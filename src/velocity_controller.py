#!/usr/bin/env python3

import sys
import os
import rospy
import numpy as np
import time
import math
from datetime import datetime
import pandas as pd
import traceback

from geometry_msgs.msg import Twist, Point, PointStamped, Quaternion, Pose, PoseStamped, Wrench, Vector3
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Float32MultiArray

from fabric_simulator.msg import SegmentStateArray
from fabric_simulator.msg import MinDistanceDataArray

from deformable_manipulations_composite_layup.msg import ControllerStatus

from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse

import tf.transformations as tf_trans

import cvxpy as cp

import threading

from scipy.spatial import KDTree

from tesseract_planner.planner import TesseractPlanner

from presaved_paths_experiments_manager import PresavedPathsExperimentsManager

# Moving average class
from convergence_checker import MovingAverageWithConvergenceAndTrend

# Set print options to reduce precision and increase line width
np.set_printoptions(precision=2, linewidth=200)

def format_number(n, digits=4):
    # Round to the specified number of decimal digits
    formatted = f"{n:.{digits}f}"
    # Remove trailing zeros and decimal point if necessary
    formatted = formatted.rstrip('0').rstrip('.')
    # Replace '-0' with '0'
    if formatted == '-0':
        return '0'
    return formatted

def pretty_print_array(arr, precision=2, width=5):
    format_spec = f"{{:>{width}.{precision}f}}"
    for row in arr:
        print(" ".join(format_spec.format(value) if value != 0 else " " * width for value in row))

        
class VelocityControllerNode:
    def __init__(self):
        # Set a flag for node initialization status
        self.initialized = False
        rospy.logwarn("VelocityControllerNode: Initializing, Please wait...")
        
        self.enabled = False  # Flag to enable/disable controller

        # To Store the time when the controller is enabled/disabled
        self.controller_enabled_time = 0.0
        self.controller_enabled_time_str = ""
        self.controller_disabled_time = 0.0
        # iteration counter for the controller to calculate the average performance
        self.controller_itr = 0 

        self.nominal_control_enabled = rospy.get_param("~nominal_control_enabled", True)
        self.obstacle_avoidance_enabled = rospy.get_param("~obstacle_avoidance_enabled", True)
        self.stress_avoidance_enabled = rospy.get_param("~stress_avoidance_enabled", True)

        # Create the service server for enable/disable the controller
        self.set_enable_controller_server = rospy.Service('~set_enable_controller', SetBool, self.set_enable_controller)

        # Create service servers for enabling/disabling each feature
        self.set_nominal_control_server = rospy.Service('~set_nominal_control_enabled', SetBool, self.set_nominal_control_enabled)
        self.set_obstacle_avoidance_server = rospy.Service('~set_obstacle_avoidance_enabled', SetBool, self.set_obstacle_avoidance_enabled)
        self.set_stress_avoidance_server = rospy.Service('~set_stress_avoidance_enabled', SetBool, self.set_stress_avoidance_enabled)

        self.initial_full_state_dict = None # To store the initial full state of the deformable object
        self.target_full_state_dict = None # To store the target full state of the deformable object
        
        # Create service servers to set and save the inital and target states of the particles
        self.set_n_save_initial_state_server = rospy.Service('~set_n_save_initial_state', SetBool, self.set_n_save_initial_state)
        self.set_n_save_target_state_server = rospy.Service('~set_n_save_target_state', SetBool, self.set_n_save_target_state)
        
        # Get the saving directory for the executions log (rosbags, plans, etc.)
        self.executions_log_directory = rospy.get_param("~executions_log_directory", "")
        
        self.reverse_after_execution = rospy.get_param("~reverse_after_execution", False)
        
        # Get the saving directory for the initial and target states of the particles
        self.state_saving_directory = rospy.get_param("~state_saving_directory", "")

        self.pub_rate_odom = rospy.get_param("~pub_rate_odom", 50)

        self.custom_static_particles = None
        self.odom_topic_prefix = None
        while (not self.custom_static_particles):
            try:
                self.custom_static_particles = rospy.get_param("/custom_static_particles") # Default static particles 
                # self.custom_static_particles = [34]
                self.odom_topic_prefix = rospy.get_param("/custom_static_particles_odom_topic_prefix") # published
            except:
                rospy.logwarn("No particles obtained from ROS parameters!.")
                time.sleep(0.5)
        
        # Create information publishers for the evaluation of the controller
        self.info_pub_controller_status = rospy.Publisher("~info_controller_status", ControllerStatus, queue_size=10) # Publishes the status of the controller when it is enabled/disabled
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up

        self.info_pub_target_pos_error_avr_norm = rospy.Publisher("~info_target_pos_error_avr_norm", Float32, queue_size=10) # average norm of the target position errors
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        self.info_pub_target_ori_error_avr_norm = rospy.Publisher("~info_target_ori_error_avr_norm", Float32, queue_size=10) # average norm of the target orientation errors
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        
        self.info_pub_overall_min_distance_collision = rospy.Publisher("~info_overall_min_distance_collision", Float32, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        
        self.info_pub_stress_avoidance_performance = rospy.Publisher("~info_stress_avoidance_performance", Float32, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        self.info_pub_stress_avoidance_performance_avr = rospy.Publisher("~info_stress_avoidance_performance_avr", Float32, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up

        self.info_pub_wildcard_array = rospy.Publisher("~info_wildcard_array", Float32MultiArray, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        self.info_pub_wildcard_scalar = rospy.Publisher("~info_wildcard_scalar", Float32, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up

        self.real_robot_mode_enabled = rospy.get_param("~real_robot_mode_enabled", False) # Flag to enable/disable the real robot mode
        
        if not self.real_robot_mode_enabled:
            # Create an (odom) Publisher for each static particle (i.e. held particles by the robots) as control output to them.
            self.odom_publishers = {}
            for particle in self.custom_static_particles:
                self.odom_publishers[particle] = rospy.Publisher(self.odom_topic_prefix + str(particle), Odometry, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        else:
            # Read the real robot topics and create the publishers
            self.robot_command_topic_names = rospy.get_param("~robot_command_topic_names", [])
            rospy.loginfo("robot_command_topic_names: " + str(self.robot_command_topic_names))
            
            # The number of robot_command_topic_names should be equal to the number of custom static particles in the DLO simulation
            if len(self.robot_command_topic_names) != len(self.custom_static_particles):
                rospy.logerr("The number of robot_command_topic_names should be equal to the number of custom static particles in the DLO simulation.\n" +
                              "The number of custom_static_particles: " + str(len(self.custom_static_particles)) + "\n" +
                                "The number of robot_command_topic_names: " + str(len(self.robot_command_topic_names)))
                rospy.signal_shutdown("The number of robot_command_topic_names should be equal to the number of custom static particles in the DLO simulation.")
                
            # Create a (twist) Publisher for each static particle (i.e. held particles by the robots) as control output to them.
            # Note that the order of the robot_command_topic_names should be the same as the order of the custom_static_particles!!
            self.twist_publishers = {}
            for i, particle in enumerate(self.custom_static_particles):
                self.twist_publishers[particle] = rospy.Publisher(self.robot_command_topic_names[i], Twist, queue_size=10)
                rospy.sleep(0.1)
            

        self.delta_x = rospy.get_param("/perturbation_publisher/delta_x", 0.1) # m
        self.delta_y = rospy.get_param("/perturbation_publisher/delta_y", 0.1) # m
        self.delta_z = rospy.get_param("/perturbation_publisher/delta_z", 0.1) # m
        self.delta_th_x = rospy.get_param("/perturbation_publisher/delta_th_x", 0.1745329) # rad
        self.delta_th_y = rospy.get_param("/perturbation_publisher/delta_th_y", 0.1745329) # rad
        self.delta_th_z = rospy.get_param("/perturbation_publisher/delta_th_z", 0.1745329) # rad

        # Controller gains  [x,y,z, Rx,Ry,Rz]
        self.kp = np.array(rospy.get_param("~kp", [1.0,1.0,1.0, 1.0,1.0,1.0]))
        self.kd = np.array(rospy.get_param("~kd", [0.0,0.0,0.0, 0.0,0.0,0.0]))

        self.k_low_pass_ft = rospy.get_param("~k_low_pass_ft", 0.9) # low pass filter coefficient for the ft values of the previous values
        self.k_low_pass_min_d = rospy.get_param("~k_low_pass_min_d", 0.5) # low pass filter coefficient for the minimum distance values of the previous values
        self.k_low_pass_convergence = rospy.get_param("~k_low_pass_convergence", 0.99) # low pass filter coefficient for the minimum distance values of the previous values

        self.max_linear_velocity = rospy.get_param("~max_linear_velocity", 0.1) # m/s
        self.max_angular_velocity = rospy.get_param("~max_angular_velocity", 0.15) # rad/s
        
        self.max_linear_acceleration = rospy.get_param("~max_linear_acceleration", 0.5) # m/s^2
        self.max_angular_acceleration = rospy.get_param("~max_angular_acceleration", 0.5) # rad/s^2

        self.acceptable_pos_err_avr_norm = rospy.get_param("~acceptable_pos_err_avr_norm", 0.01) # m
        self.acceptable_ori_err_avr_norm = rospy.get_param("~acceptable_ori_err_avr_norm", 0.1) # rad
        
        self.pos_err_avr_norm = 0.0 # float('inf') # initialize average norm of the position errors
        self.ori_err_avr_norm = 0.0 # float('inf') # initialize average norm of the orientation errors
        self.pos_err_avr_norm_prev = 0.0 # float('inf') # initialize average norm of the previous position errors
        self.ori_err_avr_norm_prev = 0.0 # float('inf') # initialize average norm of the previous orientation errors
        
        self.convergence_wait_timeout = rospy.get_param("~convergence_wait_timeout", 5.0) # seconds
        if self.convergence_wait_timeout <= 0.0:
            # set to infinite
            self.convergence_wait_timeout = float('inf')
            
        # Time when the last error change is more than the convergence thresholds
        self.time_last_error_change_is_valid = rospy.Time.now()
        
        self.convergence_threshold_pos = float(rospy.get_param("~convergence_threshold_pos", 1e-6)) # m
        self.convergence_threshold_ori = float(rospy.get_param("~convergence_threshold_ori", 1e-4)) # rad        

        # moving average of the change in the average norm of the position and orientation errors
        if np.isfinite(self.convergence_wait_timeout):
            # win_size = (self.convergence_wait_timeout*self.pub_rate_odom)//1
            win_size = (self.pub_rate_odom)//1
        else:
            win_size = 10    
            
        self.pos_err_convergence_checker = MovingAverageWithConvergenceAndTrend(n=win_size, 
                                                                                tolerance=self.convergence_threshold_pos, 
                                                                                trend_tolerance=self.convergence_threshold_pos*10, 
                                                                                stable_count=5)
        self.ori_err_convergence_checker = MovingAverageWithConvergenceAndTrend(n=win_size, 
                                                                                tolerance=self.convergence_threshold_ori, 
                                                                                trend_tolerance=self.convergence_threshold_ori*1, 
                                                                                stable_count=5)
        
        # Particle/Segment ids of the tip points of the tent pole 
        # to be placed into the grommets
        self.tip_particles = rospy.get_param("~tip_particles", [0,39])
        
        # Offset distance from the obstacles
        self.d_obstacle_offset = rospy.get_param("~d_obstacle_offset", 0.05)

        # Obstacle avoidance free zone distance
        # further than this distance, no obstacles considered by the controller 
        self.d_obstacle_freezone = rospy.get_param("~d_obstacle_freezone", 2.0)

        # Obstacle avoidance performance record variables
        self.overall_min_distance_collision = float('inf')

        # Obstacle avoidance alpha(h_obstacle) function coefficients 
        self.c1_alpha_obstacle = rospy.get_param("~c1_alpha_obstacle", 0.05)
        self.c2_alpha_obstacle = rospy.get_param("~c2_alpha_obstacle", 2.0)
        self.c3_alpha_obstacle = rospy.get_param("~c3_alpha_obstacle", 2.0)

        # Safe wrench values for the robots, assumed to be fixed and the same everywhere. 
        # TODO: Make it variable based on the robot kinematics and dynamics in the future.
        self.wrench_max = np.array(rospy.get_param("~wrench_max", [200.0, 200.0, 200.0, 15.0, 15.0, 15.0]))

        # stress offset values for each axis [Fx,Fy,Fz, Tx,Ty,Tz]
        self.w_stress_offset = np.array(rospy.get_param("~w_stress_offset", [30.0, 30.0, 30.0, 2.25, 2.25, 2.25])) 

        # alpha(h_ft) function robot stress coefficients for each axis [Fx,Fy,Fz, Tx,Ty,Tz]
        self.c1_alpha_ft = np.array(rospy.get_param("~c1_alpha_ft", [1.0, 1.0, 1.0, 0.5, 0.5, 0.5]))
        self.c2_alpha_ft = np.array(rospy.get_param("~c2_alpha_ft", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
        self.c3_alpha_ft = np.array(rospy.get_param("~c3_alpha_ft", [0.8, 0.8, 0.8, 1.0, 1.0, 1.0]))

        # stress avoidance performance record variables
        self.stress_avoidance_performance_avr = 0.0
        self.stress_avoidance_performance_sum = 0.0
        self.stress_avoidance_performance_ever_zero = False

        # Grommet poses as targets for the tip points
        # Each element holds [[x,y,z],[Rx,Ry,Rz(euler angles in degrees)]]
        target_poses_basic =  rospy.get_param("~target_poses", [[[-1, -1, 0.14], [5,   0, 90]], \
                                                                [[ 1, -1, 0.14], [175, 0, 90]]])
        
        self.target_poses = {} # Each item will be a Pose() msg class
        for i, particle in enumerate(self.tip_particles):
            self.target_poses[particle] = self.calculate_pose_msg(target_poses_basic[i])

        # Full initial and target states of the deformable object if they are provided in the parameters
        self.initial_state_file = rospy.get_param("~initial_state_file", "")
        self.target_state_file = rospy.get_param("~target_state_file", "")
        # Load the initial and target states if they are provided
        if self.initial_state_file:
            assert self.load_state("initial", self.initial_state_file)
        if self.target_state_file:
            assert self.load_state("target", self.target_state_file)

        ## ----------------------------------------------------------------------------------------
        ## SETUP FOR DEFORMABLE OBJECT STATE READINGS FROM SIMULATION PERTURBATIONS

        self.deformable_object_state_topic_name = rospy.get_param("/dlo_state_topic_name") # subscribed
        # this is also like prefix to the perturbed particles' new states

        # Dictionaries that will hold the state of the custom_static_particles and tip_particles
        self.particle_positions = {}
        self.particle_orientations = {}
        self.particle_twists = {}
        self.particle_wrenches = {}

        self.current_full_state = None # To store the current full state of the deformable object
        
        # Subscriber for deformable object states to figure out the current particle positions
        self.sub_state = rospy.Subscriber(self.deformable_object_state_topic_name, SegmentStateArray, self.state_array_callback, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        
        # Subscribers to the particle states with perturbations      
        ## We create 6 subscribers for perturbed states of each custom static particle
        self.subs_state_dx = {}
        self.subs_state_dy = {}
        self.subs_state_dz = {}
        self.subs_state_dth_x = {}
        self.subs_state_dth_y = {}
        self.subs_state_dth_z = {}

        # Dictionaries to store perturbed states of the custom static particles
        self.particle_positions_dx = {}
        self.particle_orientations_dx = {}
        self.particle_twists_dx = {}
        self.particle_wrenches_dx = {}

        self.particle_positions_dy = {}
        self.particle_orientations_dy = {}
        self.particle_twists_dy = {}
        self.particle_wrenches_dy = {}

        self.particle_positions_dz = {}
        self.particle_orientations_dz = {}
        self.particle_twists_dz = {}
        self.particle_wrenches_dz = {}

        self.particle_positions_dth_x = {}
        self.particle_orientations_dth_x = {}
        self.particle_twists_dth_x = {}
        self.particle_wrenches_dth_x = {}

        self.particle_positions_dth_y = {}
        self.particle_orientations_dth_y = {}
        self.particle_twists_dth_y = {}
        self.particle_wrenches_dth_y = {}

        self.particle_positions_dth_z = {}
        self.particle_orientations_dth_z = {}
        self.particle_twists_dth_z = {}
        self.particle_wrenches_dth_z = {}

        self.states_set_particles = [] # For bookkeeping of which custom static particles are obtained their all perturbed state readings at least once.

        ## Create the subscribers to states with perturbations
        for particle in self.custom_static_particles:
            # Prepare the topic names of that particle
            state_dx_topic_name    = self.deformable_object_state_topic_name + "_" + str(particle) + "_x" 
            state_dy_topic_name    = self.deformable_object_state_topic_name + "_" + str(particle) + "_y" 
            state_dz_topic_name    = self.deformable_object_state_topic_name + "_" + str(particle) + "_z" 
            state_dth_x_topic_name = self.deformable_object_state_topic_name + "_" + str(particle) + "_th_x" 
            state_dth_y_topic_name = self.deformable_object_state_topic_name + "_" + str(particle) + "_th_y" 
            state_dth_z_topic_name = self.deformable_object_state_topic_name + "_" + str(particle) + "_th_z" 

            # Create the subscribers (also takes the particle argument)
            self.subs_state_dx[particle]    = rospy.Subscriber(state_dx_topic_name,    SegmentStateArray, self.state_array_dx_callback,    particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_state_dy[particle]    = rospy.Subscriber(state_dy_topic_name,    SegmentStateArray, self.state_array_dy_callback,    particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_state_dz[particle]    = rospy.Subscriber(state_dz_topic_name,    SegmentStateArray, self.state_array_dz_callback,    particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_state_dth_x[particle] = rospy.Subscriber(state_dth_x_topic_name, SegmentStateArray, self.state_array_dth_x_callback, particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_state_dth_y[particle] = rospy.Subscriber(state_dth_y_topic_name, SegmentStateArray, self.state_array_dth_y_callback, particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_state_dth_z[particle] = rospy.Subscriber(state_dth_z_topic_name, SegmentStateArray, self.state_array_dth_z_callback, particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        
        ## ----------------------------------------------------------------------------------------
        ## SETUP FOR MINIMUM DISTANCE READINGS FROM SIMULATION PERTURBATIONS
        self.min_distance_topic_name = rospy.get_param("/min_dist_to_rb_topic_name") # subscribed, 
        # this is also like prefix to the perturbed particles' new minimum distances

        # Subscriber to figure out the current deformable object minimum distances to the rigid bodies in the scene 
        self.sub_min_distance = rospy.Subscriber(self.min_distance_topic_name, MinDistanceDataArray, self.min_distances_array_callback, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up

        # Subscribers to the particle minimum distances with perturbations      
        ## We create 6 subscribers for perturbed states of each custom static particle
        self.subs_min_distance_dx = {}
        self.subs_min_distance_dy = {}
        self.subs_min_distance_dz = {}
        self.subs_min_distance_dth_x = {}
        self.subs_min_distance_dth_y = {}
        self.subs_min_distance_dth_z = {}

        # Dictionaries to store minimum distances caused by the perturbation on the custom static particles
        self.min_distances = {}
        self.min_distances_dx = {}
        self.min_distances_dy = {}
        self.min_distances_dz = {}
        self.min_distances_dth_x = {}
        self.min_distances_dth_y = {}
        self.min_distances_dth_z = {}

        self.min_distances_set_particles = [] # For bookkeeping of which custom static particles are obtained their all perturbed min distance readings at least once.
        self.min_distances_set_particles_obstacles = [] 

        ## Create the subscribers to minimum distances with perturbations
        for particle in self.custom_static_particles:
            # Prepare the topic names of that particle
            min_distance_dx_topic_name    = self.min_distance_topic_name + "_" + str(particle) + "_x" 
            min_distance_dy_topic_name    = self.min_distance_topic_name + "_" + str(particle) + "_y" 
            min_distance_dz_topic_name    = self.min_distance_topic_name + "_" + str(particle) + "_z" 
            min_distance_dth_x_topic_name = self.min_distance_topic_name + "_" + str(particle) + "_th_x" 
            min_distance_dth_y_topic_name = self.min_distance_topic_name + "_" + str(particle) + "_th_y" 
            min_distance_dth_z_topic_name = self.min_distance_topic_name + "_" + str(particle) + "_th_z" 
        
            # Create the subscribers (also takes the particle argument)
            self.subs_min_distance_dx[particle]    = rospy.Subscriber(min_distance_dx_topic_name,    MinDistanceDataArray, self.min_distance_array_dx_callback,    particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_min_distance_dy[particle]    = rospy.Subscriber(min_distance_dy_topic_name,    MinDistanceDataArray, self.min_distance_array_dy_callback,    particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_min_distance_dz[particle]    = rospy.Subscriber(min_distance_dz_topic_name,    MinDistanceDataArray, self.min_distance_array_dz_callback,    particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_min_distance_dth_x[particle] = rospy.Subscriber(min_distance_dth_x_topic_name, MinDistanceDataArray, self.min_distance_array_dth_x_callback, particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_min_distance_dth_y[particle] = rospy.Subscriber(min_distance_dth_y_topic_name, MinDistanceDataArray, self.min_distance_array_dth_y_callback, particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            self.subs_min_distance_dth_z[particle] = rospy.Subscriber(min_distance_dth_z_topic_name, MinDistanceDataArray, self.min_distance_array_dth_z_callback, particle, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        ## ----------------------------------------------------------------------------------------

        ## ----------------------------------------------------------------------------------------
        ### Path planning parameters and variables
        self.path_tracking_control_enabled = rospy.get_param("~path_tracking_control_enabled", True)
        self.path_planning_tesseract_enabled = rospy.get_param("~path_planning_tesseract_enabled", False)
        self.path_planning_pre_saved_paths_enabled = rospy.get_param("~path_planning_pre_saved_paths_enabled", False)
        
        self.replanning_allowed = rospy.get_param("~replanning_allowed", False)
        self.max_replanning_attempts = rospy.get_param("~max_replanning_attempts", 3)
        self.num_replanning_attempts = 0
        self.is_replanning_needed = False

        if self.path_planning_tesseract_enabled:
            
            # Get deformable object simulator scene json file path from the ROS parameter server
            self.rb_scene_config_path = None
            self.dlo_l  = None # For plan request
            self.dlo_r = None # For plan request
            while (not self.rb_scene_config_path):
                try:
                    self.rb_scene_config_path = rospy.get_param("/rb_scene_config_path")
                    self.dlo_l = rospy.get_param("/dlo_l") # For plan request
                    self.dlo_r = rospy.get_param("/dlo_r") # For plan request
                except:
                    rospy.logwarn("Deformable object simulator scene json file path is not provided!")
                    time.sleep(2)

            # Get the tesseract resource path from the ROS parameter server
            self.tesseract_resource_path = rospy.get_param("~tesseract_resource_path") 
                    
            # Get the urdf and srdf file paths from the ROS parameter server for the simplified fabric
            self.tesseract_tent_pole_urdf = rospy.get_param("~tesseract_tent_pole_urdf", "") # string
            self.tesseract_tent_pole_srdf = rospy.get_param("~tesseract_tent_pole_srdf") # string
            
            self.tesseract_tent_pole_tcp_frame = rospy.get_param("~tesseract_tent_pole_tcp_frame")
            self.tesseract_tent_pole_manipulator_group = rospy.get_param("~tesseract_tent_pole_manipulator_group")
            self.tesseract_tent_pole_manipulator_working_frame = rospy.get_param("~tesseract_tent_pole_manipulator_working_frame")
            
            self.tesseract_use_default_viewer = rospy.get_param("~tesseract_use_default_viewer", False)
            
            # Create tessaract planner object
            self.tesseract_planner = TesseractPlanner(self.rb_scene_config_path,
                                                        self.tesseract_resource_path,
                                                        self.tesseract_tent_pole_urdf,
                                                        self.tesseract_tent_pole_srdf,
                                                        self.tesseract_tent_pole_tcp_frame,
                                                        self.tesseract_tent_pole_manipulator_group,
                                                        self.tesseract_tent_pole_manipulator_working_frame,
                                                        self.tesseract_use_default_viewer)
            
            ## Plan request parameters
            self.automatic_tent_pole_max_links = rospy.get_param("~automatic_tent_pole_max_links", 10)
            self.automatic_tent_pole_min_links = rospy.get_param("~automatic_tent_pole_min_links", 1)
            self.dlo_approximation_error_threshold = rospy.get_param("~dlo_approximation_error_threshold", 0.02)
            self.coll_depth_to_try_remove = rospy.get_param("~coll_depth_to_try_remove", 0.01)
            self.environment_limits_xyz = rospy.get_param("~environment_limits_xyz", [-1, 1, -1, 1, -1, 1])
            self.joint_angle_limits_xyz_deg = rospy.get_param("~joint_angle_limits_xyz_deg", [-90, 90, -180, 180, -40, 40])
            
        # Create variables to hold the planned path 
        self.reset_planned_path_variables()

        # Path Tracking Controller gains  [x,y,z, Rx,Ry,Rz]
        self.kp_path_tracking = np.array(rospy.get_param("~kp_path_tracking", [1.0,1.0,1.0, 1.0,1.0,1.0]))
        self.kd_path_tracking = np.array(rospy.get_param("~kd_path_tracking", [0.0,0.0,0.0, 0.0,0.0,0.0]))

        # Path tracking switch off parameters used in the transition function
        # A piecewise smooth transition function is used to switch off the path tracking controller smoothly to the nominal controller
        # when the robot is close to the end of path. See: 
        
        # the distance from the end of the path to start switching off the path tracking controller
        self.d_path_tracking_start_switch_off_distance = rospy.get_param("~d_path_tracking_start_switch_off_distance", 0.35)         
        # the distance from the end of the path to completely switch off the path tracking controller
        self.d_path_tracking_complete_switch_off_distance = rospy.get_param("~d_path_tracking_complete_switch_off_distance", 0.05)

        # Add the described parameters below:
        # Feedforward velocity scale factors for the path tracking controller
        # These values are multiplied with the maximum velocity limits to scale down the velocity commands of the path tracking
        # They are in the range of [0,1]. If the value is 1.0, the maximum velocity limits are used as the feedforward velocity commands.
        # path_tracking_feedforward_linear_velocity_scale_factor: 0.5
        # path_tracking_feedforward_angular_velocity_scale_factor: 0.5
        self.path_tracking_feedforward_linear_velocity_scale_factor =  np.clip(rospy.get_param("~path_tracking_feedforward_linear_velocity_scale_factor", 0.5), 0.0, 1.0)
        self.path_tracking_feedforward_angular_velocity_scale_factor = np.clip(rospy.get_param("~path_tracking_feedforward_angular_velocity_scale_factor", 0.5), 0.0, 1.0)
        
        if self.path_planning_tesseract_enabled:
            # Create the information publishers for the planned path
            self.info_pub_planned_path_current_target_point = rospy.Publisher("~info_planned_path_current_target_point", PointStamped, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            
            # Create the planned path publisher
            self.path_pub = rospy.Publisher("~planned_path", Path, queue_size=10)
            rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            
            # Create the planned path publisher for the particles
            # Create the information publishers for the current target point of the planned path
            self.path_pub_particles = {}
            self.info_pub_planned_path_current_target_point_particles = {}

            for particle in self.custom_static_particles:
                self.path_pub_particles[particle] = rospy.Publisher("~planned_path_particle_" + str(particle), Path, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
                self.info_pub_planned_path_current_target_point_particles[particle] = rospy.Publisher("~info_planned_path_current_target_point_particle_" + str(particle), 
                                                                                                      PointStamped, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up

            self.update_path_status_timer_callback_lock = threading.Lock()
            # Create timer to update the path status and tracking
            self.update_path_status_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.update_path_status_timer_callback)
        ## ----------------------------------------------------------------------------------------
        
        ## ----------------------------------------------------------------------------------------
        ## Experiments with pre-saved paths paramaters and variables
        self.experiments_scene_id = rospy.get_param("~experiments_scene_id", "1")
        self.experiments_dlo_type = rospy.get_param("~experiments_dlo_type", None)
        
        if self.experiments_dlo_type == "None" or self.experiments_dlo_type == "none" or self.experiments_dlo_type == "NONE" or self.experiments_dlo_type == "" or self.experiments_dlo_type == " ":
            self.experiments_dlo_type = None
                
        self.experiments_range = rospy.get_param("~experiments_range", [1,100])
        # Ensure the parameter is always treated as a list
        if not isinstance(self.experiments_range, list):
            self.experiments_range = [self.experiments_range]
        # Handle the cases according to the explanation in the parameter file
        if len(self.experiments_range) == 2:
            # If exactly two values are provided, assume it is a range
            start, end = self.experiments_range
            self.experiments_range = list(range(start, end + 1))
        else:
            # If only one value or more than two values are given, treat it as a list of experiments
            self.experiments_range = self.experiments_range
        
        self.experiments_saved_paths_directory = rospy.get_param("~experiments_saved_paths_directory")
        
        # self.experiments_enabled = False # Flag to enable/disable the experiments
        
        # Create the service server for enable/disable the experiments
        self.set_enable_experiments_server = rospy.Service('~set_enable_experiments', SetBool, self.set_enable_experiments)
        
        # Create the experiments manager object
        if self.path_planning_pre_saved_paths_enabled:
            self.experiments_manager = PresavedPathsExperimentsManager(self, 
                                                                    self.experiments_scene_id, 
                                                                    self.experiments_range, 
                                                                    self.experiments_saved_paths_directory,
                                                                    self.experiments_dlo_type)
        else:
            self.experiments_manager = PresavedPathsExperimentsManager(self,
                                                                    None, 
                                                                    None,
                                                                    None,
                                                                    None)
        
        ## ----------------------------------------------------------------------------------------

        ## ----------------------------------------------------------------------------------------
        # Control output wait timeout
        self.valid_control_output_wait_timeout = rospy.get_param("~valid_control_output_wait_timeout", 5.0) # seconds
        if self.valid_control_output_wait_timeout <= 0.0:
            # set to infinite
            self.valid_control_output_wait_timeout = float('inf')

        # Time when the last control output is valid
        self.time_last_control_output_is_valid = rospy.Time.now()
            
        
        self.control_outputs = {} 
        for particle in self.custom_static_particles:
            self.control_outputs[particle] = np.zeros(6) # initialization for the velocity command
            
        self.control_outputs_last = {}
        for particle in self.custom_static_particles:
            self.control_outputs_last[particle] = np.zeros(6)

        # Start the control
        self.calculate_control_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.calculate_control_outputs_timer_callback)
        self.odom_pub_timer          = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)

        # # Event to control the execution of one callback based on user input
        # self.proceed_event = threading.Event()
        ## ----------------------------------------------------------------------------------------
        self.initialized = True
        rospy.loginfo("Velocity controller is initialized.")

    #     # Start thread for user input
    #     self.input_thread = threading.Thread(target=self.user_input_thread)
    #     self.input_thread.start()

    # def user_input_thread(self):
    #     while not rospy.is_shutdown():
    #         input("Press Enter to allow the control calculations to proceed...")
    #         self.proceed_event.set()

    # Implement the set_n_save_initial_state and set_n_save_target_state services
    def set_n_save_initial_state(self, request=None):
        result = self.full_states_setter_n_saver("initial", save=True)
        if result:
            return SetBoolResponse(True, 'Successfully set the initial state of the particles.')
        else:
            return SetBoolResponse(False, 'Failed to set the initial state of the particles.')

    def set_n_save_target_state(self, request=None):
        result = self.full_states_setter_n_saver("target", save=True)
        if result:
            return SetBoolResponse(True, 'Successfully set the initial state of the particles.')
        else:
            return SetBoolResponse(False, 'Failed to set the initial state of the particles.')
    
    def full_states_setter_n_saver(self, state_type, save=True):
        """
        Saves the states of the particles to a file.

        Args:
            state_type (str): The type of the state to be saved. It can be either "initial" or "target". 
                              If any other string is given, it will be considered as "unknown".
        """
            
        saving_dir = os.path.expanduser(self.state_saving_directory)
        if not os.path.exists(saving_dir):
            os.makedirs(saving_dir)
            rospy.loginfo("The directory is created: {}".format(saving_dir))

        # Save the states of the particles to a file
        # The file name will be like: 2024-12-31-17-41-34_initial_states.csv
        # or 2024-12-31-17-41-34_target_states.csv
        file_name = self.get_system_timestamp() + "_" + state_type + "_states.csv"
        file_path = os.path.join(saving_dir, file_name)

        if self.current_full_state:
            try:
                if state_type == "initial":
                    # Set the initial state of the particles to the current state
                    self.initial_full_state_dict = self.parse_state_as_dict(self.current_full_state)
                    rospy.loginfo("The initial states of the particles are set.")            
                    if save:
                        self.save_state_dict_to_csv(self.initial_full_state_dict, file_path)
                    
                elif state_type == "target":
                    # Set the target state of the particles to the current state
                    self.target_full_state_dict = self.parse_state_as_dict(self.current_full_state)
                    rospy.loginfo("The target states of the particles are set.")
                    if save:
                        self.save_state_dict_to_csv(self.target_full_state_dict, file_path)
                    
                else:
                    rospy.logerr("Unknown state type: {}".format(state_type))
                    return False
                        
            # except with the full traceback
            except Exception:
                rospy.logerr("An error occurred while saving the states of the particles: {}".format(traceback.format_exc()))
                return False
            
            if save:
                rospy.loginfo("The {} states of the particles are saved to the file: {}".format(state_type, file_path))
            return True                
        else:
            rospy.logerr("The current full state of the particles is not available yet!")
            return False
    
    def log_initial_n_target_full_states(self):
        saving_dir = os.path.expanduser(self.executions_log_directory)
        if not os.path.exists(saving_dir):
            os.makedirs(saving_dir)
            rospy.loginfo("The directory is created: {}".format(saving_dir))
        
        file_name = self.controller_enabled_time_str + "_initial_states.csv"
        file_path = os.path.join(saving_dir, file_name)
        
        self.save_state_dict_to_csv(self.initial_full_state_dict, file_path)
        
        file_name = self.controller_enabled_time_str + "_target_states.csv"
        file_path = os.path.join(saving_dir, file_name)
        
        self.save_state_dict_to_csv(self.target_full_state_dict, file_path)
        
    def parse_state_as_dict(self, states_msg):
        # Parse SegmentStateArray from fabric_simulator.msg as a dictionary
        # with "id,p_x,p_y,p_z,o_x,o_y,o_z,o_w" are the keys
        # where id is the particle id and p_x, p_y, p_z are the position components
        # and o_x, o_y, o_z, o_w are the orientation components
        
        # Initialize an empty dictionary to store data
        data = {'id': [], 'p_x': [], 'p_y': [], 'p_z': [], 'o_x': [], 'o_y': [], 'o_z': [], 'o_w': []}
        
        # Iterate through each SegmentState in the SegmentStateArray
        for state in states_msg.states:
            # Extract the segment id
            data['id'].append(state.id)
            
            # Extract the position components
            data['p_x'].append(state.pose.position.x)
            data['p_y'].append(state.pose.position.y)
            data['p_z'].append(state.pose.position.z)
            
            # Extract the orientation components
            data['o_x'].append(state.pose.orientation.x)
            data['o_y'].append(state.pose.orientation.y)
            data['o_z'].append(state.pose.orientation.z)
            data['o_w'].append(state.pose.orientation.w)
        
        return data
    
    def convert_state_dict_to_numpy(self, state_dict):
        # Extract the lists from the dictionary
        p_x = state_dict['p_x']
        p_y = state_dict['p_y']
        p_z = state_dict['p_z']
        o_x = state_dict['o_x']
        o_y = state_dict['o_y']
        o_z = state_dict['o_z']
        o_w = state_dict['o_w']

        # Stack the lists into a 2D numpy array and transpose it
        dlo_state = np.array([p_x, p_y, p_z, o_x, o_y, o_z, o_w]).T # shape: (n, 7)

        return dlo_state
        
    def save_state_dict_to_csv(self, state_dict, file_path):
        """
        Save the state dictionary to a CSV file. If a file with the given 
        file_path already exists, a suffix (_1, _2, etc.) will be appended
        to prevent overwriting.

        :param state_dict: Dictionary containing states to be saved.
        :param file_path:  Full path (directory + filename) for the CSV file.
        """
        # Check if file already exists
        if os.path.exists(file_path):
            base, extension = os.path.splitext(file_path)
            counter = 1
            # Keep adding suffix until no file with that name exists
            while os.path.exists(file_path):
                file_path = f"{base}_{counter}{extension}"
                counter += 1
        
        # Create the DataFrame from the dictionary
        df = pd.DataFrame(state_dict)

        # Sort the DataFrame by 'id' in ascending order
        df.sort_values(by='id', ascending=True, inplace=True)

        # Save the DataFrame to the CSV file
        df.to_csv(file_path, index=False)

    def read_state_dict_from_csv(self, file):
        # Load the data from the CSV file
        df = pd.read_csv(file)
        
        # Convert DataFrame to dictionary
        state_dict = df.to_dict(orient='list')
        return state_dict
    
    def load_state(self, state_type, state_file):
        # Load the state from the file
        state_dict = self.read_state_dict_from_csv(state_file)
        
        if state_type == "initial":
            self.initial_full_state_dict = state_dict
            return True
        elif state_type == "target":
            self.target_full_state_dict = state_dict
            return True
        else:
            rospy.logerr("Unknown state type: {}".format(state_type))
            return False
    
    def calculate_state_rmse_error(self, state_dict1, state_dict2):
        # Calculate the position error between two states
        # The states are given as dictionaries with the following keys:
        # 'id', 'p_x', 'p_y', 'p_z', 'o_x', 'o_y', 'o_z', 'o_w'
        
        # Convert the dictionaries to numpy arrays
        state1 = self.convert_state_dict_to_numpy(state_dict1)[:,:3] # Extract the position components
        state2 = self.convert_state_dict_to_numpy(state_dict2)[:,:3] # Extract the position components
        
        # Calculate the error
        error = state1 - state2 # shape: (n, 3)
        # Calculate the Root Mean Squared Error (RMSE)
        rmse = np.sqrt(np.mean(np.linalg.norm(error, axis=1)**2))
        return rmse 
    
    def calculate_final_task_error(self):
        # Calculate the final task error between the current and target states
        current_state_dict = self.parse_state_as_dict(self.current_full_state)
        return self.calculate_state_rmse_error(current_state_dict, self.target_full_state_dict)
    
    def set_enable_controller(self, request):
        self.controller_enabler(enable=request.data, cause="manual")
        return SetBoolResponse(True, 'Successfully set enabled state to {}'.format(request.data))
    
    def controller_enabler(self, enable, cause="manual"):
        if not enable:
            self.enabled = False
            
            # Stop the particles
            self.odom_publishers_publish_zero_velocities()

        ## Publish the controller status information
        status_msg = ControllerStatus()
        # Fill in common fields
        status_msg.time = rospy.Time.now()  # Example: current time
        status_msg.cause = cause 
        status_msg.enabled = enable

        if enable:
            self.controller_enabled_time = rospy.Time.now()
            self.controller_enabled_time_str = self.get_system_timestamp()
            
            status_msg.total_duration = -1.0  # -1.0 means the controller is still enabled
            status_msg.rate = -1.0  # -1.0 means the controller is still enabled
            
            # Reset the performance metrics
            self.controller_itr = 0
            self.stress_avoidance_performance_avr = 0.0
            self.stress_avoidance_performance_sum = 0.0
            self.stress_avoidance_performance_ever_zero = False
            self.overall_min_distance_collision = float('inf')

            status_msg.stress_avoidance_performance_avr = 0.0
            status_msg.min_distance = float('inf')
            
            # Reset the number of replanning attempts and the replanning needed flag
            self.is_replanning_needed = False
            self.num_replanning_attempts = 0
            
            self.pos_err_avr_norm = 0.0 # float('inf') # initialize average norm of the position errors
            self.ori_err_avr_norm = 0.0 # float('inf') # initialize average norm of the orientation errors
            self.pos_err_avr_norm_prev = 0.0 # float('inf') # initialize average norm of the previous position errors
            self.ori_err_avr_norm_prev = 0.0 # float('inf') # initialize average norm of the previous orientation errors
            
            self.pos_err_convergence_checker.reset() # reset the position error convergence checker
            self.ori_err_convergence_checker.reset() # reset the orientation error convergence checker
            
            if cause == "manual":
                self.experiments_manager.start_rosbag_recording_manual(self.controller_enabled_time_str, 
                                                                       self.executions_log_directory)
            
            self.enabled = True
            rospy.loginfo("-------------- Controller is enabled --------------")
        else:
            self.controller_disabled_time = rospy.Time.now()
            # Calculate the duration when the controller is disabled
            dur = (self.controller_disabled_time - self.controller_enabled_time).to_sec() # seconds
            status_msg.total_duration = dur  # seconds
            # status_msg.total_duration = dur - self.convergence_wait_timeout # seconds
            status_msg.rate = self.controller_itr / dur # rate of the controller iterations
            
            status_msg.stress_avoidance_performance_avr = self.stress_avoidance_performance_avr
            status_msg.min_distance = self.overall_min_distance_collision

            # ----------------------------------------------------------------------------------------
            ## Print the performance metrics suitable for a csv file
            rospy.loginfo("-------------- Controller is Disabled --------------")
            rospy.loginfo("Performance metrics (CSV suitable):")
            rospy.loginfo("Titles: ft_on, collision_on, success, min_distance, rate, duration, stress_avoidance_performance_avr, stress_avoidance_performance_ever_zero, start_time, final_task_error, is_replanning_needed, num_replanning_attempts")

            ft_on_str = "1" if self.stress_avoidance_enabled else "0"
            collision_on_str = "1" if self.obstacle_avoidance_enabled else "0"
            success_str = "1" if (cause == "automatic") else "0"
            min_distance_str = str(self.overall_min_distance_collision)
            rate_str = str(status_msg.rate)
            duration_str = str(status_msg.total_duration)
            stress_avoidance_performance_avr_str = str(self.stress_avoidance_performance_avr)
            stress_avoidance_performance_ever_zero_str = "1" if self.stress_avoidance_performance_ever_zero else "0"
            # Create start time string with YYYY-MM-DD-Hour-Minute-Seconds format for example 2024-12-31-17-41-34
            start_time_str = self.controller_enabled_time_str
            # Calculate the final task error as RMSE between the current and target states
            final_task_error_str = str(self.calculate_final_task_error()) if (self.target_full_state_dict is not None) else "0.0"
            is_replanning_needed_str = "1" if self.is_replanning_needed else "0"
            num_replanning_attempts_str = str(int(self.num_replanning_attempts))

            execution_results = [ft_on_str, collision_on_str, success_str, 
                                 min_distance_str, rate_str, duration_str, 
                                 stress_avoidance_performance_avr_str, 
                                 stress_avoidance_performance_ever_zero_str, 
                                 start_time_str,final_task_error_str,
                                 is_replanning_needed_str, num_replanning_attempts_str]
            csv_line = "Result: " + ",".join(execution_results)
            # Print the csv line green color if the controller is successful else red color
            if (cause == "automatic"):
                # Print the csv line orange color if stress avoidance performance is ever zero
                if self.stress_avoidance_performance_ever_zero:
                    rospy.loginfo("\033[93m" + csv_line + "\033[0m")
                    # rospy.loginfo(csv_line)
                else:
                    # Print the csv line green color if the controller is successful and stress avoidance performance is never zero
                    rospy.loginfo("\033[92m" + csv_line + "\033[0m")
                    # rospy.loginfo(csv_line)
            else:
                rospy.loginfo("\033[91m" + csv_line + "\033[0m")
                # rospy.loginfo(csv_line)
            # ----------------------------------------------------------------------------------------

            # Reset the planned path variables
            self.reset_planned_path_variables()
            
            # Save the execution results to a csv file
            if self.path_planning_pre_saved_paths_enabled:
                self.experiments_manager.end_experiment(cause=cause, execution_results=execution_results)
            else:
                self.experiments_manager.stop_rosbag_recording()
                
                # Calculate the executed path length
                exec_path_length, _ = self.experiments_manager.calculate_executed_path_length()
                rospy.loginfo("Executed path length: %f" % exec_path_length)
                
                # Append path length to execution_results
                if execution_results is not None:
                    execution_results.append(str(exec_path_length))
                    
                    # Save results to csv file
                    self.experiments_manager.save_experiment_results_manual(execution_results, self.controller_enabled_time_str, self.executions_log_directory)
                else:
                    self.experiments_manager.save_experiment_results_manual(execution_results, self.controller_enabled_time_str, self.executions_log_directory)
                
                if self.reverse_after_execution:
                    # Reverse the executed path
                    self.experiments_manager.reverse_executed_path(speed_multiplier=5.0)
                    
                    # Re-apply the initial state
                    self.experiments_manager.apply_initial_state()

        # Publish the status message
        self.info_pub_controller_status.publish(status_msg)
        
        # Once again, 
        # Publish zero velocities for all particles if the controller is disabled 
        # To make sure the particles are not moving when the controller is disabled
        if not self.enabled:
            # Stop the particles
            self.odom_publishers_publish_zero_velocities()

    def set_nominal_control_enabled(self, request):
        self.nominal_control_enabled = request.data
        rospy.loginfo("Nominal control enabled state set to {}".format(request.data))
        return SetBoolResponse(True, 'Successfully set nominal control enabled state to {}'.format(request.data))

    def set_obstacle_avoidance_enabled(self, request):
        self.obstacle_avoidance_enabled = request.data
        rospy.loginfo("Obstacle avoidance enabled state set to {}".format(request.data))
        return SetBoolResponse(True, 'Successfully set obstacle avoidance enabled state to {}'.format(request.data))

    def set_stress_avoidance_enabled(self, request):
        self.stress_avoidance_enabled = request.data
        rospy.loginfo("Stress avoidance enabled state set to {}".format(request.data))
        return SetBoolResponse(True, 'Successfully set stress avoidance enabled state to {}'.format(request.data))
        
    def calculate_pose_msg(self, pose_basic):
        # pose_basic: Holds the target pose as a list formatted as [[x,y,z],[Rx,Ry,Rz(euler angles in degrees)]]
        # This function converts it to a Pose msg

        # Extract position and orientation from pose_basic
        position_data, orientation_data = pose_basic

        # Convert orientation from Euler angles (degrees) to radians
        orientation_radians = np.deg2rad(orientation_data)

        # Convert orientation from Euler angles to quaternion
        quaternion_orientation = tf_trans.quaternion_from_euler(*orientation_radians)

        # Prepare the pose msg
        pose_msg = Pose()
        pose_msg.position = Point(*position_data)
        pose_msg.orientation = Quaternion(*quaternion_orientation)
        return pose_msg

    def state_array_callback(self, states_msg):
        if self.initialized:
            self.current_full_state = states_msg
            
            for particle in (self.custom_static_particles + self.tip_particles):
                self.particle_positions[particle] = states_msg.states[particle].pose.position
                self.particle_orientations[particle] = states_msg.states[particle].pose.orientation
                self.particle_twists[particle] = states_msg.states[particle].twist

                wrench_array = self.wrench_to_numpy(states_msg.states[particle].wrench)
                if particle not in self.particle_wrenches:
                    self.particle_wrenches[particle] = wrench_array
                else:
                    # Apply low-pass filter to the force and torque values
                    self.particle_wrenches[particle] = self.k_low_pass_ft*self.particle_wrenches[particle] + (1 - self.k_low_pass_ft)*wrench_array

    def state_array_dx_callback(self, states_msg, perturbed_particle):
        if self.initialized:
            self.update_state_arrays(states_msg, perturbed_particle, self.particle_positions_dx, self.particle_orientations_dx, self.particle_twists_dx, self.particle_wrenches_dx)

    def state_array_dy_callback(self, states_msg, perturbed_particle):
        if self.initialized:
            self.update_state_arrays(states_msg, perturbed_particle, self.particle_positions_dy, self.particle_orientations_dy, self.particle_twists_dy, self.particle_wrenches_dy)

    def state_array_dz_callback(self, states_msg, perturbed_particle):
        if self.initialized:
            self.update_state_arrays(states_msg, perturbed_particle, self.particle_positions_dz, self.particle_orientations_dz, self.particle_twists_dz, self.particle_wrenches_dz)

    def state_array_dth_x_callback(self, states_msg, perturbed_particle):
        if self.initialized:
            self.update_state_arrays(states_msg, perturbed_particle, self.particle_positions_dth_x, self.particle_orientations_dth_x, self.particle_twists_dth_x, self.particle_wrenches_dth_x)

    def state_array_dth_y_callback(self, states_msg, perturbed_particle):
        if self.initialized:
            self.update_state_arrays(states_msg, perturbed_particle, self.particle_positions_dth_y, self.particle_orientations_dth_y, self.particle_twists_dth_y, self.particle_wrenches_dth_y)

    def state_array_dth_z_callback(self, states_msg, perturbed_particle):
        if self.initialized:
            self.update_state_arrays(states_msg, perturbed_particle, self.particle_positions_dth_z, self.particle_orientations_dth_z, self.particle_twists_dth_z, self.particle_wrenches_dth_z)

    def update_state_arrays(self, states_msg, perturbed_particle, positions_dict, orientations_dict, twists_dict, wrenches_dict):
        if not (perturbed_particle in positions_dict):
            positions_dict[perturbed_particle] = {}
            orientations_dict[perturbed_particle] = {}
            twists_dict[perturbed_particle] = {}
            wrenches_dict[perturbed_particle] = {}

        for particle in (self.custom_static_particles + self.tip_particles):
            positions_dict[perturbed_particle][particle] = states_msg.states[particle].pose.position
            orientations_dict[perturbed_particle][particle] = states_msg.states[particle].pose.orientation
            twists_dict[perturbed_particle][particle] = states_msg.states[particle].twist

            wrench_array = self.wrench_to_numpy(states_msg.states[particle].wrench)
            if particle not in wrenches_dict[perturbed_particle]:
                wrenches_dict[perturbed_particle][particle] = wrench_array
            else:
                # Apply low-pass filter to the force and torque values
                wrenches_dict[perturbed_particle][particle] = self.k_low_pass_ft*wrenches_dict[perturbed_particle][particle] + (1 - self.k_low_pass_ft)*wrench_array

    def is_perturbed_states_set_for_particle(self,particle):
        """
        Checks if the perturbed state parameters (dx, dy, dz, dthx, dthy, dthz) are set for a specified particle.
        This function determines if the custom static particle has obtained perturbed states along the x, y, z axes at least once.
        If these states are set, the particle is added to the 'states_set_particles' list,
        and the function returns True if the 'self.particle_positions' of that particle is not None. Otherwise, it returns False.
        """
        if particle in self.states_set_particles:
            return (particle in self.particle_positions)
        else:
            check = ((particle in self.particle_positions_dx) and  
                     (particle in self.particle_positions_dy) and 
                     (particle in self.particle_positions_dz) and
                     (particle in self.particle_positions_dth_x) and  
                     (particle in self.particle_positions_dth_y) and 
                     (particle in self.particle_positions_dth_z))
            if check:
                self.states_set_particles.append(particle)
                return (particle in self.particle_positions)
            else:
                return False

    def calculate_jacobian_tip(self):
        """
        Calculates the Jacobian matrix that defines the relation btw. 
        the robot hold points (custom_static_particles) 6DoF poses and
        the tent pole tip points (tip_particles) 6 DoF poses.
        The result is a 12x12 matrix for 2 tip and 2 holding point poses. 
        """
        J = np.zeros((6*len(self.tip_particles),6*len(self.custom_static_particles)))

        for idx_tip, tip in enumerate(self.tip_particles):
            for idx_particle, particle in enumerate(self.custom_static_particles):
                
                # Do not proceed until the initial values have been set
                if ((not self.is_perturbed_states_set_for_particle(particle))):
                    rospy.logwarn_throttle(1,"[calculate_jacobian_tip func.] particle: " + str(particle) + " state is not published yet or it does not have for perturbed states.")
                    continue

                # calculate the pose differences:
                # btw. the current tip pose and the perturbated tip pose caused by the perturbation of custom_static_particle pose in each direction
                current_pose = Pose()
                perturbed_pose = Pose()

                current_pose.position = self.particle_positions[tip]
                current_pose.orientation = self.particle_orientations[tip]
                
                # dx direction
                perturbed_pose.position = self.particle_positions_dx[particle][tip]
                perturbed_pose.orientation = self.particle_orientations_dx[particle][tip]
                pose_difference = self.calculate_pose_difference(current_pose,perturbed_pose)
                J[ (6*idx_tip) : (6*(idx_tip+1)) , 6*idx_particle+0 ] = pose_difference/self.delta_x

                # dy direction
                perturbed_pose.position = self.particle_positions_dy[particle][tip]
                perturbed_pose.orientation = self.particle_orientations_dy[particle][tip]
                pose_difference = self.calculate_pose_difference(current_pose,perturbed_pose)
                J[ (6*idx_tip) : (6*(idx_tip+1)) , 6*idx_particle+1 ] = pose_difference/self.delta_y

                # dz direction
                perturbed_pose.position = self.particle_positions_dz[particle][tip]
                perturbed_pose.orientation = self.particle_orientations_dz[particle][tip]
                pose_difference = self.calculate_pose_difference(current_pose,perturbed_pose)
                J[ (6*idx_tip) : (6*(idx_tip+1)) , 6*idx_particle+2 ] = pose_difference/self.delta_z

                # dth_x direction
                perturbed_pose.position = self.particle_positions_dth_x[particle][tip]
                perturbed_pose.orientation = self.particle_orientations_dth_x[particle][tip]
                pose_difference = self.calculate_pose_difference(current_pose,perturbed_pose)
                J[ (6*idx_tip) : (6*(idx_tip+1)) , 6*idx_particle+3 ] = pose_difference/self.delta_th_x

                # dy direction
                perturbed_pose.position = self.particle_positions_dth_y[particle][tip]
                perturbed_pose.orientation = self.particle_orientations_dth_y[particle][tip]
                pose_difference = self.calculate_pose_difference(current_pose,perturbed_pose)
                J[ (6*idx_tip) : (6*(idx_tip+1)) , 6*idx_particle+4 ] = pose_difference/self.delta_th_y

                # dz direction
                perturbed_pose.position = self.particle_positions_dth_z[particle][tip]
                perturbed_pose.orientation = self.particle_orientations_dth_z[particle][tip]
                pose_difference = self.calculate_pose_difference(current_pose,perturbed_pose)
                J[ (6*idx_tip) : (6*(idx_tip+1)) , 6*idx_particle+5 ] = pose_difference/self.delta_th_z                
        return J
    
    def calculate_error_tip(self):
        err = np.zeros(6*len(self.tip_particles))

        avr_norm_pos_err = 0.0
        avr_norm_ori_err = 0.0

        for idx_tip, tip in enumerate(self.tip_particles):
            # Do not proceed until the initial values have been set
            if not (tip in self.particle_positions):
                rospy.logwarn("Tip particle: " + str(tip) + " state is not obtained yet.")
                continue

            current_pose = Pose()
            current_pose.position = self.particle_positions[tip]
            current_pose.orientation = self.particle_orientations[tip]

            target_pose = self.target_poses[tip]

            err[(6*idx_tip) : (6*(idx_tip+1))] = self.calculate_pose_target_error(current_pose,target_pose)

            avr_norm_pos_err += np.linalg.norm(err[(6*idx_tip) : (6*(idx_tip+1))][0:3])/len(self.tip_particles)
            avr_norm_ori_err += np.linalg.norm(err[(6*idx_tip) : (6*(idx_tip+1))][3:6])/len(self.tip_particles)

        return err, avr_norm_pos_err, avr_norm_ori_err
    
    def calculate_path_tracking_error(self):
        err = np.zeros(6*len(self.custom_static_particles))

        avr_norm_pos_err = 0.0
        avr_norm_ori_err = 0.0

        for idx_particle, particle in enumerate(self.custom_static_particles):
            # Do not proceed until the initial values have been set
            if not self.is_perturbed_states_set_for_particle(particle):
                rospy.logwarn("Particle: " + str(particle) + " state is not obtained yet.")
                continue

            if particle not in self.planned_path_current_target_poses_of_particles:
                rospy.logwarn("Particle: " + str(particle) + " planned path is not obtained yet.")
                continue

            current_pose = Pose()
            current_pose.position = self.particle_positions[particle]
            current_pose.orientation = self.particle_orientations[particle]

            target_pose = self.planned_path_current_target_poses_of_particles[particle]

            err[(6*idx_particle) : (6*(idx_particle+1))] = self.calculate_pose_target_error(current_pose,target_pose)

            avr_norm_pos_err += np.linalg.norm(err[(6*idx_particle) : (6*(idx_particle+1))][0:3])/len(self.custom_static_particles)
            avr_norm_ori_err += np.linalg.norm(err[(6*idx_particle) : (6*(idx_particle+1))][3:6])/len(self.custom_static_particles)

        return err, avr_norm_pos_err, avr_norm_ori_err
    
    def calculate_path_tracking_velocity_error(self):
        err = np.zeros(6*len(self.custom_static_particles)) # Velocity error as 6n x 1 vector

        avr_norm_lin_vel_err = 0.0
        avr_norm_ang_vel_err = 0.0

        for idx_particle, particle in enumerate(self.custom_static_particles):
            # Do not proceed until the initial values have been set
            if not self.is_perturbed_states_set_for_particle(particle):
                rospy.logwarn("Particle: " + str(particle) + " twist state is not obtained yet.")
                continue

            if particle not in self.planned_path_velocity_profile_of_particles:
                rospy.logwarn("Particle: " + str(particle) + " planned path velocities are not obtained yet.")
                continue

            current_twist = self.twist_to_numpy(self.particle_twists[particle]) # (6,)

            target_twist = self.planned_path_current_target_velocities_of_particles[particle] # (6,)

            # calculate_twist_target_error
            err[(6*idx_particle) : (6*(idx_particle+1))] = target_twist - current_twist # (6,)

            avr_norm_lin_vel_err += np.linalg.norm(err[(6*idx_particle) : (6*(idx_particle+1))][0:3])/len(self.custom_static_particles)
            avr_norm_ang_vel_err += np.linalg.norm(err[(6*idx_particle) : (6*(idx_particle+1))][3:6])/len(self.custom_static_particles)

        return err, avr_norm_lin_vel_err, avr_norm_ang_vel_err

    def min_distances_array_callback(self, min_distances_msg):
        if self.initialized:
            # Create a set to track the IDs in the current message
            current_ids = set()

            # Iterate over the incoming message and update the dictionary
            for data in min_distances_msg.data:
                rigid_body_index = data.index2

                current_ids.add(rigid_body_index)
                if rigid_body_index not in self.min_distances or self.min_distances[rigid_body_index] == float('inf'):
                    # Initialize new ID or update 'inf' value with current minDistance
                    self.min_distances[rigid_body_index] = data.minDistance
                else:
                    # Apply low pass filter for existing ID
                    self.min_distances[rigid_body_index] = self.k_low_pass_min_d*self.min_distances[rigid_body_index] + (1-self.k_low_pass_min_d)*data.minDistance

            # Set values to float('inf') for IDs not in current message
            for key in list(self.min_distances.keys()):
                if key not in current_ids:
                    self.min_distances[key] = float('inf')

    def min_distance_array_dx_callback(self, min_distances_msg, perturbed_particle):
        if self.initialized:
            self.update_min_distances(self.min_distances_dx, min_distances_msg, perturbed_particle)

    def min_distance_array_dy_callback(self, min_distances_msg, perturbed_particle):
        if self.initialized:
            self.update_min_distances(self.min_distances_dy, min_distances_msg, perturbed_particle)

    def min_distance_array_dz_callback(self, min_distances_msg, perturbed_particle):
        if self.initialized:
            self.update_min_distances(self.min_distances_dz, min_distances_msg, perturbed_particle)

    def min_distance_array_dth_x_callback(self, min_distances_msg, perturbed_particle):
        if self.initialized:
            self.update_min_distances(self.min_distances_dth_x, min_distances_msg, perturbed_particle)

    def min_distance_array_dth_y_callback(self, min_distances_msg, perturbed_particle):
        if self.initialized:
            self.update_min_distances(self.min_distances_dth_y, min_distances_msg, perturbed_particle)

    def min_distance_array_dth_z_callback(self, min_distances_msg, perturbed_particle):
        if self.initialized:
            self.update_min_distances(self.min_distances_dth_z, min_distances_msg, perturbed_particle)

    def update_min_distances(self, min_distances, min_distances_msg, perturbed_particle):
        if perturbed_particle not in min_distances:
            min_distances[perturbed_particle] = {}

        # Create a set to track the IDs in the current message
        current_ids = set()

        # Iterate over the incoming message and update the dictionary
        for data in min_distances_msg.data:
            rigid_body_index = data.index2

            current_ids.add(rigid_body_index)
            if rigid_body_index not in min_distances[perturbed_particle] or min_distances[perturbed_particle][rigid_body_index] == float('inf'):
                # Initialize new ID or update 'inf' value with current minDistance
                min_distances[perturbed_particle][rigid_body_index] = data.minDistance
            else:
                # Apply low pass filter for existing ID
                min_distances[perturbed_particle][rigid_body_index] = self.k_low_pass_min_d*min_distances[perturbed_particle][rigid_body_index] + (1 - self.k_low_pass_min_d)*data.minDistance

        # Set values to float('inf') for IDs not in current message
        for key in list(min_distances[perturbed_particle].keys()):
            if key not in current_ids:
                min_distances[perturbed_particle][key] = float('inf')

    def is_perturbed_min_distances_set(self, particle):
        if particle in self.min_distances_set_particles:
            return True
        else:
            check = ((particle in self.min_distances_dx) and  
                     (particle in self.min_distances_dy) and 
                     (particle in self.min_distances_dz) and
                     (particle in self.min_distances_dth_x) and  
                     (particle in self.min_distances_dth_y) and 
                     (particle in self.min_distances_dth_z))
            if check:
                self.min_distances_set_particles.append(particle)
                return True
            else:
                return False
            
    def is_perturbed_min_distances_set_for_obstacle_id(self, particle, key):
        if not self.is_perturbed_min_distances_set(particle):
            return False

        # key here is the rigid body index of the obstacle
        if (particle, key) in self.min_distances_set_particles_obstacles:
            return True
        else:
            check = ((key in self.min_distances_dx[particle]) and  
                     (key in self.min_distances_dy[particle]) and 
                     (key in self.min_distances_dz[particle]) and
                     (key in self.min_distances_dth_x[particle]) and  
                     (key in self.min_distances_dth_y[particle]) and 
                     (key in self.min_distances_dth_z[particle]))
            if check:
                self.min_distances_set_particles_obstacles.append((particle, key))
                return True
            else:
                return False

    def calculate_jacobian_obstacle_min_distance(self, key):
        """
        Calculates the Jacobian matrix that defines the relation btw. 
        the robot hold points (custom_static_particles) 6DoF poses and
        the minimum distance to the obstacles. 
        The result is a row matrix(vector) (e.g. dimension 1x12 for two holding point poses)
        """
        J = np.zeros((1,6*len(self.custom_static_particles)))

        for idx_particle, particle in enumerate(self.custom_static_particles):
            # Do not proceed if the minimum distance is not set for the obstacle
            if not self.is_perturbed_min_distances_set_for_obstacle_id(particle, key):
                rospy.logwarn_throttle(1,"[calculate_jacobian_obstacle_min_distance func.] particle: "+str(particle)+", obstacle index: "+str(key)\
                              +", min distances are not published yet or it does not have for perturbed states")
                continue

            # dx direction
            J[0, 6*idx_particle+0] = (self.min_distances_dx[particle][key] - self.min_distances[key])/self.delta_x if self.delta_x != 0.0 else 0.0
            # dy direction
            J[0, 6*idx_particle+1] = (self.min_distances_dy[particle][key] - self.min_distances[key])/self.delta_y if self.delta_y != 0.0 else 0.0
            # dz direction
            J[0, 6*idx_particle+2] = (self.min_distances_dz[particle][key] - self.min_distances[key])/self.delta_z if self.delta_z != 0.0 else 0.0
            # dth_x direction
            J[0, 6*idx_particle+3] = (self.min_distances_dth_x[particle][key] - self.min_distances[key])/self.delta_th_x if self.delta_th_x != 0.0 else 0.0
            # dy direction
            J[0, 6*idx_particle+4] = (self.min_distances_dth_y[particle][key] - self.min_distances[key])/self.delta_th_y if self.delta_th_y != 0.0 else 0.0
            # dz direction
            J[0, 6*idx_particle+5] = (self.min_distances_dth_z[particle][key] - self.min_distances[key])/self.delta_th_z if self.delta_th_z != 0.0 else 0.0

        # Replace non-finite elements with 0
        J[~np.isfinite(J)] = 0

        return J

    def calculate_jacobian_ft(self):
        """
        Calculates the Jacobian matrix that defines the relation btw.
        the robot hold points (custom_static_particles) 6DoF poses and
        the forces and torques applied by the robots.
        The result is a 12x12 matrix for 2 holding point poses.
        """
        J = np.zeros((6*len(self.custom_static_particles),6*len(self.custom_static_particles)))

        for idx_particle1, particle1 in enumerate(self.custom_static_particles):
            for idx_particle, particle in enumerate(self.custom_static_particles):
                # Do not proceed until the initial values have been set
                if ((not self.is_perturbed_states_set_for_particle(particle))):
                    rospy.logwarn_throttle(1,"[calculate_jacobian_ft func.] particle: " + str(particle) + " state is not published yet or it does not have for perturbed states.")
                    continue

                # calculate the wrench differences:
                # btw. the current holding point wrench and the perturbated holding point wrench caused by the perturbation of custom_static_particle pose in each direction

                # Note that the forces and torques are negated to calculate the wrenches needed to be applied by the robots.
                current_wrench = self.particle_wrenches[particle1]
                
                # dx direction
                perturbed_wrench = self.particle_wrenches_dx[particle][particle1]
                J[ (6*idx_particle1) : (6*(idx_particle1+1)) , 6*idx_particle+0 ] = (perturbed_wrench-current_wrench)/self.delta_x
                # dy direction
                perturbed_wrench = self.particle_wrenches_dy[particle][particle1]
                J[ (6*idx_particle1) : (6*(idx_particle1+1)) , 6*idx_particle+1 ] = (perturbed_wrench-current_wrench)/self.delta_y
                # dz direction
                perturbed_wrench = self.particle_wrenches_dz[particle][particle1]
                J[ (6*idx_particle1) : (6*(idx_particle1+1)) , 6*idx_particle+2 ] = (perturbed_wrench-current_wrench)/self.delta_z
                # dth_x direction
                perturbed_wrench = self.particle_wrenches_dth_x[particle][particle1]
                J[ (6*idx_particle1) : (6*(idx_particle1+1)) , 6*idx_particle+3 ] = (perturbed_wrench-current_wrench)/self.delta_th_x
                # dy direction
                perturbed_wrench = self.particle_wrenches_dth_y[particle][particle1]
                J[ (6*idx_particle1) : (6*(idx_particle1+1)) , 6*idx_particle+4 ] = (perturbed_wrench-current_wrench)/self.delta_th_y
                # dz direction
                perturbed_wrench = self.particle_wrenches_dth_z[particle][particle1]
                J[ (6*idx_particle1) : (6*(idx_particle1+1)) , 6*idx_particle+5 ] = (perturbed_wrench-current_wrench)/self.delta_th_z            

        return J

    def calculate_safe_control_output(self, nominal_u):
        ## ---------------------------------------------------
        ## Define optimization variables
        u = cp.Variable(6*len(self.custom_static_particles))

        # Initialize the constraints
        constraints = []
        ## ---------------------------------------------------

        ## ---------------------------------------------------
        # DEFINE COLLISION AVOIDANCE CONTROL BARRIER CONSTRAINTS FOR EACH SCENE MINIMUM DISTANCE READINGS
        
        overall_min_distance = float('inf')
        for key in list(self.min_distances.keys()):
            # key here is the rigid body index of the obstacle

            # update the overall minimum distance
            if self.min_distances[key] < overall_min_distance:
                overall_min_distance = self.min_distances[key]

            if self.obstacle_avoidance_enabled:
                h = self.min_distances[key] - self.d_obstacle_offset # Control Barrier Function (CBF)
                alpha_h = self.alpha_collision_avoidance(h)
                
                # Calculate the obstacle minimum distance Jacobian 
                J_obs_min_dist = self.calculate_jacobian_obstacle_min_distance(key) # 1x12

                # pretty_print_array(J_obs_min_dist, precision=4)
                # print("---------------------------")
                
                # # publish J for information
                # J_msg = Float32MultiArray(data=np.ravel(J_obs_min_dist))
                # self.info_J_publisher.publish(J_msg)

                J_tolerance = 0.001
                if np.any(np.abs(J_obs_min_dist) >= J_tolerance):
                        # Add collision avoidance to the constraints
                        constraints += [J_obs_min_dist @ u >= -alpha_h]
                else:
                    pass
                    # pretty_print_array(J_obs_min_dist, precision=4)
                    # rospy.logwarn_throttle(1,"For obstacle index: " + str(key) + ", ignored J_obs_min_dist and obstacle constraint is not added")
        
        # publish the current overall minimum distance to collision for information
        self.info_pub_overall_min_distance_collision.publish(Float32(data=overall_min_distance))
        
        # Update the controller's last enabled period overall minimum distance for the performance monitoring
        if overall_min_distance < self.overall_min_distance_collision:
            self.overall_min_distance_collision = overall_min_distance

        ## ---------------------------------------------------
                
        ## ---------------------------------------------------
        # DEFINE STRESS CONTROL BARRIER CONSTRAINTS 
        
        # Initialize the variables for the stress avoidance
        h_ft = np.zeros(6*len(self.custom_static_particles)) # Control Barrier Function (CBF) for the forces and torques
        h_ft_normalized = np.zeros(6*len(self.custom_static_particles)) # CBF normalized values for performance monitoring
        alpha_h_ft = np.zeros(6*len(self.custom_static_particles)) # alpha for the forces and torques
        sign_ft = np.zeros(6*len(self.custom_static_particles)) # sign for the forces and torques
        ft = np.zeros(6*len(self.custom_static_particles)) # forces and torques

        # Calculate the stress avoidance constraints for each custom static particle
        for idx_particle, particle in enumerate(self.custom_static_particles):
            # h_ft[6*idx_particle:6*(idx_particle+1)] = self.wrench_max - self.w_stress_offset - np.abs(self.particle_wrenches[particle]) # h_ft = wrench_max - wrench_offset - |wrench| (OLD)
            h_ft[6*idx_particle:6*(idx_particle+1)] = (self.wrench_max - self.w_stress_offset)**2 - (self.particle_wrenches[particle])**2 # h_ft = (wrench_max - wrench_offset)^2 - (wrench)^2 (NEW)
            h_ft_normalized[6*idx_particle:6*(idx_particle+1)] = (self.wrench_max - np.abs(self.particle_wrenches[particle]))/self.wrench_max
            alpha_h_ft[6*idx_particle:6*(idx_particle+1)] = self.alpha_robot_stress(h_ft[6*idx_particle:6*(idx_particle+1)])
            sign_ft[6*idx_particle:6*(idx_particle+1)] = np.sign(self.particle_wrenches[particle])
            ft[6*idx_particle:6*(idx_particle+1)] = self.particle_wrenches[particle]

        # Calculate stress avoidance performance monitoring values
        stress_avoidance_performance = self.calculate_and_publish_stress_avoidance_performance(h_ft_normalized)

        if stress_avoidance_performance <= 0.0:
                # Stop the controller if the stress avoidance performance is zero
                # self.controller_enabler(enable=False, cause="Stress avoidance hit zero.")
                rospy.logwarn("Stress avoidance hit zero.")
                # return None

        if self.stress_avoidance_enabled:
            # Calculate the forces and torques Jacobian
            J_ft = self.calculate_jacobian_ft() # 12x12
            
            # print("J_ft:")
            # print(J_ft)
            # print("---------------------------")
            # print("alpha_h_ft:")
            # print(alpha_h_ft)
            # print("---------------------------")
            # print("sign_ft:")
            # print(sign_ft)
            # print("-------------------------------------------------------")

            # Mutiply the sign of the wrenches elementwise 
            # with the matrix multiplication of the Jacobian with the control input
            # to obtain the forces and torques.
            # Add stress avoidance to the constraints
            # constraints += [cp.multiply(-sign_ft, (J_ft @ u)) >= -alpha_h_ft] # (OLD)
            constraints += [cp.multiply(-2*ft, (J_ft @ u)) >= -alpha_h_ft] # (NEW)
        ## ---------------------------------------------------
            
        # ## ---------------------------------------------------
        ## Add also limit to the feasible u
        
        # With using the same limits to both linear and angular velocities
        # u_max = 0.1
        # constraints += [cp.norm(u,'inf') <= u_max] # If inf-norm used, the constraint is linear, use OSQP solver (still QP)
        # # constraints += [cp.norm(u,2)     <= u_max] # If 2-norm is used, the constraint is Quadratic, use CLARABEL or ECOS solver (Not QP anymore, a conic solver is needed)

        # With using the different limits to linear and angular velocities
        u_linear_max = self.max_linear_velocity*6.0 # 0.3 # 0.1
        u_angular_max = self.max_angular_velocity*6.0 # 0.5 # 0.15

        # The following slices select every first 3 elements of each group of 6 in the array u.
        linear_indices = np.concatenate([np.arange(i, i+3) for i in range(0, 6*len(self.custom_static_particles), 6)])
        # Apply constraint to linear components
        constraints += [cp.norm(u[linear_indices],'inf') <= u_linear_max]

        # The following slices select every second 3 elements of each group of 6 in the array u.
        angular_indices = np.concatenate([np.arange(i+3, i+6) for i in range(0, 6*len(self.custom_static_particles), 6)])
        # Apply constraint to angular components
        constraints += [cp.norm(u[angular_indices],'inf') <= u_angular_max]
        # ## ---------------------------------------------------
        
        
        # ## ---------------------------------------------------
        # ## Limit velocity and accelerations at once
        
        # # Using the different limits to linear and angular velocities
        # u_lin_max = self.max_linear_velocity*1.0 # 0.3 # 0.1
        # u_ang_max = self.max_angular_velocity*1.0 # 0.5 # 0.15
        
        # a_lin_max = self.max_linear_acceleration * 1.0
        # a_ang_max = self.max_angular_acceleration * 1.0
        
        # ## Calculate the time step based on the average control calculation rate
        # # Calculate the duration since the controller is enabled
        # dur = (rospy.Time.now() - self.controller_enabled_time).to_sec() # seconds
        # # Calculate the average control calculation rate        
        # dt = dur/self.controller_itr  # Time step size
        # # # Or use hard coded value
        # # dt = 1.0/self.pub_rate_odom
        # # print("dt: ", dt)
        
        # # Precompute useful quantities
        # a_lin_max_dt = a_lin_max * dt
        # a_ang_max_dt = a_ang_max * dt
        
        # # Assuming control_outputs_last is a dictionary or structure where velocities are stored
        # # Example structure for control_outputs_last:
        # # self.control_outputs_last[particle] = np.array([v_x, v_y, v_z, w_x, w_y, w_z])

        # for i, particle in enumerate(self.custom_static_particles):            
        #     lin_indices = np.arange(6*i, 6*i+3)  # Linear velocity indices
        #     ang_indices = np.arange(6*i+3, 6*i+6)  # Angular velocity indices
            
        #     v_prev = self.control_outputs_last[particle][0:3]  # Previous linear velocity
        #     w_prev = self.control_outputs_last[particle][3:6]  # Previous angular velocity
            
        #     # Optimization variables
        #     u_lin = u[lin_indices]
        #     u_ang = u[ang_indices]
            
        #     # Precompute useful quantities for the current particle
        #     v_min = np.minimum(u_lin_max, a_lin_max_dt + v_prev)
        #     w_min = np.minimum(u_ang_max, a_ang_max_dt + w_prev)
            
        #     v_neg_min = np.minimum(u_lin_max, a_lin_max_dt - v_prev)
        #     w_neg_min = np.minimum(u_ang_max, a_ang_max_dt - w_prev)
            
        #     # Create masks for each case
        #     mask_lin_case1 = np.abs(v_prev) <= a_lin_max_dt  # |v_prev| <= a_max * dt
        #     mask_lin_case2 = v_prev > a_lin_max_dt          # v_prev >= a_max * dt
        #     mask_lin_case3 = -v_prev >= a_lin_max_dt         # -v_prev >= a_max * dt
            
        #     mask_ang_case1 = np.abs(w_prev) <= a_ang_max_dt  # |w_prev| <= a_max * dt
        #     mask_ang_case2 = w_prev > a_ang_max_dt           # w_prev >= a_max * dt
        #     mask_ang_case3 = -w_prev >= a_ang_max_dt         # -w_prev >= a_max * dt
            
        #     # Apply constraints only if the mask selects non-empty elements
            
        #     ## Linear velocity constraints
        #     if np.any(mask_lin_case1):  # Check if mask selects any element
        #         constraints += [
        #             u_lin[mask_lin_case1] <= v_min[mask_lin_case1],
        #             -u_lin[mask_lin_case1] <= v_neg_min[mask_lin_case1]
        #         ]
        #     if np.any(mask_lin_case2):
        #         constraints += [
        #             u_lin[mask_lin_case2] <= v_min[mask_lin_case2],
        #             -u_lin[mask_lin_case2] <= 0
        #         ]
        #     if np.any(mask_lin_case3):
        #         constraints += [
        #             u_lin[mask_lin_case3] <= 0,
        #             -u_lin[mask_lin_case3] <= v_neg_min[mask_lin_case3]
        #         ]
            
        #     ## Angular velocity constraints
        #     if np.any(mask_ang_case1):
        #         constraints += [
        #             u_ang[mask_ang_case1] <= w_min[mask_ang_case1],
        #             -u_ang[mask_ang_case1] <= w_neg_min[mask_ang_case1]
        #         ]
        #     if np.any(mask_ang_case2):
        #         constraints += [
        #             u_ang[mask_ang_case2] <= w_min[mask_ang_case2],
        #             -u_ang[mask_ang_case2] <= 0
        #         ]
        #     if np.any(mask_ang_case3):
        #         constraints += [
        #             u_ang[mask_ang_case3] <= 0,
        #             -u_ang[mask_ang_case3] <= w_neg_min[mask_ang_case3]
        #         ]
                        
        # ## ---------------------------------------------------
        
        ## ---------------------------------------------------
        # Define the problem
        
        ## Define weights for control inputs
        weights = np.ones(6*len(self.custom_static_particles))
        # Assign less weight on z axis positional motions (i.e make them more dynamic)
        
        ## Note that x axis position is every 1st element of each 6 element sets in the weight vector
        # weights[0::6] = 0.5 
        ## Note that y axis position is every 2nd element of each 6 element sets in the weight vector
        # weights[1::6] = 0.5 
        ## Note that z axis position is every 3rd element of each 6 element sets in the weight vector
        # weights[2::6] = 0.1 
        # weights[2::6] = self.calculate_cost_weight_z_pos(stress_avoidance_performance, overall_min_distance-self.d_obstacle_offset)
        
        ## Note that x axis rotation is every 4th element of each 6 element sets in the weight vector
        # weights[3::6] = 0.8
        ## Note that y axis rotation is every 5th element of each 6 element sets in the weight vector
        # weights[4::6] = 0.8
        ## Note that z axis rotation is every 6th element of each 6 element sets in the weight vector
        # weights[5::6] = 0.8
        
        # Slow down the nominal control output when close to the obstacles and stress limits
        # nominal_u = self.calculate_weight_nominal_input(stress_avoidance_performance, overall_min_distance-self.d_obstacle_offset) * nominal_u

        # Define cost function with weights
        cost = cp.sum_squares(cp.multiply(weights, u - nominal_u)) / 2.0
        
        # ## ---------------------------------------------------
        # # Add cost of change in control input (acceleration) as a COST FUNCTION (not a constraint)
        
        # # Define the maximum allowable accelerations for linear and angular components
        # a_linear_max = self.max_linear_acceleration * 1.0
        # a_angular_max = self.max_angular_acceleration * 1.0
        
        # # Define the weights for the acceleration cost
        # w_lin_acc = 0.0001 # 1.0 # 0.01
        # w_ang_acc = 0.0001 # 1.0 # 0.01

        # # Calculate the time step based on the average control calculation rate
        # # Calculate the duration since the controller is enabled
        # dur = (rospy.Time.now() - self.controller_enabled_time).to_sec() # seconds
        # # Calculate the average control calculation rate        
        # dt = dur/self.controller_itr  # Time step size
        # # print("dt: ", dt)
        
        # for i, particle in enumerate(self.custom_static_particles):            
        #     linear_indices = np.arange(6*i, 6*i+3)
        #     angular_indices = np.arange(6*i+3, 6*i+6)
            
        #     delta_u_linear = (u[linear_indices] - self.control_outputs_last[particle][:3])/dt
        #     delta_u_angular = (u[angular_indices] - self.control_outputs_last[particle][3:6])/dt
            
        #     cost += (w_lin_acc/a_linear_max**2) *cp.sum_squares(delta_u_linear)/2.0
        #     cost += (w_ang_acc/a_angular_max**2) *cp.sum_squares(delta_u_angular)/2.0
        
        # ## ---------------------------------------------------
        
        problem = cp.Problem(cp.Minimize(cost), constraints)
        ## ---------------------------------------------------

        ## ---------------------------------------------------
        # Solve the problem

        # # # For warm-start
        # if hasattr(self, 'prev_optimal_u'):
        #     u.value = self.prev_optimal_u

        # init_t = time.time() # For timing
        try:
            # problem.solve() # Selects automatically
            # problem.solve(solver=cp.CLARABEL) #  
            # problem.solve(solver=cp.CLARABEL, tol_gap_abs=1e-4, tol_gap_rel=1e-4, tol_feas=1e-4) #  
            # problem.solve(solver=cp.CVXOPT) # (warm start capable)
            # problem.solve(solver=cp.ECOS) # 
            # problem.solve(solver=cp.ECOS_BB) # 
            # problem.solve(solver=cp.GLOP) # NOT SUITABLE
            # problem.solve(solver=cp.GLPK) # NOT SUITABLE
            # problem.solve(solver=cp.GUROBI) # 
            # problem.solve(solver=cp.MOSEK) # Encountered unexpected exception importing solver CBC
            # problem.solve(solver=cp.OSQP, eps_abs=1e-4, eps_rel=1e-4, time_limit=(3./ self.pub_rate_odom), warm_starting=True) #  (default) (warm start capable)
            # problem.solve(solver=cp.OSQP, eps_abs=1e-5, eps_rel=1e-3) #  (default) (warm start capable)
            problem.solve(solver=cp.OSQP) #  (default) (warm start capable)
            # problem.solve(solver=cp.PDLP) # NOT SUITABLE
            # problem.solve(solver=cp.SCIPY) # NOT SUITABLE 
            # problem.solve(solver=cp.SCS) # (warm start capable)

        except cp.error.SolverError as e:
            rospy.logwarn("QP solver Could not solve the problem: {}".format(e))
            # self.prev_optimal_u = None # to reset warm-start
            return None

        # rospy.logwarn("QP solver calculation time: " + str(1000*(time.time() - init_t)) + " ms.") # For timing

        # Print the available qp solvers
        # e.g. ['CLARABEL', 'CVXOPT', 'ECOS', 'ECOS_BB', 'GLOP', 'GLPK', 'GLPK_MI', 'GUROBI', 'MOSEK', 'OSQP', 'PDLP', 'SCIPY', 'SCS']
        rospy.loginfo_once("Installed solvers:" + str(cp.installed_solvers()))
        # Print the solver used to solve the problem (once)
        rospy.loginfo_once("Solver used: " + str(problem.solver_stats.solver_name))

        # check if problem is infeasible or unbounded
        if problem.status in ["infeasible", "unbounded"]:
            rospy.logwarn("QP solver, The problem is {}.".format(problem.status))
            # self.prev_optimal_u = None # to reset warm-start
            return None
        
        # # For warm-start in the next iteration
        # self.prev_optimal_u = u.value
        ## ---------------------------------------------------
        
        # # Find the acceleration for each particle
        # for idx_particle, particle in enumerate(self.custom_static_particles):
        #     if idx_particle == 0:
        #         a_lin = (u.value[6*idx_particle:6*idx_particle+3] - self.control_outputs_last[particle][:3] )/dt
                
        #         # If any of the linear acceleration components is above the maximum allowable value
        #         if np.any(np.abs(a_lin) > a_linear_max):
        #             rospy.logerr("a_lin is above thresholds: " +  str(a_lin))
            
        
        # Return optimal u
        return u.value

    def update_last_control_output_is_valid_time(self):
        # Take a note of the time when the control output is calculated
        self.time_last_control_output_is_valid = rospy.Time.now()
        
    def update_last_error_change_is_valid_time(self):
        # Take a note of the time when the error changes are above the threshold (valid) 
        self.time_last_error_change_is_valid = rospy.Time.now()
                        
    def assign_control_outputs(self, control_output):
        for idx_particle, particle in enumerate(self.custom_static_particles):    
            self.control_outputs[particle] = control_output[6*idx_particle:6*(idx_particle+1)]
            # print("Particle " + str(particle) + " u: " + str(self.control_outputs[particle]))
    
    def calculate_nominal_control_output(self, err_tip):
        # Initiate the nominal control output to zero
        control_output = np.zeros(6*len(self.custom_static_particles))

        # Calculate the nominal control outputs
        if self.nominal_control_enabled:
            J_tip = self.calculate_jacobian_tip() # 12x12
            # pretty_print_array(J_tip)
            # print("---------------------------")

            # Calculate the nominal control output
            control_output = np.dot(np.linalg.pinv(J_tip), err_tip) # (12,)
            # Apply the proportinal gains
            for idx_particle, particle in enumerate(self.custom_static_particles):
                # Get nominal control output of that particle
                control_output[6*idx_particle:6*(idx_particle+1)] = self.kp * control_output[6*idx_particle:6*(idx_particle+1)] # nominal

        return control_output

    def calculate_path_tracking_control_output(self):
        # Initiate the path tracking control output to zero
        control_output = np.zeros(6*len(self.custom_static_particles))

        # Calculate the path tracking control outputs
        if self.path_tracking_control_enabled and self.planned_path_current_target_velocities_of_particles:
            # ----------------------
            # Calculate the error between the current pose and the target pose of the particles
            (err_path_tracking, 
            pos_err_avr_norm_path_tracking, 
            ori_err_avr_norm_path_tracking) = self.calculate_path_tracking_error() # (12,), scalar, scalar
            
            # pretty_print_array(err_path_tracking)
            # print("---------------------------")

            # # publish error norms for information TODO
            # self.info_pub_path_tracking_pos_error_avr_norm.publish(Float32(data=pos_err_avr_norm_path_tracking))
            # self.info_pub_path_tracking_ori_error_avr_norm.publish(Float32(data=ori_err_avr_norm_path_tracking))

            # Print orientation error tracking only elements 4th 5th 6th every 6 elements
            # print(np.rad2deg(err_path_tracking[np.r_[3:6, 9:12]]))
            # ----------------------
            
            # ----------------------
            # # Calculate twist error tracking between the current twist and the target twist of the particles
            # (err_path_tracking_twist,
            # lin_vel_err_avr_norm_path_tracking,
            # ang_vel_err_avr_norm_path_tracking) = self.calculate_path_tracking_velocity_error() # 12x1, scalar, scalar
            
            # # pretty_print_array(err_path_tracking_twist)
            # # print("---------------------------")
            
            # # # publish error norms for information TODO
            # # self.info_pub_path_tracking_lin_vel_error_avr_norm.publish(Float32(data=lin_vel_err_avr_norm_path_tracking))
            # # self.info_pub_path_tracking_ang_vel_error_avr_norm.publish(Float32(data=ang_vel_err_avr_norm_path_tracking))
            # # self.info_pub_path_tracking_ang_vel_error_avr_norm.publish(Float32(data=ang_vel_err_avr_norm_path_tracking))
            
            # err_path_tracking_twist = np.squeeze(err_path_tracking_twist) # (12,)
            # # self.info_pub_path_tracking_ang_vel_error_avr_norm.publish(Float32(data=ang_vel_err_avr_norm_path_tracking))            
            
            # err_path_tracking_twist = np.squeeze(err_path_tracking_twist) # (12,)
            # ----------------------
            
            for idx_particle, particle in enumerate(self.custom_static_particles):
                # Get nominal control output of that particle and Apply the proportinal gains

                p_term = self.kp_path_tracking * err_path_tracking[6*idx_particle:6*(idx_particle+1)]
                
                # d_term = self.kd_path_tracking * err_path_tracking_twist[6*idx_particle:6*(idx_particle+1)]            
                # control_output[6*idx_particle:6*(idx_particle+1)] = p_term + d_term # path tracking

                # Feed forward control with the velocity profile of the path
                ffwd_term = self.planned_path_current_target_velocities_of_particles[particle] # (6,)
                
                # If p_term in the same direction with the ffwd_term, 
                # then use the sum of ffwd term and only the perpendicular part of the p_term to remove the lateral errors
                # otherwise, use only the p_term (i.e. the ffwd term is not used) ) because it means the ffwd_term is conflicting with the error direction
                
                # Separate linear (first 3) and angular (last 3) terms
                p_term_lin = p_term[:3]
                p_term_ang = p_term[3:]

                ffwd_term_lin = ffwd_term[:3]
                ffwd_term_ang = ffwd_term[3:]
                
                # Linear part
                if np.dot(p_term_lin, ffwd_term_lin) > 0:
                    # Calculate the perpendicular part of the p_term_lin
                    p_term_lin_perp = p_term_lin - np.dot(p_term_lin, ffwd_term_lin)/np.sqrt(np.dot(ffwd_term_lin, ffwd_term_lin)) * ffwd_term_lin
                    control_output[6*idx_particle:6*idx_particle+3] = p_term_lin_perp + ffwd_term_lin
                else:
                    control_output[6*idx_particle:6*idx_particle+3] = p_term_lin

                # Angular part
                if np.dot(p_term_ang, ffwd_term_ang) > 0:
                    # Calculate the perpendicular part of the p_term_ang
                    p_term_ang_perp = p_term_ang - np.dot(p_term_ang, ffwd_term_ang)/np.sqrt(np.dot(ffwd_term_ang, ffwd_term_ang)) * ffwd_term_ang
                    control_output[6*idx_particle+3:6*(idx_particle+1)] = p_term_ang_perp + ffwd_term_ang                    
                else:
                    control_output[6*idx_particle+3:6*(idx_particle+1)] = p_term_ang
                
                # if np.dot(p_term, ffwd_term) > 0:
                #     # Calculate the perpendicular part of the p_term
                #     p_term_perpendicular = p_term - np.dot(p_term, ffwd_term)/np.sqrt(np.dot(ffwd_term, ffwd_term)) * ffwd_term
                    
                #     control_output[6*idx_particle:6*(idx_particle+1)] = p_term_perpendicular + ffwd_term
                # else:
                #     control_output[6*idx_particle:6*(idx_particle+1)] = p_term
                
        return control_output
    
    def transition_function(self, c, c_l, c_u):
        """Piecewise smooth transition function
        
        Uses a cosine function to smoothly transition from 0 to 1 over a given range of distances.
        The transition function is defined as:
        weight = 0.5 * (1 - np.cos(np.pi * (c-c_l)/(c_u-c_l)))
        where c is the distance to the path completion and c_l and c_u are the lower and upper bounds of the range of distances.
        At c <= c_l, the weight is 0. 
        At c >= c_u, the weight is 1.
        
        See: https://www.desmos.com/calculator/lw1rirjtab
        """
        if c <= c_l:
            return 0.0
        elif c >= c_u:
            return 1.0
        else:
            return 0.5 * (1 - np.cos(np.pi * (c-c_l)/(c_u-c_l)))
    
    def blend_control_outputs(self, nominal_u, path_tracking_u):
        """
        Blends the nominal and path tracking control outputs using a transition function.
        The blending weight is calculated based on the distance to the path completion.
        The blending weight is in the range [0,1] and it is calculated using a transition function.
        The blending is performed as:
        control_output = (1.0 - weight)*nominal_u + weight*path_tracking_u
        """        
        ## Calculate a weight for blending the control outputs in the range [0,1]

        ## Centroid distance to the path completion
        # l_path = self.planned_path_cumulative_lengths[-1] # path length
        # d_to_complete = (1.0 - self.planned_path_completion_rate) * l_path # distance to path completion
        # print("d_to_complete (centroid): " + str(d_to_complete))

        ## Calculate average distance to complete from particle paths:
        d_to_complete = 0.0
        n = 0
        for particle, path_lengths_particle in self.planned_path_cumulative_lengths_of_particles.items():
            l_path_particle = path_lengths_particle[-1]
            d_to_complete += (1.0 - self.planned_path_completion_rate) * l_path_particle
            # Print below the particle, d_to_complete
            # print("Particle: " + str(particle) + ", d_to_complete: " + str(d_to_complete))
            n += 1
        
        if n == 0:
            rospy.logwarn("There is no particle path to calculate the distance to the path completion.")
            d_to_complete = float('inf')
        else:
            d_to_complete /= n
        # print("d_to_complete (average particles): " + str(d_to_complete))

        # Use the transition function to calculate the weight
        weight = self.transition_function(d_to_complete, 
                                          self.d_path_tracking_complete_switch_off_distance, 
                                          self.d_path_tracking_start_switch_off_distance)
        # print("weight: " + str(weight))
        
        return (1.0 - weight)*nominal_u + weight*path_tracking_u

    def calculate_control_outputs_timer_callback(self,event):
        """
        Calculates the control outputs (self.control_outputs) for the custom static particles.
        """
        if self.initialized:
            # if self.proceed_event.is_set():
            #     # print("Executing control calculations...")
            #     self.enabled = True
                
            # Only calculate if enabled
            if self.enabled:
                
                # Increment the controller iteration
                self.controller_itr += 1
                
                # ----------------------------------------------------------------------------------------------------
                ## Calculate the tip errors and update the error norms
                
                # Update the previous error norms
                self.pos_err_avr_norm_prev = self.pos_err_avr_norm
                self.ori_err_avr_norm_prev = self.ori_err_avr_norm
                
                # Calculate the current tip errors
                (err_tip, 
                pos_err_avr_norm, 
                ori_err_avr_norm) = self.calculate_error_tip() # (12,), scalar, scalar
                
                # pretty_print_array(err_tip)
                # print("---------------------------")
                
                # Apply low-pass filter to the error norms
                self.pos_err_avr_norm = self.k_low_pass_convergence*self.pos_err_avr_norm_prev + (1-self.k_low_pass_convergence)*pos_err_avr_norm
                self.ori_err_avr_norm = self.k_low_pass_convergence*self.ori_err_avr_norm_prev + (1-self.k_low_pass_convergence)*ori_err_avr_norm
                
                _, is_pos_err_converged = self.pos_err_convergence_checker.update(self.pos_err_avr_norm)
                _, is_ori_err_converged = self.ori_err_convergence_checker.update(self.ori_err_avr_norm)
                
                # Update the last error change is valid time if the change in the error norms is above the thresholds
                if ((np.abs(self.pos_err_avr_norm - self.pos_err_avr_norm_prev) >= self.convergence_threshold_pos) or
                    (np.abs(self.ori_err_avr_norm - self.ori_err_avr_norm_prev) >= self.convergence_threshold_ori)):
                    self.update_last_error_change_is_valid_time()
                # else:
                #     rospy.logwarn("Error norms are not changing. Current changes in error norms:\npos_err_avr_norm: " + str(np.abs(self.pos_err_avr_norm - self.pos_err_avr_norm_prev)) + ", ori_err_avr_norm: " + str(np.abs(self.ori_err_avr_norm - self.ori_err_avr_norm_prev)))
                
                # publish error norms for information
                self.info_pub_target_pos_error_avr_norm.publish(Float32(data=pos_err_avr_norm))
                self.info_pub_target_ori_error_avr_norm.publish(Float32(data=ori_err_avr_norm))
                # ----------------------------------------------------------------------------------------------------
                
                # ----------------------------------------------------------------------------------------------------
                if self.path_tracking_control_enabled:
                    if self.planned_path and self.planned_path_current_target_velocities_of_particles:
                        # Calculate path tracking control output
                        path_tracking_control_output = self.calculate_path_tracking_control_output() # (12,)
                        # print("Path tracking control_output")
                        # print(path_tracking_control_output)
                        # print("---------------------------")

                        if self.nominal_control_enabled:
                            # Calculate nominal control output
                            nominal_control_output = self.calculate_nominal_control_output(err_tip) # (12,)
                            # Blend the nominal and path tracking control outputs
                            control_output = self.blend_control_outputs(nominal_control_output, path_tracking_control_output) # (12,)
                        else:
                            control_output = path_tracking_control_output
                    else:
                        rospy.logwarn_throttle(5, "Waiting for the planned path to be available. Setting control output to zero. (throttled to 5s)")
                    
                        control_output = np.zeros(6*len(self.custom_static_particles))    
                        self.assign_control_outputs(control_output)
                        self.update_last_control_output_is_valid_time()
                        self.update_last_error_change_is_valid_time()
                        return
                    
                else: # if path tracking control is not enabled
                    # Calculate nominal control output
                    control_output = self.calculate_nominal_control_output(err_tip) # (12,)
                # ----------------------------------------------------------------------------------------------------
                        
                # ----------------------------------------------------------------------------------------------------  
                ## Check for the convergence of the controller

                # Check if the change in error norms are below the thresholds for a long time
                if (((rospy.Time.now() - self.time_last_error_change_is_valid).to_sec() > self.convergence_wait_timeout) and
                    is_pos_err_converged and is_ori_err_converged and
                    (self.nominal_control_enabled or self.path_tracking_control_enabled)): 
                    # We will either Disable the controller or replan the path depending on some conditions:
                    
                    ## 1. Check for the arrival to the target pose of the particles 
                    # i.e. Check for the error norms (position and orientation) of the particles, and
                    # disable the controller with success if the errors are below the threshold
                    
                    # if ((self.pos_err_avr_norm < (self.acceptable_pos_err_avr_norm+self.d_obstacle_offset)) and
                    if ((self.pos_err_avr_norm < (self.acceptable_pos_err_avr_norm)) and
                        (self.ori_err_avr_norm < self.acceptable_ori_err_avr_norm)):
                        
                        # Log the current error norms and the acceptable error norms
                        rospy.loginfo("Current error norms: pos_err_avr_norm: " + str(self.pos_err_avr_norm) + ", ori_err_avr_norm: " + str(self.ori_err_avr_norm))
                        rospy.loginfo("Acceptable error norms: pos_err_avr_norm: " + str(self.acceptable_pos_err_avr_norm) + ", ori_err_avr_norm: " + str(self.acceptable_ori_err_avr_norm))
                        
                        # assign the zero control output to pause the controller
                        self.update_last_control_output_is_valid_time()
                        control_output = np.zeros(6*len(self.custom_static_particles))                
                        self.assign_control_outputs(control_output)
                        
                        # call set_enable service to disable the controller
                        self.controller_enabler(enable=False, cause="automatic")
                        # Create green colored log info message
                        # rospy.loginfo("\033[92m" + "Error norms are below the thresholds. The controller is disabled." + "\033[0m")
                        rospy.loginfo("Success! Error norms are below the acceptible error thresholds.")
                        return

                    ## 2. Else if the replanning is allowed, replan the path
                    elif self.replanning_allowed and self.num_replanning_attempts < self.max_replanning_attempts:                    
                        rospy.logwarn("Replanning is triggered due to the error norms are not changing.")
                        
                        # Log the current error norms and the acceptable error norms
                        rospy.logwarn("Current error norms: pos_err_avr_norm: " + str(self.pos_err_avr_norm) + ", ori_err_avr_norm: " + str(self.ori_err_avr_norm))
                        rospy.logwarn("Acceptable error norms: pos_err_avr_norm: " + str(self.acceptable_pos_err_avr_norm) + ", ori_err_avr_norm: " + str(self.acceptable_ori_err_avr_norm))
                        
                        # assign the zero control output to pause the controller
                        self.update_last_control_output_is_valid_time()
                        control_output = np.zeros(6*len(self.custom_static_particles))
                        self.assign_control_outputs(control_output)
                        
                        # Disable the path planning with pre-saved paths to allow automatic replanning
                        self.path_planning_pre_saved_paths_enabled = False
                        
                        # Update the initial state as the current state 
                        # w/out saving it, because it'll be logged in the update_path_status_timer_callback
                        self.full_states_setter_n_saver(state_type="initial", save=False)
                        
                        # Resetting the planned path variables will force replanning
                        # w/out stopping the controller
                        self.reset_planned_path_variables() 
                        
                        # Set the replanning flag and increment the replanning attempts
                        self.is_replanning_needed = True
                        self.num_replanning_attempts += 1
                        return
                        
                    ## 3. Else (i.e. if the replanning is not allowed or the replanning attempts are exhausted)
                    # disable the controller with a failure 
                    else:
                        if not self.replanning_allowed:
                            rospy.logwarn("Replanning is needed however replanning is not allowed.")
                        elif self.num_replanning_attempts >= self.max_replanning_attempts:
                            rospy.logwarn("Replanning is needed however maximum replanning attempts are reached.")
                            
                        # Log the current error norms and the acceptable error norms
                        rospy.logwarn("Current error norms: pos_err_avr_norm: " + str(self.pos_err_avr_norm) + ", ori_err_avr_norm: " + str(self.ori_err_avr_norm))
                        rospy.logwarn("Acceptable error norms: pos_err_avr_norm: " + str(self.acceptable_pos_err_avr_norm) + ", ori_err_avr_norm: " + str(self.acceptable_ori_err_avr_norm))
                        
                        rospy.logwarn("The controller is going to be disabled.")
                        
                        self.is_replanning_needed = True
                        
                        # call set_enable service to disable the controller
                        self.controller_enabler(enable=False, cause="error norms are not changing")
                        # Create red colored log info message
                        # rospy.logerr("\033[91m" + "The controller is disabled due to the error norms are not changing." + "\033[0m")
                        rospy.logerr("The controller is disabled due to the error norms are not changing.")

                        # assign the zero control output
                        self.update_last_control_output_is_valid_time()
                        control_output = np.zeros(6*len(self.custom_static_particles))
                        self.assign_control_outputs(control_output)
                        return
                        
                else: # The change in error norms are above the thresholds, i.e. not converged yet
                    # Proceed with the safe control output calculations
                    
                    # init_t = time.time()                
                    
                    # Calculate safe control output with obstacle avoidance        
                    control_output = self.calculate_safe_control_output(control_output) # safe # (12,)
                    # rospy.logwarn("QP solver calculation time: " + str(1000*(time.time() - init_t)) + " ms.")
                    
                    if control_output is not None: # Successfully calculated the safe control output
                        self.update_last_control_output_is_valid_time()
                        self.assign_control_outputs(control_output)
                        return
                    
                    else: # if control output is None (ie. QP solver error)
                        # check if the control output is None for a long time
                        if (rospy.Time.now() - self.time_last_control_output_is_valid).to_sec() > self.valid_control_output_wait_timeout:
                            # call set_enable service to disable the controller
                            self.controller_enabler(enable=False, cause="QP solver error")
                            # Create red colored log info message
                            # rospy.logerr("\033[91m" + "The controller is disabled due to the QP solver error." + "\033[0m")
                            rospy.logerr("The controller is disabled due to the QP solver error.")

                        # assign the zero control output
                        control_output = np.zeros(6*len(self.custom_static_particles))
                        self.assign_control_outputs(control_output)
                        return
                # ----------------------------------------------------------------------------------------------------  
                

            # # Reset the event to wait for the next input
            # self.proceed_event.clear()  

            # else:
            #     self.enabled = False


    def odom_pub_timer_callback(self,event):
        """
        Integrates the self.control_outputs with time and publishes the resulting poses as Odometry messages.
        """
        if self.initialized:
            # Only publish if enabled
            if self.enabled:
                # TODO: This for loop can be parallelized for better performance
                for particle in self.custom_static_particles:
                    # Do not proceed until the initial values have been set
                    if ((not (particle in self.particle_positions)) or \
                        (not self.is_perturbed_states_set_for_particle(particle)) or \
                        (not self.is_perturbed_min_distances_set(particle))):   
                        # print("---------------------------")
                        continue
                        
                    if self.control_outputs[particle] is not None:
                        # Prepare Odometry message
                        odom = Odometry()
                        odom.header.stamp = rospy.Time.now()
                        odom.header.frame_id = "map"

                        # dt_check = self.nominal_controllers[particle].get_dt() # 
                        dt = 1./self.pub_rate_odom

                        # Scale down the calculated output if its norm is higher than the specified norm max_u
                        control_outputs_linear = self.scale_down_vector(self.control_outputs[particle][:3], max_u=self.max_linear_velocity)

                        # Control output is the new position
                        odom.pose.pose.position.x =  self.particle_positions[particle].x + control_outputs_linear[0]*dt
                        odom.pose.pose.position.y =  self.particle_positions[particle].y + control_outputs_linear[1]*dt
                        odom.pose.pose.position.z =  self.particle_positions[particle].z + control_outputs_linear[2]*dt

                        # The new orientation
                        axis_angle = np.array(self.control_outputs[particle][3:6])
                        angle = np.linalg.norm(axis_angle)
                        axis = axis_angle / angle if (angle > 1e-9) else np.array([0, 0, 1])
                        
                        # Scale down the angle as omega.
                        angle = self.scale_down_vector(angle, max_u=self.max_angular_velocity) if (angle > 1e-9) else angle

                        # Update axis_angle
                        axis_angle = angle * axis

                        # Convert axis-angle to quaternion
                        delta_orientation = tf_trans.quaternion_about_axis(angle * dt, axis)

                        # Update the current orientation by multiplying with delta orientation
                        current_orientation = [self.particle_orientations[particle].x, 
                                            self.particle_orientations[particle].y, 
                                            self.particle_orientations[particle].z, 
                                            self.particle_orientations[particle].w]
                        
                        new_orientation = tf_trans.quaternion_multiply(delta_orientation, current_orientation)

                        # Assign the new orientation to the Odometry message
                        odom.pose.pose.orientation.x = new_orientation[0]
                        odom.pose.pose.orientation.y = new_orientation[1]
                        odom.pose.pose.orientation.z = new_orientation[2]
                        odom.pose.pose.orientation.w = new_orientation[3]

                        # Assign the linear and angular velocities to the Odometry message
                        odom.twist.twist.linear.x = control_outputs_linear[0]
                        odom.twist.twist.linear.y = control_outputs_linear[1]
                        odom.twist.twist.linear.z = control_outputs_linear[2]
                        odom.twist.twist.angular.x = axis_angle[0]
                        odom.twist.twist.angular.y = axis_angle[1]
                        odom.twist.twist.angular.z = axis_angle[2]

                        # Update the pose of the particle 
                        # self.particle_positions[particle] = odom.pose.pose.position
                        # self.particle_orientations[particle] = odom.pose.pose.orientation

                        # Save the applied control output as the last control output
                        self.control_outputs_last[particle][:3] = control_outputs_linear
                        self.control_outputs_last[particle][3:6] = axis_angle

                        # Publish
                        if not self.real_robot_mode_enabled:
                            self.odom_publishers[particle].publish(odom)
                        else:
                            self.twist_publishers[particle].publish(odom.twist.twist)
                    else:
                        self.control_outputs[particle] = np.zeros(6)

    def odom_publishers_publish_zero_velocities(self):
        """
        This is a helper function that can be used to publish zero velocities for all particles.
        Publishes zero velocities for all particles.
        Useful for stopping the particles when the controller is disabled.
        """
        for particle in self.custom_static_particles:
            # Set the control outputs to zero
            self.control_outputs[particle] = np.zeros(6)

            # Prepare Odometry message
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"

            # Assign the current position and orientation
            odom.pose.pose.position = self.particle_positions[particle]
            odom.pose.pose.orientation = self.particle_orientations[particle]

            # Assign zero velocities
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0

            # Publish
            if not self.real_robot_mode_enabled:
                self.odom_publishers[particle].publish(odom)
            else:
                self.twist_publishers[particle].publish(odom.twist.twist)


    def wrench_to_numpy(self, wrench):
        """
        Converts a ROS wrench message to a numpy array.

        :param wrench: The wrench (force and torque) in ROS message format.
        :return: A numpy array representing the wrench.
        """
        # Combine force and torque arrays
        return np.array([wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z])
    
    def twist_to_numpy(self, twist):
        """
        Converts a ROS twist message to a numpy array.

        :param twist: The twist (linear and angular velocities) in ROS message format.
        :return: A numpy array representing the twist.
        """
        # Combine linear and angular velocity arrays
        return np.array([twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z])
    
    def calculate_pose_difference(self, current_pose, perturbed_pose):
        """ 
        Calculates the pose difference between the current pose and the perturbed pose.
        The input poses are given as ROS pose msgs.
        """
        # Position difference
        diff_x = perturbed_pose.position.x - current_pose.position.x
        diff_y = perturbed_pose.position.y - current_pose.position.y
        diff_z = perturbed_pose.position.z - current_pose.position.z

        # Orientation difference
        current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        perturbed_orientation = [perturbed_pose.orientation.x, perturbed_pose.orientation.y, perturbed_pose.orientation.z, perturbed_pose.orientation.w]

        # Relative rotation quaternion from current to perturbed
        quaternion_difference = tf_trans.quaternion_multiply(perturbed_orientation, tf_trans.quaternion_inverse(current_orientation))

        # Normalize the quaternion to avoid numerical issues
        quaternion_difference = self.normalize_quaternion(quaternion_difference)

        # # AXIS-ANGLE ORIENTATION ERROR/DIFFERENCE DEFINITION
        # # Convert quaternion difference to rotation vector (axis-angle representation)
        # rotation_vector = self.quaternion_to_rotation_vec(quaternion_difference)

        # EULER ANGLES POSE ERROR/DIFFERENCE DEFINITION
        # Convert quaternion to Euler angles for a more intuitive error representation
        rotation_vector = tf_trans.euler_from_quaternion(quaternion_difference)

        rotation_vector = np.array(rotation_vector)

        # Add a deadzone to the orientation error 
        deadzone_mask = np.abs(rotation_vector) < np.deg2rad(0.1) # Create a mask where values are within the deadzone
        rotation_vector[deadzone_mask] = 0 # Set values within the deadzone to zero
        # print(np.rad2deg(rotation_vector))

        # return np.array([diff_x, diff_y, diff_z]), rotation_vector
        # Combine position difference and rotation vector into a 6x1 vector
        return np.array([diff_x, diff_y, diff_z] + list(rotation_vector))
    
    def calculate_pose_target_error(self, current_pose, target_pose):
        """ 
        Calculates the pose error between the current pose and the target pose.
        The input poses are given as ROS pose msgs.
        """
        # Position error
        err_x = target_pose.position.x - current_pose.position.x
        err_y = target_pose.position.y - current_pose.position.y
        err_z = target_pose.position.z - current_pose.position.z

        # Orientation error as quaternion
        current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        target_orientation = [target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]

        # Relative rotation quaternion from current to target
        quaternion_error = tf_trans.quaternion_multiply(target_orientation, tf_trans.quaternion_inverse(current_orientation))

        # Normalize the quaternion to avoid numerical issues
        quaternion_error = self.normalize_quaternion(quaternion_error)

        # # AXIS-ANGLE ORIENTATION ERROR/DIFFERENCE DEFINITION
        # # Convert quaternion difference to rotation vector (axis-angle representation)
        # rotation_vector = self.quaternion_to_rotation_vec(quaternion_error)

        # EULER ANGLES POSE ERROR/DIFFERENCE DEFINITION
        # Convert quaternion to Euler angles for a more intuitive error representation
        rotation_vector = tf_trans.euler_from_quaternion(quaternion_error)

        rotation_vector = np.array(rotation_vector)

        # Add a deadzone to the orientation error 
        deadzone_mask = np.abs(rotation_vector) < np.deg2rad(0.1) # Create a mask where values are within the deadzone
        rotation_vector[deadzone_mask] = 0 # Set values within the deadzone to zero
        # print(np.rad2deg(rotation_vector))

        # Combine position difference and rotation vector into a 6x1 vector
        return np.array([err_x, err_y, err_z] + list(rotation_vector))
    
    def get_system_timestamp(self):
        # Get the current system time
        now = datetime.now()
        # Format the datetime object into a string similar to rosbag filenames
        timestamp = now.strftime("%Y-%m-%d-%H-%M-%S")
        return timestamp

    def multiply_quaternions(self, q1, q2):
        """
        Multiply two quaternions.
        """
        w0, x0, y0, z0 = q1.w, q1.x, q1.y, q1.z
        w1, x1, y1, z1 = q2.w, q2.x, q2.y, q2.z

        w = w0*w1 - x0*x1 - y0*y1 - z0*z1
        x = w0*x1 + x0*w1 + y0*z1 - z0*y1
        y = w0*y1 - x0*z1 + y0*w1 + z0*x1
        z = w0*z1 + x0*y1 - y0*x1 + z0*w1

        return Quaternion(x, y, z, w)

    def inverse_quaternion(self, q):
        """
        Calculate the inverse of a quaternion.
        """
        w, x, y, z = q.w, q.x, q.y, q.z
        norm = w**2 + x**2 + y**2 + z**2

        return Quaternion(-x/norm, -y/norm, -z/norm, w/norm)
    
    def quaternion_to_matrix(self, quat):
        """
        Return 3x3 rotation matrix from quaternion geometry msg.
        """
        # Extract the quaternion components
        q = np.array([quat.x, quat.y, quat.z, quat.w], dtype=np.float64, copy=True)

        n = np.dot(q, q)
        if n < np.finfo(float).eps:
            return np.identity(3)
        
        q *= np.sqrt(2.0 / n)
        q = np.outer(q, q)

        return np.array((
            (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3]),
            (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3]),
            (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1])
            ), dtype=np.float64)
    
    def hat(self,k):
        """
        Returns a 3 x 3 cross product matrix for a 3 x 1 vector
        
                [  0 -k3  k2]
        khat =  [ k3   0 -k1]
                [-k2  k1   0]
        
        :type    k: numpy.array
        :param   k: 3 x 1 vector
        :rtype:  numpy.array
        :return: the 3 x 3 cross product matrix    
        """
        
        khat=np.zeros((3,3))
        khat[0,1]=-k[2]
        khat[0,2]=k[1]
        khat[1,0]=k[2]
        khat[1,2]=-k[0]
        khat[2,0]=-k[1]
        khat[2,1]=k[0]    
        return khat
    
    def quaternion_to_rotation_vec(self, quaternion):
        """
        Converts a quaternion to axis-angle representation.
        Minimal representation of the orientation error. (3,) vector.
        Arguments:
            quaternion: The quaternion to convert as a numpy array in the form [x, y, z, w].
        Returns:
            rotation_vector: Minimal axis-angle representation of the orientation error. (3,) vector.
            Norm if the axis_angle is the angle of rotation.
        """
        angle = 2 * np.arccos(quaternion[3])
        
        # Wrap the angle to [-pi, pi]
        angle = np.arctan2(np.sin(angle), np.cos(angle))

        # Handling small angles with an approximation
        small_angle_threshold = 1e-6
        if np.abs(angle) < small_angle_threshold:
            # Use small angle approximation
            rotation_vector = np.array([0.,0.,0.])
            
            # # Also calculate the representation Jacobian inverse
            # J_inv = np.eye((3,3))
        else:
            # Regular calculation for larger angles
            axis = quaternion[:3] / np.sin(angle/2.0)
            # Normalize the axis
            axis = axis / np.linalg.norm(axis)
            rotation_vector = angle * axis 
            
            # # Also calculate the representation Jacobian inverse
            # k_hat = self.hat(axis)
            # cot = 1.0 / np.tan(angle/2.0)
            # J = -angle/2.0 * (k_hat + cot*(k_hat @ k_hat)) + np.outer(axis, axis)
            # J_inv = np.linalg.inv(J)
            
        return rotation_vector # , J_inv

    def normalize_quaternion(self, quaternion):
        norm = np.linalg.norm(quaternion)
        if norm == 0:
            raise ValueError("Cannot normalize a quaternion with zero norm.")
        return quaternion / norm

    def scale_down_vector(self, u, max_u):
        norm_u = np.linalg.norm(u)

        if norm_u > max_u:
            u = u / norm_u * max_u

        return u

    def calculate_cost_weight_z_pos(self, stress_avoidance_performance, overall_min_distance):
        """
        Calculates the weight for the z position control based on the stress avoidance performance and the overall minimum distance to collision.
        """
        # Weight limits
        w_max = 1.0
        w_min = 0.4 # 0.05

        # Compute the geometric mean of the stress avoidance performance and the overall minimum distance to collision
        # Below, both values are in the range [0, 1].
        w = np.sqrt(stress_avoidance_performance * min(1.0, max(0.0, overall_min_distance / self.d_obstacle_freezone)))

        ## Compute the weight in the range [w_min, w_max] 
        # OPTION 1:
        # w_boost = 0.95 # must be in the range [0, 1]
        # weight = (w_max - w_min)*(w**w_boost) + w_min 

        # OPTION 2: 
        # See: https://www.desmos.com/calculator/obvcltohjs for the function visualizations
        # Sigmoid adjustment
        k = 20  # Steepness of the sigmoid function transition
        d = 0.4  # Midpoint of the sigmoid function
        # Compute the sigmoid function to smooth transition
        s = 1 / (1 + np.exp(-k * (w - d)))
        weight = (w_max - w_min) * (w + (1 - w) * s) + w_min

        # rospy.loginfo("Weight for z position control: " + str(weight))

        return weight
    
    def calculate_weight_nominal_input(self, stress_avoidance_performance, overall_min_distance):
        """
        Calculates the weight for the z position control based on the stress avoidance performance and the overall minimum distance to collision.
        """
        # Weight limits
        w_max = 1.0
        w_min = 0.1 # 0.4 (for z weight) # 0.05 # 0.1 (for nominal control scaling)

        # Compute the geometric mean of the stress avoidance performance and the overall minimum distance to collision
        # Below, both values are in the range [0, 1].
        w = np.sqrt(stress_avoidance_performance * min(1.0, max(0.0, overall_min_distance / (self.d_obstacle_freezone) )))

        ## Compute the weight in the range [w_min, w_max] 
        # OPTION 1:
        # w_boost = 0.95 # must be in the range [0, 1]
        # weight = (w_max - w_min)*(w**w_boost) + w_min 

        # OPTION 2: 
        # See: https://www.desmos.com/calculator/obvcltohjs for the function visualizations
        # Sigmoid adjustment
        k = 20  # Steepness of the sigmoid function transition
        d = 0.4  # Midpoint of the sigmoid function
        # Compute the sigmoid function to smooth transition
        s = 1 / (1 + np.exp(-k * (w - d)))
        weight = (w_max - w_min) * (w + (1 - w) * s) + w_min

        # rospy.loginfo("Weight for z position control: " + str(weight))

        return weight

    def calculate_and_publish_stress_avoidance_performance(self, h_ft_normalized):
        """
        Calculates and publishes the performance of the stress avoidance.
        """

        def f(x, c_p=2.773):
            """
            Function to boost values further away from zero.
            c_p is a constant that determines the steepness of the function, 
            c_p must be positive for boosting.
            """
            return (np.exp(-c_p * x) - 1) / (np.exp(-c_p) - 1)

        # Check if any of the h_ft_normalized values are less than 0
        if np.any(h_ft_normalized < 0.0):
            # Publish the performance as 0 if any of the h_ft_normalized values are less than 0
            performance = 0.0
        else:
            # Multiply all the h_ft_normalized values
            performance = np.prod(h_ft_normalized)
            # Apply the f(x) function to the product
            performance = f(performance)

        # Publish the performance
        self.info_pub_stress_avoidance_performance.publish(Float32(data=performance))

        # Update the performance history
        self.stress_avoidance_performance_sum += performance
        self.stress_avoidance_performance_avr = self.stress_avoidance_performance_sum / self.controller_itr

        if performance <= 0.0:
            self.stress_avoidance_performance_ever_zero = True

        # Publish the average performance
        self.info_pub_stress_avoidance_performance_avr.publish(Float32(data=self.stress_avoidance_performance_avr))
        return performance

    def alpha_collision_avoidance(self,h):
        """
        Calculates the value of extended_class_K function \alpha(h) for COLLISION AVOIDANCE
        Piecewise Linear function is used when h is less than 0,
        when h is greater or equal to 0 a nonlinear function is used.
        See: https://www.desmos.com/calculator/dtsdrcczge for the function visualizations
        """        
        if (h < -self.d_obstacle_offset):
            # alpha_h = -self.c3_alpha_obstacle*self.d_obstacle_offset # Use this if you want to have a fixed value for alpha when h < -d_obstacle_offset
            alpha_h = self.c3_alpha_obstacle*h # Use this if you want to have a linear function for alpha when h < -d_obstacle_offset
        elif (-self.d_obstacle_offset <= h < 0 ):
            alpha_h = self.c3_alpha_obstacle*h
        elif (0 <= h < (self.d_obstacle_freezone - self.d_obstacle_offset)):
            alpha_h = (self.c1_alpha_obstacle*h)/((self.d_obstacle_freezone - self.d_obstacle_offset)-h)**self.c2_alpha_obstacle
        else:
            alpha_h = float('inf')
        
        return alpha_h
    
    # def alpha_collision_avoidance(self,h):
    #     """
    #     Calculates the value of extended_class_K function \alpha(h) for COLLISION AVOIDANCE
    #     Piecewise Linear function is used when h is less than 0,
    #     when h is greater or equal to 0 a nonlinear function is used.
    #     See: https://www.desmos.com/calculator/owyqxbpbn3 for the function visualizations
    #     """        
    #     if (h < -self.d_obstacle_offset):
    #         alpha_h = -self.c3_alpha_obstacle*self.d_obstacle_offset # Use this if you want to have a fixed value for alpha when h < -d_obstacle_offset
    #         # alpha_h = self.c3_alpha_obstacle*h # Use this if you want to have a linear function for alpha when h < -d_obstacle_offset
    #     elif (-self.d_obstacle_offset <= h < 0 ):
    #         alpha_h = self.c3_alpha_obstacle*h
    #     elif (0 <= h < (self.d_obstacle_freezone - self.d_obstacle_offset)):
    #         alpha_h = np.tan((self.c1_alpha_obstacle*np.pi*h)/(2*(self.d_obstacle_freezone - self.d_obstacle_offset)))
    #     else:
    #         alpha_h = np.tan((self.c1_alpha_obstacle*np.pi)/2)
        
    #     return alpha_h
    
    def alpha_robot_stress(self, h):
        """
        Calculates the value of extended_class_K function \alpha(h) for ROBOT STRESS.
        Returns 6x1 alpha_h vector.
        Note that h is 6x1 vector and adjusted to be less than or equal to the wrench_max (h_ft = (wrench_max - wrench_offset)^2 - (wrench)^2).
        (self.wrench_max - self.w_stress_offset)**2 - (self.particle_wrenches[particle])**2 
        
        Piecewise Linear function is used when h is less than 0,
        when h is greater or equal to 0 a nonlinear function is used.
        
        See: https://www.desmos.com/calculator/hc6lc7nzkk for the function visualizations
        NEW: https://www.desmos.com/calculator/mwiakn4s5j (FORCE)
        NEW: https://www.desmos.com/calculator/tnflamxjvx (TORQUE)
        """
        # Initialize alpha_h with zeros
        alpha_h = np.zeros(h.shape)

        # Boolean masks for the conditions
        condition_positive_or_zero = h >= 0
        condition_negative = h < 0
        condition_close_to_limit = ((self.wrench_max - self.w_stress_offset)**2 - h) < 1e-6  # Add an epsilon threshold to avoid division by zero
        
        # Default values for when h is close to wrench_max
        alpha_h[condition_close_to_limit] = float('inf')  # Assign a default value or handle appropriately
        
        # Calculate for h values greater or equal to 0, excluding values close to the limit
        valid_condition_positive_or_zero = condition_positive_or_zero & ~condition_close_to_limit
        alpha_h[valid_condition_positive_or_zero] = (self.c1_alpha_ft[valid_condition_positive_or_zero] * h[valid_condition_positive_or_zero] ) / \
                                                    ( (self.wrench_max[valid_condition_positive_or_zero] - self.w_stress_offset[valid_condition_positive_or_zero])**2 -  h[valid_condition_positive_or_zero] ) ** self.c2_alpha_ft[valid_condition_positive_or_zero]

        # Calculate for h values less than 0
        alpha_h[condition_negative] = self.c3_alpha_ft[condition_negative] * h[condition_negative]

        return alpha_h

    # def alpha_robot_stress(self, h):
    #     """
    #     Calculates the value of extended_class_K function \alpha(h) for ROBOT STRESS.
    #     Returns 6x1 alpha_h vector.
    #     Note that h is 6x1 vector and adjusted to be less than or equal to the wrench_max (h_ft = wrench_max - |wrench|).
    #     Piecewise Linear function is used when h is less than 0,
    #     when h is greater or equal to 0 a nonlinear function is used.
    #     See: https://www.desmos.com/calculator/hc6lc7nzkk for the function visualizations
    #     """
    #     # Initialize alpha_h with zeros
    #     alpha_h = np.zeros(h.shape)

    #     # Boolean masks for the conditions
    #     condition_positive_or_zero = h >= 0
    #     condition_negative = h < 0
    #     condition_close_to_limit = (self.wrench_max - h) < 1e-6  # Add an epsilon threshold to avoid division by zero

    #     # Default values for when h is close to wrench_max
    #     alpha_h[condition_close_to_limit] = float('inf')  # Assign a default value or handle appropriately

    #     # Calculate for h values greater or equal to 0, excluding values close to the limit
    #     valid_condition_positive_or_zero = condition_positive_or_zero & ~condition_close_to_limit
    #     alpha_h[valid_condition_positive_or_zero] = (self.c1_alpha_ft[valid_condition_positive_or_zero] * h[valid_condition_positive_or_zero]) / \
    #                                                 (self.wrench_max[valid_condition_positive_or_zero] - h[valid_condition_positive_or_zero]) ** self.c2_alpha_ft[valid_condition_positive_or_zero]

    #     # Calculate for h values less than 0
    #     alpha_h[condition_negative] = self.c3_alpha_ft[condition_negative] * h[condition_negative]

    #     return alpha_h
    
    ## ----------------------------------------------------------------------------------------
    ## --------------------------- PATH PLANNING AND TRACKING ---------------------------------
    def reset_planned_path_variables(self):
        self.planned_path = [] # (it is a list of PoseStamped() msgs)
        self.planned_path_points = None # path 3D xyz points as numpy array
        self.planned_path_cumulative_lengths = [0.0] # cumulative lengths of the path segments
        self.planned_path_cumulative_rotations = [0.0] # cumulative rotations of the path segments obtained from the angles in axis-angle representation consecutive rotations in radians
        self.planned_path_direction_vectors = [] # directions of the path segments as unit vectors
        self.planned_path_rotation_vectors = [] # rotation axes of the path segments as unit vectors

        self.planned_path_completion_rate = 0.0 # completion rate of the planned path
        self.planned_path_current_target_index = 1 # index of the current waypoint in the planned path
        self.planned_path_current_target_pose = None # current target pose of the planned path as a Pose() msg
                
        self.planned_path_of_particles = {} # planned path of the particles as a list of Pose() msgs
        self.planned_path_points_of_particles = {} # path 3D xyz points of the particles as numpy array
        self.planned_path_cumulative_lengths_of_particles = {} # cumulative lengths of the path segments of the particles
        self.planned_path_cumulative_rotations_of_particles = {} # cumulative rotations of the path segments of the particles obtained from the angles in axis-angle representation consecutive rotations in radians
        self.planned_path_direction_vectors_of_particles = {} # directions of the path segments of the particles as unit vectors (list of n elements with each element is a 3D vector for each particle)
        self.planned_path_rotation_vectors_of_particles = {} # rotation axes of the path segments of the particles as unit vectors (list of n elements with each element is a 3D vector for each particle)

        self.planned_path_current_target_poses_of_particles = {} # current target poses of the particles as Pose() msgs
        
        self.planned_path_velocity_profile_of_particles = {} # velocity profile of the particles
        self.planned_path_current_target_velocities_of_particles = {} # current target velocities of the particles based on the velocity profile

    def average_quaternions(self, Q, weights):
        '''
        Averaging Quaternions using Markley's method with weights.[1]
        See: https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
        
        [1] Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman. 
        "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30, no. 4 (2007): 1193-1197.

        Arguments:
            Q (ndarray): An Mx4 ndarray of quaternions. Each quaternion has [w, x, y, z] format.
            weights (list): An M elements list, a weight for each quaternion.

        Returns:
            ndarray: The weighted average of the input quaternions in [w, x, y, z] format. 
        '''
        # Use the optimized numpy functions for a more efficient computation
        A = np.einsum('ij,ik,i->...jk', Q, Q, weights)
        # Compute the eigenvectors and eigenvalues, and return the eigenvector corresponding to the largest eigenvalue
        return np.linalg.eigh(A)[1][:, -1]
    
    def update_path_status_timer_callback(self, event):
        if self.initialized:
            # Only publish if enabled
            if self.enabled and self.update_path_status_timer_callback_lock.acquire(blocking=False):
                try:
                    ## ----------------------------------------------------------------------------------------------
                    if self.path_planning_tesseract_enabled:
                        ## ----------------------------------------------------------------------------------------------
                        if (not self.planned_path) and (not self.path_planning_pre_saved_paths_enabled):
                            """
                            Calculate the tesseract planned path
                            """
                            
                            # If the user did not provide an initial state file, update the initial state as the current state
                            if not self.initial_state_file:
                                rospy.logwarn("Initial state file is not provided. Updating the initial state as the current state.")
                                # Update the initial state as the current state 
                                self.full_states_setter_n_saver(state_type="initial", save=False)
                                
                            # Check if the initial and target states are set
                            if self.initial_full_state_dict is None:
                                rospy.logwarn("Please set the Initial state using the 'set_n_save_initial_state' service.")
                            
                            elif self.target_full_state_dict is None:
                                rospy.logwarn("Please set the Target state using the 'set_n_save_target_state' service.")
                                
                            else:
                                # Log both the initial and target states for the current execution
                                self.log_initial_n_target_full_states()
                                
                                try:
                                    plan_data = self.tesseract_planner.plan(self.initial_full_state_dict,
                                                                            self.target_full_state_dict,
                                                                            self.dlo_l,
                                                                            self.dlo_r,
                                                                            self.custom_static_particles,
                                                                            self.automatic_tent_pole_max_links,
                                                                            self.automatic_tent_pole_min_links,
                                                                            self.environment_limits_xyz,
                                                                            self.joint_angle_limits_xyz_deg,
                                                                            self.dlo_approximation_error_threshold,
                                                                            self.coll_depth_to_try_remove,
                                                                            return_all_data=True,
                                                                            plot_for_debugging=False)
                                    
                                    # Unpack the plan data
                                    (plan_data_ompl, 
                                    plan_data_trajopt, 
                                    performance_data, 
                                    initial_n_target_states,
                                    performance_data_extra) = plan_data
                                    
                                    planning_success = performance_data[3]
                                    
                                    if planning_success:
                                        # Set the planned path variables for the execution
                                        (self.planned_path,
                                        self.planned_path_points,
                                        self.planned_path_cumulative_lengths,
                                        self.planned_path_cumulative_rotations,
                                        self.planned_path_direction_vectors,
                                        self.planned_path_rotation_vectors,
                                        self.planned_path_of_particles,
                                        self.planned_path_points_of_particles,
                                        self.planned_path_cumulative_lengths_of_particles,
                                        self.planned_path_cumulative_rotations_of_particles,
                                        self.planned_path_direction_vectors_of_particles,
                                        self.planned_path_rotation_vectors_of_particles, 
                                        planned_path_approximated_dlo_joint_values) = plan_data_trajopt
                                    
                                    # Save and log the performance data and plan data
                                    self.tesseract_planner.save_performance_results(performance_data=performance_data,
                                                                                    performance_data_extra=performance_data_extra,
                                                                                    experiment_id=self.controller_enabled_time_str,
                                                                                    mingruiyu_scene_id=None,
                                                                                    dlo_type=None,
                                                                                    saving_folder_name=self.executions_log_directory,
                                                                                    is_to_current_file_dir=False)
                                    
                                    # Save the planned path data
                                    self.tesseract_planner.save_generated_paths(plan_data=plan_data,
                                                                                experiment_id=self.controller_enabled_time_str,
                                                                                mingruiyu_scene_id=None,
                                                                                dlo_type=None,
                                                                                saving_folder_name=self.executions_log_directory,
                                                                                is_to_current_file_dir=False)
                                except Exception as e:
                                    rospy.logerr("An error occurred while planning the path using Tesseract: " + str(e))
                                    rospy.logerr("{}".format(traceback.format_exc()))
                                                    
                        ## ----------------------------------------------------------------------------------------------
                        if (self.planned_path) and (not self.planned_path_velocity_profile_of_particles):
                            """
                            Calculate the velocity profile for the particles.
                            """
                            
                            rospy.loginfo("Calculating the velocity profile for the particles.")
                            # Calculate the velocity profile for the particles
                            self.planned_path_velocity_profile_of_particles = self.calculate_path_tracking_velocity_profile_of_particles()
                            
                            # print("Planned path velocity profile of the particles: \n", self.planned_path_velocity_profile_of_particles)
                            
                            # Set the current target index and pose for the new planned path
                            self.planned_path_current_target_index = 1
                            self.update_current_path_target_poses_and_velocities(self.planned_path_current_target_index)

                            # Publish the planned path for visualization
                            self.publish_path(self.planned_path)

                            # Publish the planned path of the particles for visualization
                            self.publish_paths_of_particles(self.planned_path_of_particles)
                        ## ----------------------------------------------------------------------------------------------
                        if (self.planned_path) and (self.planned_path_velocity_profile_of_particles):
                            """
                            Path tracking...
                            """
                            ## ---------------------------------------------------
                            """
                            Calculate how much of the planned path is completed and, update and publish the current target point as a geometry_msgs/PointStamped.
                            """
                            # Calculate the completion rate of the planned path
                            # Also update the current target pose and index on the path
                            self.planned_path_current_target_index = self.update_current_path_target_index(self.planned_path_current_target_index)
                            self.update_current_path_target_poses_and_velocities(self.planned_path_current_target_index)

                            # Create a PointStamped message for the current target points and publish for visualization
                            self.publish_current_target_points()
                            ## ---------------------------------------------------
                        ## ----------------------------------------------------------------------------------------------
                    ## ----------------------------------------------------------------------------------------------
                finally:
                    self.update_path_status_timer_callback_lock.release()  
                    # pass      

    def set_path(self, path_variables):
        """
        Sets the path variables.
        """
        rospy.loginfo("Setting the path variables.")
        
        (self.planned_path,
        self.planned_path_points,
        self.planned_path_cumulative_lengths,
        self.planned_path_cumulative_rotations,
        self.planned_path_direction_vectors,
        self.planned_path_rotation_vectors,
        self.planned_path_of_particles,
        self.planned_path_points_of_particles,
        self.planned_path_cumulative_lengths_of_particles,
        self.planned_path_cumulative_rotations_of_particles,
        self.planned_path_direction_vectors_of_particles,
        self.planned_path_rotation_vectors_of_particles, _) = path_variables
        return
            
    def calculate_centroid_pose_of_target_poses(self):
        """
        Calculates the centroid pose of the target poses.
        """
        # Initialize the centroid pose
        centroid_pose = Pose()
        
        # Calculate the centroid position
        centroid_position = Point()
        num_poses = 0
        quaternions = []
        
        for idx_tip, tip in enumerate(self.tip_particles):
            # Do not proceed until the initial values have been set
            if not (tip in self.particle_positions):
                rospy.logwarn("Tip particle: " + str(tip) + " state is not obtained yet (2).")
                continue

            pose = self.target_poses[tip]
        
            centroid_position.x += pose.position.x
            centroid_position.y += pose.position.y
            centroid_position.z += pose.position.z
            
            # Collect quaternion for averaging
            quaternions.append([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ])
            
            # Increase valid poses count
            num_poses += 1
            
        # Avoid division by zero if no valid poses found
        if num_poses == 0:
            return centroid_pose
            
        # Calculate the average position
        centroid_position.x /= num_poses
        centroid_position.y /= num_poses
        centroid_position.z /= num_poses
        
        # Set the centroid position to the Pose
        centroid_pose.position = centroid_position
        
        # Calculate the centroid orientation
        centroid_orientation = Quaternion()
        
        # Prepare quaternion data for averaging
        quaternions = np.array(quaternions)
        weights = np.array([1] * num_poses)  # Assuming equal weight for simplicity
        
        # Calculate the average quaternion
        avg_quaternion = self.average_quaternions(quaternions, weights)
        
        # Normalize the quaternion to avoid numerical issues
        avg_quaternion = self.normalize_quaternion(avg_quaternion)
        
        # Set the average quaternion to the centroid orientation
        centroid_orientation = Quaternion(avg_quaternion[1], avg_quaternion[2], avg_quaternion[3], avg_quaternion[0]) # Note x, y, z, w
        
        # Set the centroid orientation to the Pose
        centroid_pose.orientation = centroid_orientation
        
        return centroid_pose
                
    def update_current_path_target_index(self, current_target_index):
        """
        Calculates how much of the path is completed and updates the current target index.
        The completion rate is the ratio of the distance traveled to the total path length.
        The current target point is the next point on the path that the robot should reach.
        This function should be called periodically to update the current target point.
        """

        self.planned_path_completion_rate = float(current_target_index)/len(self.planned_path)
            
        ## Update the current_target_index
        # Check if the current target index should be updated
        if current_target_index <= (len(self.planned_path) - 1):
            next_index = current_target_index + 1
            prev_index = current_target_index - 1
            
            # list of votes to move to the next index for each particle
            votes_for_next_index = []
            
            # for particle in self.custom_static_particles:
            for particle, path in self.planned_path_of_particles.items():
                current_target_point = np.array([self.planned_path_current_target_poses_of_particles[particle].position.x, 
                                                 self.planned_path_current_target_poses_of_particles[particle].position.y, 
                                                 self.planned_path_current_target_poses_of_particles[particle].position.z])

                prev_target_point = np.array([self.planned_path_of_particles[particle][prev_index].pose.position.x,
                                              self.planned_path_of_particles[particle][prev_index].pose.position.y,
                                              self.planned_path_of_particles[particle][prev_index].pose.position.z])
                
                current_position = np.array([self.particle_positions[particle].x, 
                                             self.particle_positions[particle].y, 
                                             self.particle_positions[particle].z])
                
                # Project current position on the line segment from current target to next target
                line_vector = current_target_point - prev_target_point
                point_vector = current_position - current_target_point

                line_length = np.linalg.norm(line_vector)
                if line_length > 0:
                    line_unit_vector = line_vector / line_length
                else:
                    line_unit_vector = np.zeros(3) # Avoid division by zero

                # Calculate the projection length    
                projection_length = np.dot(point_vector, line_unit_vector)
                
                # If projection length is greater than zero, vote to update the target index
                # projection_distance_buffer = 0.0
                # projection_distance_buffer = self.acceptable_pos_err_avr_norm
                projection_distance_buffer = 10.0*self.convergence_threshold_pos
                if not projection_length >= -projection_distance_buffer:
                    votes_for_next_index.append(False) # Vote not to move to the next index
                else:
                    votes_for_next_index.append(True) # Vote to move to the next index
                
            # Get all the votes, make sure it's not empty and check if all the particles agree to move to the next index
            if votes_for_next_index and all(votes_for_next_index):        
                current_target_index = next_index
                
        return current_target_index    
    
    def update_current_path_target_poses_and_velocities(self, current_target_index):
        """
        Updates the current target poses and velocities of the particles based on the current target index.
        """
        # update the current target pose
        if current_target_index < len(self.planned_path):
            self.planned_path_current_target_pose = self.planned_path[current_target_index].pose

        # update the current target poses of the particles
        for particle, path in self.planned_path_of_particles.items():
            if current_target_index < len(self.planned_path):
                self.planned_path_current_target_poses_of_particles[particle] = path[current_target_index].pose

        # Update the current target velocities of the particles based on the velocity profile
        for particle, velocity_profile in self.planned_path_velocity_profile_of_particles.items():
            self.planned_path_current_target_velocities_of_particles[particle] = velocity_profile[current_target_index-1]
                
    def calculate_path_tracking_velocity_profile_of_particles(self):
        """
        Calculates the velocity profile of the particles for path tracking.
        planned_path_velocity_profile_of_particles = {particle: velocity_profile as a list of 6D vectors}
        """
        planned_path_velocity_profile_of_particles = {}

        # For each particle, calculate segment durations
        # Assuming the number of segments are the same for each particle
        # Find the maximum segment durations of all paths
        max_segment_durations = None # to hold the maximum segment durations of the all paths as np.array of n elements
        for idx_particle, particle in enumerate(self.custom_static_particles):
            if particle not in self.planned_path_current_target_poses_of_particles:
                rospy.logwarn("Particle: " + str(particle) + " planned path is not obtained yet.")
                continue

            segment_durations_particle = self.calculate_segment_durations(self.planned_path_cumulative_lengths_of_particles[particle], 
                                                                          self.planned_path_cumulative_rotations_of_particles[particle], 
                                                                          self.max_linear_velocity*self.path_tracking_feedforward_linear_velocity_scale_factor, 
                                                                          self.max_angular_velocity*self.path_tracking_feedforward_angular_velocity_scale_factor)
            
            # print("Segment durations for particle: ", particle, " are: ", segment_durations_particle)
            
            if max_segment_durations is None:
                max_segment_durations = segment_durations_particle
            else:
                max_segment_durations = np.maximum(max_segment_durations, segment_durations_particle)
                
        # print("Max segment durations: ", max_segment_durations)

        # Calculate the velocity profile of the particles
        for idx_particle, particle in enumerate(self.custom_static_particles):
            if particle not in self.planned_path_current_target_poses_of_particles:
                continue

            segment_lengths = np.diff(self.planned_path_cumulative_lengths_of_particles[particle]) # np.array n elements
            # segment_rotations = np.diff(self.planned_path_cumulative_rotations_of_particles[particle]) # np.array of n elements
            segment_rotations = np.mod(np.diff(self.planned_path_cumulative_rotations_of_particles[particle]) + np.pi, 2 * np.pi) - np.pi # wrap to [-pi, pi]
            
            segment_directions = self.planned_path_direction_vectors_of_particles[particle]  # list of n elements with each element a unit 3D vector
            segment_rotation_vectors = self.planned_path_rotation_vectors_of_particles[particle]  # list of n elements with each element a unit 3D vector

            linear_speed_profile = segment_lengths / max_segment_durations  # np.array of n elements
            angular_speed_profile = segment_rotations / max_segment_durations  # np.array of n elements

            # Multiply the speed profiles with the direction vectors and rotation vectors
            # and concatenate them to obtain the velocity profile as a list of 6D vectors
            velocity_profile = [np.concatenate((speed * direction, angular_speed * rotation))
                                for speed, angular_speed, direction, rotation in zip(linear_speed_profile, 
                                                                                     angular_speed_profile, 
                                                                                     segment_directions, 
                                                                                     segment_rotation_vectors)]

            planned_path_velocity_profile_of_particles[particle] = velocity_profile

            # Add zero velocity at the end of the velocity profile
            planned_path_velocity_profile_of_particles[particle].append(np.zeros(6))

        return planned_path_velocity_profile_of_particles
    
    def calculate_segment_durations(self, cumulative_lenghts, cumulative_rotations, max_linear_velocity, max_angular_velocity):
        """
        Calculates the duration of each segment in the path based on the maximum linear and angular velocities.

        Args:
            cumulative_lenghts: A list of cumulative lengths of the segments in the path.
            cumulative_rotations: A list of cumulative rotations of the segments in the path.
            max_linear_velocity: The maximum linear velocity of the robot.
            max_angular_velocity: The maximum angular velocity of the robot.
        Returns:
            np.array required durations of the segments in the path.
        """
        segment_lengths = np.diff(cumulative_lenghts)
        # segment_rotations = np.diff(cumulative_rotations)
        segment_rotations = np.mod(np.diff(cumulative_rotations) + np.pi, 2 * np.pi) - np.pi # wrap to [-pi, pi]
        
        linear_times = segment_lengths / max_linear_velocity
        angular_times = segment_rotations / max_angular_velocity
        
        # print("cumulative_lenghts: ", cumulative_lenghts)   
        # print("cumulative_rotations: ", cumulative_rotations)
        
        # print("Linear times: ", linear_times)
        # print("Angular times: ", angular_times)

        return np.maximum(linear_times, angular_times)

    def publish_path(self, planned_path):
        """
        Publishes the planned path of the robot to the /path topic.
        """
        # Create a Path message
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        # Add the poses to the path
        for pose in planned_path:
            path.poses.append(pose)

        # Publish the path
        self.path_pub.publish(path)

    def publish_paths_of_particles(self, planned_path_of_particles):
        """
        Publishes the planned path of the particles to the /path_of_particles topic.
        """
        # Create a Path message
        path_of_particles = {}
        for particle, path in planned_path_of_particles.items():
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = rospy.Time.now()

            # Add the poses to the path
            for pose in path:
                path_msg.poses.append(pose)

            # Publish the path
            path_of_particles[particle] = path_msg

            self.path_pub_particles[particle].publish(path_msg)

    def publish_current_target_points(self):
        """
        Publishes the current target points on the path to the /current_target_points topic.
        """
        time_now = rospy.Time.now()
        frame = "map"

        # Create a PointStamped message for the current target point in the planned path of **centroid**
        current_target_point = PointStamped()
        current_target_point.header.stamp = time_now
        current_target_point.header.frame_id = frame
        current_target_point.point = self.planned_path_current_target_pose.position
        self.info_pub_planned_path_current_target_point.publish(current_target_point) # Publish the current target point

        # Publish the current target points of the **particles** 
        for particle, pose in self.planned_path_current_target_poses_of_particles.items():
            # Create a PointStamped message for the current target point of the particle
            current_target_point_of_particle = PointStamped()
            current_target_point_of_particle.header.frame_id = frame
            current_target_point_of_particle.header.stamp = time_now
            current_target_point_of_particle.point = pose.position
            self.info_pub_planned_path_current_target_point_particles[particle].publish(current_target_point_of_particle) # Publish the current target point of the particle

    ## ----------------------------------------------------------------------------------------
    
    ## ----------------------------------------------------------------------------------------
    ## -------------------------- EXPERIMENTS WITH PRE-SAVED PATHS ----------------------------
    def set_enable_experiments(self, request):
        self.experiments_enabler(request.data, cause="manual")
        return SetBoolResponse(True, 'Successfully set Experiments enabled state to {}'.format(request.data))
    
    def experiments_enabler(self, enable, cause="manual"):
        if enable:
            if self.path_planning_pre_saved_paths_enabled:
                self.experiments_manager.start_next_experiment()
                rospy.loginfo("Experiments enabled by " + cause + ".")
            else:
                rospy.logwarn("Experiments are not enabled. Enable the path_planning_pre_saved_paths_enabled parameter")
        else:
            self.controller_enabler(enable=False, cause="manual")
            rospy.loginfo("Experiments disabled by " + cause + ".")
                    
    ## ----------------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('velocity_controller_node', anonymous=False)

    node = VelocityControllerNode()

    rospy.spin()
    # node.input_thread.join()  # Ensure the input thread closes cleanly