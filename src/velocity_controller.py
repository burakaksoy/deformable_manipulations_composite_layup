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
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, Float32MultiArray

from fabric_simulator.msg import SegmentStateArray
from fabric_simulator.msg import MinDistanceDataArray

from deformable_manipulations_composite_layup.msg import ControllerStatus

from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse

import tf.transformations as tf_trans

import cvxpy as cp

from experiments_manager import ExperimentsManager

import threading

def run_in_thread(callback):
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=callback, args=args, kwargs=kwargs)
        thread.daemon = True
        thread.start()
    return wrapper

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
        self.paused = False  # Flag to pause/resume controller

        # To Store the time when the controller is enabled/disabled
        self.controller_enabled_time = 0.0
        self.controller_enabled_time_str = ""
        self.controller_disabled_time = 0.0
        # iteration counter for the controller to calculate the average performance
        self.controller_itr = 0 

        self.nominal_control_enabled = rospy.get_param("~nominal_control_enabled", True)
        self.obstacle_avoidance_enabled = rospy.get_param("~obstacle_avoidance_enabled", True)
        self.stress_avoidance_enabled = rospy.get_param("~stress_avoidance_enabled", True)

        # Create the service server for pause/resume the controller
        self.set_pause_controller_server = rospy.Service('~set_pause_controller', SetBool, self.set_pause_controller)
        # Create the service server for enable/disable the controller
        self.set_enable_controller_server = rospy.Service('~set_enable_controller', SetBool, self.set_enable_controller)

        # Create service servers for enabling/disabling each feature
        self.set_nominal_control_server = rospy.Service('~set_nominal_control_enabled', SetBool, self.set_nominal_control_enabled)
        self.set_obstacle_avoidance_server = rospy.Service('~set_obstacle_avoidance_enabled', SetBool, self.set_obstacle_avoidance_enabled)
        self.set_stress_avoidance_server = rospy.Service('~set_stress_avoidance_enabled', SetBool, self.set_stress_avoidance_enabled)
        
        # Create parameters for logging
        self.log_enabled = rospy.get_param("~log_enabled", True)
        self.rosbag_log_enabled = rospy.get_param("~rosbag_log_enabled", True)
        self.compressed_rosbag = rospy.get_param("~compressed_rosbag", False)
        # Get the saving directory for the executions log (rosbags, plans, etc.)
        self.executions_log_directory = rospy.get_param("~executions_log_directory", "")

        self.pub_rate_odom = rospy.get_param("~pub_rate_odom", 100)

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
        
        self.info_pub_overall_min_distance_collision = rospy.Publisher("~info_overall_min_distance_collision", Float32, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        
        self.info_pub_stress_avoidance_performance = rospy.Publisher("~info_stress_avoidance_performance", Float32, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        self.info_pub_stress_avoidance_performance_avr = rospy.Publisher("~info_stress_avoidance_performance_avr", Float32, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        
        self.info_target_pose_publisher = rospy.Publisher("~info_controller_marker_target_pose", MarkerArray, queue_size=1)
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

        # Controller gains  [x,y,z, Rx,Ry,Rz]
        self.kp = np.array(rospy.get_param("~kp", [1.0,1.0,1.0]))
        self.kd = np.array(rospy.get_param("~kd", [0.0,0.0,0.0]))

        self.k_low_pass_ft = rospy.get_param("~k_low_pass_ft", 0.9) # low pass filter coefficient for the ft values of the previous values
        self.k_low_pass_min_d = rospy.get_param("~k_low_pass_min_d", 0.5) # low pass filter coefficient for the minimum distance values of the previous values
        self.k_low_pass_convergence = rospy.get_param("~k_low_pass_convergence", 0.99) # low pass filter coefficient for the minimum distance values of the previous values

        self.max_linear_velocity = rospy.get_param("~max_linear_velocity", 0.1) # m/s
        
        self.max_linear_acceleration = rospy.get_param("~max_linear_acceleration", 0.5) # m/s^2

        self.pos_err_avr_norm = 0.0 # float('inf') # initialize average norm of the position errors
        self.pos_err_avr_norm_prev = 0.0 # float('inf') # initialize average norm of the previous position errors
            
        self.convergence_threshold_pos = float(rospy.get_param("~convergence_threshold_pos", 1e-6)) # m
                    
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
        # To do: Make it variable based on the robot kinematics and dynamics in the future.
        self.wrench_max = np.array(rospy.get_param("~wrench_max", [200.0, 200.0, 200.0]))

        # stress offset values for each axis [Fx,Fy,Fz]
        self.w_stress_offset = np.array(rospy.get_param("~w_stress_offset", [30.0, 30.0, 30.0])) 

        # alpha(h_ft) function robot stress coefficients for each axis [Fx,Fy,Fz]
        self.c1_alpha_ft = np.array(rospy.get_param("~c1_alpha_ft", [1.0, 1.0, 1.0]))
        self.c2_alpha_ft = np.array(rospy.get_param("~c2_alpha_ft", [1.0, 1.0, 1.0]))
        self.c3_alpha_ft = np.array(rospy.get_param("~c3_alpha_ft", [0.8, 0.8, 0.8]))

        # stress avoidance performance record variables
        self.stress_avoidance_performance_avr = 0.0
        self.stress_avoidance_performance_sum = 0.0
        self.stress_avoidance_performance_ever_zero = False
        
        ## ----------------------------------------------------------------------------------------
        ## SETUP FOR DEFORMABLE OBJECT STATE READINGS FROM SIMULATION PERTURBATIONS

        self.deformable_object_state_topic_name = rospy.get_param("/fabric_state_topic_name") # subscribed
        # this is also like prefix to the perturbed particles' new states

        # Dictionaries that will hold the state of the custom_static_particles
        self.particle_positions = {}
        self.particle_orientations = {}
        self.particle_twists = {}
        self.particle_wrenches = {}

        # self.current_full_state = None # To store the current full state of the deformable object
        
        experimental_run_subscribers_in_thread = False
        
        # Subscriber for deformable object states to figure out the current particle positions
        if experimental_run_subscribers_in_thread:
            self.sub_state = rospy.Subscriber(self.deformable_object_state_topic_name + "_minimal", SegmentStateArray, run_in_thread(self.state_array_callback), queue_size=10)
        else:
            self.sub_state = rospy.Subscriber(self.deformable_object_state_topic_name + "_minimal", SegmentStateArray, self.state_array_callback, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        
        # Subscribers to the particle states with perturbations      
        ## We create 3 subscribers for perturbed states of each custom static particle
        self.subs_state_dx = {}
        self.subs_state_dy = {}
        self.subs_state_dz = {}

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

        self.states_set_particles = [] # For bookkeeping of which custom static particles are obtained their all perturbed state readings at least once.

        ## Create the subscribers to states with perturbations
        for particle in self.custom_static_particles:
            # Prepare the topic names of that particle
            state_dx_topic_name    = self.deformable_object_state_topic_name + "_" + str(particle) + "_x" + "_minimal" 
            state_dy_topic_name    = self.deformable_object_state_topic_name + "_" + str(particle) + "_y" + "_minimal" 
            state_dz_topic_name    = self.deformable_object_state_topic_name + "_" + str(particle) + "_z" + "_minimal" 

            # Create the subscribers (also takes the particle argument)
            if experimental_run_subscribers_in_thread:
                self.subs_state_dx[particle] = rospy.Subscriber(state_dx_topic_name, SegmentStateArray, run_in_thread(self.state_array_dx_callback), particle, queue_size=10)
                rospy.sleep(0.1)
                self.subs_state_dy[particle] = rospy.Subscriber(state_dy_topic_name, SegmentStateArray, run_in_thread(self.state_array_dy_callback), particle, queue_size=10)
                rospy.sleep(0.1)
                self.subs_state_dz[particle] = rospy.Subscriber(state_dz_topic_name, SegmentStateArray, run_in_thread(self.state_array_dz_callback), particle, queue_size=10)
                rospy.sleep(0.1)
            else:
                self.subs_state_dx[particle]    = rospy.Subscriber(state_dx_topic_name, SegmentStateArray, self.state_array_dx_callback, particle, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
                self.subs_state_dy[particle]    = rospy.Subscriber(state_dy_topic_name, SegmentStateArray, self.state_array_dy_callback, particle, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
                self.subs_state_dz[particle]    = rospy.Subscriber(state_dz_topic_name, SegmentStateArray, self.state_array_dz_callback, particle, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        
        ## ----------------------------------------------------------------------------------------
        ## SETUP FOR MINIMUM DISTANCE READINGS FROM SIMULATION PERTURBATIONS
        self.min_distance_topic_name = rospy.get_param("/min_dist_to_rb_topic_name") # subscribed, 
        # this is also like prefix to the perturbed particles' new minimum distances

        # Subscriber to figure out the current deformable object minimum distances to the rigid bodies in the scene 
        if experimental_run_subscribers_in_thread:
            self.sub_min_distance = rospy.Subscriber(self.min_distance_topic_name, MinDistanceDataArray, run_in_thread(self.min_distances_array_callback), queue_size=10)
        else:
            self.sub_min_distance = rospy.Subscriber(self.min_distance_topic_name, MinDistanceDataArray, self.min_distances_array_callback, queue_size=10)
        rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up

        # Subscribers to the particle minimum distances with perturbations      
        ## We create 3 subscribers for perturbed states of each custom static particle
        self.subs_min_distance_dx = {}
        self.subs_min_distance_dy = {}
        self.subs_min_distance_dz = {}

        # Dictionaries to store minimum distances caused by the perturbation on the custom static particles
        self.min_distances = {}
        self.min_distances_dx = {}
        self.min_distances_dy = {}
        self.min_distances_dz = {}

        self.min_distances_set_particles = [] # For bookkeeping of which custom static particles are obtained their all perturbed min distance readings at least once.
        self.min_distances_set_particles_obstacles = [] 

        ## Create the subscribers to minimum distances with perturbations
        for particle in self.custom_static_particles:
            # Prepare the topic names of that particle
            min_distance_dx_topic_name    = self.min_distance_topic_name + "_" + str(particle) + "_x" 
            min_distance_dy_topic_name    = self.min_distance_topic_name + "_" + str(particle) + "_y" 
            min_distance_dz_topic_name    = self.min_distance_topic_name + "_" + str(particle) + "_z" 
        
            # Create the subscribers (also takes the particle argument)
            if experimental_run_subscribers_in_thread:
                self.subs_min_distance_dx[particle]    = rospy.Subscriber(min_distance_dx_topic_name, MinDistanceDataArray, run_in_thread(self.min_distance_array_dx_callback),    particle, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
                self.subs_min_distance_dy[particle]    = rospy.Subscriber(min_distance_dy_topic_name, MinDistanceDataArray, run_in_thread(self.min_distance_array_dy_callback),    particle, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
                self.subs_min_distance_dz[particle]    = rospy.Subscriber(min_distance_dz_topic_name, MinDistanceDataArray, run_in_thread(self.min_distance_array_dz_callback),    particle, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
            else:
                self.subs_min_distance_dx[particle]    = rospy.Subscriber(min_distance_dx_topic_name,    MinDistanceDataArray, self.min_distance_array_dx_callback,    particle, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
                self.subs_min_distance_dy[particle]    = rospy.Subscriber(min_distance_dy_topic_name,    MinDistanceDataArray, self.min_distance_array_dy_callback,    particle, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
                self.subs_min_distance_dz[particle]    = rospy.Subscriber(min_distance_dz_topic_name,    MinDistanceDataArray, self.min_distance_array_dz_callback,    particle, queue_size=10)
                rospy.sleep(0.1)  # Small delay to ensure publishers are fully set up
        ## ----------------------------------------------------------------------------------------
        
        self.experiments_manager = ExperimentsManager(self)
        
        ## ----------------------------------------------------------------------------------------
        self.target_poses_wrt_leader = {}
        
        self.target_poses = {} # Each item will be a Pose() msg class
        self.target_relative_poses = {} # Each item will be a Pose() msg class
        
        self.reset_target_poses_wrt_leader_server = rospy.Service('~reset_target_poses_wrt_leader', Empty, self.reset_target_poses_wrt_leader)
        
        # Read the followed odom topic names and create the subscribers to them
        self.followed_odom_topic_names = rospy.get_param("~followed_odom_topic_names", [])
        rospy.loginfo("followed_odom_topic_names: " + str(self.followed_odom_topic_names))
        
        self.followed_positions = {}
        self.followed_orientations = {}
        self.followed_twists = {}
        
        self.leader_odom_topic = ""
        
        self.followed_odom_subscribers = {}
        for i, topic_name in enumerate(self.followed_odom_topic_names):
            # Select the first topic as the leader
            if i == 0:
                self.leader_odom_topic = topic_name
            
            # Create the subscribers
            if experimental_run_subscribers_in_thread:
                self.followed_odom_subscribers[topic_name] = rospy.Subscriber(topic_name, Odometry, run_in_thread(self.followed_odom_subscriber_callback), topic_name, queue_size=10)
            else:
                self.followed_odom_subscribers[topic_name] = rospy.Subscriber(topic_name, Odometry, self.followed_odom_subscriber_callback, topic_name, queue_size=10)
            rospy.sleep(0.1)
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
            self.control_outputs[particle] = np.zeros(3) # initialization for the velocity command
            
        self.control_outputs_last = {}
        for particle in self.custom_static_particles:
            self.control_outputs_last[particle] = np.zeros(3)

        # Start the control
        self.calculate_control_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.calculate_control_outputs_timer_callback)
        self.odom_pub_timer          = rospy.Timer(rospy.Duration(1. / self.pub_rate_odom), self.odom_pub_timer_callback)
        ## ----------------------------------------------------------------------------------------
        
        self.initialized = True
        rospy.loginfo("Velocity controller is initialized.")








    def set_pause_controller(self, request):
        self.paused = request.data
        rospy.loginfo("Pause state set to {}".format(request.data))
        return SetBoolResponse(True, 'Successfully set pause state to {}'.format(request.data))
    
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
            
            self.pos_err_avr_norm = 0.0 # float('inf') # initialize average norm of the position errors
            self.pos_err_avr_norm_prev = 0.0 # float('inf') # initialize average norm of the previous position errors
            
            if cause == "manual" and self.log_enabled and self.rosbag_log_enabled:
                self.experiments_manager.start_rosbag_recording_manual(self.controller_enabled_time_str, 
                                                                        self.executions_log_directory,
                                                                        compress=self.compressed_rosbag)
            
            self.enabled = True
            rospy.loginfo("-------------- Controller is enabled --------------")
        else:
            self.controller_disabled_time = rospy.Time.now()
            # Calculate the duration when the controller is disabled
            dur = (self.controller_disabled_time - self.controller_enabled_time).to_sec() # seconds
            status_msg.total_duration = dur  # seconds
            status_msg.rate = self.controller_itr / dur # rate of the controller iterations
            
            status_msg.stress_avoidance_performance_avr = self.stress_avoidance_performance_avr
            status_msg.min_distance = self.overall_min_distance_collision

            # ----------------------------------------------------------------------------------------
            ## Print the performance metrics suitable for a csv file
            rospy.loginfo("-------------- Controller is Disabled --------------")
            rospy.loginfo("Performance metrics (CSV suitable):")
            rospy.loginfo("Titles: ft_on, collision_on, success, min_distance, rate, duration, stress_avoidance_performance_avr, stress_avoidance_performance_ever_zero, start_time, final_task_error")

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
            final_task_error_str = "0.0"

            execution_results = [ft_on_str, collision_on_str, success_str, 
                                min_distance_str, rate_str, duration_str, 
                                stress_avoidance_performance_avr_str, 
                                stress_avoidance_performance_ever_zero_str, 
                                start_time_str,final_task_error_str]
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
            if self.log_enabled:
                if self.rosbag_log_enabled:    
                    # Save the execution results to a csv file
                    self.experiments_manager.stop_rosbag_recording()
            
                # Save results to csv file
                self.experiments_manager.save_experiment_results_manual(execution_results, self.controller_enabled_time_str, self.executions_log_directory)

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
        
    def state_array_callback(self, states_msg):
        if self.initialized:
            # self.current_full_state = states_msg
            
            # for particle in self.custom_static_particles:
            #     self.particle_positions[particle] = states_msg.states[particle].pose.position
            #     self.particle_orientations[particle] = states_msg.states[particle].pose.orientation
            #     self.particle_twists[particle] = states_msg.states[particle].twist

            #     wrench_array = self.wrench_to_numpy(states_msg.states[particle].wrench)
            #     if particle not in self.particle_wrenches:
            #         self.particle_wrenches[particle] = wrench_array
            #     else:
            #         # Apply low-pass filter to the force and torque values
            #         self.particle_wrenches[particle] = self.k_low_pass_ft*self.particle_wrenches[particle] + (1 - self.k_low_pass_ft)*wrench_array
            
            # Turn the list of states into a dict keyed by the ID
            # e.g. { 0: SegmentState, 7: SegmentState, 80: SegmentState, ... }
            states_dict = {s.id: s for s in states_msg.states}

            # Now iterate over your custom_static_particles
            for particle in self.custom_static_particles:
                # Only update if this particle actually exists in the message
                if particle in states_dict:
                    seg_state = states_dict[particle]

                    # Update your dictionaries
                    self.particle_positions[particle] = seg_state.pose.position
                    self.particle_orientations[particle] = seg_state.pose.orientation
                    self.particle_twists[particle] = seg_state.twist

                    wrench_array = self.wrench_to_numpy(seg_state.wrench)
                    if particle not in self.particle_wrenches:
                        self.particle_wrenches[particle] = wrench_array
                    else:
                        # Apply low-pass filter
                        self.particle_wrenches[particle] = (
                            self.k_low_pass_ft * self.particle_wrenches[particle]
                            + (1 - self.k_low_pass_ft) * wrench_array
                        )

    def state_array_dx_callback(self, states_msg, perturbed_particle):
        if self.initialized:
            self.update_state_arrays(states_msg, perturbed_particle, self.particle_positions_dx, self.particle_orientations_dx, self.particle_twists_dx, self.particle_wrenches_dx)

    def state_array_dy_callback(self, states_msg, perturbed_particle):
        if self.initialized:
            self.update_state_arrays(states_msg, perturbed_particle, self.particle_positions_dy, self.particle_orientations_dy, self.particle_twists_dy, self.particle_wrenches_dy)

    def state_array_dz_callback(self, states_msg, perturbed_particle):
        if self.initialized:
            self.update_state_arrays(states_msg, perturbed_particle, self.particle_positions_dz, self.particle_orientations_dz, self.particle_twists_dz, self.particle_wrenches_dz)

    def update_state_arrays(self, states_msg, perturbed_particle, positions_dict, orientations_dict, twists_dict, wrenches_dict):
        if not (perturbed_particle in positions_dict):
            positions_dict[perturbed_particle] = {}
            orientations_dict[perturbed_particle] = {}
            twists_dict[perturbed_particle] = {}
            wrenches_dict[perturbed_particle] = {}

        # for particle in self.custom_static_particles:
        #     positions_dict[perturbed_particle][particle] = states_msg.states[particle].pose.position
        #     orientations_dict[perturbed_particle][particle] = states_msg.states[particle].pose.orientation
        #     twists_dict[perturbed_particle][particle] = states_msg.states[particle].twist

        #     wrench_array = self.wrench_to_numpy(states_msg.states[particle].wrench)
        #     if particle not in wrenches_dict[perturbed_particle]:
        #         wrenches_dict[perturbed_particle][particle] = wrench_array
        #     else:
        #         # Apply low-pass filter to the force and torque values
        #         wrenches_dict[perturbed_particle][particle] = self.k_low_pass_ft*wrenches_dict[perturbed_particle][particle] + (1 - self.k_low_pass_ft)*wrench_array
                
        # Build a dictionary from segment ID -> SegmentState
        states_dict = {s.id: s for s in states_msg.states}

        for particle in self.custom_static_particles:
            if particle in states_dict:
                seg_state = states_dict[particle]
                
                positions_dict[perturbed_particle][particle] = seg_state.pose.position
                orientations_dict[perturbed_particle][particle] = seg_state.pose.orientation
                twists_dict[perturbed_particle][particle] = seg_state.twist

                wrench_array = self.wrench_to_numpy(seg_state.wrench)
                if particle not in wrenches_dict[perturbed_particle]:
                    wrenches_dict[perturbed_particle][particle] = wrench_array
                else:
                    # Apply low-pass filter
                    wrenches_dict[perturbed_particle][particle] = (
                        self.k_low_pass_ft * wrenches_dict[perturbed_particle][particle]
                        + (1 - self.k_low_pass_ft) * wrench_array
                    )

    def is_perturbed_states_set_for_particle(self,particle):
        """
        Checks if the perturbed state parameters (dx, dy, dz) are set for a specified particle.
        This function determines if the custom static particle has obtained perturbed states along the x, y, z axes at least once.
        If these states are set, the particle is added to the 'states_set_particles' list,
        and the function returns True if the 'self.particle_positions' of that particle is not None. Otherwise, it returns False.
        """
        if particle in self.states_set_particles:
            return (particle in self.particle_positions)
        else:
            check = ((particle in self.particle_positions_dx) and  
                     (particle in self.particle_positions_dy) and 
                     (particle in self.particle_positions_dz))
            if check:
                self.states_set_particles.append(particle)
                return (particle in self.particle_positions)
            else:
                return False
    
    def calculate_target_poses(self):
        if self.leader_odom_topic in self.followed_positions:
            
            leader_position = self.followed_positions[self.leader_odom_topic]
            
            while True:  # Loop until all target poses are set
                reset_occurred = False  # Flag to track if reset happens

                for particle in self.custom_static_particles:
                    if particle in self.target_poses_wrt_leader:
                        relative_position = self.target_poses_wrt_leader[particle].position

                        self.target_poses[particle] = Pose()
                        self.target_poses[particle].position.x = relative_position.x + leader_position.x
                        self.target_poses[particle].position.y = relative_position.y + leader_position.y
                        self.target_poses[particle].position.z = relative_position.z + leader_position.z    
                    else:
                        rospy.logwarn("Particle: " + str(particle) + " target pose wrt leader is not set yet. Trying to set it.")
                        self.reset_target_poses_wrt_leader(None)
                        reset_occurred = True  # Mark that a reset happened
                        break  # Exit the for-loop to restart

                if not reset_occurred:
                    break  # Exit the while-loop if no reset occurred

            # After updating all target poses, publish the target poses as markers
            self.publish_arrows_marker_array()
        else:
            rospy.logwarn("Leader position is not available yet.")
    
    def calculate_pose_errors(self):
        err = np.zeros(3*len(self.custom_static_particles))
        avr_norm_pos_err = 0.0
        
        self.calculate_target_poses() # Updates self.target_poses

        for idx_particle, particle in enumerate(self.custom_static_particles):
            # Do not proceed until the initial values have been set
            if not (particle in self.particle_positions):
                rospy.logwarn("particle: " + str(particle) + " state is not obtained yet.")
                continue
            
            if not (particle in self.target_poses):
                rospy.logwarn("particle: " + str(particle) + " target pose is not set yet.")
                continue

            current_pose = Pose()
            current_pose.position = self.particle_positions[particle]
            # current_pose.orientation = self.particle_orientations[particle]

            target_pose = self.target_poses[particle]

            err[(3*idx_particle) : (3*(idx_particle+1))] = self.calculate_pose_target_error(current_pose,target_pose)

            avr_norm_pos_err += np.linalg.norm(err[(3*idx_particle) : (3*(idx_particle+1))][0:3])/len(self.custom_static_particles)

        return err, avr_norm_pos_err
    
    
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
                     (particle in self.min_distances_dz))
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
                     (key in self.min_distances_dz[particle]))
            if check:
                self.min_distances_set_particles_obstacles.append((particle, key))
                return True
            else:
                return False

    def calculate_jacobian_obstacle_min_distance(self, key):
        """
        Calculates the Jacobian matrix that defines the relation btw. 
        the robot hold points (custom_static_particles) 3DoF poses and
        the minimum distance to the obstacles. 
        The result is a row matrix(vector) (e.g. dimension 1x6 for two holding point poses)
        """
        J = np.zeros((1,3*len(self.custom_static_particles)))

        for idx_particle, particle in enumerate(self.custom_static_particles):
            # Do not proceed if the minimum distance is not set for the obstacle
            if not self.is_perturbed_min_distances_set_for_obstacle_id(particle, key):
                rospy.logwarn_throttle(1,"[calculate_jacobian_obstacle_min_distance func.] particle: "+str(particle)+", obstacle index: "+str(key)\
                              +", min distances are not published yet or it does not have for perturbed states")
                continue

            # dx direction
            J[0, 3*idx_particle+0] = (self.min_distances_dx[particle][key] - self.min_distances[key])/self.delta_x if self.delta_x != 0.0 else 0.0
            # dy direction
            J[0, 3*idx_particle+1] = (self.min_distances_dy[particle][key] - self.min_distances[key])/self.delta_y if self.delta_y != 0.0 else 0.0
            # dz direction
            J[0, 3*idx_particle+2] = (self.min_distances_dz[particle][key] - self.min_distances[key])/self.delta_z if self.delta_z != 0.0 else 0.0

        # Replace non-finite elements with 0
        J[~np.isfinite(J)] = 0

        return J

    def calculate_jacobian_ft(self):
        """
        Calculates the Jacobian matrix that defines the relation btw.
        the robot hold points (custom_static_particles) 6DoF poses and
        the forces and torques applied by the robots.
        The result is a 6x6 matrix for 2 holding point poses.
        """
        J = np.zeros((3*len(self.custom_static_particles),3*len(self.custom_static_particles)))

        for idx_particle1, particle1 in enumerate(self.custom_static_particles):
            for idx_particle, particle in enumerate(self.custom_static_particles):
                # Do not proceed until the initial values have been set
                if ((not self.is_perturbed_states_set_for_particle(particle))):
                    rospy.logwarn_throttle(1,"[calculate_jacobian_ft func.] particle: " + str(particle) + " state is not published yet or it does not have for perturbed states.")
                    continue

                # calculate the wrench differences:
                # btw. the current holding point wrench and the perturbated holding point wrench caused by the perturbation of custom_static_particle pose in each direction

                # Note that the forces and torques are negated to calculate the wrenches needed to be applied by the robots.
                current_wrench = self.particle_wrenches[particle1][:3]
                
                # dx direction
                perturbed_wrench = self.particle_wrenches_dx[particle][particle1][:3]
                J[ (3*idx_particle1) : (3*(idx_particle1+1)) , 3*idx_particle+0 ] = (perturbed_wrench-current_wrench)/self.delta_x
                # dy direction
                perturbed_wrench = self.particle_wrenches_dy[particle][particle1][:3]
                J[ (3*idx_particle1) : (3*(idx_particle1+1)) , 3*idx_particle+1 ] = (perturbed_wrench-current_wrench)/self.delta_y
                # dz direction
                perturbed_wrench = self.particle_wrenches_dz[particle][particle1][:3]
                J[ (3*idx_particle1) : (3*(idx_particle1+1)) , 3*idx_particle+2 ] = (perturbed_wrench-current_wrench)/self.delta_z

        return J

    def calculate_safe_control_output(self, nominal_u):
        ## ---------------------------------------------------
        ## Define optimization variables
        u = cp.Variable(3*len(self.custom_static_particles))

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
                J_obs_min_dist = self.calculate_jacobian_obstacle_min_distance(key) # 1x6

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
        h_ft = np.zeros(3*len(self.custom_static_particles)) # Control Barrier Function (CBF) for the forces
        h_ft_normalized = np.zeros(3*len(self.custom_static_particles)) # CBF normalized values for performance monitoring
        alpha_h_ft = np.zeros(3*len(self.custom_static_particles)) # alpha for the forces 
        sign_ft = np.zeros(3*len(self.custom_static_particles)) # sign for the forces 
        ft = np.zeros(3*len(self.custom_static_particles)) # forces

        # Calculate the stress avoidance constraints for each custom static particle
        for idx_particle, particle in enumerate(self.custom_static_particles):
            h_ft[3*idx_particle:3*(idx_particle+1)] = (self.wrench_max - self.w_stress_offset)**2 - (self.particle_wrenches[particle][:3])**2 # h_ft = (wrench_max - wrench_offset)^2 - (wrench)^2 (NEW)
            h_ft_normalized[3*idx_particle:3*(idx_particle+1)] = (self.wrench_max - np.abs(self.particle_wrenches[particle][:3]))/self.wrench_max
            alpha_h_ft[3*idx_particle:3*(idx_particle+1)] = self.alpha_robot_stress(h_ft[3*idx_particle:3*(idx_particle+1)])
            sign_ft[3*idx_particle:3*(idx_particle+1)] = np.sign(self.particle_wrenches[particle][:3])
            ft[3*idx_particle:3*(idx_particle+1)] = self.particle_wrenches[particle][:3]

        # Calculate stress avoidance performance monitoring values
        stress_avoidance_performance = self.calculate_and_publish_stress_avoidance_performance(h_ft_normalized)

        if stress_avoidance_performance <= 0.0:
                # Stop the controller if the stress avoidance performance is zero
                # self.controller_enabler(enable=False, cause="Stress avoidance hit zero.")
                rospy.logwarn("Stress avoidance hit zero.")
                # return None

        if self.stress_avoidance_enabled:
            # Calculate the forces and torques Jacobian
            J_ft = self.calculate_jacobian_ft() # 6x6
            
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

        # With using the different limits to linear velocities
        u_linear_max = self.max_linear_velocity*6.0 # 0.3 # 0.1

        # The following slices select every first 3 elements of each group of 3 in the array u.
        linear_indices = np.concatenate([np.arange(i, i+3) for i in range(0, 3*len(self.custom_static_particles), 3)])
        # Apply constraint to linear components
        constraints += [cp.norm(u[linear_indices],'inf') <= u_linear_max]

        # ## ---------------------------------------------------
        
        
        # ## ---------------------------------------------------
        # ## Limit velocity and accelerations at once
        
        # # Using the different limits to linear and angular velocities
        # u_lin_max = self.max_linear_velocity*1.0 # 0.3 # 0.1
        
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
        # # self.control_outputs_last[particle] = np.array([v_x, v_y, v_z])

        # for i, particle in enumerate(self.custom_static_particles):            
        #     lin_indices = np.arange(3*i, 3*i+3)  # Linear velocity indices
            
        #     v_prev = self.control_outputs_last[particle][0:3]  # Previous linear velocity
            
        #     # Optimization variables
        #     u_lin = u[lin_indices]
            
        #     # Precompute useful quantities for the current particle
        #     v_min = np.minimum(u_lin_max, a_lin_max_dt + v_prev)
            
        #     v_neg_min = np.minimum(u_lin_max, a_lin_max_dt - v_prev)
            
        #     # Create masks for each case
        #     mask_lin_case1 = np.abs(v_prev) <= a_lin_max_dt  # |v_prev| <= a_max * dt
        #     mask_lin_case2 = v_prev > a_lin_max_dt          # v_prev >= a_max * dt
        #     mask_lin_case3 = -v_prev >= a_lin_max_dt         # -v_prev >= a_max * dt
            
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
                        
        # ## ---------------------------------------------------
        
        ## ---------------------------------------------------
        # Define the problem
        
        ## Define weights for control inputs
        weights = np.ones(3*len(self.custom_static_particles))
        # Assign less weight on z axis positional motions (i.e make them more dynamic)
        
        ## Note that x axis position is every 1st element of each 3 element sets in the weight vector
        # weights[0::3] = 0.5 
        ## Note that y axis position is every 2nd element of each 3 element sets in the weight vector
        # weights[1::3] = 0.5 
        ## Note that z axis position is every 3rd element of each 3 element sets in the weight vector
        weights[2::3] = 0.1 
        # weights[2::3] = self.calculate_cost_weight_z_pos(stress_avoidance_performance, overall_min_distance-self.d_obstacle_offset)
        
        # Slow down the nominal control output when close to the obstacles and stress limits
        # nominal_u = self.calculate_weight_nominal_input(stress_avoidance_performance, overall_min_distance-self.d_obstacle_offset) * nominal_u

        # Define cost function with weights
        cost = cp.sum_squares(cp.multiply(weights, u - nominal_u)) / 2.0
        
        # ## ---------------------------------------------------
        # # Add cost of change in control input (acceleration) as a COST FUNCTION (not a constraint)
        
        # # Define the maximum allowable accelerations for linear and angular components
        # a_linear_max = self.max_linear_acceleration * 1.0
        
        # # Define the weights for the acceleration cost
        # w_lin_acc = 0.0001 # 1.0 # 0.01

        # # Calculate the time step based on the average control calculation rate
        # # Calculate the duration since the controller is enabled
        # dur = (rospy.Time.now() - self.controller_enabled_time).to_sec() # seconds
        # # Calculate the average control calculation rate        
        # dt = dur/self.controller_itr  # Time step size
        # # print("dt: ", dt)
        
        # for i, particle in enumerate(self.custom_static_particles):            
        #     linear_indices = np.arange(3*i, 3*i+3)
            
        #     delta_u_linear = (u[linear_indices] - self.control_outputs_last[particle][:3])/dt
            
        #     cost += (w_lin_acc/a_linear_max**2) *cp.sum_squares(delta_u_linear)/2.0
        
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
        #         a_lin = (u.value[3*idx_particle:3*idx_particle+3] - self.control_outputs_last[particle][:3] )/dt
                
        #         # If any of the linear acceleration components is above the maximum allowable value
        #         if np.any(np.abs(a_lin) > a_linear_max):
        #             rospy.logerr("a_lin is above thresholds: " +  str(a_lin))
            
        
        # Return optimal u
        return u.value

    def update_last_control_output_is_valid_time(self):
        # Take a note of the time when the control output is calculated
        self.time_last_control_output_is_valid = rospy.Time.now()
                                
    def assign_control_outputs(self, control_output):
        for idx_particle, particle in enumerate(self.custom_static_particles):    
            self.control_outputs[particle] = control_output[3*idx_particle:3*(idx_particle+1)]
            # print("Particle " + str(particle) + " u: " + str(self.control_outputs[particle]))
    
    def calculate_nominal_control_output(self, err_particles):
        # Initiate the nominal control output to zero
        control_output = np.zeros(3*len(self.custom_static_particles))

        # Calculate the nominal control outputs
        if self.nominal_control_enabled:
            # Calculate the nominal control output
            control_output = err_particles # (6,)
            # Apply the proportinal gains
            for idx_particle, particle in enumerate(self.custom_static_particles):
                # Get nominal control output of that particle
                control_output[3*idx_particle:3*(idx_particle+1)] = self.kp * control_output[3*idx_particle:3*(idx_particle+1)] # nominal

        return control_output
    
    def calculate_control_outputs_timer_callback(self,event):
        """
        Calculates the control outputs (self.control_outputs) for the custom static particles.
        """
        if self.initialized:                
            # Only calculate if enabled
            if self.enabled:
                
                # Increment the controller iteration
                self.controller_itr += 1
                
                # ----------------------------------------------------------------------------------------------------
                ## Calculate the controlled particle pose errors and update the error norms
                
                # Update the previous error norms
                self.pos_err_avr_norm_prev = self.pos_err_avr_norm
                
                # Calculate the current particle errors
                (err_particles, pos_err_avr_norm) = self.calculate_pose_errors() # (6,), scalar, scalar
                
                # pretty_print_array(err_particles)
                # print("err_particles: " + str(err_particles))
                # print("---------------------------")
                
                # Apply low-pass filter to the error norms
                self.pos_err_avr_norm = self.k_low_pass_convergence*self.pos_err_avr_norm_prev + (1-self.k_low_pass_convergence)*pos_err_avr_norm
                
                # publish error norms for information
                self.info_pub_target_pos_error_avr_norm.publish(Float32(data=pos_err_avr_norm))
                # ----------------------------------------------------------------------------------------------------
                
                # Calculate nominal control output
                control_output = self.calculate_nominal_control_output(err_particles) # (6,)
                        
                # print("control_output: " + str(control_output))
                # print("---------------------------")
                
                # ----------------------------------------------------------------------------------------------------  
                # Proceed with the safe control output calculations
                
                # init_t = time.time()                
                
                # Calculate safe control output with obstacle avoidance        
                control_output = self.calculate_safe_control_output(control_output) # safe # (6,)
                # rospy.logwarn("QP solver calculation time: " + str(1000*(time.time() - init_t)) + " ms.")
                
                if control_output is not None: # Successfully calculated the safe control output
                    self.update_last_control_output_is_valid_time()
                    self.assign_control_outputs(control_output)
                    
                    # print("self.control_outputs: " + str(self.control_outputs))
                    # print("---------------------------")
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
                    control_output = np.zeros(3*len(self.custom_static_particles))
                    self.assign_control_outputs(control_output)
                    return
                # ----------------------------------------------------------------------------------------------------  
            

    def odom_pub_timer_callback(self,event):
        """
        Integrates the self.control_outputs with time and publishes the resulting poses as Odometry messages.
        """
        if self.initialized:
            # Only publish if enabled
            if self.enabled:                
                if self.paused:                    
                    # Publish zero velocities for all particles
                    self.odom_publishers_publish_zero_velocities()                    
                    return
                
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

                        # Assign the new orientation to the Odometry message
                        odom.pose.pose.orientation = self.particle_orientations[particle]

                        # Assign the linear and angular velocities to the Odometry message
                        odom.twist.twist.linear.x = control_outputs_linear[0]
                        odom.twist.twist.linear.y = control_outputs_linear[1]
                        odom.twist.twist.linear.z = control_outputs_linear[2]
                        odom.twist.twist.angular.x = 0.0
                        odom.twist.twist.angular.y = 0.0
                        odom.twist.twist.angular.z = 0.0

                        # Update the pose of the particle 
                        # self.particle_positions[particle] = odom.pose.pose.position
                        # self.particle_orientations[particle] = odom.pose.pose.orientation

                        # Save the applied control output as the last control output
                        self.control_outputs_last[particle][:3] = control_outputs_linear

                        # Publish
                        if not self.real_robot_mode_enabled:
                            self.odom_publishers[particle].publish(odom)
                        else:
                            self.twist_publishers[particle].publish(odom.twist.twist)
                    else:
                        self.control_outputs[particle] = np.zeros(3)

    def odom_publishers_publish_zero_velocities(self):
        """
        This is a helper function that can be used to publish zero velocities for all particles.
        Publishes zero velocities for all particles.
        Useful for stopping the particles when the controller is disabled.
        """
        for particle in self.custom_static_particles:
            # Set the control outputs to zero
            self.control_outputs[particle] = np.zeros(3)

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
        
    def calculate_pose_target_error(self, current_pose, target_pose):
        """ 
        Calculates the pose error between the current pose and the target pose.
        The input poses are given as ROS pose msgs.
        """
        # Position error
        err_x = target_pose.position.x - current_pose.position.x
        err_y = target_pose.position.y - current_pose.position.y
        err_z = target_pose.position.z - current_pose.position.z

        # Combine position difference and rotation vector into a 6x1 vector
        return np.array([err_x, err_y, err_z])
    
    def get_system_timestamp(self):
        # Get the current system time
        now = datetime.now()
        # Format the datetime object into a string similar to rosbag filenames
        timestamp = now.strftime("%Y-%m-%d-%H-%M-%S")
        return timestamp

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
    
    def alpha_robot_stress(self, h):
        """
        Calculates the value of extended_class_K function \alpha(h) for ROBOT STRESS.
        Returns 3x1 alpha_h vector.
        Note that h is 3x1 vector and adjusted to be less than or equal to the wrench_max (h_ft = (wrench_max - wrench_offset)^2 - (wrench)^2).
        (self.wrench_max - self.w_stress_offset)**2 - (self.particle_wrenches[particle])**2 
        
        Piecewise Linear function is used when h is less than 0,
        when h is greater or equal to 0 a nonlinear function is used.
        
        See: https://www.desmos.com/calculator/hc6lc7nzkk for the function visualizations
        NEW: https://www.desmos.com/calculator/mwiakn4s5j (FORCE)
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
    
    ## ----------------------------------------------------------------------------------------
    def followed_odom_subscriber_callback(self, odom ,topic_name):
        if self.initialized:
            self.followed_positions[topic_name] = odom.pose.pose.position
            self.followed_orientations[topic_name] = odom.pose.pose.orientation
            self.followed_twists[topic_name] = odom.twist.twist
                
    def publish_arrows_marker_array(self):
        marker_array = MarkerArray()
        now = rospy.Time.now()

        for i, (particle, target_pose) in enumerate(self.target_poses.items()):
            # Create a marker for the arrow
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now
            marker.ns = "target_arrows"
            
            # Use a unique ID for each particle so RViz knows they are different markers
            marker.id = i
            
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Set the scale of the arrow
            marker.scale.x = 0.015  # Shaft diameter
            marker.scale.y = 0.05   # Head diameter
            marker.scale.z = 0.3    # Head length
            
            # Set the color (you can differentiate color if you want)
            marker.color.a = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            # Pose not used in ARROW mode except for orientation,
            # but we need to set orientation to a valid quaternion
            marker.pose.orientation.w = 1.0

            # Suppose you have a `leader_position` for each arrow.
            # Here I'm assuming you store that in a structure or compute it on the fly.
            leader_pos = self.followed_positions[self.leader_odom_topic]

            start_point = Point()
            start_point.x = leader_pos.x
            start_point.y = leader_pos.y
            start_point.z = leader_pos.z

            end_point = Point()
            end_point.x = target_pose.position.x
            end_point.y = target_pose.position.y
            end_point.z = target_pose.position.z

            marker.points = [start_point, end_point]
            
            marker_array.markers.append(marker)
        
        # Publish once for all markers
        self.info_target_pose_publisher.publish(marker_array)
        
    def reset_target_poses_wrt_leader(self, request):
        if self.leader_odom_topic in self.followed_positions:
            leader_position = self.followed_positions[self.leader_odom_topic]
            # leader_orientation = self.followed_orientations[self.leader_odom_topic]
        
            for particle in self.custom_static_particles:
                particle_position = self.particle_positions[particle]
                # particle_orientation = self.particle_orientations[particle]
                
                self.target_poses_wrt_leader[particle] = Pose()
                self.target_poses_wrt_leader[particle].position.x = particle_position.x - leader_position.x
                self.target_poses_wrt_leader[particle].position.y = particle_position.y - leader_position.y
                self.target_poses_wrt_leader[particle].position.z = particle_position.z - leader_position.z
        else:
            rospy.logwarn("Leader position is not available yet.")
                
        return EmptyResponse()
    ## ----------------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('velocity_controller_node', anonymous=False)

    node = VelocityControllerNode()

    rospy.spin()
    # node.input_thread.join()  # Ensure the input thread closes cleanly