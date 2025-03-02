import os
import re
import traceback
import numpy as np
import time
import sys
import csv
from pathlib import Path

try:
    import cPickle as pickle
except ModuleNotFoundError:
    import pickle

import rospy
import rosbag 
from .rosbag_controlled_recording import RosbagControlledRecorder

from geometry_msgs.msg import Twist, Point, PointStamped, Quaternion, Pose, PoseStamped, Wrench, Vector3
from nav_msgs.msg import Odometry, Path

class ExperimentsManager(object):
    def __init__(self, velocity_controller_node):
        
        self.velocity_controller_node = velocity_controller_node
        self.experiment_number = None
        
        self.is_experiments_completed = False
        
        self.rosbag_file = None # The rosbag file name with full path
        self.odom_topics = None # List of odom topics to record and reverse the executed path
        self.rosbag_recorder = None # RosbagControlledRecorder object, None if not recording
                                        
    def start_rosbag_recording_manual(self, experiment_number, saving_dir, compress=False):
        rospy.loginfo("Starting rosbag recording for manual execution")
        
        if self.rosbag_recorder is not None:
            rospy.logwarn("Rosbag recorder is already running. Stopping the current recording and starting a new one.")
            self.stop_rosbag_recording()
        
        # Create the rosbag file name         
        file_name = f"{experiment_number}_experiment_execution.bag"  # e.g. "1_experiment_execution.bag"
        
        saving_dir = os.path.expanduser(saving_dir)
        
        if not os.path.exists(saving_dir):
            os.makedirs(saving_dir)
            print(f"Directory '{saving_dir}' created.")
        
        self.rosbag_file = os.path.expanduser(os.path.join(saving_dir, file_name))
        
        # Topics to record
        topics = ["/fabric_markers", "/fabric_state", "/min_dist_markers", 
                    "/min_dist_to_rigid_bodies", "/rigid_body_markers", 
                    "/spacenav/twist", "/rosout", "/rosout_agg"]
        
        # All velocity controller specific topics
        vel_controller_topics = ["-e \"/composite_layup_velocity_controller.*\""]
        topics.extend(vel_controller_topics)
        
        # All odom particles topics
        self.odom_topics = [f"/odom_particle_{i}" for i in self.velocity_controller_node.custom_static_particles]
        # e.g. ["/odom_particle_1", "/odom_particle_2"]
        topics.extend(self.odom_topics)
        
        # Real robot topics
        real_robot_topics = ["/oarbot_blue/cmd_vel_world", "/oarbot_silver/cmd_vel_world", "/e_stop",
                             "/nuc/rgb/camera_info", "/nuc/rgb/image_rect_color/compressed", 
                             "/tf", "/tf_static",
                             "/oarbot_silver/constraint_marker", "/oarbot_blue/constraint_marker",
                             "/d1/viz_base_obs_dist_hard_thres","/d2/viz_base_obs_dist_hard_thres",
                             "/d1/viz_base_obs_dist_thres", "/d2/viz_base_obs_dist_thres",
                             "/d1/viz_robots_d1_tf_base_link", "/d2/viz_robots_d2_tf_base_link",
                             "/d1/viz_workspace_polygon", "/d2/viz_workspace_polygon"]
        
        topics.extend(real_robot_topics)
        
        # Convert to a single string with spaces
        topics_str = " ".join(topics)
        
        # Compress flag
        compress_str = "--bz2" if compress else ""
        
        # Create the rosbag command
        rosbag_command = f"rosbag record --output-name={self.rosbag_file} {compress_str} {topics_str}"
        
        # ROSbag recorder object
        self.rosbag_recorder = RosbagControlledRecorder(rosbag_command)
        
        # Start recording
        self.rosbag_recorder.start_recording_srv()
        
        # Wait for a few seconds to make sure the recording has started
        time_to_wait = 2
        rospy.loginfo(f"Waiting for {time_to_wait} seconds for the rosbag recording to start completely..")
        time.sleep(time_to_wait)
        return
        
    def stop_rosbag_recording(self):
        rospy.loginfo("Stopping rosbag recording")
        
        if self.rosbag_recorder is not None:
            self.rosbag_recorder.stop_recording_srv()
            self.rosbag_recorder = None
        else:
            rospy.logwarn("Rosbag recorder is not running. Nothing to stop.")
        return
        
    def save_experiment_results_manual(self,execution_results, experiment_id, saving_dir):
        rospy.loginfo("Saving experiment results")
        
        # Create the results csv file name
        file_name = f"experiment_execution_results.csv"
        
        saving_dir = os.path.expanduser(saving_dir)
        
        if not os.path.exists(saving_dir):
            os.makedirs(saving_dir)
            print(f"Directory '{saving_dir}' created.")
        
        self.csv_file = os.path.join(saving_dir, file_name)
        
        row_title = ["experiment_id", "ft_on", "collision_on", "success", 
                    "min_distance", "rate", "duration", "stress_avoidance_performance_avr", 
                    "stress_avoidance_performance_ever_zero", "start_time", "final_task_error"]
        
        # If the file does not exist, create it and write the header
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row_title)
                rospy.loginfo(f"File '{self.csv_file}' created and header written.")
                
        # Append the results to the csv file
        with open(self.csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            if execution_results is not None:
                writer.writerow([str(experiment_id)] + execution_results)
            else:
                writer.writerow([str(experiment_id)] + ["Nan" for _ in range(len(row_title)-1)])
                rospy.logwarn("No execution results to save, writing 'Nan' values.")                
            rospy.loginfo(f"Results appended to the CSV file: '{self.csv_file}'.")
            