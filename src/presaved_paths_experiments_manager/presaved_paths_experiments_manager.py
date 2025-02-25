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

class PresavedPathsExperimentsManager(object):
    def __init__(self, velocity_controller_node, 
                        scene_id, 
                        experiments_range, 
                        saved_paths_dir,
                        dlo_type=None):
        
        self.velocity_controller_node = velocity_controller_node
        self.scene_id = scene_id
        self.dlo_type = dlo_type
        self.experiments_range = experiments_range
        self.experiment_number = None
        self.saved_paths_dir = saved_paths_dir
        
        self.is_experiments_completed = False
        
        self.rosbag_file = None # The rosbag file name with full path
        self.odom_topics = None # List of odom topics to record and reverse the executed path
        self.rosbag_recorder = None # RosbagControlledRecorder object, None if not recording
        
    def start_next_experiment(self):
        self.velocity_controller_node.path_planning_pre_saved_paths_enabled = True
        
        if not self.experiments_range:            
            self.is_experiments_completed = True
            rospy.loginfo("No more experiments to run.")
            
        if not self.is_experiments_completed:
            self.experiment_number = self.experiments_range.pop(0)
            self.run_experiment(self.experiment_number)
        else:
            rospy.logwarn("Experiments are completed. Plese restart the node to run again.")
        
    def run_experiment(self, experiment_number):
        rospy.loginfo("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        rospy.loginfo("Running experiment number %d" % experiment_number)
        
        # Read presaved path for the experiment
        presaved_path_variables, planning_success = self.load_presaved_path(experiment_number)
        
        if planning_success:
            # Assign the path to the controller
            self.velocity_controller_node.set_path(presaved_path_variables)
            
            # Start rosbag recording
            self.start_rosbag_recording(experiment_number)
            
            # Call the controller_enabler(true)
            self.velocity_controller_node.controller_enabler(enable=True, cause="automatic")
        else:
            rospy.logwarn("Planning was not successful. Skipping the execution of the experiment.")
            
            # Save the results with Nan values
            self.save_experiment_results(execution_results=None, experiment_id=experiment_number)
            if not self.is_experiments_completed:
                time_to_wait = 2
                rospy.loginfo("Starting next experiment in %d seconds.." % time_to_wait)
                time.sleep(time_to_wait)
                self.start_next_experiment()
        
    def end_experiment(self, cause="manual", execution_results=None):
        if not self.is_experiments_completed:
            rospy.loginfo("Experiment ended")
            rospy.loginfo("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
            
            # End rosbag recording
            self.stop_rosbag_recording()
            
            # Calculate the executed path length
            exec_path_length, _ = self.calculate_executed_path_length()
            rospy.loginfo("Executed path length: %f" % exec_path_length)
            
            # Append path length to execution_results
            if execution_results is not None:
                execution_results.append(str(exec_path_length))
                
                # Save results to csv file
                self.save_experiment_results(execution_results, self.experiment_number)
            else:
                self.save_experiment_results(execution_results, self.experiment_number)
            
            # Reverse the executed path
            self.reverse_executed_path(speed_multiplier=5.0)
            
            # Re-apply the initial state
            self.apply_initial_state()
            
            if cause == "manual":
                # self.is_experiments_completed = True
                rospy.loginfo("Experiment ended manually")
                if not self.is_experiments_completed:
                    time_to_wait = 2
                    rospy.loginfo("Starting next experiment in %d seconds.." % time_to_wait)
                    time.sleep(time_to_wait)
                    self.start_next_experiment()
                
            else:
                if not self.is_experiments_completed:
                    time_to_wait = 2
                    rospy.loginfo("Starting next experiment in %d seconds.." % time_to_wait)
                    time.sleep(time_to_wait)
                    self.start_next_experiment()
        else:
            rospy.logwarn("Cannot end the experiment since Experiments are already completed. Plese restart the node to run again.")
                
    def load_presaved_path(self, experiment_number):
        rospy.loginfo("Loading presaved path for experiment number %d" % experiment_number)
        
        if self.dlo_type is None:
            scene_dir = f"scene_{self.scene_id}"
        else:
            scene_dir = f"scene_{self.scene_id}_dlo_{self.dlo_type}"
        
        formatted_experiment_id = f"{experiment_number:03d}"
        
        if self.dlo_type is None:
            file_name = f"scene_{self.scene_id}_experiment_{formatted_experiment_id}_data.pkl"  # e.g. "scene_1_experiment_001_data.pkl"
        else:
            file_name = f"scene_{self.scene_id}_dlo_{self.dlo_type}_experiment_{formatted_experiment_id}_data.pkl"  # e.g. "scene_1_dlo_1_experiment_001_data.pkl"
        
        path_file = os.path.expanduser(os.path.join(self.saved_paths_dir, scene_dir, file_name))
        rospy.loginfo("File: " + path_file)
        
        with open(path_file, 'rb') as inp:
            plan_data = pickle.load(inp)
        
            plan_data_ompl = plan_data[0]
            plan_data_trajopt = plan_data[1] 
            performance_data = plan_data[2]
            initial_n_target_states = plan_data[3]
            # performance_data_extra = plan_data[4]
            
            # GUIDE:
            # plan_data_ompl = (ompl_path, ompl_path_points, ompl_path_cumulative_lengths, ompl_path_cumulative_rotations,
            #                 ompl_path_direction_vectors, ompl_path_rotation_vectors, ompl_path_of_particles,
            #                 ompl_path_points_of_particles, ompl_path_cumulative_lengths_of_particles,
            #                 ompl_path_cumulative_rotations_of_particles, ompl_path_direction_vectors_of_particles,
            #                 ompl_path_rotation_vectors_of_particles, ompl_path_approximated_dlo_joint_values)
            # plan_data_trajopt = (trajopt_path, trajopt_path_points, trajopt_path_cumulative_lengths, trajopt_path_cumulative_rotations,
            #                     trajopt_path_direction_vectors, trajopt_path_rotation_vectors, trajopt_path_of_particles,
            #                     trajopt_path_points_of_particles, trajopt_path_cumulative_lengths_of_particles,
            #                     trajopt_path_cumulative_rotations_of_particles, trajopt_path_direction_vectors_of_particles,
            #                     trajopt_path_rotation_vectors_of_particles, trajopt_path_approximated_dlo_joint_values)
            # performance_data = (experiment_id, simplified_dlo_num_segments, avr_state_approx_error, planning_success,
            #                     planning_time_ompl, planning_time_trajopt, total_planning_time, ompl_path_length, trajopt_path_length)
            # initial_n_target_states = (initial_full_state, initial_approximated_state_pos, initial_approximated_state_joint_pos, 
            #                         target_full_state, target_approximated_state_pos, target_approximated_state_joint_pos)
            # performance_data_extra = (dlo_simplification_time, urdf_generation_time)
            
            planning_success = performance_data[3]

        return plan_data_trajopt, planning_success
        
    def start_rosbag_recording(self, experiment_number, compress=True):
        rospy.loginfo("Starting rosbag recording")
        
        if self.rosbag_recorder is not None:
            rospy.logwarn("Rosbag recorder is already running. Stopping the current recording and starting a new one.")
            self.stop_rosbag_recording()
        
        # Create the rosbag file name 
        if self.dlo_type is None:
            scene_dir = f"scene_{self.scene_id}"
        else:
            scene_dir = f"scene_{self.scene_id}_dlo_{self.dlo_type}"
        
        formatted_experiment_id = f"{experiment_number:03d}"
        
        if self.dlo_type is None:
            file_name = f"scene_{self.scene_id}_experiment_{formatted_experiment_id}.bag" # e.g. "scene_1_experiment_001.bag"
        else:
            file_name = f"scene_{self.scene_id}_dlo_{self.dlo_type}_experiment_{formatted_experiment_id}.bag"  # e.g. "scene_1_dlo_1_experiment_001.bag"
        
        saving_dir = os.path.expanduser(os.path.join(self.saved_paths_dir, scene_dir))
        
        if not os.path.exists(saving_dir):
            os.makedirs(saving_dir)
            print(f"Directory '{saving_dir}' created.")
        
        self.rosbag_file = os.path.join(saving_dir, file_name)
        
        # Topics to record
        topics = ["/dlo_markers", "/dlo_state", "/min_dist_markers", 
                    "/min_dist_to_rigid_bodies", "/rigid_body_markers", 
                    "/spacenav/twist", "/rosout", "/rosout_agg"]
        
        # All velocity controller specific topics
        vel_controller_topics = ["-e \"/composite_layup_velocity_controller.*\""]
        topics.extend(vel_controller_topics)
        
        # All odom particles topics
        self.odom_topics = [f"/odom_particle_{i}" for i in self.velocity_controller_node.custom_static_particles]
        # e.g. ["/odom_particle_1", "/odom_particle_2"]
        topics.extend(self.odom_topics)
        
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
        topics = ["/dlo_markers", "/dlo_state", "/min_dist_markers", 
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

    def calculate_executed_path_length(self):
        """
        Calculate the executed path length by reading the /dlo_state messages recorded in the rosbag file.
        
        Description:
        - "/dlo_state" topic stores the executed path for each particle. (This topic is a SegmentStateArray.msg type.)
        - For the path length calculation, we take the average of the path lengths of:
        - - the holding particles (custom_static_particles) and
        - - the center particle (If the number of particles is even, we take the average of the two middle particles as the center particle).
        
        Returns:
            - executed_path_length: float, the average of the path lengths of the holding particles (custom_static_particles) and the center particle.
            - executed_path_cumulative_lengths_of_frames: dict, a dictionary keyed by the particle IDs and "center" with the values as a list of cumulative lengths of the path segments.
        """
        rospy.loginfo("Calculating executed path length")
        
        # Read the recorded rosbag file
        bag = rosbag.Bag(self.rosbag_file)
        
        # A dictionary keyed by the particle IDs and "center" with the values as a list of cumulative lengths of the path segments
        executed_path_cumulative_lengths_of_frames = {} 
        # A dictionary keyed by the particle IDs and "center" with the values as the last positions
        last_positions = {}
        
        keys = self.velocity_controller_node.custom_static_particles + ["center"]
        
        for key in keys:
            executed_path_cumulative_lengths_of_frames[key] = [0.0] # Initialize with 0.0
            last_positions[key] = None  # Initialize with None
            
        for topic, msg, t in bag.read_messages(topics="/dlo_state"):            
            # Read the dlo_state topic, read the positions of the particles and calculate the path length for each particle (ie. custom_static_particles and the "center")
            
            num_particles = len(msg.states) # Assumed to be particle IDs are 0, 1, 2, 3, ... (n-1) (n = num_particles)
            
            # Read the positions of the relevant particles
            particle_positions = {}
            
            # Set the center position 
            if num_particles % 2 == 0:
                mid1 = num_particles // 2 - 1
                mid2 = num_particles // 2
                center_position =(np.array([msg.states[mid1].pose.position.x, 
                                            msg.states[mid1].pose.position.y, 
                                            msg.states[mid1].pose.position.z]) +
                                  np.array([msg.states[mid2].pose.position.x, 
                                            msg.states[mid2].pose.position.y, 
                                            msg.states[mid2].pose.position.z])) / 2.0
            else:
                center_position = np.array([msg.states[num_particles // 2].pose.position.x,
                                            msg.states[num_particles // 2].pose.position.y,
                                            msg.states[num_particles // 2].pose.position.z])                
            particle_positions["center"] = center_position
            
            # Set the positions of the custom_static_particles
            for particle_id in self.velocity_controller_node.custom_static_particles:
                particle_positions[particle_id] = np.array([msg.states[particle_id].pose.position.x,
                                                            msg.states[particle_id].pose.position.y,
                                                            msg.states[particle_id].pose.position.z])
            
                
            # Update the cumulative path length for each particle
            for key in keys:
                if last_positions[key] is not None:
                    segment_length = np.linalg.norm(particle_positions[key] - last_positions[key])
                    executed_path_cumulative_lengths_of_frames[key].append(
                        executed_path_cumulative_lengths_of_frames[key][-1] + segment_length
                    )
                last_positions[key] = particle_positions[key]
                
        bag.close()
            
        # Calculate the executed path length
        executed_path_length = np.mean([executed_path_cumulative_lengths_of_frames[particle][-1] for particle in keys])
        
        return executed_path_length, executed_path_cumulative_lengths_of_frames
        
    def save_experiment_results(self,execution_results, experiment_id):
        rospy.loginfo("Saving experiment results")
        
        # Create the results csv file name
        if self.dlo_type is None:
            scene_dir = f"scene_{self.scene_id}"
        else:
            scene_dir = f"scene_{self.scene_id}_dlo_{self.dlo_type}"
        
        if self.dlo_type is None:
            file_name = f"scene_{self.scene_id}_experiment_execution_results.csv"  # e.g. "scene_1_experiment_execution_results.csv"
        else:
            file_name = f"scene_{self.scene_id}_dlo_{self.dlo_type}_experiment_execution_results.csv" # e.g. "scene_1_dlo_1_experiment_execution_results.csv"
            
        saving_dir = os.path.expanduser(os.path.join(self.saved_paths_dir, scene_dir))
        
        if not os.path.exists(saving_dir):
            os.makedirs(saving_dir)
            print(f"Directory '{saving_dir}' created.")
        
        self.csv_file = os.path.join(saving_dir, file_name)
        
        row_title = ["experiment_id", "ft_on", "collision_on", "success", 
                     "min_distance", "rate", "duration", "stress_avoidance_performance_avr", 
                     "stress_avoidance_performance_ever_zero", "start_time", "final_task_error", 
                     "is_replanning_needed", "num_replanning_attempts",
                     "executed_path_length"]
        
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
                writer.writerow([str(int(experiment_id))] + execution_results)
            else:
                writer.writerow([str(int(experiment_id))] + ["Nan" for _ in range(len(row_title)-1)])
                rospy.logwarn("No execution results to save, writing 'Nan' values.")                
            rospy.loginfo(f"Results appended to the CSV file: '{self.csv_file}'.")
            
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
                     "stress_avoidance_performance_ever_zero", "start_time", "final_task_error", 
                     "is_replanning_needed", "num_replanning_attempts",
                     "executed_path_length"]
        
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
            
    def reverse_executed_path(self, speed_multiplier=1.0):
        rospy.loginfo("Reversing executed path")
        
        # Timed history of Odometry topics (self.odom_topics) stores the executed path for each particle
        
        # Read the recorded rosbag file
        bag = rosbag.Bag(self.rosbag_file)
        
        # Dictionaries to store commands and time deltas for each particle
        commands = {particle: [] for particle in self.velocity_controller_node.custom_static_particles}
        time_deltas = {particle: [] for particle in self.velocity_controller_node.custom_static_particles}
        last_timestamps = {particle: None for particle in self.velocity_controller_node.custom_static_particles}
        
        # Read and store commands with time differences for each topic
        for topic, msg, t in bag.read_messages(topics=self.odom_topics):
            # Extract the particle number from the topic name
            particle = int(topic.split('_')[-1])
            
            if last_timestamps[particle] is not None:
                time_deltas[particle].append((1.0/speed_multiplier)*(t - last_timestamps[particle]).to_sec())
            last_timestamps[particle] = t
            commands[particle].append(self.reverse_odom_msg(msg, speed_multiplier))  # Append in the normal order
        
        bag.close()
        
        # Reverse the lists after the loop
        for particle in self.velocity_controller_node.custom_static_particles:
            commands[particle].reverse()
            time_deltas[particle].reverse()
        
        # Determine the minimum length of command sequences to handle synchronized publishing
        min_len = min(len(commands[particle]) for particle in self.velocity_controller_node.custom_static_particles)
        
        for i in range(min_len):
            # Publish commands to all particles simultaneously using pre-existing publishers
            for particle in self.velocity_controller_node.custom_static_particles:
                self.velocity_controller_node.odom_publishers[particle].publish(commands[particle][i])
            
            # Filter out empty sequences before calling min()
            valid_time_deltas = [time_deltas[particle][i] for particle in self.velocity_controller_node.custom_static_particles if i < len(time_deltas[particle])]
            
            if valid_time_deltas:  # Check if the list is not empty
                sleep_time = min(valid_time_deltas)
                time.sleep(sleep_time)
        
        # Handle any remaining commands for each particle
        for particle in self.velocity_controller_node.custom_static_particles:
            for i in range(min_len, len(commands[particle])):
                self.velocity_controller_node.odom_publishers[particle].publish(commands[particle][i])
                if i < len(time_deltas[particle]):
                    time.sleep(time_deltas[particle][i])
        
    def reverse_odom_msg(self, odom_msg, speed_multiplier=1.0):
        odom = Odometry()
        odom.header = odom_msg.header # No need to reverse the header
        odom.child_frame_id = odom_msg.child_frame_id # No need to reverse the child_frame_id
        odom.pose = odom_msg.pose # No need to reverse the pose
        # Reverse the twist
        odom.twist.twist.linear.x = -odom_msg.twist.twist.linear.x*speed_multiplier
        odom.twist.twist.linear.y = -odom_msg.twist.twist.linear.y*speed_multiplier
        odom.twist.twist.linear.z = -odom_msg.twist.twist.linear.z*speed_multiplier
        odom.twist.twist.angular.x = -odom_msg.twist.twist.angular.x*speed_multiplier
        odom.twist.twist.angular.y = -odom_msg.twist.twist.angular.y*speed_multiplier
        odom.twist.twist.angular.z = -odom_msg.twist.twist.angular.z*speed_multiplier
        return odom 
        
    def apply_initial_state(self):
        rospy.loginfo("Applying initial state")
        
        # Read the initial "/dlo_state" topic from the rosbag file
        bag = rosbag.Bag(self.rosbag_file)
        initial_state = None
        for topic, msg, t in bag.read_messages(topics="/dlo_state"):
            initial_state = msg
            break
        bag.close()
        
        for particle in self.velocity_controller_node.custom_static_particles:
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()  # Corrected to use stamp
            odom.header.frame_id = "map"
            
            # Assign the pose from the initial state to the odometry message
            odom.pose.pose = initial_state.states[particle].pose
            
            # Twist remains zero as we are setting the initial state
            
            # Publish the initial state to the corresponding odom topic
            self.velocity_controller_node.odom_publishers[particle].publish(odom)
        
        return
        