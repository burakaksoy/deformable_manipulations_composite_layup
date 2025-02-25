# ROS message to represent controller status

# Time that the controller action started or stopped
time time
# Cause of action (manual or automatic)
string cause
# Action (started or stopped)
bool enabled

# Total duration in seconds if the controller is enabled
float32 total_duration # -1.0 means the controller is still enabled
# Overall rate of the controller of the last enabled period
float32 rate # -1.0 means the controller is still enabled

# Overall stress avoidance performance of the controller of the last enabled period 
float32 stress_avoidance_performance_avr

# Overall minimum distance recorded to all obstacles during the last enabled period
float32 min_distance
