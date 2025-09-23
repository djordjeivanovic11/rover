# Rover status message
std_msgs/Header header

# Battery information
float32 battery_voltage
float32 battery_percentage

# System status
bool motors_enabled
bool emergency_stop
string current_mode

# Position information
geometry_msgs/Pose current_pose
geometry_msgs/Twist current_velocity
