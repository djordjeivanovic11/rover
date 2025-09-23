# =============================================================================
# ROVER STATUS MESSAGE
# =============================================================================
# Comprehensive rover status message for telemetry, diagnostics, and
# mission monitoring. Includes URC competition-specific fields.
# =============================================================================

# Header
std_msgs/Header header

# Current rover state
string current_action                      # Current action being executed
string mission_mode                       # Current mission mode
bool is_autonomous                         # True if in autonomous mode
bool safety_ok                            # True if all safety checks pass
bool emergency_stop_active                # True if emergency stop is active

# Position and motion
geometry_msgs/PoseStamped current_pose     # Current pose in map frame
geometry_msgs/Twist current_velocity       # Current linear and angular velocity
nav_msgs/Odometry odometry                # Full odometry information

# Mission progress
float32 mission_progress_percentage        # Overall mission progress [0.0-100.0]
float32 autonomy_time_remaining           # Remaining autonomy timer (seconds)
int32 waypoints_completed                 # Number of waypoints completed
int32 total_waypoints                     # Total waypoints in current mission

# Hardware status
float32 battery_voltage                   # Battery voltage (V)
float32 battery_current                   # Battery current (A)
float32 battery_soc                       # State of charge [0.0-1.0]
float32 battery_temperature               # Battery temperature (C)
float32[] motor_currents                  # Current draw for each motor (A)
float32[] motor_temperatures              # Temperature for each motor (C)

# Communication and performance
float32 communication_quality             # Link quality [0.0-1.0]
float32 bandwidth_usage                   # Current bandwidth usage (Mbps)
float32 cpu_usage                         # CPU usage percentage [0.0-100.0]
float32 memory_usage                      # Memory usage percentage [0.0-100.0]

# Safety and diagnostics
int32[] active_faults                     # Array of active fault codes
string[] fault_messages                   # Human-readable fault descriptions
bool stuck_detected                       # True if rover is stuck
float32 slip_ratio                        # Current wheel slip ratio [0.0-1.0]
bool collision_detected                   # True if collision detected

# Environmental
float32 ambient_temperature               # Ambient temperature (C)
float32 internal_temperature              # Internal temperature (C)
float32 humidity                          # Humidity percentage [0.0-100.0]
geometry_msgs/Vector3 acceleration        # Current acceleration (m/s²)
geometry_msgs/Vector3 angular_velocity    # Angular velocity from IMU (rad/s)

# URC-specific telemetry
string competition_mode                   # URC competition mode
float32 judge_score_estimate              # Estimated score for current task
bool visual_servoing_active               # True if visual servoing is active
int32 tennis_balls_detected               # Number of tennis balls detected
geometry_msgs/Point sun_direction         # Sun direction vector (for navigation bias) 