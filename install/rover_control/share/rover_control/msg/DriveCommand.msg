# =============================================================================
# DRIVE COMMAND MESSAGE
# =============================================================================
# Low-level drive command message for rover movement control.
# Used internally by action servers and path followers.
# =============================================================================

# Header
std_msgs/Header header

# Basic movement command
geometry_msgs/Twist velocity              # Target linear and angular velocity
float32 acceleration                      # Target acceleration (m/s²)
float32 duration                          # Command duration (seconds, 0 = continuous)

# Control mode
string control_mode                       # "velocity", "position", "trajectory"
bool relative_command                     # True if command is relative to current state

# Safety and limits
float32 max_velocity                      # Maximum allowed velocity (m/s)
float32 max_acceleration                  # Maximum allowed acceleration (m/s²)
bool enforce_safety_limits               # Apply mission-specific safety limits
string mission_mode                      # Mission mode for parameter selection

# Advanced control
float32 curvature                        # Path curvature (1/radius in m⁻¹)
float32 lookahead_distance              # Pure pursuit lookahead distance (m)
bool use_slip_compensation               # Enable wheel slip compensation

# Execution parameters
int32 priority                           # Command priority (higher = more important)
float32 timeout                         # Command timeout (seconds)
string source                           # Source of command (for logging) 