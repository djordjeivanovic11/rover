# =============================================================================
# ARM STATUS MESSAGE
# =============================================================================
# Comprehensive status message for dashboards, ROS bags, and judge telemetry
# Published at regular intervals for system monitoring
# =============================================================================

# Header for timestamp and frame information
std_msgs/Header header

# Current arm state enum
uint8 IDLE = 0
uint8 PLANNING = 1
uint8 EXECUTING = 2
uint8 TOOL_CHANGE = 3
uint8 FAULT = 4
uint8 EMERGENCY_STOP = 5
uint8 CALIBRATING = 6
uint8 state

# Current end-effector pose
geometry_msgs/PoseStamped current_pose

# Current joint state
sensor_msgs/JointState joint_state

# Current tool information
string current_tool_name
bool tool_attached
float32 gripper_opening     # Current gripper opening (meters)
float32 gripper_force       # Current gripper force (Newtons)

# Safety status
bool safety_ok              # Overall safety status
bool estop_active          # Emergency stop status
uint8 error_code           # Current error code (0 = no error)
string error_message       # Human-readable error description

# Motion status
bool in_motion             # True if arm is currently moving
float32 velocity_norm      # Current velocity magnitude (rad/s)
geometry_msgs/Vector3 tcp_velocity  # TCP linear velocity (m/s)
geometry_msgs/Vector3 tcp_angular_velocity  # TCP angular velocity (rad/s)

# Force/Torque status
geometry_msgs/WrenchStamped current_wrench  # Current F/T at TCP
bool force_limit_active    # True if force limiting is active

# Temperature monitoring
float32[] joint_temperatures  # Joint temperatures (Celsius)
float32 max_temperature      # Highest joint temperature
bool temperature_warning     # True if any joint over warning temp

# Current monitoring  
float32[] joint_currents     # Joint currents (Amps)
float32 max_current         # Highest joint current
bool current_warning        # True if any joint over current limit

# Workspace status
bool workspace_valid        # True if current pose is within workspace
float32 workspace_margin    # Distance to workspace boundary (meters)

# Mission context
string current_mission      # Current mission type (ES, SC, DM, etc.)
string current_action       # Current action being executed
float32 action_progress     # Action progress percentage [0.0-100.0]

# System health
float32 system_uptime       # System uptime (seconds)
uint32 command_count        # Total commands executed
uint32 fault_count         # Total faults encountered
float32 success_rate        # Recent command success rate [0.0-1.0] 