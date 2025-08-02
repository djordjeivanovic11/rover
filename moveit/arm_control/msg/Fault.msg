# =============================================================================
# FAULT MESSAGE
# =============================================================================
# Structured fault reporting for safety monitor and error recovery systems
# Provides detailed fault information for logging and automated response
# =============================================================================

# Header for timestamp and frame information
std_msgs/Header header

# Fault severity levels
uint8 INFO = 0
uint8 WARNING = 1
uint8 ERROR = 2
uint8 CRITICAL = 3
uint8 EMERGENCY = 4
uint8 severity

# Fault code (matches safety_params.yaml fault_codes)
uint32 fault_code

# Fault category for filtering and handling
string fault_category       # "JOINT", "FORCE_TORQUE", "ESTOP", "COMMUNICATION", "WORKSPACE", "SYSTEM"

# Descriptive information
string fault_description    # Human-readable fault description
string component_name       # Component that generated the fault
string joint_name          # Specific joint name (if applicable)

# Fault data
float64 fault_value        # Measured value that caused fault
float64 fault_threshold    # Threshold that was exceeded
string fault_units         # Units for fault_value and fault_threshold

# Spatial information
geometry_msgs/Point fault_location  # 3D location of fault (if spatial)
string reference_frame     # Reference frame for fault_location

# Recovery information
bool auto_recoverable      # True if fault can be automatically recovered
bool recovery_attempted    # True if recovery was attempted
bool recovery_successful   # True if recovery was successful
string recovery_action     # Description of recovery action taken

# System state at fault time
uint8 system_state         # System state when fault occurred (from ArmStatus)
string current_action      # Action being executed when fault occurred
float32 action_progress    # Action progress when fault occurred [0.0-100.0]

# Additional context
string[] related_faults    # IDs of related/cascading faults
uint32 occurrence_count    # Number of times this fault has occurred
duration time_since_last   # Time since last occurrence of this fault type

# Operator guidance
string recommended_action  # Recommended operator action
uint8 operator_intervention_required  # Level of intervention needed
string troubleshooting_info  # Additional troubleshooting information 