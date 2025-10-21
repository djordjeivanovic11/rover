# =============================================================================
# FAULT MESSAGE (Simplified for URC)
# =============================================================================
# Structured fault reporting for safety monitor
# =============================================================================

# Timestamp
builtin_interfaces/Time timestamp

# Fault identification
string fault_code              # Fault code (e.g., "ESTOP", "JOINT_LIMIT")
string message                 # Human-readable description

# Fault severity
bool critical                  # True if fault requires immediate stop
