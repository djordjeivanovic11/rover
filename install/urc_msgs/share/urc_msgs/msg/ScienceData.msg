# Science data message
std_msgs/Header header

# Sample information
string sample_id
geometry_msgs/Point sample_location

# Sensor readings
float32 ph_level
float32 temperature
float32 moisture
string[] detected_compounds
