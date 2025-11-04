#include <VescUart.h>

VescUart vesc;

// Joystick mode (legacy x/y)
int16_t joy_x = 0;
int16_t joy_y = 0;

// Wheel mode (new L/R)
int rpm_left_cmd = 0;
int rpm_right_cmd = 0;
bool use_wheel_mode = false;
uint32_t last_wheel_cmd_ms = 0;

const int MAX_SPEED = 15000;
const int MAX_LINE  = 30;
const uint32_t READ_TIMEOUT_MS = 20;
const uint32_t WATCHDOG_TIMEOUT_MS = 300;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  vesc.setSerialPort(&Serial2);
  vesc.setSendCan(true);
  delay(100);
}

bool read_line_from_serial(char* out, size_t out_cap) {
  size_t i = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < READ_TIMEOUT_MS) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (i < out_cap) out[i] = '\0';
        else out[out_cap - 1] = '\0';
        return true;
      }
      if (i < out_cap - 1) out[i++] = c;
    }
    delayMicroseconds(200);
  }
  return false;
}

void loop() {
  char line[MAX_LINE];
  
  // Parse incoming commands
  if (read_line_from_serial(line, sizeof(line))) {
    // NEW: Atomic wheel commands "L<rpm> R<rpm>" (preferred for autonomous)
    if (line[0] == 'L' && strchr(line, 'R')) {
      int l = 0, r = 0;
      if (sscanf(line, "L%d R%d", &l, &r) == 2) {
        rpm_left_cmd = l;
        rpm_right_cmd = r;
        last_wheel_cmd_ms = millis();
        use_wheel_mode = true;
      }
    }
    // LEGACY: Individual wheel commands (backwards compatible)
    else if (line[0] == 'L') {
      rpm_left_cmd = atoi(line + 1);
      last_wheel_cmd_ms = millis();
      use_wheel_mode = true;
    } 
    else if (line[0] == 'R') {
      rpm_right_cmd = atoi(line + 1);
      last_wheel_cmd_ms = millis();
      use_wheel_mode = true;
    }
    // LEGACY: Joystick mode (x/y for teleop)
    else if (line[0] == 'x') {
      joy_x = (int16_t)atoi(line + 1);
      use_wheel_mode = false;
    } 
    else if (line[0] == 'y') {
      joy_y = (int16_t)atoi(line + 1);
      use_wheel_mode = false;
    }
  }

  // Execute motor commands based on mode
  if (use_wheel_mode) {
    // WHEEL MODE: Direct left/right RPM control
    
    // Clamp to safe limits
    int rpm_L = constrain(rpm_left_cmd,  -MAX_SPEED, MAX_SPEED);
    int rpm_R = constrain(rpm_right_cmd, -MAX_SPEED, MAX_SPEED);
    
    // Send to VESCs
    vesc.setRPM(rpm_L);
    vesc.setRPM(rpm_L, 3);
    vesc.setRPM(rpm_R, 4);
    vesc.setRPM(rpm_R, 6);
    
    // Watchdog: stop if no commands for 300ms
    if (millis() - last_wheel_cmd_ms > WATCHDOG_TIMEOUT_MS) {
      rpm_left_cmd = 0;
      rpm_right_cmd = 0;
      vesc.setRPM(0);
      vesc.setRPM(0, 3);
      vesc.setRPM(0, 4);
      vesc.setRPM(0, 6);
    }
    
  } else {
    // JOYSTICK MODE: Mix x/y into left/right (existing behavior)
    
    const float xAmt = joy_x / 32768.0f;
    const float yAmt = joy_y / 32768.0f;
    
    const float ml_f = constrain(yAmt + xAmt * 0.5f, -1.0f, 1.0f);
    const float mr_f = constrain(yAmt - xAmt * 0.5f, -1.0f, 1.0f);
    
    const int ml_rpm = (int)(MAX_SPEED * ml_f);
    const int mr_rpm = (int)(MAX_SPEED * mr_f);
    
    vesc.setRPM(ml_rpm);
    vesc.setRPM(ml_rpm, 3);
    vesc.setRPM(mr_rpm, 4);
    vesc.setRPM(mr_rpm, 6);
  }

  // Read actual motor speeds and send feedback
  int left_meas = 0, right_meas = 0;
  if (vesc.getVescValues())   left_meas  = (int)vesc.data.rpm;
  if (vesc.getVescValues(4))  right_meas = (int)vesc.data.rpm;

  // Output format depends on mode
  if (use_wheel_mode) {
    // Odometry format for ROS
    Serial.print("ODOM ");
    Serial.print(left_meas);
    Serial.print(" ");
    Serial.print(right_meas);
    Serial.print("\n");
  } else {
    // Legacy format for debugging
    Serial.print("RPM L=");
    Serial.print(left_meas);
    Serial.print(" R=");
    Serial.print(right_meas);
    Serial.print("\n");
  }

  delay(50);
}

