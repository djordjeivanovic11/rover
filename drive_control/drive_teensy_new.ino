#include <VescUart.h>

VescUart vesc;

int16_t joy_x = 0;
int16_t joy_y = 0;

const int MAX_SPEED = 15000;
const int MAX_LINE  = 30;
const uint32_t READ_TIMEOUT_MS = 20; //50 Hz

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
  uint8_t nb = 0
  if (read_line_from_serial(line, sizeof(line))) {
    if (line[0] == 'x') {
      joy_x = (int16_t)atoi(line + 1);
    } else if (line[0] == 'y') {
      joy_y = (int16_t)atoi(line + 1);
    }
  }

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

  delay(50);
}
