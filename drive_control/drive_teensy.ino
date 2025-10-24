#include <VescUart.h>

VescUart vesc;
uint32_t startTime;
uint32_t elapsedTime;

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial2.begin(115200);
  vesc.setSerialPort(&Serial2);
  startTime = millis();
}

int16_t joy_x = 0;
int16_t joy_y = 0;

const int MAX_SPEED = 15000;

const int MAX_LINE = 30;

void loop() {
  while (Serial1.available()) {
    uint8_t charInd = 0;
    char line[MAX_LINE];
    uint8_t nb = 0;
    while (nb != '\n') {
      nb = Serial1.read();
      line[charInd] = nb;
      charInd++;
    }
    line[charInd] = 0;
    if (line[0] == 'x') {
      joy_x = atoi(line+1);
    } else if (line[0] == 'y') {
      joy_y = atoi(line+1);
    }
  }
  float xAmt = joy_x/32768.0;
  float yAmt = joy_y/32768.0;

  float ml = constrain(yAmt+xAmt/2,-1,1);
  float mr = constrain(yAmt - xAmt/2,-1,1);
  vesc.setRPM(MAX_SPEED*ml);
  vesc.setRPM(MAX_SPEED*ml,3);
  vesc.setRPM(MAX_SPEED*mr,4);
  vesc.setRPM(MAX_SPEED*mr,6);
  delay(50);
}
