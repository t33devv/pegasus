#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();

#define servoMIN 150
#define servoMAX 600

int angleToPulse(int angle) {
  return map(angle, 0, 180, servoMIN, servoMAX);
}

void setup() {
  Serial.begin(9600);
  srituhobby.begin();
  srituhobby.setPWMFreq(60);
  delay(10);

  srituhobby.setPWM(0, 0, angleToPulse(0));
}

void loop() {
}