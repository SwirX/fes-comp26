```
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <LiquidCrystal_I2C.h>

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

LiquidCrystal_I2C lcd(0x27, 16, 2); 

#define BACK_RIGHT  0
#define BACK_LEFT   2
#define FRONT_LEFT  4
#define FRONT_RIGHT 6

#define STOP_PULSE    1500
#define FORWARD_LEFT  2000
#define BACKWARD_LEFT 1000
#define FORWARD_RIGHT 1000
#define BACKWARD_RIGHT 2000

void setup() {
  Serial.begin(115200);
  
  SerialBT.begin("ESP32_Rover"); 
  Serial.println("Bluetooth started! Ready to pair.");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("    Team RC2    "); 
  lcd.setCursor(0, 1);
  lcd.print("  Ready to GO!  ");

  stopRobot();
}

void loop() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.print("Command received: ");
    Serial.println(command);

    if (command == 'F') {
      moveForward();
    } 
    else if (command == 'B') {
      moveBackward();
    } 
    else if (command == 'L') {
      turnLeft();
    } 
    else if (command == 'R') {
      turnRight();
    } 
    else if (command == 'S') {
      stopRobot();
    }
  }
}

void moveForward() {
  pwm.writeMicroseconds(FRONT_LEFT, FORWARD_LEFT);
  pwm.writeMicroseconds(BACK_LEFT,  FORWARD_LEFT); 
  pwm.writeMicroseconds(FRONT_RIGHT, FORWARD_RIGHT);
  pwm.writeMicroseconds(BACK_RIGHT,  FORWARD_RIGHT); 
}

void moveBackward() {
  pwm.writeMicroseconds(FRONT_LEFT, BACKWARD_LEFT);
  pwm.writeMicroseconds(BACK_LEFT,  BACKWARD_LEFT);
  pwm.writeMicroseconds(FRONT_RIGHT, BACKWARD_RIGHT);
  pwm.writeMicroseconds(BACK_RIGHT,  BACKWARD_RIGHT);
}

void turnLeft() {
  pwm.writeMicroseconds(FRONT_LEFT, BACKWARD_LEFT);
  pwm.writeMicroseconds(BACK_LEFT,  BACKWARD_LEFT);
  pwm.writeMicroseconds(FRONT_RIGHT, FORWARD_RIGHT);
  pwm.writeMicroseconds(BACK_RIGHT,  FORWARD_RIGHT);
}

void turnRight() {
  pwm.writeMicroseconds(FRONT_LEFT, FORWARD_LEFT);
  pwm.writeMicroseconds(BACK_LEFT,  FORWARD_LEFT);
  pwm.writeMicroseconds(FRONT_RIGHT, BACKWARD_RIGHT);
  pwm.writeMicroseconds(BACK_RIGHT,  BACKWARD_RIGHT);
}

void stopRobot() {
  pwm.setPWM(FRONT_LEFT, 0, 4096);
  pwm.setPWM(BACK_LEFT,  0, 4096);
  pwm.setPWM(FRONT_RIGHT, 0, 4096);
  pwm.setPWM(BACK_RIGHT,  0, 4096);
}
```
