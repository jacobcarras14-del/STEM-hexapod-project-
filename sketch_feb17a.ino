// ============================================================================
// HEXAPOD SERVO CALIBRATION - PCA9685 (270-Degree Servos)
// ============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// 270-degree servo settings - ADJUST THESE for your specific servos
#define SERVOMIN  125  // Pulse for 0 degrees (try 100-150)
#define SERVOMAX  625  // Pulse for 270 degrees (try 575-650)
#define SERVO_FREQ 50

// Leg 1 servo channels
#define LEG1_COXA_CHANNEL  0
#define LEG1_FEMUR_CHANNEL 1
#define LEG1_TIBIA_CHANNEL 2

int current_angle = 135;  // Start at middle (270/2 = 135)
int current_servo = 0;

void setup() {
  Serial.begin(115200);
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  Serial.println("========================================");
  Serial.println("270-DEGREE SERVO CALIBRATION");
  Serial.println("========================================");
  Serial.println();
  Serial.println("COMMANDS:");
  Serial.println("  +/-  : Change angle by 10 degrees");
  Serial.println("  w/s  : Change angle by 1 degree");
  Serial.println("  n/p  : Next/Previous servo");
  Serial.println("  h    : Go to center (135 deg)");
  Serial.println("  r    : Print current state");
  Serial.println("  0    : 0 degrees");
  Serial.println("  5    : 135 degrees (center)");
  Serial.println("  9    : 270 degrees (max)");
  Serial.println();
  
  setAllServos(135);
  printCurrentState();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case '+':
        current_angle += 10;
        current_angle = constrain(current_angle, 0, 270);
        updateServo();
        break;
        
      case '-':
        current_angle -= 10;
        current_angle = constrain(current_angle, 0, 270);
        updateServo();
        break;
        
      case 'w':
      case 'W':
        current_angle += 1;
        current_angle = constrain(current_angle, 0, 270);
        updateServo();
        break;
        
      case 's':
      case 'S':
        current_angle -= 1;
        current_angle = constrain(current_angle, 0, 270);
        updateServo();
        break;
        
      case 'n':
      case 'N':
        current_servo = (current_servo + 1) % 3;
        printCurrentState();
        break;
        
      case 'p':
      case 'P':
        current_servo = (current_servo - 1 + 3) % 3;
        printCurrentState();
        break;
        
      case 'h':
      case 'H':
        current_angle = 135;
        setAllServos(135);
        printCurrentState();
        break;
        
      case 'r':
      case 'R':
        printCurrentState();
        break;
        
      case '0':
        current_angle = 0;
        updateServo();
        break;
        
      case '5':
        current_angle = 135;
        updateServo();
        break;
        
      case '9':
        current_angle = 270;
        updateServo();
        break;
    }
  }
}

void updateServo() {
  int channel = getCurrentChannel();
  setServoAngle(channel, current_angle);
  printCurrentState();
}

void setAllServos(int angle) {
  setServoAngle(LEG1_COXA_CHANNEL, angle);
  setServoAngle(LEG1_FEMUR_CHANNEL, angle);
  setServoAngle(LEG1_TIBIA_CHANNEL, angle);
}

int getCurrentChannel() {
  if (current_servo == 0) return LEG1_COXA_CHANNEL;
  if (current_servo == 1) return LEG1_FEMUR_CHANNEL;
  return LEG1_TIBIA_CHANNEL;
}

String getServoName() {
  if (current_servo == 0) return "COXA ";
  if (current_servo == 1) return "FEMUR";
  return "TIBIA";
}

void setServoAngle(uint8_t channel, int angle) {
  // Map 0-270 degrees to pulse width
  int pulse = map(angle, 0, 270, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void printCurrentState() {
  Serial.println();
  Serial.println("========================================");
  Serial.print(">>> Servo: ");
  Serial.print(getServoName());
  Serial.print(" (Ch");
  Serial.print(getCurrentChannel());
  Serial.println(")");
  Serial.print(">>> Angle: ");
  Serial.print(current_angle);
  Serial.println(" degrees (0-270)");
  Serial.println("========================================");
  Serial.println();
  Serial.println("RECORD WHEN LEG MATCHES SIM:");
  Serial.println("  Coxa zero:  _____ deg");
  Serial.println("  Femur zero: _____ deg");
  Serial.println("  Tibia zero: _____ deg");
  Serial.println();
}
