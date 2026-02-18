// ============================================================================
// HEXAPOD LEG CONTROLLER - 270-Degree Servos
// ============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// 270-degree servo settings (same as calibration)
#define SERVOMIN  125
#define SERVOMAX  625
#define SERVO_FREQ 50

// Leg 1 channels
#define LEG1_COXA_CHANNEL  0
#define LEG1_FEMUR_CHANNEL 1
#define LEG1_TIBIA_CHANNEL 2

// ============================================================================
// CALIBRATION VALUES - UPDATE THESE!
// ============================================================================
// Set these to the angles you found during calibration
int COXA_ZERO = 135;   // TODO: Change to your value
int FEMUR_ZERO = 135;  // TODO: Change to your value
int TIBIA_ZERO = 135;  // TODO: Change to your value
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  Serial.println("Hexapod Ready - 270deg Servos");
  
  // Move to home position
  setServoAngle(LEG1_COXA_CHANNEL, COXA_ZERO);
  setServoAngle(LEG1_FEMUR_CHANNEL, FEMUR_ZERO);
  setServoAngle(LEG1_TIBIA_CHANNEL, TIBIA_ZERO);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("LEG")) {
      parseLegCommand(command);
    }
  }
}

void parseLegCommand(String cmd) {
  // Format: LEG,1,coxa_deg,femur_deg,tibia_deg
  int comma1 = cmd.indexOf(',');
  int comma2 = cmd.indexOf(',', comma1 + 1);
  int comma3 = cmd.indexOf(',', comma2 + 1);
  int comma4 = cmd.indexOf(',', comma3 + 1);
  
  if (comma1 == -1 || comma2 == -1 || comma3 == -1 || comma4 == -1) {
    return;
  }
  
  int leg_num = cmd.substring(comma1 + 1, comma2).toInt();
  float coxa_deg = cmd.substring(comma2 + 1, comma3).toFloat();
  float femur_deg = cmd.substring(comma3 + 1, comma4).toFloat();
  float tibia_deg = cmd.substring(comma4 + 1).toFloat();
  
  if (leg_num == 1) {
    // Convert from IK output (-180 to +180) to servo range (0-270)
    // IK gives angles relative to zero, we add to zero position
    int coxa_pos = COXA_ZERO + (int)coxa_deg;
    int femur_pos = FEMUR_ZERO + (int)femur_deg;
    int tibia_pos = TIBIA_ZERO + (int)tibia_deg;
    
    // Safety limits for 270-degree servos
    coxa_pos = constrain(coxa_pos, 0, 270);
    femur_pos = constrain(femur_pos, 0, 270);
    tibia_pos = constrain(tibia_pos, 0, 270);
    
    setServoAngle(LEG1_COXA_CHANNEL, coxa_pos);
    setServoAngle(LEG1_FEMUR_CHANNEL, femur_pos);
    setServoAngle(LEG1_TIBIA_CHANNEL, tibia_pos);
  }
}

void setServoAngle(uint8_t channel, int angle) {
  int pulse = map(angle, 0, 270, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}