#include <Servo.h>

Servo leg1;
Servo leg2;
Servo leg3;

#define M_PI 3.14159265358979323846


// add these globals
float current_pos1 = 0, current_pos2 = 0, current_pos3 = 0;
float target_pos1  = 0, target_pos2  = 0, target_pos3  = 0;
float smooth = 0.05;  // lower = smoother but slower (0.01–0.2)

// Servo calibration
// Mapping: angle = 0.135 * us - 67.5
// => us = (angle + 67.5) / 0.135
int start_angle = 135;  // CHANGE TO CORRECT STARTING ANGLE
int center_us;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Attach servo with full range
  leg1.attach(9, 500, 2500); //check that this has pwm
  leg2.attach(10, 500, 2500);
  leg3.attach(11, 500, 2500);
  
  // Compute center position in microseconds
  center_us = int((start_angle + 67.5) / 0.135);

  // Move to center
  leg1.writeMicroseconds(center_us);
  leg2.writeMicroseconds(center_us);
  leg3.writeMicroseconds(center_us);
  delay(2000);

  Serial.println("READY");  // tell MATLAB we're good to go
}


void loop() {
  // smoothly step toward target
  current_pos1 += smooth * (target_pos1 - current_pos1);
  current_pos2 += smooth * (target_pos2 - current_pos2);
  current_pos3 += smooth * (target_pos3 - current_pos3);

  leg1.writeMicroseconds(center_us + int(current_pos1 / 0.135));
  leg2.writeMicroseconds(center_us + int(current_pos2 / 0.135));
  leg3.writeMicroseconds(center_us + int(current_pos3 / 0.135));

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');

    float a1 = line.substring(0, line.indexOf(',')).toFloat();
    line = line.substring(line.indexOf(',') + 1);
    float a2 = line.substring(0, line.indexOf(',')).toFloat();
    float a3 = line.substring(line.indexOf(',') + 1).toFloat();

    int pos1 = center_us + int(a1 / 0.135);
    int pos2 = center_us + int(a2 / 0.135);
    int pos3 = center_us + int(a3 / 0.135);

    if (pos1 < 500 || pos1 > 2500 ||
        pos2 < 500 || pos2 > 2500 ||
        pos3 < 500 || pos3 > 2500) {
      Serial.print("OUT_OF_RANGE:");
      Serial.print(a1); Serial.print(",");
      Serial.print(a2); Serial.print(",");
      Serial.println(a3);
    } 
    else {
      // update targets, don't move directly
      target_pos1 = a1;
      target_pos2 = a2;
      target_pos3 = a3;
      Serial.println("OK");
    }
  }
  delay(10);  // 100Hz update rate
}
  
