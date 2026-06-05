#include <Servo.h>

Servo leg1;

#define M_PI 3.14159265358979323846

// Motion parameters
float freq = 1;   // Hz
float dt   = 0.01;  // seconds (100 Hz update)
float amp  = 30.0;  // degrees (± amplitude)
float duration = 2.0/freq;

// Time tracking
unsigned long t0 = 0;
unsigned long t_c = 0;

// Servo calibration
// Mapping: angle = 0.135 * us - 67.5
// => us = (angle + 67.5) / 0.135
int center_angle = 135;  // midpoint of 0–270°
int center_us;

void setup() {
  delay(2000);

  // Attach servo with full range
  // Compute center position in microseconds
  center_us = int((center_angle + 67.5) / 0.135);

  // Attach servo with full range
  leg1.attach(9, 500, 2500);

  // Move to center
  leg1.writeMicroseconds(center_us);
  delay(2000);

  t0 = millis();
  t_c = t0;
}

void loop() {
  unsigned long t_d = millis();

  float t = (t_d - t0) / 1000.0;
  float delta_t = (t_d - t_c) / 1000.0;

  if (t >= duration) {
      // 5 cycles done — go to center and stop
      leg1.writeMicroseconds(center_us);
      while (true) {}  // halt
    }

  if (delta_t >= dt) {

    // Sinusoidal motion (centered at 0°)
    float angle = amp * sin(2 * M_PI * freq * t);

    // Convert angle → microseconds
    int pos = center_us + int(angle / 0.135);

    // Safety clamp (important!)
    if (pos < 500)  pos = 500;
    if (pos > 2500) pos = 2500;


    leg1.writeMicroseconds(pos);

    t_c = t_d;
  }
}
