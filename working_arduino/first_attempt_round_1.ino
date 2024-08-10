#include <Arduino.h>
#include "distance.h"
#include "PID.h"
#include "MotorCtrl.h"

uint8_t new_angle;

const uint8_t motor_pins_d[3] = {3, 7, 8};
const uint8_t servo_pin_d     = 10;

unsigned long previousMillis = 0;
const unsigned long servoInterval = 10; // Update interval in milliseconds

DistanceClass frontDistance(13, 12, 0, 2);  // Create an instance for front sensor with 3 samples per measurement
DistanceClass rightDistance(5, 4, 0, 1);
DistanceClass leftDistance(6, 2, 0, 1);

motor_ctrl MOTORS(motor_pins_d, servo_pin_d, 254);
PID pid_c(0.8, 0.0, 0.01);


void setup() {
  Serial.begin(9600);
  MOTORS.init_pins();
  MOTORS.stop_motor();
  MOTORS.turn_forward();
  delay(1000);
}

void loop() {
  // MOTORS.move_forward();

  frontDistance.read_sensor();  // Update the sensor readings and state
  rightDistance.read_sensor();
  leftDistance.read_sensor();

  float leftDist = leftDistance.get_distance();
  float rightDist = rightDistance.get_distance();
  float frontDist = frontDistance.get_distance();

  float error = rightDist - leftDist;
  double correction = pid_c.compute(error);
  short mid_angle = MOTORS.get_default_angle(1);
  short new_angle = mid_angle + correction;

  // Serial.print("angle: ");
  // Serial.print(new_angle);
  // Serial.print(", Correctiong: ");
  // Serial.print(correction);
  // Serial.print(", right distance: ");
  // Serial.print(rightDist);
  // Serial.print(", left distance: ");
  // Serial.print(leftDist);
  // Serial.println();

  // SPEED ADJUSTMENT BASED ON FRONT DISTANCE
  if (abs(error) < 130) {
    uint8_t speed_pid = (frontDist / 200) * 254;
    MOTORS.update_speed(speed_pid < 200 ? 200 : speed_pid); 
  } else {
    MOTORS.update_speed(254);
  }


  // Servo angle editing
  unsigned long servoMillis = millis();
  if (servoMillis - previousMillis > servoInterval) {
    MOTORS.set_servo_angle(new_angle);
  }
}