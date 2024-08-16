#include <MotorCtrl.h>
#include <distance.h>
#include <pid.h>
#include <Arduino.h>


const uint8_t motor_pins_d[3] = {3, 7, 8};
const uint8_t servo_pin_d     = 10;

unsigned long previousMillis = 0;
const unsigned long servoInterval = 10; // Update interval in milliseconds
bool debug = false;
uint8_t new_angle;

// Create instances for ultrasonci sensors
DistanceClass frontDistance(13, 12, 0, 2);
DistanceClass rightDistance(5, 4, 0, 1);   
DistanceClass leftDistance(6, 2, 0, 1);

motor_ctrl MOTORS(motor_pins_d, servo_pin_d, 254); // initializing Motor control class
PID pid_c(0.6, 0.0, 0.03); // initializing pid Controller for left and right distances


void setup() {
  Serial.begin(57600);
  MOTORS.init_pins();
  MOTORS.stop_motor();
  MOTORS.turn_forward();
  delay(1000);
}

void loop() {
  MOTORS.move_forward();

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

  if (debug == true) {
    Serial.print("angle: ");
    Serial.print(new_angle);
    Serial.print(", Correctiong: ");
    Serial.print(correction);
    Serial.print(", right distance: ");
    Serial.print(rightDist);
    Serial.print(", left distance: ");
    Serial.print(leftDist);
  }


  // SPEED ADJUSTMENT BASED ON FRONT DISTANCE
  if (abs(error) < 130) {
    uint8_t speed_pid = (frontDist / 200) * 254;
    MOTORS.update_speed(speed_pid < 200 ? 200 : speed_pid); 
  } else {
    MOTORS.update_speed(254); // if the robot is turning , turn with max speed
  }


  // Servo angle editing
  unsigned long servoMillis = millis();
  if (servoMillis - previousMillis > servoInterval) {
    MOTORS.set_servo_angle(new_angle);
  }

  if (debug == true) {
    Serial.print(", servo anngle: ");
    Serial.print(MOTORS.get_servo_angle());
    Serial.println();
  }
}