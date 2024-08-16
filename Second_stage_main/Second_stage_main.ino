#include <MotorCtrl.h>
#include <distance.h>
#include <pid.h>
#include <Arduino.h>

// PINS
const uint8_t motor_pins_d[3] = {3, 7, 8};
const uint8_t servo_pin_d     = 10;

// servo intervals
unsigned long servoMillis = 0;
const unsigned long servoInterval = 10; // 0.01 second

// turn intervals
unsigned long turnStartMillis = 0;
const unsigned long turnInterval = 1000; // 1 second 


// vars
double correction = 0;
float error = 0;
float pillar_error = 0.0;
short base_angle = 90;
short new_angle = 90;
bool debug = false;

enum State { TURNING, MOVING };
State state = MOVING; 

// Flags 
bool turn_flag = false;
bool stop_flag = false;

// Create instances for ultrasonci sensors
DistanceClass frontDistance(13, 12, 0, 2);
DistanceClass rightDistance(5, 4, 0, 1);   
DistanceClass leftDistance(6, 2, 0, 1);

motor_ctrl MOTORS(motor_pins_d, servo_pin_d, 254); // initializing Motor control class
PID distance_pid(0.6, 0.0, 0.03); // initializing pid Controller for left and right distances
PID color_pid(0.5, 0.0, 0.01);


void setup() {
  Serial.begin(57600);
  MOTORS.init_pins();
  MOTORS.stop_motor();
  MOTORS.turn_forward();
  delay(1000);
  base_angle = MOTORS.get_default_angle(1);
}

void loop() {
  unsigned long currentMillis = millis();
  if (!stop_flag) {
    MOTORS.move_forward(200);
  } else {
    handle_finishing(); // function because we might need to add final adjustments to make the robot stop from where it started
  }

  frontDistance.read_sensor();  // Update the sensor readings and state
  rightDistance.read_sensor();
  leftDistance.read_sensor();

  float leftDist = leftDistance.get_distance();
  float rightDist = rightDistance.get_distance();
  float frontDist = frontDistance.get_distance();

  // read and handle flags from the rasperry to handle second stage and stopping 
  if (Serial.available() > 0) {
    process_serial();
  };
  
  switch (state) {
    case TURNING:
      if (currentMillis - turnStartMillis >= turnInterval) {
        state = MOVING;
        error = rightDist - leftDist;
        correction = distance_pid.compute(error);
        base_angle = MOTORS.get_default_angle(1); // making base angle the mid angle for the servo
      } else {
        error = pillar_error;
        correction = color_pid.compute(error);
        base_angle = MOTORS.get_default_angle(2); // making base angle the right angle for the servo (based on turn direction this changes to left or right angle of the servo)
      }
      break;

    case MOVING:
      if (pillar_error == 0) { // if there is no pillars e.g (red or green pillar) adjust based on distance from walls
        error = rightDist - leftDist;
        correction = distance_pid.compute(error);
      } else { // if there is pillars e.g (red or green pillar) adjust based on the pillar error
        error = pillar_error; 
        correction = color_pid.compute(error);
      };
      base_angle = MOTORS.get_default_angle(1); // in both cases we are still moving forward so the error adjustment is on the mid angle of the servo
      break;
  }

  new_angle = base_angle + correction;
  
  // Servo angle adjustment
  if (currentMillis - servoMillis > servoInterval) {
    MOTORS.set_servo_angle(new_angle);
    servoMillis = currentMillis; 
  }

}

void process_serial() {
  String message = Serial.readStringUntil("\n");
  char flag      = message.charAt(0);
  String data    = message.substring(2);

  switch (flag) {
    case 'r':
    case 'g':
      // handle green pillar
      pillar_error = data.toInt();
      if (debug) {
        Serial.print("data: ");
        Serial.print(data);
        Serial.print(", error: ");
        Serial.print(pillar_error);
      }
      break;
    case 's':
      // set stopping flag to true when its the last turn
      stop_flag = (data == "true");
      break;
    case 't':
      // set turning flag to true 
      // this works whenever the camera reads a ground color e.g (blue or orange)
      // it also checks if there is at least one second delay between each ground color reading and that for avoiding error
      turn_flag = (data == "true");
      if (turn_flag) {
        turnStartMillis = millis();
        state = TURNING;
      }
      break;
  };
}


void handle_finishing() {
  // do other stuff before stopping maybe adjusting to the section where the robot started etc ...
  MOTORS.stop_motor();
}
