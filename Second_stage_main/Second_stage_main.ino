#include <MotorCtrl.h>
#include <distance.h>
#include <pid.h>
#include <Arduino.h>

const uint8_t motor_pins_d[3] = {3, 7, 8};
const uint8_t servo_pin_d     = 10;

// servo intervals
unsigned long servoMillis = 0;

// turn intervals
unsigned long turnStartMillis = 0;
const unsigned long turnInterval = 800; // 0.8 second 

bool debug = false;

// vars
double correction = 0;
float error = 0;
short base_angle = 90;
short new_angle = 90;
enum State { TURNING, MOVING };
State state = MOVING; 

// Flags & Serial Vars
bool turn_flag = false;
bool stop_flag = false;
double pillar_error = 0.0;
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
  MOTORS.move_forward(200);

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
      error = pillar_error;
      if (error == 0) {
        error = rightDist - leftDist;
        correction = distance_pid.compute(error);
      } else {
        correction = color_pid.compute(error);
      };
      base_angle = MOTORS.get_default_angle(1);
      break;
  }

  new_angle = base_angle + correction;
  
  // Servo angle editing
  if (currentMillis - servoMillis > 10) {
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
  MOTORS.stop_motor();
}

