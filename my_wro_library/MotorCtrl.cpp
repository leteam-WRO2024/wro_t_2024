#include <MotorCtrl.h>

motor_ctrl::motor_ctrl(uint8_t* motor_pins_e, uint8_t servo_pin_e, uint8_t speed_e) : servo_pin(servo_pin_e), speed(speed_e) {
  for (int i = 0; i < 3; i++) {
    this->motor_pins[i] = motor_pins_e[i];
  }
}

void motor_ctrl::init_pins() {
  for (int i = 0; i < 3; i++) {
    pinMode(this->motor_pins[i], OUTPUT);
  }

  this->servo_ctrl.attach(this->servo_pin);
}

void motor_ctrl::set_servo_angle(short angle) {
  angle = constrain(angle, this->angles[0], this->angles[2]);

  this->servo_ctrl.write(angle);
  this->servo_angle = angle;
}

short motor_ctrl::get_servo_angle() const {
  return (short) this->servo_ctrl.read();
}

short motor_ctrl::get_default_angle(uint8_t which) const {
  which = constrain(which, 0, 2);
  return this->angles[which];
}

void motor_ctrl::turn_forward() {
  this->set_servo_angle(this->angles[1]);
}

void motor_ctrl::turn_left() {
  this->set_servo_angle(this->angles[0]);
}

void motor_ctrl::turn_right() {
  this->set_servo_angle(this->angles[2]);
}

void motor_ctrl::reverse_angle() {
  short new_angle = (this->angles[1] - this->servo_angle) + this->angles[1];
  this->set_servo_angle(new_angle);
}

void motor_ctrl::update_default_angles(short* angles_e) {
  for (int i = 0; i < 3; i++) {
    this->angles[i] = constrain(angles_e[i], 0, 180);
  }
}

void motor_ctrl::update_speed(uint8_t speed_e) {
  this->speed = constrain(speed_e, 0, 254);
}

void motor_ctrl::move_forward(uint8_t speed_e) {
  this->move_motor(speed_e, true);
}

void motor_ctrl::move_backward(uint8_t speed_e) {
  this->move_motor(speed_e, false);
}

void motor_ctrl::move_motor(uint8_t speed_e, bool forward) {
  // Set motor direction
  // digitalWrite(this->motor_pins[2], forward ? LOW : HIGH);

  // Set motor speed
  if (speed_e == 255) {
    speed_e = this->speed;
  }

  digitalWrite(this->motor_pins[1], forward ? HIGH : LOW);
  analogWrite(this->motor_pins[0], constrain(speed_e, 0, 254));
}

void motor_ctrl::stop_motor() {
  digitalWrite(this->motor_pins[2], LOW);
  analogWrite(this->motor_pins[0], 0);
}



