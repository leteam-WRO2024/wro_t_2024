#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <Arduino.h>
#include <PWMServo.h>

class motor_ctrl { 
  public:
    motor_ctrl(uint8_t* motor_pins_e, uint8_t servo_pin_e, uint8_t speed_e);
    void init_pins();

    // dc motor methods
    void move_forward(uint8_t speed_e = 255);  // Default values
    void move_backward(uint8_t speed_e = 255);
    void stop_motor();

    // servo methods
    void turn_forward();
    void turn_left();
    void turn_right();
    void reverse_angle();

    // getters & setters
    void     update_speed(uint8_t speed_e);
    short    get_servo_angle() const;
    void     set_servo_angle(short angle = 90);
    void     update_default_angles(short* angles);

    short get_default_angle(uint8_t which) const;

  private:
    void move_motor(uint8_t speed_e = 255, bool forward = true);
    uint8_t    speed = 240;
    uint8_t    motor_pins[3];
    uint8_t    servo_pin;
    PWMServo   servo_ctrl;

    short      servo_angle = 90;
    short      angles[3]   = {40, 90, 140};
};

#endif