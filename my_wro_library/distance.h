#ifndef ULTRA_DISTANCE_H
#define ULTRA_DISTANCE_H

#include <HCSR04.h>

class DistanceClass {
  public:
    DistanceClass(uint8_t trigPin, uint8_t echoPin, uint8_t state, uint8_t samples = 3);

    void read_sensor();

    // Different Updates
    void average_distance();
    void max_value();
    void min_value();

    // getters & setters
    float get_distance() const;
    float set_distance(float value);

  private:
    uint8_t state;
    uint8_t SAMPLES; 
    float* ultra_data;
    float distance;

    HCSR04 ultrasonic;

};

#endif