#include "distance.h"

DistanceClass::DistanceClass(uint8_t trigPin, uint8_t echoPin, uint8_t state_e = 0, uint8_t samples) : 
  SAMPLES(samples), 
  ultrasonic(trigPin, echoPin), 
  state(state_e),
  distance(0)
{
  ultra_data = new float[samples];
}

void DistanceClass::read_sensor() {
  // for (int i = 0; i < this->SAMPLES; i++) {
  //   ultra_data[i] = this->ultrasonic.dist();
  // }

  this->set_distance(this->ultrasonic.dist());

  // switch (state) {
  //   case 0:
  //     this->average_distance();
  //     break;
  //   case 1:
  //     this->max_value();
  //     break;
  //   case 2:
  //     this->min_value();
  //     break;
  //   default:
  //     this->set_distance(this->ultrasonic.dist());
  //     break;
  // }
}

void DistanceClass::average_distance() {
  float sum = 0;
  for (int i = 0; i < this->SAMPLES; i++) {
    sum += ultra_data[i];
  }
  this->set_distance((sum / this->SAMPLES));
}

void DistanceClass::max_value() {
  float curr_max = ultra_data[0];
  for (int i = 0; i < this->SAMPLES; i++) {
    if (ultra_data[i] > curr_max) {
      curr_max = ultra_data[i];
    }
  }
  this->set_distance(curr_max);
}

void DistanceClass::min_value() {
  float curr_min = ultra_data[0];
  for (int i = 0; i < this->SAMPLES; i++) {
    if (ultra_data[i] < curr_min) {
      curr_min = ultra_data[i];
    }
  }
  this->set_distance(curr_min);
}

float DistanceClass::get_distance() const {
  return this->distance;
}

float DistanceClass::set_distance(float value) {
  if (value == 0) { value = 200; }
  this->distance = constrain(value, 0, 200);
}
