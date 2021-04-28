#include "TurbiditySensor.h"
#include <Arduino.h>

TurbiditySensor::TurbiditySensor(const int pin) : pin(pin) {}

float TurbiditySensor::read() {
    int value = analogRead(pin);
    return -0.2973 * value + 858.5;
}
