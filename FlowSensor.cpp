#include <Arduino.h>
#include "FlowSensor.h"

volatile int count;

void rpm()
{
    count++;
}

FlowSensor::FlowSensor(const int pin, const float factor) : pin(pin), calibrationFactor(factor)
{
}

void FlowSensor::init()
{
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), rpm, RISING);
}

void FlowSensor::start()
{
    startTime = millis();
    count = 0;
    sei();
}

void FlowSensor::stop()
{
    cli();
    stopTime = millis();
}

float FlowSensor::read()
{
    return count * 1000 / 60 / calibrationFactor; // in milliliters
}

unsigned long FlowSensor::timer()
{
    return stopTime - startTime; // in millis
}
