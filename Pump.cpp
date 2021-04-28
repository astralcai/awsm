#include <Arduino.h>
#include "Pump.h"

Pump::Pump(const int fw, const int rv, const int fw_v, const int rv_v)
    : fw(fw), rv(rv), fw_v(fw_v), rv_v(rv_v)
{
    pinMode(fw, OUTPUT);
    pinMode(rv, OUTPUT);
    pinMode(fw_v, OUTPUT);
    pinMode(rv_v, OUTPUT);
    off(); // starting state should be off
}

void Pump::off()
{
    digitalWrite(fw_v, 0);
    digitalWrite(rv_v, 0);
    digitalWrite(fw, 0);
    digitalWrite(rv, 0);
}

void Pump::forward()
{
    digitalWrite(rv_v, 0);
    digitalWrite(fw_v, 1);
    digitalWrite(rv, 0);
    digitalWrite(fw, 1);
}

void Pump::reverse()
{
    digitalWrite(fw_v, 0);
    digitalWrite(rv_v, 1);
    digitalWrite(fw, 0);
    digitalWrite(rv, 1);
}