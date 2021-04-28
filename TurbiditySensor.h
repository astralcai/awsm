#ifndef _TURBIDITY_SENSOR_H_
#define _TURBIDITY_SENSOR_H_

class TurbiditySensor
{
public:
    TurbiditySensor(const int pin);
    float read(); // get the turbidity value

private:
    const int pin;
};

#endif // _TURBIDITY_SENSOR_H_
