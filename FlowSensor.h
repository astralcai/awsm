#ifndef _FLOW_SENSOR_H_
#define _FLOW_SENSOR_H_

class FlowSensor
{
public:
    FlowSensor(const int pin, const float factor);
    void init();           // initialize the sensor
    void start();          // start taking interrupts
    void stop();           // stop taking interrupts
    float read();          // reads the amount of flow in mL
    unsigned long timer(); // sampling time in milliseconds

private:
    const int pin;
    const float calibrationFactor;
    unsigned long startTime;
    unsigned long stopTime;
};

#endif // _FLOW_SENSOR_H_
