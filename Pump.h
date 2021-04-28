#ifndef _PUMP_H_
#define _PUMP_H_

class Pump
{
public:
    Pump(const int fw, const int rv, const int fw_v, const int rv_v);
    void off();
    void forward();
    void reverse();

private:
    const int fw;
    const int rv;
    const int fw_v;
    const int rv_v;
};

#endif // _PUMP_H_