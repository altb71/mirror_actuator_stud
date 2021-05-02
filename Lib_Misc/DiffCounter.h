#ifndef DIFFCOUNTER_H_
#define DIFFCOUNTER_H_

#include <mbed.h>

class DiffCounter
{
public:

    DiffCounter(float T, float Ts);
    DiffCounter(float T, float Ts, uint32_t incPerRev);
    DiffCounter(float T, float Ts, uint32_t incPerRev, float direction);

    float operator()(short inc) {
        return doStep(inc);
    }
    
    virtual     ~DiffCounter();
    
    void        reset(float initValue, short inc);
    float       doStep(int16_t inc);

private:

        double  b;
        double  a;
        int16_t incPast;
        int16_t inc;
        double  vel;
        double  inc2rad;
        int32_t incPerRev;
        float       direction;

};

#endif /* DIFFCOUNTER_H_ */