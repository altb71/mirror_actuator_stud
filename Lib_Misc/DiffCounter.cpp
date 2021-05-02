/*  
    DiffCounter Class, differentiate encoder counts for cuboid, applies LP filter and unwrapping
    
              b*(1 - z^-1)                      s
      G(z) = -------------  <-- tustin --  ----------- = G(s)
              1 - a*z^-1                      T*s + 1
*/

#include "DiffCounter.h"
#define   pi 3.141592653589793
using namespace std;

DiffCounter::DiffCounter(float T, float Ts)
{   
    this->incPerRev = 4.0*1000.0;    // incr encoder with 6400inc/rev
        this->direction = 1.0f;

        b = 2.0/(2.0*(double)T + (double)Ts);
    a = -(2.0*(double)T - (double)Ts)/(2.0*(double)T + (double)Ts);
    
        incPast = 0;
    vel = 0.0;
    
        inc2rad = 2.0*pi/static_cast<double>(incPerRev);  
}
DiffCounter::DiffCounter(float T, float Ts,uint32_t incPerRev)
{   
    this->incPerRev = incPerRev;
        this->direction = 1.0f;
        
        b = 2.0/(2.0*static_cast<double>(T) + static_cast<double>(Ts));
    a = -(2.0*static_cast<double>(T) - static_cast<double>(Ts))/(2.0*static_cast<double>(T) + static_cast<double>(Ts));
    
        incPast = 0;
    vel = 0.0;
    
        inc2rad = 2.0*pi/static_cast<double>(incPerRev);   // incr encoder with inc/rev
}
DiffCounter::DiffCounter(float T, float Ts,uint32_t incPerRev, float direction)
{   
        this->incPerRev = incPerRev;
        this->direction = direction;
        
        b =   2.0/(2.0*static_cast<double>(T) + static_cast<double>(Ts));
        a = -(2.0*static_cast<double>(T) - static_cast<double>(Ts))/(2.0*static_cast<double>(T) + static_cast<double>(Ts));

        incPast = 0;
    
        vel = 0.0;
        inc2rad = 2.0*pi/static_cast<double>(incPerRev);   // incr encoder with inc/rev
}
DiffCounter::~DiffCounter() {}

void DiffCounter::reset(float initValue, short inc)
{
    vel = static_cast<double>(initValue);
    incPast = inc;
    
        return;
}

float DiffCounter::doStep(int16_t inc)
{
        int16_t delta(inc - incPast);
        incPast = inc;
        vel = b*static_cast<double>(delta)*inc2rad - a*vel;
            
        return direction*(static_cast<float>(vel));
}