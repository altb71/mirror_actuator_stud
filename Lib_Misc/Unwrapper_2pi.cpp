/*  
*/

#include "Unwrapper_2pi.h"
#define   pi 3.141592653589793
using namespace std;

Unwrapper_2pi::Unwrapper_2pi(void)
{   
    last_value = 0.0;
    turns = 0;
}

Unwrapper_2pi::~Unwrapper_2pi() {}

void Unwrapper_2pi::reset(void)
{
    last_value = 0.0;
    turns = 0;
}

float Unwrapper_2pi::doStep(float in)
{
    float temp = in + 2*pi*(float)turns;
    if((temp - last_value) > pi){
        temp -= 2*pi;
        turns--;
        }
    else if((temp - last_value) < -pi){
        temp += 2*pi;
        turns++;
        }
    last_value = temp;
    return (temp);
}
