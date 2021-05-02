/*  
*/

#include "Unwrapper.h"
#define   pi 3.141592653589793
using namespace std;

Unwrapper::Unwrapper(double i2r)
{   
    inc2rad = i2r;
    last_value = 0;
}

Unwrapper::~Unwrapper() {}

void Unwrapper::reset(void)
{
    last_value = 0;
}

double Unwrapper::doStep(short inc)
{
    long temp = inc;
    if((temp - last_value) > 32000)
        temp -= 0xFFFF;
    else if((temp - last_value) < -32000)
        temp += 0xFFFF;
    last_value = temp;
    return (temp*inc2rad);
}
