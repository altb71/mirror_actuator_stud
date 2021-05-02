#include "path_1d.h"
using namespace std;

path_1d::path_1d(void)
{
    this->finished = false;
    this->started  = false;
    this->initialized = false;
    
}


path_1d::~path_1d() {}


void path_1d::initialize(float a,float v, float s,float v_0,float v_end, float s_start){
    this->a_max = a;
    this->v_max = v;
    this->s_start = s_start;
    this->sig = (float)(((s-s_start)>=0) -((s-s_start)<0));
    this->v_0 = abs(v_0);
    this->dT=-this->v_0/this->a_max;
    v_end = abs(v_end);
    this->s_0 = abs(s-s_start);
    this->s_end = this->s_0+.5f*v_end*v_end/this->a_max+.5f*this->v_0*this->v_0/this->a_max;
    this->T_end=2*sqrt(this->s_end/this->a_max);
    float VT= a * this->T_end/2.0f;
    if(VT<=this->v_max)
        {
        this->T1=this->T_end/2.0f;
        this->T2=this->T_end/2.0f;
        }
    else
        {
        float V_ = VT-this->v_max;
        float TX = 2.0f*V_ / this->a_max;
        float DS = .5f*V_ * TX;
        float T_ = TX + DS / this->v_max;
        this->T_end = this->T_end+DS / this->v_max;
        this->T1=(this->T_end-T_)/2.0f;          // beschleunige bis hier
        this->T2=(this->T_end+T_)/2.0f;          // bis hier konstant
        this->X1=0.5f*this->a_max*this->T1*this->T1;
        this->X2=this->X1 + this->v_max*(this->T2-this->T1);
        this->X3=this->X2 + .5f*a_max*(this->T_end - this->T2)*(this->T_end-this->T2);
        }
    finished = false;
    started  = false;
    initialized = true;
}
void path_1d::get_x_v(float t, float *x,float *v)
{
 t = t-t_offset-dT;
 if(t <=T1)
    {
    *x = 0.5f*a_max*t*t;
    *v = a_max * t;
    }
else if(t<=T2)
    {
     *x = X1+v_max*(t-T1);
     *v = v_max;
    }
else if(t<=T_end)
    {
    *x = X3-.5*a_max*(t-T_end)*(t-T_end);
    *v = -a_max*(t-T_end);
    }
else
    {
    *x = s_end;
    *v = 0;
    }
if(t>=T_end)
    {
    finished = true;
    started = false;
    }
*x -= .5*v_0*v_0/a_max;
if((*x) >= s_0)
    {
    finished = true;    
    started = false;
    }
*x *= sig;
*x += s_start; 
*v *= sig;
}
void path_1d::start(float t)
{   
    t_offset = t;
    finished = false;
    started = true;
    }