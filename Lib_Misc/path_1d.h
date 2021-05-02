#ifndef PATH_1D_H_
#define PATH_1D_H_


#include "mbed.h"
class path_1d{
public:
    path_1d(void);
    virtual ~path_1d();
    void initialize(float,float,float,float,float,float);
    void get_x_v(float,float *, float *);
    void start(float);
    bool finished;
    bool started;
    bool initialized;
    

private:
    float dT;
    float sig;
    float T_end,T1,T2;
    float X1,X2,X3;
    float t_offset;
    float a_max,v_max,s_end,v_0,s_0,s_start;
    //-----------------------------------
    
};

#endif