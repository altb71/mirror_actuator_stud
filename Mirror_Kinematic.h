#ifndef MIRROR_KINEMATIC_H_
#define MIRROR_KINEMATIC_H_

#include "mbed.h"
#include "data_structs.h"

extern DATA_Xchange data;


class Mirror_Kinematic
{
public:
    Mirror_Kinematic(void);
    float screen_h;
    float  n ;          //        Brechungsindex
    float dist_L;       // distance laser0 to 2nd axis
    float screen_d;     // thickness of screen / mmm
    void set_offsets(int16_t,int16_t);
    void set_additional_offsets(int16_t,int16_t);
    void add_additional_offsets(int16_t,int16_t);
    int16_t get_additional_offsets(uint8_t axis);
    int16_t add_additional_offsets(uint8_t axis);
    int16_t inc_offset[2];
    int16_t inc_additional_offset[2];
    float mot_inc_to_rad;
    float mot_rad_to_inc;
    bool P2X(float *,float *);
    bool X2P(float *,float *);
    bool trafo_is_on;
    bool external_control;
    bool controller_is_on;
private:
    float old_phi[2];
    
    };
    
#endif
    

