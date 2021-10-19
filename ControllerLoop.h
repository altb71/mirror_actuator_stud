#include "mbed.h"
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "DiffCounter.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "path_1d.h"
#include "PID_Cntrl.h"
#include "Unwrapper_2pi.h"
#include "Mirror_Kinematic.h"
#include "data_structs.h"
#include "GPA.h"
#include "FastPWM.h"

extern EncoderCounter counter1,counter2;
extern EncoderCounterIndex index1,index2;
extern DiffCounter diff1,diff2;
extern path_1d *current_path;
extern LinearCharacteristics i2u;
extern LinearCharacteristics u2i;
//extern FastPWM i_des1;
extern AnalogOut i_des1;
extern AnalogOut i_des2;
extern DigitalOut i_enable;
extern DigitalIn big_button;
extern Timer glob_ti;
extern Mirror_Kinematic mk;
extern DigitalOut laser_on;
extern DATA_Xchange data;
extern GPA myGPA;
//extern AnalogIn i_act2;

// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class ControllerLoop
{
public:
    ControllerLoop(float Ts);
    virtual     ~ControllerLoop();
    void start_loop(void);
    void init_controllers(void);
    void reset_pids(void);


private:
    void loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    void sendSignal();
    bool is_initialized;
    void find_index(void);
    PID_Cntrl v_cntrl[2];
    Unwrapper_2pi uw2pi1;
    Unwrapper_2pi uw2pi2;
    float pos_cntrl(float);
    float Kv;
    DigitalOut dout1;
};
