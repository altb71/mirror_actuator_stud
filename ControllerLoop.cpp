#include "ControllerLoop.h"
using namespace std;

// contructor for controller loop
ControllerLoop::ControllerLoop(float Ts) : thread(osPriorityHigh,4096), dout1(PB_9)
{
    this->Ts = Ts;
    diff1.reset(0.0f,0);
    diff2.reset(0.0f,0);
    is_initialized = false;
    ti.reset();
    ti.start();
    data.laser_on = true;
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    float w01=2*3.1415927 * 2.0f;
    float xy[2];
    float exc = 0;
    float Amp = 0.025;
    PID_Cntrl vel_cntrl1(0.0158,3.17,0,0,Ts,-.8,.8); 
    PID_Cntrl vel_cntrl2(0.0158,3.17,0,0,Ts,-.8,.8);
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        short c1 = counter1 - index1.positionAtIndexPulse - mk.inc_offset[0]- mk.inc_additional_offset[0];            // get counts from Encoder
        short c2 = counter2 - index2.positionAtIndexPulse - mk.inc_offset[1]- mk.inc_additional_offset[1];            // get counts from Encoder
        data.sens_phi[0] = uw2pi1(2.0f*3.1415927f/4000.0f*(float)c1);
        data.sens_Vphi[0] = diff1(c1);                 // motor velocity 
        data.sens_phi[1] = uw2pi2(2.0f*3.1415927f/4000.0f*(float)c2);
        data.sens_Vphi[1] = diff2(c2);                 // motor velocity
        // -------------------------------------------------------------
        // at very beginning: move system slowly to find the zero pulse
        // set "if(0)" if you like to ommit at beginning
        if(!is_initialized)
            {
            find_index();
            if(index1.positionAtIndexPulse != 0 && index2.positionAtIndexPulse != 0) 
                is_initialized=true;
            }
        else
            {
            // ------------------------ do the control first
            // calculate desired currents here, you can do "anything" here, 
            // if you like to refer to values e.g. from the gui or from the trafo,
            // please use data.xxx values, they are calculated 30 lines below
            //float v_des1 = exc;//10.0f*sinf(2.0f* 3.14159f*8.0f*ti.read());
            //float v_des2 = 0;//10.0f*cosf(2.0f* 3.14159f*8.0f*ti.read());
            
            float Kv = 140;
            float v_des1 = data.cntrl_Vphi_des[0] + Kv * (data.cntrl_phi_des[0] - data.sens_phi[0]);
            float v_des2 = data.cntrl_Vphi_des[1] + Kv * (data.cntrl_phi_des[1] - data.sens_phi[1]);
            data.i_des[0] = vel_cntrl1(v_des1 - data.sens_Vphi[0]);
            data.i_des[1] = vel_cntrl2(v_des2 - data.sens_Vphi[1]);
        
            // ------------------------ write outputs
            i_des1.write(i2u(data.i_des[0]));
            i_des2.write(i2u(data.i_des[1]));
            // GPA: if you want to use the GPA, uncomment and improve following line:
            //exc = myGPA(data.i_des[0],data.sens_Vphi[0]);
            exc = myGPA(v_des1,data.sens_phi[0]);
            
            // now do trafos etc

            if(mk.external_control) // get desired values from external source (GUI)
                {
                if(mk.trafo_is_on)  // use desired xy values from xternal source and transform
                                    // otherwise external source delivers phi1, phi2 values directly
                    {
                    bool dum = mk.X2P(data.cntrl_xy_des,data.cntrl_phi_des);
                    }
                }
            else        // this is called, when desired values are calculated here internally (e.g. pathplanner)
                {
                if(mk.trafo_is_on)
                    {
                    data.cntrl_xy_des[0] = 50.0f*cosf(w01*glob_ti.read());      // make a circle in xy-co-ordinates
                    data.cntrl_xy_des[1] = 50.0f*sinf(w01*glob_ti.read());
                    bool dum = mk.X2P(data.cntrl_xy_des,data.cntrl_phi_des);
                    }
                else
                    {
                    float ti2 = glob_ti.read();
                    data.cntrl_phi_des[0] = Amp * cosf(w01 * ti2);     // make some harmonic movements directly on phi1/phi2
                    data.cntrl_phi_des[1] = Amp * sinf(w01 * ti2);
                    data.cntrl_Vphi_des[0] = -Amp * w01 * sinf(w01 * ti2);     // make some harmonic movements directly on phi1/phi2
                    data.cntrl_Vphi_des[1] =  Amp * w01 * cosf(w01 * ti2);
                    
                    }
                }
            bool dum = mk.P2X(data.sens_phi,data.est_xy);       // calculate actual xy-values, uncomment this if there are timing issues
            //current_path->get_x_v(glob_ti.read(),&phi_des,&v_des);
            }       // else(..) 
        laser_on = data.laser_on;
        i_enable = big_button;
        }// endof the main loop
}

void ControllerLoop::sendSignal() {
    thread.flags_set(threadFlag);
}
void ControllerLoop::start_loop(void)
{
    thread.start(callback(this, &ControllerLoop::loop));
    ticker.attach(callback(this, &ControllerLoop::sendSignal), Ts);
}

float ControllerLoop::pos_cntrl(float d_phi)
{
   
   // write position controller here
   return 0.0;
    }

void ControllerLoop::init_controllers(void)
{
    // set values for your velocity and position controller here!
    
    
}
// find_index: move axis slowly to detect the zero-pulse
void ControllerLoop::find_index(void)
{
    // use a simple P-controller to get system spinning, add a constant current to overcome friction
    float Kp = 0.005;
    float i1 = 0.2f + Kp*(50.0f - data.sens_Vphi[0]);
    float i2 = 0.2f + Kp*(50.0f - data.sens_Vphi[1]) ;
    i_des1.write(i2u(i1));
    i_des2.write(i2u(i2));
    }
    
void ControllerLoop::reset_pids(void)
{
    // reset all cntrls.
    
    
    }