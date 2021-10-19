#include "mbed.h"
#include "math.h" 
//------------------------------------------
#define PI 3.1415927f
//------------------------------------------
#include "EncoderCounter.h"
#include "EncoderCounterIndex.h"
#include "DiffCounter.h"
#include "IIR_filter.h"
#include "LinearCharacteristics.h"
#include "PID_Cntrl.h"
#include "Unwrapper_2pi.h"
#include "path_1d.h"
#include "GPA.h"
#include "ControllerLoop.h"
#include "Mirror_Kinematic.h"
#include "data_structs.h"
#include "uart_comm_thread.h"
#include "FastPWM.h"
 
static BufferedSerial serial_port(USBTX, USBRX);
DigitalIn big_button(PC_3);         // Enable button an backside
bool key_was_pressed = false;       //
float Ts=.0002f;                    // sampling time
void pressed(void);
void released(void); 
//------------- DEFINE FILTERS ----------------
// missing
//------------- Define In/Out -----------------
AnalogOut i_des1(PA_5);
AnalogOut i_des2(PA_4);
//AnalogIn i_act1(PA_3);
//AnalogIn i_act2(PC_0);
//FastPWM i_des1(PB_10);
//FastPWM i_des2(PA_15);
DigitalOut i_enable(PC_4);
DigitalOut laser_on(PB_0);
///------------- Encoder -----------------------
EncoderCounter counter1(PA_6, PC_7);    // initialize counter on PA_6 and PC_7
InterruptIn indexpulse1(PA_8);
EncoderCounterIndex index1(counter1,indexpulse1);   
// ------------------------------------
EncoderCounter counter2(PB_6, PB_7);    // initialize counter on PB_6 and PB_7
InterruptIn indexpulse2(PB_4);
EncoderCounterIndex index2(counter2,indexpulse2);    // initialize counter on PA_6 and PC_7
// ------------------------------------
DiffCounter diff1(0.0005f,Ts,4000);              // discrete differentiate, based on encoder data
DiffCounter diff2(0.0005f,Ts,4000);              // discrete differentiate, based on encoder data
//LinearCharacteristics i2pwm(-1.0,1.0,0.02,0.98,.02,.98);
LinearCharacteristics i2u(-.80,.80,0.0f,1.0f);
LinearCharacteristics u2i(0.0,1.0,-1.0,1.0);

Unwrapper_2pi uw2pi1;
Unwrapper_2pi uw2pi2;
//------------------------------------------
// ----- User defined functions -----------
ControllerLoop loop(Ts);                        // this is forthe main controller loop
uart_comm_thread uart_com(&serial_port,.05f);   // this is the communication thread
Timer glob_ti;                                  // the global timer
path_1d p1;             // pathplanner (under constr.)
path_1d p2;             // pathplanner (under constr.)
path_1d *current_path;
// --------- GPA -----------------------------
//init values: f0,   f1, nbPts, A0, A1, Ts
GPA      myGPA(5 , 2400,    40, 60, 50, Ts);
//------------------------------------------------------------------------------
// --------- Mirror kinematik, define values, trafos etc there
Mirror_Kinematic mk;
//------------------------------------------------------------------------------
// --------- data: overall data structure for x-change
DATA_Xchange data;

//******************************************************************************
//---------- main loop -------------
//******************************************************************************

int main()
{
    serial_port.set_baud(115200);
    serial_port.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
    serial_port.set_blocking(false); // force to send whenever possible and data is there
    i_enable = 0;       // disable current first
    counter1.reset();   // encoder reset
    counter2.reset();   // encoder reset
    mk.set_offsets(982,-167);          // individal set values for global position
    mk.trafo_is_on = true;
    glob_ti.start();
    glob_ti.reset();
    loop.init_controllers();
    uart_com.start_uart();
    loop.start_loop();
    i_des1.write(i2u(0));
    i_des2.write(i2u(0));
    ThisThread::sleep_for(200);
    uart_com.send_text((char *)"Start Mirroractuator 1.2");
   /* p1.initialize(300,10,A,0,0,0);
    p2.initialize(300,10,-A,0,0,A);*/
    laser_on = true;
    //for(int wk =0;wk<5;wk++)
    while(0)
        {
        short c1 = counter1;            // get counts from Encoder
        short c2 = counter2;            // get counts from Encoder
        current_path = &p1;
        current_path->start(glob_ti.read());
        while(!current_path->finished)
            ThisThread::sleep_for(100);
        current_path = &p2;
        current_path->start(glob_ti.read());
        while(!current_path->finished)
            ThisThread::sleep_for(100);
        ThisThread::sleep_for(100);
        laser_on = !laser_on;
        }   // end of while(..)
        i_enable = 0;
    while(1)
        {
        ThisThread::sleep_for(200);
        }
}   // END OF main
