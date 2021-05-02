#ifndef UART_COMM_THREAD_H_
#define UART_COMM_THREAD_H_

#include "GPA.h"
#include "mbed.h"
#include "ThreadFlag.h"
#include "data_structs.h"
#include "data_structs.h"
#include "Mirror_Kinematic.h"

using namespace std;

extern DATA_Xchange data;
extern Mirror_Kinematic mk;


// "protocol" specifics

#define BUF_LEN      	20  // max 256
#define DATA_LEN      	20  // max 256

// states
#define IDLE    0
#define WAIT    1
#define RECEIVE 2
#define DONE    3

#define LEN_OF_EXP_TYPE2 1   // length in bytes of expected Type
#define NUM_OF_VALUE    7   // number of expected values
#define EXPECTED2        LEN_OF_EXP_TYPE2 * NUM_OF_VALUE  // byte per Value * expected values = total expected bytes

extern Mirror_Kinematic mc;

// predefiniton for callback (couldn't implement as member)

class uart_comm_thread{
public:
// public members
    uart_comm_thread(BufferedSerial*,float);
    virtual ~uart_comm_thread();
    void run(void);             // runs the statemachine, call this function periodicly, returns true when new data ready (only true for 1 cycle)
   // void request();         // request new set of data
    void start_uart(void);

    // public vars
    // public vars
	const uint8_t N = DATA_LEN;

	uint16_t 	head[6];
	float 		f_values[20];
	uint8_t 	checksum;
	uint8_t 	buffer[80];     // RX buffer
	uint8_t buffCnt;            // max 255
	uint8_t expected;
	uint8_t num_floats;
	uint8_t k_write;
	void send_text(const char *);
	void send_f_data(char,char,uint16_t,float*);
	void send_data(char,char,uint16_t,int16_t*);
	void send_data(char,char,int16_t);
	void send_char_data(char,char,uint8_t);
private:

    // private members 
    void sendCmd(char);     // sends comand to device
    void callBack();        // ISR for storing serial bytes
    void callBack_2();        // ISR for storing serial bytes
    void init();            // re initializes the buffers and the statemachine
    float Ts;
    EventQueue printfQueue;
    bool analyse_received_data(void);
    bool gpa_stop_sent;
    
// -------------------
//	uint8_t buffer[BUF_LEN];     // RX buffer
//	uint8_t buffCnt;            // max 255
	uint8_t state;              // statemachine state variable
	BufferedSerial* uart;   // pointer to uart for communication with device
    ThreadFlag              threadFlag;
    Thread                  thread;
    Ticker                  ticker;
    Mutex mutex;
	void sendThreadFlag();
};

#endif


