// includes
#include "uart_comm_thread.h"
/*
-------- DATA PROTOCOL----------------------------
   254	 1	255	201	1	  4	     0		...
    n1	n2	rec id1	id2	#Byte1 #Byte2 thedata
--------------------------------------------------
1-20 sensor values,
		id1		id2	
		10			 	Counter values
				1		c1
				2		c2	(increments)
		11		actual current
				1		i1 / A
				2		i2 / A
--------------------------------------------------
// NOT USED: 21-40 cntrl values,		21		desired values			1 		Phi1 / rad				2 		Phi2 / rad				3 		x / mm				4 		y / mm
--------------------------------------------------
101-120 estimates/actual values,
		101		angles and calculated x,y
				1 		Phi1 / rad
				2 		Phi2 / rad
				3		x / mm
				4		y / mm

--------------------------------------------------
121-140 send techn. values, like offsets
		id1		id2	
		121
				1 		inc_offset phi1 / increments	int16_t
				2		inc_offset phi2	/ 		"			"
				3 		inc_additional_offset phi1 / increments	int16_t
				4		inc_additional_offset phi2	/ 		"			"
		125
				1		num_it of X2P trafo
--------------------------------------------------

2xx: set-value and commands (like disable controller...)
		id1		id2	
	 	201: 			Set values/parameters
				1 		inc_additional_offset phi1 / increments	int16_t
				2		inc_additional_offset phi2	/ 		"			"
		
		202:			set desired absolute values	
				1		phi1		rad		float
				2		phi2		rad		float
				3		x			mm		float
				4		y			mm		float
		203				Increment values
				1		dphi1		rad		float
				2		dphi2		rad		float
				3		dx			mm		float
				4		dy			mm		float
		220			Laser on/off
				1	0 = off, 1 = on
		221			Trafo on/off
				1	0 = off, 1 = on
		230			external control on/off
				1	0 = off, 1 = on
		241     
				1 Send text
		250 
				1 GPA message

*/


extern GPA myGPA;

// #### constructor
uart_comm_thread::uart_comm_thread(BufferedSerial *com, float Ts): thread(osPriorityBelowNormal, 4096)
 {  
    // init serial
    uart = com;
    //uart->attach(callback(this, &uart_comm_thread::callBack), RawSerial::RxIrq);
    this->Ts = Ts;
    gpa_stop_sent = false;
    init();
}

// #### destructor
uart_comm_thread::~uart_comm_thread() {
}

// #### request data from device
void uart_comm_thread::init(){
       // init statemachine
}


// #### run the statemachine
void uart_comm_thread::run(void)
{
    // returnvalue
    bool retVal = false;
	uint8_t checksum,k;
	uint16_t send_state =1011;
	while(true)
    	{
        ThisThread::flags_wait_any(threadFlag);
        //---  The LOOP --------------------------------------------------------
        uint32_t num = uart->read(buffer, sizeof(buffer));
        if (num >0) 
    		{
        	if(buffer[0] == 254 && buffer[1] == 1)
            	{
            	if(analyse_received_data())
            			;//		led1 = !led1;
            	}
            }
		switch(send_state)
			{
			case 1011:
				send_f_data(101,12,2*4,data.sens_phi);		// send actual phi values (1 and 2)
				send_state = 1012;
				break;	
			case 1012:
				send_f_data(101,34,2*4,data.est_xy);		// send actual xy values 
				send_state = 202;
				break;	
			case 121:		// currently omitted
				send_data(121,34,2*2,mk.inc_additional_offset);	
				send_state = 202;
				break;
			case 202:		// desired phi
				send_f_data(202,12,2*4,data.cntrl_phi_des);
				send_state = 250;
				break;
			case 125:		// number of iterations in the trafo
				send_data(125,1,data.num_it);		
				send_state = 250;
				break;
			case 250:		// send GPA values
				if(myGPA.new_data_available)
					{
					float dum[8];
					myGPA.getGPAdata(dum);
					send_f_data(250,1,(uint16_t)32,dum);	// send new values (8 floats)
					}
				else if(myGPA.start_now)
					{
					send_char_data(250,2,0);			// send start flag
					myGPA.start_now = false;
					gpa_stop_sent  = false;
					}
				else if(myGPA.meas_is_finished && !gpa_stop_sent && !myGPA.new_data_available)
					{
                    send_char_data(250,255,1);		// send stop flag
					gpa_stop_sent = true;
					}
				send_state = 1011;
				break;			
			default:
				break;
			}
        }// loop
}



// #### receive data from hardware buffer, ensuring no dropped bytes

// ------------------- start uart ----------------
void uart_comm_thread::start_uart(void){
		
		thread.start(callback(this, &uart_comm_thread::run));
		ticker.attach(callback(this, &uart_comm_thread::sendThreadFlag), Ts);
}
// this is for realtime OS
void uart_comm_thread::sendThreadFlag() {
    thread.flags_set(threadFlag);
}

void uart_comm_thread::send_f_data(char id1,char id2,uint16_t N,float *dat)
{	
	buffer[0]=254;buffer[1]=1;buffer[2]=255;	// standard pattern
	buffer[3] = id1;
	buffer[4] = id2;
	buffer[5] = N%256;
	buffer[6] = N/256;
	uart->write(buffer, 7);
	char *float_data = (char *)dat;
	uart->write(float_data,N);
	uart->write("\r",2);		// line end
}
// send uint16 data
void uart_comm_thread::send_data(char id1,char id2,uint16_t N,int16_t *dat)
{	
	buffer[0]=254;buffer[1]=1;buffer[2]=255;	// standard pattern
	buffer[3] = id1;
	buffer[4] = id2;
	buffer[5] = *(char *)N;
	uart->write(buffer, 7);
	char *int_data = (char *)dat;
	uart->write(int_data,N);
	uart->write("\r",2);		// line end
}
// send one uint16 data
void uart_comm_thread::send_data(char id1,char id2,int16_t dat)
{	
	buffer[0]=254;buffer[1]=1;buffer[2]=255;	// standard pattern
	buffer[3] = id1;
	buffer[4] = id2;
	buffer[5] = 2;
	buffer[6] = 0;
	uart->write(buffer, 7);
	char *int_data = (char *)dat;
	uart->write(int_data,2);
	uart->write("\r",2);		// line end
}
// send text
void uart_comm_thread::send_char_data(char id1,char id2,uint8_t dat)
{	
	buffer[0]=254;buffer[1]=1;buffer[2]=255;	// standard pattern
	buffer[3] = id1;
	buffer[4] = id2;
	buffer[5] = 1;
	buffer[6] = 0;
	uart->write(buffer, 7);
	char int8_data = (char)dat;
	uart->write(&int8_data,1);
	uart->write("\r",2);		// line end
}
void uart_comm_thread::send_text(const char *txt)
{	
	uint16_t N=0;
   	while(txt[N] != '\0')		// get length of text
     	N++;
	buffer[0]=254;buffer[1]=1;buffer[2]=255;	// standard pattern
	buffer[3] = 241;
	buffer[4] = 1;
	buffer[5] = *(char *)N;
	uart->write(buffer, 7);
	uart->write(txt,N);
	uart->write("\r",2);		// line end
}
// -----------------------------------------------------------------------------
// analyse data, see comments at top of this file for numbering
bool uart_comm_thread::analyse_received_data(void){
	char msg_id1 = buffer[3];
	char msg_id2 = buffer[4];
	uint16_t N = 256 * buffer[6] + buffer[5];
	switch(msg_id1)
		{
		case 201:   // add additional offsets to offsets
			if(N != 2)
				return false;
			switch(msg_id2)
				{
				case 1:			// for phi1
					mk.add_additional_offsets(256 * buffer[8] + buffer[7],0);
					return true;
					break;
				case 2:			// for phi2
					mk.add_additional_offsets(0,256 * buffer[8] + buffer[7]);
					return true;
					break;
				default:
					break;
				}
			break;		// case 201
		case 202:				// set desired phi or xy-values
			if(N != 4)
				return false;
			switch(msg_id2)
				{
				case 1:
					data.cntrl_phi_des[0] = *(float *)&buffer[7];
					return true;
					break;
				case 2:
					data.cntrl_phi_des[1] = *(float *)&buffer[7];
					return true;
					break;
				case 3:
					data.cntrl_xy_des[0] = 	*(float *)&buffer[7];
					return true;
					break;
				case 4:
					data.cntrl_xy_des[1] = *(float *)&buffer[7];
					return true;
					break;
				default:
					break;
				}
			break;		// case 202
		case 203: 		// increment desired values
			if(N != 4)
				return false;
			switch(msg_id2)
				{
				case 1:
					data.cntrl_phi_des[0] += *(float *)&buffer[7];
					return true;
					break;
				case 2:
					data.cntrl_phi_des[1] += *(float *)&buffer[7];
					return true;
					break;
				case 3:
					data.cntrl_xy_des[0] += 	*(float *)&buffer[7];
					return true;
					break;
				case 4:
					data.cntrl_xy_des[1] += *(float *)&buffer[7];
					return true;
					break;
				default:
					break;
				}
			break;		// case 203
		case 220:				// switch laser on/off
			if(N != 1)
				return false;
			switch(msg_id2)
				{
				case 1:
					if(buffer[7] == 1)
						data.laser_on = true;
					else 
						data.laser_on = false;
					return true;
					break;
				}
			break;
		case 221:				// switch trafo on/off
			if(N != 1)
				return false;
			switch(msg_id2)
				{
				case 1:
					if(buffer[7] == 1)
						mk.trafo_is_on = true;
					else 
						mk.trafo_is_on = false;
					return true;
					break;
				}
			break;
		case 230:				// set internal/external control
			if(N != 1)
				return false;
			switch(msg_id2)
				{
				case 1:
					if(buffer[7] == 1)
						mk.external_control = true;
					else 
						mk.external_control = false;
					return true;
					break;
				}
			break;
		}
	return false;	
}

