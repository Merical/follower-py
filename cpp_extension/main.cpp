#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <iostream>  
#include <vector>
#include "communication.h"
//#include "jetsonGPIO.h"
//  sudo chmod 666 /dev/ttyUSB0
using namespace std;


string handsfree_state;
char handsfree_message[1024] = {0};

bool debugFlag = 0;
string doubleToString(double num);
//double checkdist(jetsonTX1GPIONumber pin_out, jetsonTX1GPIONumber pin_in);
void set_speed(unsigned char fd, int speed);
double GetTickCount();
int speed_arr[] = {B921600, B38400, B19200, B9600, B4800, B2400, B1200, B300,
                        B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {921600, 38400, 19200, 9600, 4800, 2400, 1200, 300, 38400,
                    19200, 9600, 4800, 2400, 1200, 300, };


extern "C"
{
	char* handsfreeDriver( DriverValue objval)
	{
        if (debugFlag == 1) cout << "LCH: C++ work begin! " << endl;
		//double current_time;
		//double start_time;
		//double run_time = 0.00;
		unsigned char fd;
		bool system_info_flag = false;
		unsigned char handsShakeState = false;
		HF_HW hw_touch;
		//unsigned char usart=system("sudo chmod 666 /dev/ttyUSB0");    
		handsfree_state = "Ready to go!";
        fd=open("/dev/ttyUSB0",O_RDWR|O_NOCTTY|O_NDELAY);
		//set_speed(fd, 921600);

		hw_touch.hf_hw(0x01,0x11,fd);	
		if (debugFlag == 1) cout << "LCH: Task Start" << endl;
		//hw_touch.updateCommand(SHAKING_HANDS);
        if (debugFlag == 1) cout << "PARA_INITIAL" << endl;
		hw_touch.paraInitial();
        if (debugFlag == 1) cout << "READ_SYSTEM_INFO" << endl;
        hw_touch.updateCommand(READ_SYSTEM_INFO);

		//start_time = GetTickCount();
		//cout << "LCH: Mainloop runs the " << count << " time.  " << endl;
		//checkHandshake();

        if (debugFlag == 1) cout << "READ_MOTOR_SPEED" << endl;
        hw_touch.updateCommand(READ_MOTOR_SPEED);
        if (debugFlag == 1) cout << "READ_GLOBAL_COORDINATE" << endl;
        hw_touch.updateCommand(READ_GLOBAL_COORDINATE);
        if (debugFlag == 1) cout << "READ_ROBOT_SPEED" << endl;
        hw_touch.updateCommand(READ_ROBOT_SPEED);
        //hw_touch.updateCommand(READ_HEAD_STATE);
        //hw_touch.updateCommand(READ_IMU_BASE_DATA);
        if (debugFlag == 1) cout << "READ_IMU" << endl;
        hw_touch.updateCommand(READ_IMU_FUSION_DATA);
        hw_touch.readBufferUpdate();

        hw_touch.paraUpdate(objval);

        hw_touch.writeBufferUpdate();
        if (debugFlag == 1) cout << "SET_ROBOT_SPEED" << endl;
        //hw_touch.updateCommand(SET_MOTOR_SPEED);
        hw_touch.updateCommand(SET_ROBOT_SPEED);
        //hw_touch.updateCommand(SET_HEAD_STATE);
        
        //hw_touch.systemInfo();
        //hw_touch.sendHandsShake(SHAKING_HANDS);

		//current_time = GetTickCount();
		//run_time = double((current_time - start_time)/1000);
		//cout << "LCH: The run_time is " << run_time << endl;
		//handsfree_state = to_string(run_time); 
		//handsfree_state = doubleToString(run_time); 
		//cout << handsfree_state << endl;
		//strcpy(handsfree_message,handsfree_state.c_str());
		//strcpy(handsfree_message, sensor_message);
        close(fd);
		return handsfree_message;
	}


	/*char* hc_sensor()
	{
		string dist_string;
		cout << "Testing the HC sensor" << endl;

		jetsonTX1GPIONumber hc_out = gpio219 ;     // Ouput
		jetsonTX1GPIONumber hc_in = gpio38 ; // Input

		gpioExport(hc_out);
		gpioExport(hc_in);
		gpioSetDirection(hc_in,inputPin);
		gpioSetDirection(hc_out,outputPin);
		dist_string = doubleToString(checkdist(hc_out, hc_in));
		strcpy(dist_message, dist_string.c_str());
		gpioUnexport(hc_in);     // unexport the LED
		gpioUnexport(hc_out);      // unexport the push button
		return dist_message;
	}*/
}

/********************************set_speed***********************/
void set_speed(unsigned char fd, int speed)
{
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);
    for ( i= 0; i < sizeof(speed_arr) / sizeof(int); i++) 
   {
      if (speed == name_arr[i]) 
      {    
        tcflush(fd, TCIOFLUSH);    
        cfsetispeed(&Opt, speed_arr[i]);
        cfsetospeed(&Opt, speed_arr[i]);  
        status = tcsetattr(fd, TCSANOW, &Opt);
        if (status != 0) 
          {   
            perror("tcsetattr fd1");
	    printf("set speed error\n");
            return;    
          }   
        tcflush(fd,TCIOFLUSH);  
      }
   }
}


string doubleToString(double num)
{
	char str[256];
	sprintf(str,"%.4lf",num);
	string result = str;
	return result;
}

/*
double GetTickCount()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

double checkdist(jetsonTX1GPIONumber pin_out, jetsonTX1GPIONumber pin_in)
{
	//cout << "LCH: Checkdist start!" << endl;
	unsigned int in_value = low;
	gpioSetValue(pin_out, high);
	usleep(15);
	gpioSetValue(pin_out, low);
	gpioGetValue(pin_in, &in_value);
	while (in_value != high)
	{	
		gpioGetValue(pin_in, &in_value);
	}
//	long double tim1 = GetTickCount()/1000;
	clock_t tim1 = clock();
	//cout << "LCH: The tim1 is " << tim1 << endl;
	while (in_value != low)
	{	
		gpioGetValue(pin_in, &in_value);
	}
//	long double tim2 = GetTickCount()/1000;
	clock_t tim2 = clock();
	double delta_time = (double)(tim2-tim1)/CLOCKS_PER_SEC;
	//cout << "LCH: The tim2 is " << tim2 << endl;
	double distance = delta_time * 34000/2;
	return distance;

}
*/
