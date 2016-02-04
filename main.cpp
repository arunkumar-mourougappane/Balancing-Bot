#include "mbed.h"
#include "lpc17xx.h"
#include <stdio.h>
#include <mpu6050.cpp>
#include "Kalman.h"
#include <math.h>
#define   GUARD_GAIN   20.0
#define PI 3.142857142857143
#define RAD_TO_DEG 57.295779513082320876798154814105
Serial pc(USBTX, USBRX);
Timer t;
PwmOut  forward(p23); //PWM for pins 22 and 23
PwmOut  reverse(p22);
float K = 1.4;
int   Kp = 3;                      
int   Ki = 1;                   
int   Kd = 6;  
int last_error = 0;
int integrated_error = 0;
int pTerm = 0, iTerm = 0, dTerm = 0;
int setPoint = 90;
int drive = 0;
float finaldrive;
double timer = 0;
volatile uint32_t temp;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
double kalAngleX, kalAngleY;
double accXangle, accYangle, accZangle; // Angle calculate using the accelerometer
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter


		
void delay( uint32_t del)
{
	uint32_t i;
	for ( i=0; i<del; i++)
		temp = i;
}

int constrain(int x, int a,int b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

int updatePid(int targetPosition, int currentPosition)   {
	//determine error
  int error = targetPosition - currentPosition; 
  pTerm = Kp * error;
  integrated_error += error;                                       
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);                            
  last_error = error;
	//compute and return position values
  return -constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}

float drivemotor(int drive)
{
	//check for drive position
	if(drive < 0)
	{
		
		float driveneg = abs(drive);
		float driveabs = drive;
		driveneg/= 30;
		driveabs/=30;
		forward = 0;
		reverse = driveneg;
		//set opposite drive to restore position
		return driveabs;
	}
		else if (drive > 0)
		{
			float drivepos = drive;
			drivepos/= 30;
			reverse = 0;
			forward = drivepos;
			//set opposite drive to restore position
			return drivepos;
		}
		return 0;
	}





int main()
{
	t.start();


int16_t ax=0,ay=0,az=0,gx=0,gy=0,gz =0,temp_read = 0;
double accX,accY,accZ,gyroX,gyroY ; //gyroscope variable intialisation
//pc.printf("test1\t\n");
MPU6050 mpu;
mpu.initialize();
int j = mpu.testConnection();
	while(1)
	{
		temp_read = mpu.getTemperature();
		temp_read = temp_read/340 + 36.53;
		
	mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
	accX=(double)ax;
	accY=(double)ay;
	accZ=(double)az;	
	gyroX=(double)gx;	
	gyroY=(double)gy;	
	
	// determining drift and correction	
	double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
	accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
		
	compAngleX = accXangle;
  compAngleY = accYangle;
	kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
		
		
	gyroXangle += gyroXrate*((double)(t.read_us()-timer)/1000000);
  gyroYangle += gyroYrate*((double)(t.read_us()-timer)/1000000);
		
	compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(t.read_us()-timer)/100000)))+(0.07*accXangle); // Calculate the angle using a Complimentary filter // value comparison and check
  compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(t.read_us()-timer)/100000)))+(0.07*accYangle);
		
		
	kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(t.read_us()-timer)/100000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(t.read_us()-timer)/100000);
	
	timer = t.read_us(); // read and use time interrupts
	drive = 90 - kalAngleX;
	//updatePid(setPoint, kalAngleX;
	
	// setting drive data
	drive = constrain(drive, -30, 30);

		 finaldrive=drivemotor(drive*4  );
	pc.printf("%0.2f\t\t%d\t\t%0.2f\t\t0\t\t0\n",finaldrive,drive,kalAngleX); // USB serial read data for reference
	
	}

}
