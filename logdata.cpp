#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <string.h>
#include "Kalman.h"
#include <sys/time.h>
#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//struct data{
//	int16_t ax,ay,az;
//	int16_t gx,gy,gz;
//};
MPU6050 accelgyro;

Kalman kalmanX;
Kalman kalmanY;


int16_t ax, ay, az;
int16_t gx, gy, gz;

//uint32_t timer; 
struct timeval microtime, microtime2;

double accXangle; 
double accYangle; 

double gyroXangle, gyroYangle; 
double compAngleX, compAngleY;
double kalAngleX, kalAngleY;

//uint32_t timer;


int fd;
//struct data bit; 
int i;
char *filename;

void setup() {
	
   	        

    i = 0;
    filename = "data.txt";
    // initialize device
    printf("Initializing I2C devices...\n");
    accelgyro.initialize();
     

    // verify connection
    printf("Testing device connections...\n");
    printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    
    accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  
    accXangle = (atan2(ax,az)+M_PI)*RAD_TO_DEG; 
    accYangle = (atan2(ay,az)+M_PI)*RAD_TO_DEG;

    kalmanX.setAngle(accXangle);
    kalmanY.setAngle(accYangle);

    gyroXangle = accXangle; 
    gyroYangle = accYangle; 

    compAngleX = accXangle; 
    compAngleY = accYangle; 
    
    gettimeofday(&microtime,NULL);

    //timer = (uint32_t) microtime.tv_usec;   	

}


void loop(char* file) {
    // read raw accel/gyro measurements from device
    
    if (( fd = open(file,O_RDWR | O_APPEND | O_CREAT)) < 0)
    {
	perror("Blad otwarcia pliku");
	exit (-1);
    }

    
    usleep(80000);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    accXangle = (atan2(ax,az)+M_PI)*RAD_TO_DEG;
    accYangle = (atan2(ay,az)+M_PI)*RAD_TO_DEG; 

//printf("%d\n", accXangle);
    double gyroXrate = (double)gx/131.0;
    double gyroYrate = (double)gy/131.0;

//printf("	%f %f 			%f %f		\n",accXangle, kalAngleX,accYangle, kalAngleY);
    gettimeofday(&microtime2,NULL);
    gyroXangle += gyroXrate*((double)(microtime2.tv_usec - microtime.tv_usec)/1000000) - 180;
    gyroYangle += gyroYrate*((double)(microtime2.tv_usec - microtime.tv_usec)/1000000) - 180;

    gettimeofday(&microtime2,NULL);	
    compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(microtime2.tv_usec - microtime.tv_usec)/1000000)))+(0.07*accXangle) - 180;
    compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(microtime2.tv_usec - microtime.tv_usec)/1000000)))+(0.07*accYangle) - 180;
    
    gettimeofday(&microtime2,NULL);
    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(microtime2.tv_usec - microtime.tv_usec)/1000000) - 180;
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(microtime2.tv_usec - microtime.tv_usec)/1000000) - 180;
    
    gettimeofday(&microtime,NULL);
    
    dprintf(fd,"%d.%d,,%f,%f,%f,,%f,%f,%f\n",microtime.tv_sec,microtime.tv_usec,accXangle,compAngleX,kalAngleX,accYangle,compAngleY,kalAngleY);
    
    
	//dprintf(fd,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",i,ax,ay,az,gx,gy,gz,accXangle,gyroXangle,compAngleX,kalAngleX, accYangle, gyroYangle,compAngleY,kalAngleY);  
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    //if (write(fd, &bit, sizeof(bit)) < 0)
    //{
	//perror("Blad zapisu!");
    //}
    // display accel/gyro x/y/z values
    close(fd);
    i++;
    //printf("%hd  %hd  %hd  %hd  %hd  %hd  %hd  %hd  %hd  %hd  %hd  %hd  %hd  %hd  %hd  \n",i,ax,ay,az,gx,gy,gz,accXangle,gyroXangle,compAngleX,kalAngleX, accYangle, gyroYangle,compAngleY,kalAngleY);  
    //printf("a/g: %6hd %6hd    %6hd %6hd   %6f %f \n",ax,ay,gx,gy,kalAngleX,kalAngleY);
}

int main(int argc, char* argv[])
{
    
    setup();
    printf ("ARGC : %d, %s, %s,%s %d\n",argc,argv[0],argv[1],argv[2],strcmp(argv[1],"-f"));
    if (argc > 2)
    {
       	if (strcmp(argv[1],"-f") == 0)
	{
		filename = argv[2];
	}
    }
    for (;;)
		loop(filename);
}

