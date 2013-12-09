CC=g++
CFLAGS=-Wall -lm


logdata: logdata.cpp MPU6050.o I2Cdev.o 
	g++ logdata.cpp MPU6050.o I2Cdev.o -lm -o logdata 

I2Cdev.o: I2Cdev.cpp I2Cdev.h
	g++ -c I2Cdev.cpp -o I2Cdev.o
	
MPU6050.o: MPU6050.cpp MPU6050.h
	g++ -c MPU6050.cpp -o MPU6050.o

