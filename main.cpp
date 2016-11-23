/*
 * HandsFreeMouse.cpp
 *
 * Created: 11/20/2016 9:21:43 PM
 * Author : Jessica
 */ 

#include <avr/io.h>
#include "usart_ATmega1284.h"
#include "i2c.c"
#include <util/delay.h>

#define NULL 0
#define ADDRESS_W 0x50
#define ADDRESS_R 0x51	

#define BNO055_CHIP_ID_ADDR 0x00
#define OPERATION_MODE_CONFIG 0X00
#define BNO055_OPR_MODE_ADDR 0X3D
#define OPERATION_MODE_NDOF 0X0C
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0X08
#define BNO055_CALIB_STAT_ADDR 0X35
#define BNO055_GYRO_DATA_X_LSB_ADDR 0X14

uint8_t read8(uint8_t reg){
	uint8_t readValue;
	i2c_start(ADDRESS_W);
	i2c_write(reg);
	i2c_stop();
	
	i2c_start(ADDRESS_R);
	readValue = i2c_read_nack();
	i2c_stop();
	return readValue;
}

void write8(uint8_t reg, uint8_t value){
	i2c_start(ADDRESS_W);
	i2c_write(reg);
	i2c_write(value);
	i2c_stop();
}

void setMode(uint8_t mode)
{
	write8(BNO055_OPR_MODE_ADDR, mode);
	_delay_ms(30);
}

void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
	uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
	if (sys != NULL) {
		*sys = (calData >> 6) & 0x03;
	}
	if (gyro != NULL) {
		*gyro = (calData >> 4) & 0x03;
	}
	if (accel != NULL) {
		*accel = (calData >> 2) & 0x03;
	}
	if (mag != NULL) {
		*mag = calData & 0x03;
	}
}

void beginIMU(){
	setMode(OPERATION_MODE_NDOF);
}

// void sendUart(uint8_t data){
// 	if(USART_IsSendReady(0)){
// 		USART_Flush(0);
// 		USART_Send(data,0);
// 	}
// }

void sendData(){	
	i2c_start(ADDRESS_W);
	i2c_write(BNO055_ACCEL_DATA_X_LSB_ADDR);
	i2c_stop();
	
	i2c_start(ADDRESS_R);
	
	uint8_t leftX;
	uint8_t rightX;
	
	uint8_t leftY;
	uint8_t rightY;
	
	uint8_t leftZ;
	uint8_t rightZ;
	
	rightX = i2c_read_ack();	
	leftX = i2c_read_ack();
	
	rightY = i2c_read_ack();
	leftY = i2c_read_ack();
	
	rightZ = i2c_read_ack();
	leftZ = i2c_read_nack();
	
	i2c_stop();
	
	sendUart(leftX);
	sendUart(rightX);
	sendUart(leftY);
	sendUart(rightY);
	sendUart(leftZ);
	sendUart(rightZ);
	//_delay_ms(300);
	
}

void displayCalStatus(){
	 uint8_t system, gyro, accel, mag;
	 system = gyro = accel = mag = 0;
	 
	getCalibration(&system, &gyro, &accel, &mag);
	//sendUart(accel);
}

int main(void)
{
	DDRD = 0xFF;
	DDRA = 0x00; PORTA = 0x00;
	
    initUSART(0);
	i2c_init();
	beginIMU(); 
	//displayCalStatus();
	
	while(PINA == 0x00);
	
    while (1) 
    {
		sendData();
		_delay_ms(10);
    }
}