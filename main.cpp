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
#define OPERATION_MODE_CONFIG 0x00
#define BNO055_OPR_MODE_ADDR 0X3D
#define OPERATION_MODE_NDOF 0x0C
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_CALIB_STAT_ADDR 0X35
#define BNO055_GYRO_DATA_X_LSB_ADDR 0X14

double accelX0,velX0,velY0,velZ0,posX0,posY0,posZ0 = 0;


//**I2C functions
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

//**Functions not just for I2C

//combine left unsigned 8 bits and right 8 bits into signed 16 bits
//divide by 100 to normalize values
double comb8To16(uint8_t left,uint8_t right){
	int16_t val = ((int16_t)left << 8) | ((int16_t)(right));
	double output = (double)val / 100.00;
	
	return output;
}

uint8_t readNewAddress(uint8_t reg){
	i2c_start(ADDRESS_W);
	i2c_write(reg);
	//i2c_stop();
	
	i2c_start(ADDRESS_R);
	
	uint8_t value = i2c_read_ack();
	
	i2c_stop();
	
	return value;
}

//gathers data from IMU, streams coordinates into Arduino
void sendData(){	
	i2c_start(ADDRESS_W);
	i2c_write(BNO055_ACCEL_DATA_X_LSB_ADDR);
	
	i2c_start(ADDRESS_R);
	
	uint8_t leftX;
	uint8_t rightX;
	uint8_t leftY;
	uint8_t rightY;
	
	//data gathered
	rightX = i2c_read_ack();	
	leftX = i2c_read_ack();
	
	rightY = i2c_read_ack();
	leftY = i2c_read_nack();
	
	i2c_stop();
	
	//send data to Arduino through UART
	sendUart(leftX); 
	sendUart(rightX); 
	sendUart(leftY); 
	sendUart(rightY); 

// 	sendUart(2);
// 	sendUart(1);
// 	sendUart(3);
// 	sendUart(4);
	
	//TEST
	 
	//put into unsigned 16 bits
	double accelX = comb8To16(leftX,rightX);
	double accelY = comb8To16(leftY,rightY);
	
	if(accelX > 5){
		PORTB = PINB | 0x01;
	}
	else{
		PORTB = PINB & 0xFE;
	}
	if(accelY > 5){
		PORTB = PINB | 0x02;
	}
	else{
		PORTB = PINB & 0xFD;
	}
	//TEST
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
	DDRB = 0xFF; PORTB = 0x00;
	
    initUSART(0);
	i2c_init();
	beginIMU(); 
	//displayCalStatus();
 	
// 	while(PINA == 0x00);
// 	sendUart('A');
	
    while (1) 
    {
		if(PINA == 0x01){
			sendData();
			_delay_ms(50);
		}
    }
}