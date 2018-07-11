/*
 * iic.h
 *
 *  Created on: 2018Äê6ÔÂ5ÈÕ
 *      Author: Administrator
 */

#ifndef IIC_H_
#define IIC_H_

unsigned char I2C_readReg8(unsigned char slave, unsigned char add);
//uint16_t I2C_readReg16(uint8_t slave, uint8_t add);
void I2C_readMany(unsigned char slave, unsigned char startAddress, unsigned char count,
                  unsigned char* buffer);

void I2C_writeReg8(unsigned char slave, unsigned char add, unsigned char data);

void I2C_writeRegs(unsigned char slave, unsigned char add, unsigned char *data, unsigned char count);
//void I2C_writeReg16(uint8_t slave, uint8_t add, uint16_t data);
void Setting_System_Clock_For_On_Chip_RC( char Frequency );
void Delay_MS( unsigned int m );
#endif /* IIC_H_ */
