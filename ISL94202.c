/*
 * ISL94202.c
 *
 *  Created on: 12Jun.,2017
 *      Author: Ben V. Brown
 */

#include "ISL94202.h"

unsigned char CurrentStatusBuffer[5]; //Stores current device status + balancing information
unsigned char CurrentVoltageBuffer[22];
unsigned char EEPROMScratch[4]; //Used to buffer the user EEPOM [used for cal data]

/////////////////////////////////// SETTINGS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//Thresholds and releases
void ISL94202_setOVThres(unsigned int mV)
{
    ISL94202_writeEEPROMVoltage(0x00, mV, 1);
}
void ISL94202_setOVRecovery(unsigned int mV)
{
    ISL94202_writeEEPROMVoltage(0x02, mV, 0x00);
}
void ISL94202_setUVThres(unsigned int mV)
{
    ISL94202_writeEEPROMVoltage(0x04, mV, 1);
}
void ISL94202_setUVRecovery(unsigned int mV)
{
    ISL94202_writeEEPROMVoltage(0x06, mV, 0x00);
}
void ISL94202_setOVLockout(unsigned int mV)
{
    ISL94202_writeEEPROMVoltage(0x08, mV, 0x00);
}
void ISL94202_setUVLockout(unsigned int mV)
{
    ISL94202_writeEEPROMVoltage(0x0A, mV, 0x00);
}
void ISL94202_setEOCThreshold(unsigned int mV)
{
    ISL94202_writeEEPROMVoltage(0x0C, mV, 0x00);
}

unsigned int ISL94202_ReadOVThres()
{
    return ISL94202_readEEPROMWord(0x00) & 0x0FFF;
}
unsigned int ISL94202_ReadOVRecovery()
{
    return ISL94202_readEEPROMWord(0x02);
}
unsigned int ISL94202_ReadUVThres()
{
    return ISL94202_readEEPROMWord(0x04) & 0x0FFF;
}
unsigned int ISL94202_ReadUVRecovery()
{
    return ISL94202_readEEPROMWord(0x06);
}
unsigned int ISL94202_ReadOVLockout()
{
    return ISL94202_readEEPROMWord(0x08);
}
unsigned int ISL94202_ReadUVLockout()
{
    return ISL94202_readEEPROMWord(0x0A);
}
unsigned int ISL94202_ReadEOCThreshold()
{
    return ISL94202_readEEPROMWord(0x0C);
}
//Current Settings
/*
 * 0    4mV
 * 1    8mV
 * 2    16mV
 * 3    24mV
 * 4    32mV
 * 5    48mV
 * 6    64mV
 * 7    96mV
 */
void ISL94202_setDischargeOC(unsigned char mV, unsigned int time, unsigned char timeBase)
{
    //Threshold 000 is 4mV, each step ={4,8,16,24,32,48,64,96}
    //timescale = {us,ms,s,min}
    //default is 32mV,160ms
    if (mV > 7)
        return;
    ISL94202_writeEEPROMTimeout(0x16, time, timeBase, mV);
}
/*
 * 0    1mV
 * 1    2mV
 * 3    6mV
 * 4    8mV
 * 5    12mV
 * 6    16mV
 * 7    24mV
 */
void ISL94202_setChargeOC(unsigned char mV, unsigned int time, unsigned char timeBase)
{
    //Threshold 000 is 1mV, each step ={1,2,4,6,8,12,16,24}
    //timescale = {us,ms,s,min}
    //default is 8mV,160ms
    if (mV > 7)
        return;

    ISL94202_writeEEPROMTimeout(0x18, time, timeBase, mV);
}

/*
 * 0    16mV
 * 1    24mV
 * 2    32mV
 * 3    48mV
 * 4    64mV
 * 5    96mV
 * 6    128mV
 * 7    256mV
 */
void ISL94202_setintCircuit(unsigned char mV, unsigned int time, unsigned char timeBase)
{
    //Threshold 000 is 16mV, each step ={16,24,32,48,64,96,128,256}
    //timescale = {us,ms,s,min}
    //default is 128mV,200uS
    if (mV > 7)
        return;
    ISL94202_writeEEPROMTimeout(0x1A, time, timeBase, mV);
}

//Cell Balancing
void ISL94202_setCellBalanceTimes(unsigned char onTime, unsigned char offTime,
                                  unsigned char timeBase)
{
    //Sets on / off time for balancing cycle
    ISL94202_writeEEPROMTimeout(0x24, onTime, timeBase, 0);
    ISL94202_writeEEPROMTimeout(0x26, offTime, timeBase, 0);

}
void ISL94202_setCellCountSleepTimes(unsigned char cellCount, unsigned char idleModeTimeout,
                                     unsigned char sleepModeTimeout)
{
    //cellCount needs to be turned into the input bitmask
    unsigned char cellMask = 0b10000011;
    if (cellCount == 4)
        cellMask = 0b11000011;
    if (cellCount == 5)
        cellMask = 0b11000111;
    if (cellCount == 6)
        cellMask = 0b11100111;
    if (cellCount == 7)
        cellMask = 0b11101111;
    if (cellCount == 8)
        cellMask = 0b01111111;
    idleModeTimeout = idleModeTimeout & 0x0F;
    sleepModeTimeout = sleepModeTimeout & 0xF0;
    unsigned int output = cellMask << 8;
    output |= idleModeTimeout;
    output |= sleepModeTimeout;
    ISL94202_writeEEPROMWord(0x48, output);
}
void ISL94202_setCellBalanceDifference(unsigned int mV)
{
    //If the cells are closer than this the balancing stops
    //defaults to 20mV
    ISL94202_writeEEPROMVoltage(0x20, mV, 0x00);
}
void ISL94202_setCellBalanceStartV(unsigned int mV)
{
    ISL94202_writeEEPROMVoltage(0x1C, mV, 0x00);
}
void ISL94202_setCellBalanceStopV(unsigned int mV)
{
    ISL94202_writeEEPROMVoltage(0x1E, mV, 0x00);
}
void ISL94202_setCellBalanceFaultLevel(unsigned int mV)
{
    //If the difference is bigger than this its a fault and system gives up
    //default 500mV
    ISL94202_writeEEPROMVoltage(0x22, mV, 0x00);
}

void ISL94202_setFeature1(bool CellFActivatesPSD, bool XT2Mode, bool TGain,
bool PreChargeFETEnabled,
                          bool disableOpenWireScan,
                          bool OpenWireSetsPSD)
{
    unsigned char output = 0;
    output |= CellFActivatesPSD << 7;
    output |= XT2Mode << 5;
    output |= TGain << 4;
    output |= PreChargeFETEnabled << 2;
    output |= disableOpenWireScan << 1;
    output |= OpenWireSetsPSD;

    ISL94202_writeEEPROM(0x4A, output);
}
void ISL94202_setFeature2(bool CellBalanceDuringDischarge,
bool CellbalanceDuringCharge,
                          bool keepDFETonDuringCharge,
                          bool keepCFETonDuringDischarge,
                          bool shutdownOnUVLO,
                          bool enableBalanceAtEOC)
{
    unsigned char output = 0;
    output |= CellBalanceDuringDischarge ? 1 << 7 : 0;
    output |= CellbalanceDuringCharge ? 1 << 6 : 0;
    output |= keepDFETonDuringCharge ? 1 << 5 : 0;
    output |= keepCFETonDuringDischarge ? 1 << 4 : 0;
    output |= shutdownOnUVLO ? 1 << 3 : 0;
    output |= enableBalanceAtEOC ? 1 : 0;

    ISL94202_writeEEPROM(0x4B, output);
}

///////////////////////////////Support Functions \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


unsigned char ISL94202_readUserEEPROM(unsigned char address)
{
    volatile unsigned char r, r2;
    do
    {
        r = I2C_readReg8(ISLADDRESS, 0x50 + (address & 0x0F));
        __delay_cycles(1000);    //1ms
        r2 = I2C_readReg8(ISLADDRESS, 0x50 + (address & 0x0F));

    }
    while (r != r2);
    return r;
}
void ISL94202_writeUserEEPROM(unsigned char address, unsigned char value)
{
    ISL94202_writeEEPROM(0x50 + (address & 0x0F), value);
}
unsigned int ISL94202_milliVoltsToVScaleRaw(unsigned int mV)
{
    //float retvalue =((float)mV)*((4095*3)/(1.8*8));
    float retvalue = (((float) mV) / 1000.0) * 853.125;
    return retvalue;
}
unsigned int ISL94202_milliVScaleRawToVolts(unsigned int Raw)
{
    //float retvalue =((float)mV)*((4095*3)/(1.8*8));
    float retvalue = ((float) Raw) * 1.172;
    return retvalue;
}
void enableEEPROMAccess()
{
    //Set the bit to change to read/write EEPROM
    I2C_writeReg8(ISLADDRESS, 0x89, 0x01);
}
void disableEEPROMAccess()
{
    //Set the bit to change to read/write EEPROM
    I2C_writeReg8(ISLADDRESS, 0x89, 0x00);
}
void ISL94202_writeEEPROMTimeout(unsigned char add, unsigned int timeout,
                                 unsigned char timeScale, unsigned char headerFourBits)
{
    unsigned int outputValue = 0;
    outputValue = headerFourBits << 12;
    outputValue |= (timeout & 0x3FF);
    outputValue |= ((timeScale & 0x03) << 10);
    ISL94202_writeEEPROMWord(add, outputValue);
}
void ISL94202_writeEEPROMVoltage(unsigned char add, unsigned int mV,
                                 unsigned char headerFourBits)
{
    unsigned int outputValue = 0;
    outputValue = headerFourBits << 12;
    unsigned int calcValue = ISL94202_milliVoltsToVScaleRaw(mV);
    if (calcValue > 0x0FFF)
    {
        //Over Range
        return;
    }
    //else we are in range
    outputValue |= (calcValue & 0x0FFF);
    //Write out LSB in lowest address
    float reading = (unsigned int) (outputValue & 0x0FFF);

    reading *= (1.8 * 8);
    reading /= (4.095 * 3);

    ISL94202_writeEEPROMWord(add, outputValue);
}

unsigned int ISL94202_readEEPROMVoltage(unsigned char add)
{
    unsigned int outputValue = 0;
    unsigned int calcValue = 0;
    outputValue = ISL94202_readEEPROMWord(add);
    calcValue = ISL94202_milliVScaleRawToVolts(outputValue & 0x0FFF);
    return calcValue;
}

void ISL94202_writeEEPROMWord(unsigned char reg, unsigned int value)
{
    if ((reg > 0x58 && reg < 0x7F) || reg > 0xAB)
        return;
    //^ Datasheet mentions a warning not to write to these areas as they are for factory use

    enableEEPROMAccess();
    unsigned char buffer[4];    //We need to write in pages
    unsigned char i;
    unsigned char base = reg & 0xFC;
    buffer[0] = I2C_readReg8(ISLADDRESS, base);
    __delay_cycles(1000);        //delay to allow EEPROM refresh
    I2C_readMany(ISLADDRESS, base, 4, buffer);

    //We have read in the buffer
    //Update the buffer
    buffer[(unsigned char) (reg & 0x03)] = value;
    buffer[(unsigned char) (reg & 0x03) + 1] = value >> 8;

    I2C_writeReg8(ISLADDRESS, base, buffer[0]);
    __delay_cycles(35000);    //pause for EEPROM write

    //^Special case for first unsigned char causing EEPROM reload
    for (i = 0; i < 4; i++)
    {
        I2C_writeReg8(ISLADDRESS, base + i, buffer[i]);
        __delay_cycles(35000);    //35ms
        //pause for EEPROM write
    }
//    I2C_writeRegs(ISLADDRESS,base,buffer,4);
    disableEEPROMAccess();

}
//This reads the 4 byte page from the EEPROM, updates the given value, and then writes this page back to the unit
void ISL94202_writeEEPROM(unsigned char reg, unsigned char value)
{
    if ((reg > 0x58 && reg < 0x7F) || reg > 0xAB)
        return;
    //^ Datasheet mentions a warning not to write to these areas as they are for factory use

    enableEEPROMAccess();
    unsigned char buffer[4];    //We need to write in pages
    unsigned char i;
    unsigned char base = reg & 0xFC;
    buffer[0] = I2C_readReg8(ISLADDRESS, base);
    __delay_cycles(1000);        //delay to allow EEPROM refresh
    I2C_readMany(ISLADDRESS, base, 4, buffer);

    //We have read in the buffer
    //Update the buffer
    buffer[(unsigned char) (reg & 0x03)] = value;

    I2C_writeReg8(ISLADDRESS, base, buffer[0]);
    __delay_cycles(35000);        //delay to allow EEPROM write

    //^Special case for first unsigned char causing EEPROM reload
    for (i = 0; i < 4; i++)
    {
        I2C_writeReg8(ISLADDRESS, base + i, buffer[i]);
        __delay_cycles(35000);    //35ms
        //pause for EEPROM write
    }

//    I2C_writeRegs(ISLADDRESS,base,buffer,4);
    disableEEPROMAccess();
}

unsigned int ISL94202_readEEPROMWord(unsigned char reg)
{
    unsigned int value = 0;
    if ((reg > 0x58 && reg < 0x7F) || reg > 0xAB)
        return 0;
    //^ Datasheet mentions a warning not to write to these areas as they are for factory use

    enableEEPROMAccess();
    unsigned char buffer[4];    //We need to write in pages
//    unsigned char i;
    unsigned char base = reg & 0xFC;
    buffer[0] = I2C_readReg8(ISLADDRESS, base);
    __delay_cycles(1000);        //delay to allow EEPROM refresh
    I2C_readMany(ISLADDRESS, base, 4, buffer);

    //We have read in the buffer
    //Update the buffer

    value = buffer[(unsigned char) (reg & 0x03)];
    value = value + buffer[(unsigned char) (reg & 0x03) + 1] * 256;

    disableEEPROMAccess();

    return value;
}

unsigned char ISL94202_readEEPROM(unsigned char reg)
{
    unsigned char value = 0;
    if ((reg > 0x58 && reg < 0x7F) || reg > 0xAB)
        return 0;
    //^ Datasheet mentions a warning not to write to these areas as they are for factory use

    enableEEPROMAccess();
    unsigned char buffer[4];    //We need to write in pages
    unsigned char base = reg & 0xFC;
    buffer[0] = I2C_readReg8(ISLADDRESS, base);
    __delay_cycles(1000);        //delay to allow EEPROM refresh
    I2C_readMany(ISLADDRESS, base, 4, buffer);

    //We have read in the buffer
    //Update the buffer

    value = buffer[(unsigned char) (reg & 0x03)];

    disableEEPROMAccess();

    return value;
}

//Update the cell voltage and current readings from the unit
void ISL94202_updateReadings()
{
//read from 0x8A 22 bytes
    I2C_readMany(ISLADDRESS, 0x8A, 22, CurrentVoltageBuffer);
}

//Updates the buffer of the status registers
void ISL94202_updateStatus()
{
//Read from 0x80 5 bytes
    I2C_readMany(ISLADDRESS, 0x80, 5, CurrentStatusBuffer);
}

//returns the voltage measured at that cells input in millivolts
unsigned int ISL94202_getCellVoltageMV(unsigned char index)
{
    unsigned int r1 = CurrentVoltageBuffer[(index * 2) + 6];
    unsigned int r2 = CurrentVoltageBuffer[(index * 2) + 7] & 0x0F;
    unsigned int reading = (unsigned int) (r2 << 8 | r1);
    float value = 0;

    value = (float)reading * 32.0 / 27.0;
//    reading *= 32;
//    reading /= 27;
    //^ This is an integer approximation of the ideal 1.17216 (This one is 1.18)
    return value;

}

unsigned char ISL94202_getCurrentStatus(unsigned char index)
{
    return CurrentStatusBuffer[index];
}
//Returns the pack current in milliamps
unsigned int ISL94202_getPackCurrentMA(unsigned int divisor)
{
    unsigned int reading = CurrentVoltageBuffer[4]
            | ((CurrentVoltageBuffer[5] << 8) & 0x0F);
    float value = 0;
    value = (float)reading * 2.0 / 2047 / (float)divisor;
//    reading *= 2;
//    reading /= 2047;
//    reading /= divisor;        //to allow for calibration

    return reading;
}

//Returns a bitmask of the currently balancing cells
unsigned char ISL94202_getBalancingCells()
{
    return CurrentStatusBuffer[4];
}
