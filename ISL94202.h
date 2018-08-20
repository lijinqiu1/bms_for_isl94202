/*
 * ISL94202.h
 *
 *  Created on: 12 June ,2017
 *      Author: Ben V. Brown
 *
 *      Driver for the ISL94202 BMS IC
 *      Uses i2c.* for handling the I2C communications
 *
 *      Provides access to both live readings and also the ability to change settings on the unit
 *
 */

#ifndef ISL94202_H_
#define ISL94202_H_
#include <stdbool.h>
#include "iic.h"

#define ISLADDRESS 0x28

#define ISL_OVTHRES            4500
#define ISL_OVRECEOVERY        4500
#define ISL_UVTHRES            1500
#define ISL_UVRECEOVERY        1500
#define ISL_OVLOCKOUT          4500
#define ISL_UVLOCKOUT          1500
#define ISL_EOCTHRESHOLD       4500
#define ISL_CELLBALANCESTARTV  4200
#define ISL_CELLBALANCESTOP    4400
#define ISL_CELLBALANCELEVEL    500

#define ISL_setFeature1(CellFActivatesPSD, XT2Mode, TGain, PreChargeFETEnabled, disableOpenWireScan, OpenWireSetsPSD) \
    (unsigned char)((CellFActivatesPSD ? 1 << 7 : 0)|\
            (XT2Mode ? 1 << 5 : 0)|\
            (TGain ? 1 << 4 : 0)|\
            (PreChargeFETEnabled ? 1 << 2 : 0)|\
            (disableOpenWireScan ? 1 << 1 : 0)|\
            (OpenWireSetsPSD ? 1 : 0))

#define ISL_setFeature2(CellBalanceDuringDischarge, CellbalanceDuringCharge, keepDFETonDuringCharge, keepCFETonDuringDischarge,shutdownOnUVLO, enableBalanceAtEOC) \
    (unsigned char)((CellBalanceDuringDischarge ? 1 << 7 : 0)|\
            (CellbalanceDuringCharge ? 1 << 6 : 0)|\
            (keepDFETonDuringCharge ? 1 << 5 : 0)|\
            (keepCFETonDuringDischarge ? 1 << 4 : 0)|\
            (shutdownOnUVLO ? 1 << 3 : 0)|\
            (enableBalanceAtEOC ? 1 : 0))

//Update the cell voltage and current readings from the unit
void ISL94202_updateReadings();
//Updates the buffer of the status registers
void ISL94202_updateStatus();

//returns the voltage measured at that cells input in millivolts
unsigned int ISL94202_getCellVoltageMV(unsigned char index);

//Returns the pack current in milliamps
unsigned int ISL94202_getPackCurrentMA();

//Returns a bitmask of the currently balancing cells
unsigned char ISL94202_getBalancingCells();

unsigned char ISL94202_getCurrentStatus(unsigned char index);

unsigned int ISL94202_getCurrentTemperature(unsigned char index);

enum
{
    DischargeOC_4mV  = 0x00,
    DischargeOC_8mV  = 0x01,
    DischargeOC_16mV = 0x02,
    DischargeOC_24mV = 0x03,
    DischargeOC_32mV = 0x04,
    DischargeOC_48mV = 0x05,
    DischargeOC_64mV = 0x06,
    DischargeOC_96mV = 0x07,
};

enum
{
    Time_Base_us  = 0x00,
    Time_Base_ms  = 0x01,
    Time_Base_s   = 0x02,
    Time_Base_min = 0x03,
};
/////////////////////////////////// SETTINGS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//Thresholds and releases
void ISL94202_setOVLockout(unsigned int mV);
void ISL94202_setOVThres(unsigned int mV);
void ISL94202_setOVRecovery(unsigned int mV);
void ISL94202_setUVThres(unsigned int mV);
void ISL94202_setUVRecovery(unsigned int mV);
void ISL94202_setUVLockout(unsigned int mV);
void ISL94202_setEOCThreshold(unsigned int mV);
unsigned int ISL94202_ReadOVThres();
unsigned int ISL94202_ReadOVRecovery();
unsigned int ISL94202_ReadUVThres();
unsigned int ISL94202_ReadUVRecovery();
unsigned int ISL94202_ReadOVLockout();
unsigned int ISL94202_ReadUVLockout();
unsigned int ISL94202_ReadEOCThreshold();
//Current Settings
void ISL94202_setDischargeOC(unsigned char mV, unsigned int time, unsigned char timeBase);
void ISL94202_setChargeOC(unsigned char mV, unsigned int time, unsigned char timeBase);
void ISL94202_setintCircuit(unsigned char mV, unsigned int time, unsigned char timeBase);

//Cell Balancing
void ISL94202_setCellBalanceTimes(unsigned char onTime, unsigned char offTime,
                                  unsigned char timeBase);
void ISL94202_setCellCountSleepTimes(unsigned char cellCount, unsigned char idleSleep,
                            unsigned char deepsleep);
void ISL94202_setCellBalanceDifference(unsigned int mV);
void ISL94202_setCellBalanceStartV(unsigned int mV);
void ISL94202_setCellBalanceStopV(unsigned int mV);
void ISL94202_setCellBalanceFaultLevel(unsigned int mV);

unsigned int ISL94202_ReadCellBalanceDifference();
unsigned int ISL94202_ReadCellBalanceStartV();
unsigned int ISL94202_ReadCellBalanceStopV();
unsigned int ISL94202_ReadCellBalanceFaultLevel();

unsigned int ISL94202_milliVoltsToVScaleRaw(unsigned int mV);
//Chip Features Control
void ISL94202_setFeature1(bool CellFActivatesPSD, bool XT2Mode, bool TGain,
                 bool PreChargeFETEnabled, bool disableOpenWireScan,
                 bool OpenWireSetsPSD);

void ISL94202_setFeature2(bool CellBalanceDuringDischarge,
                          bool CellbalanceDuringCharge,
                 bool keepDFETonDuringCharge, bool keepCFETonDuringDischarge,
                 bool shutdownOnUVLO, bool enableBalanceAtEOC);

//////////////////////// EEPROM \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

void ISL94202_writeUserEEPROM(unsigned char address, unsigned char value);
unsigned char ISL94202_readUserEEPROM(unsigned char address);

//////////////////////// I2C Support  \\\\\\\\\\\\\\\\\\\\\\\\\\\\\

void ISL94202_writeEEPROM(unsigned char reg, unsigned char value);
void ISL94202_writeEEPROMWord(unsigned char reg, unsigned int value);
unsigned int ISL94202_readEEPROMWord(unsigned char reg);
unsigned char ISL94202_readEEPROM(unsigned char reg);
void ISL94202_writeEEPROMVoltage(unsigned char add, unsigned int mV,
                                 unsigned char headerFourBits);
void ISL94202_writeEEPROMTimeout(unsigned char add, unsigned int timeout,
                                 unsigned char timeScale,
                        unsigned char headerFourBits);

void ISL94202_Init();
#endif /* ISL94202_H_ */
