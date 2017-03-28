// Copyright 2016 Frederic Rible
// Algorithms derived form Toolerator3000 project
//
// This file is part of EATC-120, an Arduino based controller to drive an EmcoTurn 120/220 8 tools turret.
// 
// EATC-120 is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// EATC-120 is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with EATC-120.  If not, see <http://www.gnu.org/licenses/>.


#define noPinDefs

#include <ModbusRtu.h>
#include <plcLib.h>
#include <TimerOne.h>
#include <EEPROM.h>

#include "plcModbus.h"

// Modbus configuration

typedef struct {
	uint16_t version;
	uint16_t slaveId;
	uint16_t bitRateIndex;	// bitRate = 1200 * bitRateIndex 
	
	uint16_t forwardCurrent;
	uint16_t reverseCurrent;
	uint16_t lockCurrent;

	uint16_t forwardTimeout;
	uint16_t reverseTimeout;
	uint16_t ignoreOverCurrentTimeout;
	uint16_t forwardPawlTime;
	uint16_t masterTimeout;
} Config_t;

const Config_t defaultConfig = { 
	1234,			// version
	3,				// slaveId
	9600 / 1200,	// bitRateIndex

	3000,			// forwardCurrent;
	2500,			// reverseCurrent;
	800,			// lockCurrent;

	7000,			// forwardTimeout
	2000,			// reverseTimeout
	500,			// ignoreOverCurrentTimeout
	200,			// forwardPawlTime
	10000,			// masterTimeout, 0 means no timeout
};

Config_t config;

// HW pinout

const int PIN_MOTOR_DIR = 7;
const int PIN_BRAKE = 8;
const int PIN_M1 = 2;
const int PIN_M2 = 3;
const int PIN_M3 = 4;
const int PIN_M4 = 5;
const int PIN_OVER_CURRENT = 6;
const int PIN_SW2 = A0;
const int PIN_SW1 = A1;
const int PIN_S1 = 9;
const int PIN_S2 = 10;
const int PIN_S3 = 11;
const int PIN_S4 = 12;
const int PIN_LED = 13;
const int PIN_TXEN = A2;	// For modbus transceiver

// map inputs on D9-D12 to encoder position 1...8
const byte encoderToPosition[] = { 0,7,5,6,3,0,4,0,1,8,0,0,2,0,0,0 };

// Define states

const unsigned int ST_INIT = 1;
const unsigned int ST_WAIT = 2;
const unsigned int ST_FORW = 4;
const unsigned int ST_REV = 8;
const unsigned int ST_DONE = 16;
const unsigned int ST_ERROR = 32;

// Modbus mapping

Modbus *modbus;
uint16_t ModbusRegs[16] = { 0 };

// Status output
#define MB_READY		(ModbusAddr_t)0
#define MB_ERROR		(ModbusAddr_t)1
#define MB_TOOLCHANGED	(ModbusAddr_t)2

#define MB_S1			(ModbusAddr_t)4
#define MB_S2			(ModbusAddr_t)5
#define MB_S3			(ModbusAddr_t)6
#define MB_S4			(ModbusAddr_t)7

// Commands
#define MB_POWER_ON		(ModbusAddr_t)8
#define MB_TOOLCHANGE 	(ModbusAddr_t)9

#define MB_SAVE_CONFIG	(ModbusAddr_t)15

// int variables
#define MB_COILS		ModbusRegs[0]	// See previous declaration of coils and contacts
#define MB_COMMAND		ModbusRegs[1]
#define MB_POSITION		ModbusRegs[2]
#define MB_STATE		ModbusRegs[3]
#define MB_ERROR_CODE	ModbusRegs[4]
#define MB_CONFIG		ModbusRegs[5]	// First config word

// Error codes
#define ERROR_MODBUS		1	// The master has not polled the EATC120 since a long time
#define ERROR_SEARCH		2	// Target position cannot be reached in forward direction		
#define ERROR_LOCK			3	// Lock cannot be achieved
#define ERROR_POSITION		4	// Position read after lock is not consistant

// Internal variables
unsigned int targetPosition = 0;
unsigned int masterOK = 0;
unsigned int restoreFactoryDefaults = 0;
unsigned int saveConfig = 0;

// plclib extensions

extern unsigned int scanValue; 

unsigned int compare(unsigned int reg)
{
	scanValue = (scanValue == reg) ? 1 : 0;
	return scanValue;
}

unsigned int compareNot(unsigned int reg)
{
	scanValue = (scanValue != reg) ? 1 : 0;
	return scanValue;
}

unsigned int andCompareGT(unsigned int input, unsigned int value) {
	if (input <= value) {
		scanValue = 0;
	}
	return(scanValue);
}

// LMD18245 access

unsigned int setMotor(bool forward, uint16_t current_mA)
{
	if (!scanValue) return scanValue;

	// set direction
	if (forward) digitalWrite(PIN_MOTOR_DIR, 0);
	else digitalWrite(PIN_MOTOR_DIR, 1);

	if (current_mA > 3350) current_mA = 3350;
	unsigned int motorCurrent = (15 * current_mA) / 3350;

	// set current
	digitalWrite(PIN_M1, motorCurrent & 0x01);
	digitalWrite(PIN_M2, motorCurrent & 0x02);
	digitalWrite(PIN_M3, motorCurrent & 0x04);
	digitalWrite(PIN_M4, motorCurrent & 0x08);
	
	return scanValue;
}

unsigned int readPosition()
{
	unsigned int encoder = 0;
	encoder = digitalRead(PIN_S1);
	encoder += digitalRead(PIN_S2) << 1;
	encoder += digitalRead(PIN_S3) << 2;
	encoder += digitalRead(PIN_S4) << 3;
	return(encoderToPosition[encoder]);
}

unsigned int state(unsigned int var, unsigned int value)
{
	scanValue = (var & value) ? 1 : 0;
	return scanValue;
}

unsigned int setState(unsigned int &var, unsigned int value)
{
	if (scanValue) var = value;
	return scanValue;
}

unsigned int setValue(unsigned int &output, unsigned int value)
{
	if (scanValue) {
		output = value;
	}
	return scanValue;
}

// Logical section

Stack stack1;

void setup()
{
	Serial.begin(9600);

	// Read config from EEPROM, and restore default if invalid
	EEPROM.get(0, config);
	if (config.version != defaultConfig.version) {
		Serial.println("Loading factory default");
		config = defaultConfig;
		EEPROM.put(0, config);
	}
	memcpy(&MB_CONFIG, &config, sizeof(config));

	// Setup I/O pins

	pinMode(PIN_MOTOR_DIR, OUTPUT);
	pinMode(PIN_BRAKE, OUTPUT);
	pinMode(PIN_M1, OUTPUT);
	pinMode(PIN_M2, OUTPUT);
	pinMode(PIN_M3, OUTPUT);
	pinMode(PIN_M4, OUTPUT);
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW2, INPUT_PULLUP);
	pinMode(PIN_S1, INPUT_PULLUP);
	pinMode(PIN_S2, INPUT_PULLUP);
	pinMode(PIN_S3, INPUT_PULLUP);
	pinMode(PIN_S4, INPUT_PULLUP);
	pinMode(PIN_OVER_CURRENT, INPUT_PULLUP);
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_SW1, INPUT_PULLUP);

	modbus = new Modbus(config.slaveId, 0, PIN_TXEN);

	MB_STATE = ST_INIT;
	MB_POSITION = 0;

	digitalWrite(PIN_LED, 0);
	digitalWrite(PIN_BRAKE, 0);

	Timer1.initialize(10000L);
	Timer1.attachInterrupt(pollPLC);

	modbus->begin(config.bitRateIndex * 1200);
	Serial.println("Ready");
}

unsigned long TIMER_FORWARD = 0;
unsigned long TIMER_REVERSE = 0;
unsigned long TIMER_OVER_CURRENT_FORW = 0;
unsigned long TIMER_OVER_CURRENT_REV = 0;
unsigned long TIMER_PAWL = 0;
unsigned long TIMER_MASTER = 0;
unsigned long TIMER_FACTORY_DEFAULTS = 0;

Pulse pulseFactoryDefaults;
Pulse pulseSaveConfig;
Pulse pulseToolChange;
unsigned int pToolChange;

void loop()
{
	masterOK |= (modbus->poll(ModbusRegs, 16) > 4) ? 1 : 0;

	if (restoreFactoryDefaults) {
		config = defaultConfig;
		EEPROM.put(0, config);
		Serial.println("Restoring factory defaults");
		memcpy(&MB_CONFIG, &config, sizeof(config));
		restoreFactoryDefaults = 0;
	}

	if (saveConfig) {
		Serial.println("Saving config");
		memcpy(&config, &MB_CONFIG, sizeof(config));
		EEPROM.put(0, config);
		saveConfig = 0;
	}
}

unsigned int atTargetPosition()
{
	return (targetPosition == MB_POSITION) ? 1 : 0;
}

void pollPLC()
{
	MB_POSITION = readPosition();

	// *** Init state
	// Waiting for POWER_ON command

	state(MB_STATE, ST_INIT);
	setMotor(true, 0);
	reset(PIN_LED);
	setValue(MB_POSITION, 0);
	setValue(MB_ERROR_CODE, 0);
	andBit(MB_POWER_ON);
	setState(MB_STATE, ST_WAIT);

	// *** Wait state
	
	// Update status
	state(MB_STATE, ST_WAIT);
	out(MB_READY);
	setMotor(false, config.lockCurrent);
	set(PIN_LED);

	// Waiting for new tool change command

	in(MB_TOOLCHANGE);
	pulseToolChange.inClock();

	pulseToolChange.rising();
	out(pToolChange);

	state(MB_STATE, ST_WAIT);
	andBit(pToolChange);
	setValue(targetPosition, MB_COMMAND);
	andNotBit(atTargetPosition());
	setState(MB_STATE, ST_FORW);

	// We are already at position, give feedback
	state(MB_STATE, ST_WAIT);
	andBit(pToolChange);
	andBit(atTargetPosition());
	setState(MB_STATE, ST_DONE);

	// Go back to error state if we loose modbus communication
	state(MB_STATE, ST_WAIT);
	andNotBit(masterOK);
	andCompareGT(config.masterTimeout, 0);
	timerOn(TIMER_MASTER, config.masterTimeout);
	setValue(MB_COILS, 0);	// We have lost master, set commands to 0
	setValue(MB_ERROR_CODE, ERROR_MODBUS);
	setState(MB_STATE, ST_ERROR);

	// *** Forward state

	// Check if the motor is not blocked
	// Leave enough time for motor start (high current during startup)

	state(MB_STATE, ST_FORW);
	setMotor(true, config.forwardCurrent);
	//timerOn(TIMER_OVER_CURRENT_FORW, IGNORE_OVER_CURRENT_TIMEOUT);
	//andBit(PIN_OVER_CURRENT);
	//setState(MB_STATE, ST_ERROR);

	// Error if the motion take too much time

	state(MB_STATE, ST_FORW);;
	timerOn(TIMER_FORWARD, config.forwardTimeout);
	setValue(MB_ERROR_CODE, ERROR_SEARCH);
	setState(MB_STATE, ST_ERROR);

	// Check if reached target position
	// Keep moving for a small amount of time to let the pawl fall in the hole

	state(MB_STATE, ST_FORW);
	andBit(atTargetPosition());
	timerOn(TIMER_PAWL, config.forwardPawlTime);
	setState(MB_STATE, ST_REV);

	// *** Reverse state

	// Move until we can detect the motor is blocked
	// Leave enough time for motor start (high current during startup)

	state(MB_STATE, ST_REV);
	setMotor(false, config.reverseCurrent);
	timerOn(TIMER_OVER_CURRENT_REV, config.ignoreOverCurrentTimeout);	// Leave enough time for motor start
	andBit(PIN_OVER_CURRENT);
	setState(MB_STATE, ST_DONE);

	// Error if the motion take too much time

	state(MB_STATE, ST_REV);
	timerOn(TIMER_REVERSE, config.reverseTimeout);
	setValue(MB_ERROR_CODE, ERROR_LOCK);
	setState(MB_STATE, ST_ERROR);

	// *** DONE state

	// Check if we are at commanded position
	state(MB_STATE, ST_DONE);
	andNotBit(atTargetPosition());
	setValue(MB_ERROR_CODE, ERROR_POSITION);
	setState(MB_STATE, ST_ERROR);

	// Waiting for toolchange command disable
	state(MB_STATE, ST_DONE);
	out(MB_TOOLCHANGED);
	setMotor(false, config.lockCurrent);
	andNotBit(MB_TOOLCHANGE);
	setState(MB_STATE, ST_WAIT);

	// *** Error state

	state(MB_STATE, ST_ERROR);
	out(MB_ERROR);
	setMotor(true, 0);
	setValue(MB_POSITION, 0);

	// Manage power off

	inNot(MB_POWER_ON);
	setState(MB_STATE, ST_INIT);

	// Copy position encoder input for debug

	in(PIN_S1);
	out(MB_S1);
	
	in(PIN_S2);
	out(MB_S2);
	
	in(PIN_S3);
	out(MB_S3);
	
	in(PIN_S4);
	out(MB_S4);

	// Manage the SW1 button

	inNot(PIN_SW1);
	timerOn(TIMER_FACTORY_DEFAULTS, 3000);
	pulseFactoryDefaults.inClock();

	pulseFactoryDefaults.rising();
	set(restoreFactoryDefaults);

	// Manage config

	in(MB_SAVE_CONFIG);
	pulseSaveConfig.inClock();

	pulseSaveConfig.rising();
	set(saveConfig);

	masterOK = false;
}
