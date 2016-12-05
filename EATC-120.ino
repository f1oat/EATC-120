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

// Modbus configuration

#define MODBUS_SLAVE_ID 3
#define MODBUS_BITRATE 9600

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

const unsigned int ST_INIT = 0;
const unsigned int ST_WAIT = 1;
const unsigned int ST_FORW = 2;
const unsigned int ST_REV = 3;
const unsigned int ST_DONE = 4;
const unsigned int ST_ERROR = 5;

// Modbus mapping

Modbus modbus(MODBUS_SLAVE_ID, 0, PIN_TXEN);
uint16_t ModbusRegs[8] = { 0 };

// Status output
#define MB_READY		0
#define MB_ERROR		1
#define MB_TOOLCHANGED	2

#define MB_S1			4
#define MB_S2			5
#define MB_S3			6
#define MB_S4			7

// Commands
#define MB_POWER_ON		8
#define MB_TOOLCHANGE 	9

// int variables
#define MB_COILS	ModbusRegs[0]	// See previous declaration of coils and contacts
#define MB_COMMAND	ModbusRegs[1]
#define MB_POSITION	ModbusRegs[2]
#define MB_STATE	ModbusRegs[3]
//#define MB_ENCODER	ModbusRegs[4]

unsigned int targetPosition = 0;
unsigned int masterOK = 0;

// Modbus access function

extern unsigned int scanValue;

unsigned int inModbus(int reg)
{
	scanValue = bitRead(MB_COILS, reg);
	return scanValue;
}

unsigned int inNotModbus(int reg)
{
	scanValue = bitRead(MB_COILS, reg) ? 0 : 1;
	return scanValue;
}

unsigned int outModbus(int reg) 
{
	bitWrite(MB_COILS, reg, scanValue);
	return(scanValue);
}

unsigned int setModbus(int reg) {
	scanValue = scanValue | bitRead(MB_COILS, reg);		// Self latch by ORing with Output pin
	if (scanValue == 1) {
		bitSet(ModbusRegs[0], reg);
	}
	return(scanValue);
}

unsigned int resetModbus(int reg) {
	if (scanValue == 1) {
		bitClear(MB_COILS, reg);
	}
	return(scanValue);
}

unsigned int andModbus(int reg) {
	scanValue = scanValue & bitRead(MB_COILS, reg);;
	return(scanValue);
}

unsigned int andNotModbus(int reg) {
	scanValue = scanValue & ~bitRead(MB_COILS, reg);;
	return(scanValue);
}

// plclib extensions

unsigned int compare(unsigned int reg)
{
	scanValue = (scanValue == reg) ? 1 : 0;
	return scanValue;
}

unsigned int compareNEG(unsigned int reg)
{
	scanValue = (scanValue != reg) ? 1 : 0;
	return scanValue;
}

// LMD18245 access

unsigned int setMotor(bool forward, float current)
{
	if (!scanValue) return scanValue;

	// set direction
	if (forward) digitalWrite(PIN_MOTOR_DIR, 0);
	else digitalWrite(PIN_MOTOR_DIR, 1);

	unsigned int motorCurrent = current / 3.35 * 15;

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
	scanValue = (var == value) ? 1 : 0;
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

	modbus.begin(MODBUS_BITRATE);

	MB_STATE = ST_INIT;
	MB_POSITION = 0;

	digitalWrite(PIN_LED, 0);
	digitalWrite(PIN_BRAKE, 0);

	Timer1.initialize(10000L);
	Timer1.attachInterrupt(pollPLC);
}

unsigned long TIMER_FORWARD = 0;
unsigned long TIMER_REVERSE = 0;
unsigned long TIMER_OVER_CURRENT_FORW = 0;
unsigned long TIMER_OVER_CURRENT_REV = 0;
unsigned long TIMER_PAWL = 0;
unsigned long TIMER_MASTER = 0;

#define FORWARD_TIMEOUT 7000
#define REVERSE_TIMEOUT 2000
#define IGNORE_OVER_CURRENT_TIMEOUT 500
#define FORWARD_PAWL_TIME 200
#define MASTER_TIMEOUT 1000

void loop()
{
	masterOK = (modbus.poll(ModbusRegs, 8) > 4) ? 1 : 0;
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
	setMotor(true, 0.0);
	reset(PIN_LED);
	setValue(MB_POSITION, 0);
	andModbus(MB_POWER_ON);
	setState(MB_STATE, ST_WAIT);

	// *** Wait state
	
	// Update status
	state(MB_STATE, ST_WAIT);
	outModbus(MB_READY);
	setMotor(false, 0.8);
	set(PIN_LED);

	// Waiting for new tool change command

	state(MB_STATE, ST_WAIT);
	andModbus(MB_TOOLCHANGE);
	setValue(targetPosition, MB_COMMAND);

	state(MB_STATE, ST_WAIT);
	andModbus(MB_TOOLCHANGE);
	andNotBit(atTargetPosition());
	setState(MB_STATE, ST_FORW);

	// We are already at position, give feedback
	state(MB_STATE, ST_WAIT);
	andModbus(MB_TOOLCHANGE);
	andBit(atTargetPosition());
	setState(MB_STATE, ST_DONE);

	// Go back to init state if we loose modbus communication
	//state(MB_STATE, ST_WAIT);
	//andNotBit(MasterOK);
	//timerOn(TIMER_MASTER, MASTER_TIMEOUT);
	//setValue(MB_COILS, 0);	// We have lost master, set commands to 0
	//setState(MB_STATE, ST_INIT);

	// *** Forward state

	// Check if the motor is not blocked
	// Leave enough time for motor start (high current during startup)

	state(MB_STATE, ST_FORW);
	setMotor(true, 3.0);
	//timerOn(TIMER_OVER_CURRENT_FORW, IGNORE_OVER_CURRENT_TIMEOUT);
	//andBit(PIN_OVER_CURRENT);
	//setState(MB_STATE, ST_ERROR);

	// Error if the motion take too much time

	state(MB_STATE, ST_FORW);;
	timerOn(TIMER_FORWARD, FORWARD_TIMEOUT);
	setState(MB_STATE, ST_ERROR);

	// Check if reached target position
	// Keep moving for a small amount of time to let the pawl fall in the hole

	state(MB_STATE, ST_FORW);
	andBit(atTargetPosition());
	timerOn(TIMER_PAWL, FORWARD_PAWL_TIME);
	setState(MB_STATE, ST_REV);

	// *** Reverse state

	// Move until we can detect the motor is blocked
	// Leave enough time for motor start (high current during startup)

	state(MB_STATE, ST_REV);
	setMotor(false, 2.0);
	timerOn(TIMER_OVER_CURRENT_REV, IGNORE_OVER_CURRENT_TIMEOUT);	// Leave enough time for motor start
	andBit(PIN_OVER_CURRENT);
	setState(MB_STATE, ST_DONE);

	// Error if the motion take too much time

	state(MB_STATE, ST_REV);
	timerOn(TIMER_REVERSE, REVERSE_TIMEOUT);
	setState(MB_STATE, ST_ERROR);

	// *** DONE state

	// Check if we are at commanded position
	state(MB_STATE, ST_DONE);
	andNotBit(atTargetPosition());
	setState(MB_STATE, ST_ERROR);

	// Waiting for toolchange command disable
	state(MB_STATE, ST_DONE);
	outModbus(MB_TOOLCHANGED);
	setMotor(false, 0.8);
	andNotModbus(MB_TOOLCHANGE);
	setState(MB_STATE, ST_WAIT);

	// *** Error state

	state(MB_STATE, ST_ERROR);
	outModbus(MB_ERROR);
	setMotor(true, 0.0);
	setValue(MB_POSITION, 0);

	// Manage power off

	inNotModbus(MB_POWER_ON);
	setState(MB_STATE, ST_INIT);

	// Copy position encoder input for debug

	in(PIN_S1);
	outModbus(MB_S1);
	
	in(PIN_S2);
	outModbus(MB_S2);
	
	in(PIN_S3);
	outModbus(MB_S3);
	
	in(PIN_S4);
	outModbus(MB_S4);
}
