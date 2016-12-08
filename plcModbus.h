#pragma once

#include <Arduino.h>

extern uint16_t ModbusRegs[8];

typedef unsigned char ModbusAddr_t;

extern unsigned int scanValue;

unsigned int in(ModbusAddr_t reg)
{
	scanValue = bitRead(ModbusRegs[reg/16], reg);
	return scanValue;
}

unsigned int inNot(ModbusAddr_t reg)
{
	scanValue = bitRead(ModbusRegs[reg / 16], reg) ? 0 : 1;
	return scanValue;
}

unsigned int out(ModbusAddr_t reg)
{
	bitWrite(ModbusRegs[reg / 16], reg, scanValue);
	return(scanValue);
}

unsigned int set(ModbusAddr_t reg) {
	scanValue = scanValue | bitRead(ModbusRegs[reg / 16], reg);		// Self latch by ORing with Output pin
	if (scanValue == 1) {
		bitSet(ModbusRegs[reg / 16], reg);
	}
	return(scanValue);
}

unsigned int reset(ModbusAddr_t reg) {
	if (scanValue == 1) {
		bitClear(ModbusRegs[reg / 16], reg);
	}
	return(scanValue);
}

unsigned int andBit(ModbusAddr_t reg) {
	scanValue = scanValue & bitRead(ModbusRegs[reg / 16], reg);;
	return(scanValue);
}

unsigned int andNotBit(ModbusAddr_t reg) {
	scanValue = scanValue & ~bitRead(ModbusRegs[reg / 16], reg);;
	return(scanValue);
}