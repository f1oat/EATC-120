#pragma once

#include <Arduino.h>

extern uint16_t ModbusRegs[8];

typedef struct {
	int addr;
} ModbusAddr_t;

extern unsigned int scanValue;

unsigned int in(ModbusAddr_t reg)
{
	scanValue = bitRead(ModbusRegs[0], reg.addr);
	return scanValue;
}

unsigned int inNot(ModbusAddr_t reg)
{
	scanValue = bitRead(ModbusRegs[0], reg.addr) ? 0 : 1;
	return scanValue;
}

unsigned int out(ModbusAddr_t reg)
{
	bitWrite(ModbusRegs[0], reg.addr, scanValue);
	return(scanValue);
}

unsigned int set(ModbusAddr_t reg) {
	scanValue = scanValue | bitRead(ModbusRegs[0], reg.addr);		// Self latch by ORing with Output pin
	if (scanValue == 1) {
		bitSet(ModbusRegs[0], reg.addr);
	}
	return(scanValue);
}

unsigned int reset(ModbusAddr_t reg) {
	if (scanValue == 1) {
		bitClear(ModbusRegs[0], reg.addr);
	}
	return(scanValue);
}

unsigned int andBit(ModbusAddr_t reg) {
	scanValue = scanValue & bitRead(ModbusRegs[0], reg.addr);;
	return(scanValue);
}

unsigned int andNotBit(ModbusAddr_t reg) {
	scanValue = scanValue & ~bitRead(ModbusRegs[0], reg.addr);;
	return(scanValue);
}