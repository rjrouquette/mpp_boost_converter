//
// Created by robert on 11/16/20.
//

#ifndef AVR_MPPBC_REGULATOR_H
#define AVR_MPPBC_REGULATOR_H

#include <stdint.h>
#include <stdbool.h>

void initRegulator();
void updateRegulatorPulseWidth();
void enableRegulator(bool enabled);
void setOutputVoltage(uint16_t limit);
void setInputVoltage(uint16_t limit);
uint16_t getOutputVoltage();
uint16_t getOutputVoltageStep();
uint16_t getInputVoltage();
uint16_t getInputVoltageStep();
uint16_t measureOutputVoltage();
uint16_t measureInputVoltage();
uint16_t measureFanSpeed();

#endif //AVR_MPPBC_REGULATOR_H
