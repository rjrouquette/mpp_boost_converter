//
// Created by robert on 11/16/20.
//

#ifndef AVR_MPPBC_REGULATOR_H
#define AVR_MPPBC_REGULATOR_H

#include <stdint.h>

void initRegulator();
void setOutputVoltage(uint8_t target);
void setInputVoltage(uint8_t target);
uint8_t getOutputVoltage();
uint8_t getInputVoltage();
uint16_t measureOutputVoltage();
uint16_t measureInputVoltage();

#endif //AVR_MPPBC_REGULATOR_H
