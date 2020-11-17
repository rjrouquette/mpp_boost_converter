//
// Created by robert on 11/16/20.
//

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include "regulator.h"

#define ENABLE_PORT PORTB
#define ENABLE_PIN (0x01u)

#define COMPARATOR_PORT PORTA
#define COMPARATOR_OUT (0xc0u)
#define COMPARATOR_IN (0x03u)

void initRegulator() {
    // configure regulator enable pin
    ENABLE_PORT.DIRSET = ENABLE_PIN;
    ENABLE_PORT.OUTCLR = ENABLE_PIN;

    // configure comparator output (wired and)
    COMPARATOR_PORT.PIN6CTRL = 0x2bu;   // wired-and, sense level
    COMPARATOR_PORT.PIN7CTRL = 0x68u;   // wired-and, inverted
    COMPARATOR_PORT.DIRSET = COMPARATOR_OUT;
    COMPARATOR_PORT.OUTCLR = COMPARATOR_OUT;

    // configure analog comparators
    COMPARATOR_PORT.DIRCLR = COMPARATOR_IN;
    COMPARATOR_PORT.PIN0CTRL = 0x07u;
    COMPARATOR_PORT.PIN1CTRL = 0x07u;

    ACA.AC0MUXCTRL = 0x05u; // pin 0 vs DAC0
    ACA.AC1MUXCTRL = 0x0fu; // pin 1 vs Vcc Scaler
    ACA.CTRLA = 0x03u;      // enable comparator outputs
    ACA.AC0CTRL = 0x01u;    // enable comparator 0
    ACA.AC1CTRL = 0x01u;    // enable comparator 1

    // configure DACB
    DACB.CH0OFFSETCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB0OFFCAL));
    DACB.CH0GAINCAL   = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB0GAINCAL));
    DACB.CTRLC = DAC_REFSEL_AVCC_gc;
    DACB.CTRLB = DAC_CHSEL_SINGLE_gc;
    DACB.CTRLA = DAC_IDOEN_bm | DAC_CH0EN_bm | DAC_ENABLE_bm;


    // configure ADCA
    ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;
    ADCA.EVCTRL = 0x00;
    ADCA.REFCTRL = ADC_REFSEL_INTVCC2_gc; // Vcc / 2
    ADCA.CTRLB = ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc; // Signed 12-bit right-adjusted
    NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
    ADCA.CALL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
    ADCA.CALH = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
    NVM.CMD = 0x00;

    // Channel 0 (pin 0)
    ADCA.CH0.INTCTRL = 0x00u; // no interrupt
    ADCA.CH0.CTRL = ADC_CH_GAIN_DIV2_gc | ADC_CH_INPUTMODE_DIFFWGAIN_gc;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc | ADC_CH_MUXNEG_INTGND_MODE4_gc;

    // Channel 1 (pin 1)
    ADCA.CH1.INTCTRL = 0x00u; // no interrupt
    ADCA.CH1.CTRL = ADC_CH_GAIN_DIV2_gc | ADC_CH_INPUTMODE_DIFFWGAIN_gc;
    ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc | ADC_CH_MUXNEG_INTGND_MODE4_gc;

    // Enable ADC
    ADCA.CTRLA = 0x01;
}

void enableRegulator(bool enabled) {
    if(enabled)
        ENABLE_PORT.OUTSET = ENABLE_PIN;
    else
        ENABLE_PORT.OUTCLR = ENABLE_PIN;
}

void setOutputVoltage(uint16_t limit) {
    // safety check (190.00 V)
    if(limit > 19000u) limit = 19000u;

    uint32_t tmp = limit;
    tmp <<= 6u;
    tmp += 11750u;
    tmp /= 23500u;
    uint8_t scalefac = (uint8_t) tmp;

    if(scalefac > 64u) scalefac = 64u;
    else if(scalefac < 1u) scalefac = 1u;
    ACA.CTRLB = scalefac - 1u;
}

void setInputVoltage(uint16_t limit) {
    // safety check (100.00 V)
    if(limit > 10000u) limit = 10000u;

    uint32_t tmp = limit;
    tmp *= 4095u;
    tmp += 5431u;
    tmp /= 10862u;
    DACB.CH0DATA = (uint16_t) tmp;
}

uint16_t getOutputVoltage() {
    uint32_t tmp = ACA.CTRLB & 0x3fu;
    tmp += 1u;
    tmp *= 23500u;
    tmp += 32u;
    tmp /= 64u;
    return (uint16_t) tmp;
}

uint16_t getOutputVoltageStep() {
    return (23500u + 32u) / 64u;
}

uint16_t getInputVoltage() {
    uint32_t tmp = DACB.CH0DATA & 0xfffu;
    tmp *= 10862u;
    tmp += 2047u;
    tmp /= 4095u;
    return (uint16_t) tmp;
}

uint16_t getInputVoltageStep() {
    return (10862u + 2047u) / 4095u;
}

uint16_t measureOutputVoltage() {
    ADCA.CH1.CTRL |= ADC_CH_START_bm;
    while(!(ADCA.CH1.INTFLAGS & 0x01u));
    ADCA.CH1.INTFLAGS = 0x01u;

    uint32_t tmp = ADCA.CH1.RES & 0xfffu;
    if(tmp & 0x800u) tmp = 0;
    tmp *= 23500u;
    tmp += 1023u;
    tmp /= 2047u;
    return (uint16_t) tmp;
}

uint16_t measureInputVoltage() {
    ADCA.CH0.CTRL |= ADC_CH_START_bm;
    while(!(ADCA.CH0.INTFLAGS & 0x01u));
    ADCA.CH0.INTFLAGS = 0x01u;

    uint32_t tmp = ADCA.CH0.RES & 0xfffu;
    if(tmp & 0x800u) tmp = 0;
    tmp *= 10862u;
    tmp += 1023u;
    tmp /= 2047u;
    return (uint16_t) tmp;
}

