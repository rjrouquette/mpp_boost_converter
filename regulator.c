//
// Created by robert on 11/16/20.
//

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include "regulator.h"

#define ENABLE_PORT PORTB
#define ENABLE_PIN (0x01u)

#define COMPARATOR_PORT PORTA
#define COMPARATOR_OUT (0xc0u)
#define COMPARATOR_IN (0x38u)

volatile uint16_t fanSpeed;

static volatile struct {
    uint32_t acc;
    uint16_t res;
    uint16_t raw;
    uint8_t cnt;
    uint8_t rdy;
} adcFilter[2];

void initRegulator() {
    // configure regulator enable pin
    ENABLE_PORT.DIRSET = ENABLE_PIN;
    ENABLE_PORT.OUTCLR = ENABLE_PIN;

    // configure comparator output (wired and)
    COMPARATOR_PORT.PIN7CTRL = 0x68u;   // wired-and, inverted (ACA0)
    COMPARATOR_PORT.PIN6CTRL = 0x2bu;   // wired-and, sense level (ACA1)
    COMPARATOR_PORT.DIRSET = COMPARATOR_OUT;
    COMPARATOR_PORT.OUTCLR = COMPARATOR_OUT;

    // configure analog comparators
    COMPARATOR_PORT.DIRCLR = COMPARATOR_IN;
    COMPARATOR_PORT.PIN3CTRL = 0x07u;
    COMPARATOR_PORT.PIN4CTRL = 0x07u;
    COMPARATOR_PORT.PIN5CTRL = 0x07u;

    ACA.AC0MUXCTRL = AC_MUXPOS_PIN3_gc | AC_MUXNEG_DAC_gc; // pin PA3 vs DAC0 (internal)
    ACA.AC1MUXCTRL = AC_MUXPOS_PIN4_gc | AC_MUXNEG_PIN5_gc; // pin PA4 vs DAC1 (pin PA5)
    ACA.CTRLA = 0x03u;      // enable comparator outputs
    ACA.AC0CTRL = 0x01u;    // enable comparator 0
    ACA.AC1CTRL = 0x01u;    // enable comparator 1

    // configure DACB
    PORTB.DIRSET = PIN3_bm;
    PORTB.PIN3CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
    NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
    DACB.CH0OFFSETCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB0OFFCAL));
    DACB.CH0GAINCAL   = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB0GAINCAL));
    DACB.CH1OFFSETCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB1OFFCAL));
    DACB.CH1GAINCAL   = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB1GAINCAL));
    NVM.CMD = 0x00;
    DACB.CTRLC = DAC_REFSEL_AREFA_gc; // Vcc (3.3V)
    DACB.CTRLB = DAC_CHSEL_DUAL_gc;
    DACB.CTRLA = DAC_IDOEN_bm | DAC_CH0EN_bm| DAC_CH1EN_bm | DAC_ENABLE_bm;


    // configure event 0 for RTC (1.024 kHz)
    EVSYS.CH0MUX = EVSYS_CHMUX_RTC_OVF_gc;

    // configure ADCA for sweep on event 0
    ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;
    ADCA.EVCTRL = ADC_SWEEP_01_gc | ADC_EVACT_SWEEP_gc | ADC_EVSEL_0123_gc;
    ADCA.REFCTRL = ADC_REFSEL_AREFA_gc; // Vcc (3.3V)
    ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc; // Unsigned 12-bit right-adjusted
    NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
    ADCA.CALL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
    ADCA.CALH = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));
    NVM.CMD = 0x00;

    // Channel 0 (pin 0)
    ADCA.CH0.INTCTRL = ADC_CH_INTLVL_MED_gc;
    ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;

    // Channel 1 (pin 1)
    ADCA.CH1.INTCTRL = ADC_CH_INTLVL_MED_gc;
    ADCA.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc;

    // initialize ADC accumulators
    adcFilter[0].acc = 0;
    adcFilter[0].res = 0;
    adcFilter[0].raw = 0;
    adcFilter[0].cnt = 0;
    adcFilter[0].rdy = 0;
    adcFilter[1].acc = 0;
    adcFilter[1].res = 0;
    adcFilter[1].raw = 0;
    adcFilter[1].cnt = 0;
    adcFilter[1].rdy = 0;

    // Enable ADC
    ADCA.CTRLA = 0x01;

    // configure TCC0 for measuring fan speed
    fanSpeed = 0;
    PORTB.DIRCLR = PIN2_bm;
    PORTB.PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;
    EVSYS.CH1MUX = EVSYS_CHMUX_PORTB_PIN2_gc;
    TCC0.PER = 0xffffu;
    TCC0.CTRLD = TC_EVACT_FRQ_gc | TC_EVSEL_CH1_gc;
    TCC0.CTRLB = TC0_CCAEN_bm;
    TCC0.CTRLA = TC_CLKSEL_DIV1024_gc;
    TCC0.INTCTRLB = TC_CCAINTLVL_LO_gc;

    // configure PWM outputs
    PORTD.DIRSET = PIN5_bm;
    PORTE.DIRSET = PIN3_bm;
    EVSYS.CH2MUX = EVSYS_CHMUX_TCE0_CCA_gc;
    TCE0.CTRLB = TC0_CCDEN_bm | TC_WGMODE_SINGLESLOPE_gc;
    TCD1.CTRLB = TC1_CCBEN_bm | TC_WGMODE_SINGLESLOPE_gc;
    TCD1.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH2_gc;
    TCE0.CCD = 4480u;
    TCE0.CCA = 4482u;
    TCE0.PER = 8964u;
    TCD1.CCB = 4480u;
    TCD1.PER = 8964u;
    // start timers
    TCE0.CTRLA = TC_CLKSEL_DIV1_gc;
    TCD1.CTRLA = TC_CLKSEL_DIV1_gc;
}

void enableRegulator(bool enabled) {
    if(enabled)
        ENABLE_PORT.OUTSET = ENABLE_PIN;
    else
        ENABLE_PORT.OUTCLR = ENABLE_PIN;
}

void setOutputVoltage(uint16_t limit) {
    // safety check (195.00 V)
    if(limit > 19500u) limit = 19500u;

    uint32_t tmp = limit;
    tmp *= 4096u;
    tmp += 11750u;
    tmp /= 23500u;
    DACB.CH0DATA = (uint16_t) tmp;
}

void setInputVoltage(uint16_t limit) {
    // safety check (95.00 V)
    if(limit > 9500u) limit = 9500u;

    uint32_t tmp = limit;
    tmp *= 4096u;
    tmp += 5431u;
    tmp /= 10862u;
    DACB.CH1DATA = (uint16_t) tmp;
}

uint16_t getOutputVoltage() {
    uint32_t tmp = DACB.CH0DATA & 0xfffu;
    tmp *= 23500u;
    tmp += 2048u;
    tmp /= 4096u;
    return (uint16_t) tmp;
}

uint16_t getOutputVoltageStep() {
    return (23500u + 2048u) / 4096u;
}

uint16_t getInputVoltage() {
    uint32_t tmp = DACB.CH1DATA & 0xfffu;
    tmp *= 10862u;
    tmp += 2048u;
    tmp /= 4096u;
    return (uint16_t) tmp;
}

uint16_t getInputVoltageStep() {
    return (10862u + 2048u) / 4096u;
}

inline uint16_t computeOutputVoltage(uint16_t adc) {
    adc = (adc > 200) ? (adc - 200) : 0;
    uint32_t tmp = adc;
    tmp *= 23500u;
    tmp += 1948u;
    tmp /= 3896u;
    return (uint16_t) tmp;
}

uint16_t measureOutputVoltage() {
    cli();
    uint16_t adc = adcFilter[0].res;
    sei();
    return computeOutputVoltage(adc);
}

ISR(ADCA_CH0_vect) {
    adcFilter[0].raw = ADCA.CH0.RES;
    adcFilter[0].acc += ADCA.CH0.RES;
    if(++(adcFilter[0].cnt) == 0) {
        adcFilter[0].res = (uint16_t) (adcFilter[0].acc >> 8u);
        adcFilter[0].acc = 0;
    }
    adcFilter[0].rdy = 1;
}

inline uint16_t computeInputVoltage(uint16_t adc) {
    adc = (adc > 200) ? (adc - 200) : 0;
    uint32_t tmp = adc;
    tmp *= 10862u;
    tmp += 1948u;
    tmp /= 3896u;
    return (uint16_t) tmp;
}

uint16_t measureInputVoltage() {
    cli();
    uint16_t adc = adcFilter[1].res;
    sei();
    return computeInputVoltage(adc);
}

ISR(ADCA_CH1_vect) {
    adcFilter[1].raw = ADCA.CH1.RES;
    adcFilter[1].acc += ADCA.CH1.RES;
    if(++(adcFilter[1].cnt) == 0) {
        adcFilter[1].res = (uint16_t) (adcFilter[1].acc >> 8u);
        adcFilter[1].acc = 0;
    }
    adcFilter[1].rdy = 1;
}

uint16_t measureFanSpeed() {
    // snapshot of fan speed
    cli();
    uint16_t fspeed = fanSpeed;
    sei();

    // assume 4-pole fan
    uint32_t temp = 937500ul;
    temp /= fspeed;
    return (uint16_t) temp;
}

ISR(TCC0_CCA_vect) {
    fanSpeed = TCC0.CCA;
}

void updateRegulatorPulseWidth() {
    uint32_t tmp;

    // wait for next ADC samples
    if(!(adcFilter[0].rdy & adcFilter[1].rdy)) return;
    adcFilter[0].rdy = 0;
    adcFilter[1].rdy = 0;

    // snapshot adc values
    cli();
    uint16_t ov = adcFilter[0].raw;
    uint16_t iv = adcFilter[1].raw;
    sei();

    // compute voltage
    iv = computeInputVoltage(iv);
    ov = computeOutputVoltage(ov);

    // compute inductor discharge voltage
    if(iv < 100) iv = 100;
    if(ov < iv) ov = iv;
    uint16_t dv = ov - iv + 50;

    // compute charge width
    tmp = 448000ul;
    tmp /= iv;
    uint16_t cw = tmp;

    // compute discharge width
    tmp = 448000ul;
    tmp /= dv;
    uint16_t dw = tmp;

    // compute pwm period
    uint16_t per = cw + dw + 4;

    // wait for overflow (prevent glitching)
    TCE0.INTFLAGS = TC0_OVFIF_bm;
    while(!(TCE0.INTFLAGS & TC0_OVFIF_bm));
    // update timer values
    TCE0.CCD = cw;
    TCE0.CCA = per >> 1u;
    TCE0.PER = per;

    // wait for overflow (prevent glitching)
    TCD1.INTFLAGS = TC0_OVFIF_bm;
    while(!(TCD1.INTFLAGS & TC0_OVFIF_bm));
    // update timer values
    TCD1.CCB = cw;
    TCD1.PER = per;
}
