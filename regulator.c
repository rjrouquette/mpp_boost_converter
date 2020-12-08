//
// Created by robert on 11/16/20.
//

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stddef.h>
#include "regulator.h"

#define ENABLE_PORT PORTB
#define ENABLE_PIN PIN0_bm

volatile uint16_t fanSpeed;

static volatile struct {
    uint32_t acc;
    uint16_t res;
    uint8_t cnt;
    uint8_t rdy;
} adcFilter[2];

volatile uint16_t pwCharge;
volatile uint16_t pwPeriod;
volatile uint8_t pwUpdateA;
volatile uint8_t pwUpdateB;

void initRegulator() {
    // configure regulator enable pin
    ENABLE_PORT.DIRSET = ENABLE_PIN;
    ENABLE_PORT.OUTCLR = ENABLE_PIN;

    // configure analog reference pin
    PORTA.DIRCLR = PIN0_bm;
    PORTA.PIN0CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;

    // configure comparator output (wired and)
    PORTA.PIN7CTRL = PORT_OPC_WIREDAND_gc | PORT_INVEN_bm;      // wired-and, inverted (ACA0)
    PORTA.PIN6CTRL = PORT_OPC_WIREDAND_gc | PORT_ISC_LEVEL_gc;  // wired-and, sense level (ACA1)
    PORTA.DIRSET = PIN6_bm | PIN7_bm;
    PORTA.OUTCLR = PIN6_bm | PIN7_bm;

    // configure analog comparators
    PORTA.DIRCLR = PIN3_bm | PIN4_bm | PIN5_bm;
    PORTA.PIN3CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
    PORTA.PIN4CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
    PORTA.PIN5CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;

    ACA.AC0MUXCTRL = AC_MUXPOS_PIN3_gc | AC_MUXNEG_DAC_gc; // pin PA3 vs DAC0 (internal)
    ACA.AC1MUXCTRL = AC_MUXPOS_PIN4_gc | AC_MUXNEG_PIN5_gc; // pin PA4 vs DAC1 (pin PA5)
    ACA.AC0CTRL = AC_HYSMODE_SMALL_gc | AC_HSMODE_bm | AC_ENABLE_bm; // 13 mV hysteresis
    ACA.AC1CTRL = AC_HYSMODE_SMALL_gc | AC_HSMODE_bm | AC_ENABLE_bm; // 13 mV hysteresis

    // configure DACB
    PORTB.DIRSET = PIN3_bm;
    PORTB.PIN3CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
    NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
    DACB.CH0OFFSETCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB0OFFCAL));
    DACB.CH0GAINCAL   = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB0GAINCAL));
    DACB.CH1OFFSETCAL = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB1OFFCAL));
    DACB.CH1GAINCAL   = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, DACB1GAINCAL));
    NVM.CMD = 0x00;
    DACB.CTRLC = DAC_REFSEL_AVCC_gc; // Vcc (3.3V)
    DACB.CTRLB = DAC_CHSEL_DUAL_gc;
    DACB.CTRLA = DAC_IDOEN_bm | DAC_CH0EN_bm| DAC_CH1EN_bm | DAC_ENABLE_bm;


    // configure event 0 for RTC (1.024 kHz)
    EVSYS.CH0MUX = EVSYS_CHMUX_RTC_OVF_gc;

    // configure ADCA for sweep on event 0
    ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc;
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
    adcFilter[0].cnt = 0;
    adcFilter[0].rdy = 0;
    adcFilter[1].acc = 0;
    adcFilter[1].res = 0;
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

    // configure PWM outputs
    PORTD.DIRSET = PIN5_bm;
    PORTE.DIRSET = PIN3_bm;
    EVSYS.CH2MUX = EVSYS_CHMUX_TCE0_CCA_gc;
    TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc;
    TCE0.INTCTRLB = TC_CCAINTLVL_HI_gc;
    TCE0.CTRLB = TC0_CCDEN_bm | TC_WGMODE_SINGLESLOPE_gc;
    TCD1.CTRLB = TC1_CCBEN_bm | TC_WGMODE_SINGLESLOPE_gc;
    TCD1.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH2_gc;
    pwCharge = 4480u;
    pwPeriod = 8964u;
    pwUpdateA = 0;
    pwUpdateB = 0;
    TCE0.CCD = pwCharge;
    TCE0.CCA = pwPeriod >> 1u;
    TCE0.PER = pwPeriod;
    TCD1.CCB = pwCharge;
    TCD1.PER = 0xffffu;
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
    // safety check (90.00 V)
    if(limit > 9000u) limit = 9000u;

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
    adc = (adc > 190u) ? (adc - 190u) : 0;
    uint32_t tmp = adc;
    tmp *= 20534u;
    tmp += 2048u;
    tmp /= 4096u;
    return (uint16_t) tmp;
}

uint16_t measureOutputVoltage() {
    uint16_t adc = adcFilter[0].res;
    return computeOutputVoltage(adc);
}

ISR(ADCA_CH0_vect) {
    adcFilter[0].acc += ADCA.CH0.RES;
    if(++(adcFilter[0].cnt) == 0) {
        adcFilter[0].res = (uint16_t) (adcFilter[0].acc >> 8u);
        adcFilter[0].acc = 0;
    }
    adcFilter[0].rdy = 1;
}

inline uint16_t computeInputVoltage(uint16_t adc) {
    adc = (adc > 190u) ? (adc - 190u) : 0;
    uint32_t tmp = adc;
    tmp *= 9479u;
    tmp += 2048u;
    tmp /= 4096u;
    return (uint16_t) tmp;
}

uint16_t measureInputVoltage() {
    uint16_t adc = adcFilter[1].res;
    return computeInputVoltage(adc);
}

ISR(ADCA_CH1_vect) {
    adcFilter[1].acc += ADCA.CH1.RES;
    if(++(adcFilter[1].cnt) == 0) {
        adcFilter[1].res = (uint16_t) (adcFilter[1].acc >> 8u);
        adcFilter[1].acc = 0;
    }
    adcFilter[1].rdy = 1;
}

uint16_t measureFanSpeed() {
    // assume 4-pole fan
    uint32_t temp = 937500ul;
    temp /= TCC0.CCA;
    return (uint16_t) temp;
}

void updateRegulatorPulseWidth() {
    uint32_t tmp;

    // wait for next ADC samples
    if(!(adcFilter[0].rdy & adcFilter[1].rdy)) return;
    adcFilter[0].rdy = 0;
    adcFilter[1].rdy = 0;

    // snapshot adc values
    uint16_t ov = ADCA.CH0.RES;
    uint16_t iv = ADCA.CH1.RES;

    // compute voltage
    iv = computeInputVoltage(iv);
    ov = computeOutputVoltage(ov);

    // cap input voltage
    if(iv < 100) iv = 100;
    // cap output voltage
    if(ov < iv) ov = iv;
    // compute inductor discharge voltage (add 0.5V diode drop)
    uint16_t dv = ov - iv + 50;

    // compute charge width
    tmp = 448000ul;
    tmp /= iv;
    pwCharge = tmp;

    // compute discharge width
    tmp = 448000ul;
    tmp /= dv;
    uint16_t dw = tmp;

    // compute pwm period (pad with 250 ns)
    pwPeriod = pwCharge + dw + 8;
    pwUpdateA = 1;
    pwUpdateB = 1;
}

ISR(TCE0_OVF_vect) {
    uint8_t comp = ACA.STATUS & 0x30u;
    if(comp == 0x30u) {
        PORTA.OUTSET = PIN6_bm | PIN7_bm;
    } else {
        PORTA.OUTCLR = PIN6_bm | PIN7_bm;
    }

    if(pwUpdateA) {
        TCE0.CCD = pwCharge;
        TCE0.CCA = pwPeriod >> 1u;
        TCE0.PER = pwPeriod;
        pwUpdateA = 0;
    }
}

ISR(TCE0_CCA_vect) {
    if(pwUpdateB) {
        TCD1.CCB = pwCharge;
        pwUpdateB = 0;
    }
}
