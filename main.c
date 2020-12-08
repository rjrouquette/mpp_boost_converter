#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <string.h>

#include "descriptors.h"

#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#include <stddef.h>
#include <stdbool.h>

#include "cli.h"
#include "regulator.h"

static volatile bool ok_to_send = false;

void initSysClock(void);
inline void sendString(char *msg);

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
        {
                .Config =
                        {
                                .ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
                                .DataINEndpoint           =
                                        {
                                                .Address          = CDC_TX_EPADDR,
                                                .Size             = CDC_TXRX_EPSIZE,
                                                .Banks            = 1,
                                        },
                                .DataOUTEndpoint =
                                        {
                                                .Address          = CDC_RX_EPADDR,
                                                .Size             = CDC_TXRX_EPSIZE,
                                                .Banks            = 1,
                                        },
                                .NotificationEndpoint =
                                        {
                                                .Address          = CDC_NOTIFICATION_EPADDR,
                                                .Size             = CDC_NOTIFICATION_EPSIZE,
                                                .Banks            = 1,
                                        },
                        },
        };

static volatile char rxBuffer[256];
static volatile uint8_t rxCnt = 0;

int main(void) {
    initSysClock();
    initRegulator();
    sei();
    PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm;
    USB_Init();

    for(;;) {
        if (USB_DeviceState == DEVICE_STATE_Configured) {
            int16_t c = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
            if(c >= 0) {
                if(c == '\n' || c == '\r') {
                    sendString("\r\n");
                    rxBuffer[rxCnt] = 0;
                    processCommand((char *)rxBuffer);
                    sendString("\r\n> ");
                    rxCnt = 0;
                } else {
                    if(c == '\b') {
                        if(rxCnt)
                            --rxCnt;
                    } else {
                        rxBuffer[rxCnt++] = c;
                    }
                    if (ok_to_send) {
                        CDC_Device_SendByte(&VirtualSerial_CDC_Interface, c);
                        CDC_Device_Flush(&VirtualSerial_CDC_Interface);
                    }
                }
            }
        }

        CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
        USB_USBTask();
    }

    return 0;
}

void initSysClock(void) {
    // enable external 32 kHz crystal
    OSC.XOSCCTRL = OSC_XOSCSEL_32KHz_gc;

    CCP = CCP_IOREG_gc; // Security Signature to modify clock
    OSC.CTRL = OSC_XOSCEN_bm; // enable external 32kHz crystal
    while(!(OSC.STATUS & OSC_XOSCRDY_bm)); // wait for oscillator ready

    // Configure DFLL for 48MHz and 2MHz using external crystal
    // DFLL for USB
    DFLLRC32M.COMP1 = 0x1B; //Xmega AU manual, 4.17.19
    DFLLRC32M.COMP2 = 0xB7;
    DFLLRC32M.CTRL = DFLL_ENABLE_bm;
    DFLLRC2M.COMP1 = 0xA1; //Xmega AU manual, 4.17.19
    DFLLRC2M.COMP2 = 0x07;
    DFLLRC2M.CTRL = DFLL_ENABLE_bm;

    OSC.DFLLCTRL = OSC_RC32MCREF_XOSC32K_gc | OSC_RC2MCREF_XOSC32K_gc;
    NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
    DFLLRC32M.CALB = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBRCOSC));
    DFLLRC32M.CALA = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBRCOSCA));
    DFLLRC2M.CALB = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, RCOSC2M));
    DFLLRC2M.CALA = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, RCOSC2MA));
    NVM.CMD = 0x00;

    CCP = CCP_IOREG_gc; // Security Signature to modify clock
    OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_XOSCEN_bm; // enable internal 32MHz oscillator

    while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
    while(!(OSC.STATUS & OSC_RC2MRDY_bm)); // wait for oscillator ready

    // enable pll
    OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | 16u; // 2MHz * 16 = 32MHz

    CCP = CCP_IOREG_gc;
    OSC.CTRL = OSC_PLLEN_bm | OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_XOSCEN_bm; // Enable PLL

    while(!(OSC.STATUS & OSC_PLLRDY_bm)); // wait for PLL ready

    DFLLRC2M.CTRL = DFLL_ENABLE_bm;

    CCP = CCP_IOREG_gc; // Security Signature to modify clock
    CLK.CTRL = CLK_SCLKSEL_PLL_gc; // Select PLL
    CLK.PSCTRL = CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc; // CPU 32 MHz
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
    ok_to_send = false;
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
    ok_to_send = false;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
    CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo) {
    ok_to_send = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
}

void sendString(char *msg) {
    if (USB_DeviceState == DEVICE_STATE_Configured && ok_to_send) {
        CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
        CDC_Device_Flush(&VirtualSerial_CDC_Interface);
    }
}
