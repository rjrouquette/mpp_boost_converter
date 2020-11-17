//
// Created by robert on 11/16/20.
//

#include "cli.h"
#include "regulator.h"
#include <stdio.h>
#include <string.h>

static volatile char txBuff[64];

// from main .c
void sendString(char *msg);

void processCommand(const char *cmd) {
    if(strcmp(cmd, "enable") == 0) {
        enableRegulator(true);
        sendString("enabled");
        return;
    }
    if(strcmp(cmd, "disable") == 0) {
        enableRegulator(false);
        sendString("disabled");
        return;
    }

    if(strcmp(cmd, "status") == 0) {
        sprintf((char *)txBuff, "output target = %d\r\n", getOutputVoltage());
        sendString((char *)txBuff);

        sprintf((char *)txBuff, "output voltage = %d\r\n", measureOutputVoltage());
        sendString((char *)txBuff);

        sprintf((char *)txBuff, "input target = %d\r\n", getInputVoltage());
        sendString((char *)txBuff);

        sprintf((char *)txBuff, "input voltage = %d\r\n", measureInputVoltage());
        sendString((char *)txBuff);
        return;
    }

    if(strncmp(cmd, "set output ", 11) == 0) {
        int voltage;
        sscanf(cmd+11, "%d", &voltage);
        setOutputVoltage((uint16_t)voltage);
        sprintf((char *)txBuff, "output voltage = %d\r\n", getOutputVoltage());
        sendString((char *)txBuff);
        return;
    }

    if(strncmp(cmd, "set input ", 10) == 0) {
        int voltage;
        sscanf(cmd+10, "%d", &voltage);
        setInputVoltage((uint16_t)voltage);
        sprintf((char *)txBuff, "input voltage = %d\r\n", getInputVoltage());
        sendString((char *)txBuff);
        return;
    }

    if(strcmp(cmd, "get output step") == 0) {
        sprintf((char *)txBuff, "output voltage step = %d\r\n", getOutputVoltageStep());
        sendString((char *)txBuff);
        return;
    }

    if(strcmp(cmd, "get input step") == 0) {
        sprintf((char *)txBuff, "input voltage step = %d\r\n", getInputVoltageStep());
        sendString((char *)txBuff);
        return;
    }

    sendString("invalid command\r\n");
}
