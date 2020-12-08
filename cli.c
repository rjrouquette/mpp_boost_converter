//
// Created by robert on 11/16/20.
//

#include "cli.h"
#include "regulator.h"
#include <string.h>
#include <avr/io.h>

// from main .c
void sendString(char *msg);

char *appendStr(char *buff, const char *str);
char *appendDecimal(char *buff, uint16_t value);
bool parseDecimal(const char *str, uint16_t *value);

void processCommand(const char *cmd) {
    char txBuff[256];
    char *ptr;

    if(strcmp(cmd, "help") == 0) {
        sendString("commands:\r\n");
        sendString("  enable                - enable regulator output\r\n");
        sendString("  disable               - disable regulator output\r\n");
        sendString("  status                - display regulator status\r\n");
        sendString("  set output <voltage>  - set regulator output voltage\r\n");
        sendString("  set input <voltage>   - set regulator input voltage\r\n");
        return;
    }
    if(strcmp(cmd, "enable") == 0) {
        enableRegulator(true);
        sendString("regulator output enabled");
        return;
    }
    if(strcmp(cmd, "disable") == 0) {
        enableRegulator(false);
        sendString("regulator output disabled");
        return;
    }

    if(strcmp(cmd, "status") == 0) {
        ptr = appendStr(txBuff, "output target = ");
        ptr = appendDecimal(ptr, getOutputVoltage());

        ptr = appendStr(ptr, " V\r\noutput voltage = ");
        ptr = appendDecimal(ptr, measureOutputVoltage());

        ptr = appendStr(ptr, " V\r\ninput target = ");
        ptr = appendDecimal(ptr, getInputVoltage());

        ptr = appendStr(ptr, " V\r\ninput voltage = ");
        ptr = appendDecimal(ptr, measureInputVoltage());

        ptr = appendStr(ptr, " V\r\ncomparators = ");
        *(ptr++) = (char) ('0' + (ACA.STATUS & 0x3u));
        *ptr = 0;

        sendString((char *)txBuff);
        return;
    }

    if(strncmp(cmd, "set output ", 11) == 0) {
        uint16_t voltage;
        if(!parseDecimal(cmd+11, &voltage)) {
            ptr = appendStr(txBuff, "Invalid Value: ");
            ptr = appendStr(ptr, cmd+11);
        } else {
            setOutputVoltage(voltage);
            ptr = appendStr(txBuff, "output target = ");
            ptr = appendDecimal(ptr, getOutputVoltage());
            ptr = appendStr(ptr, " V");
        }
        sendString((char *)txBuff);
        return;
    }

    if(strncmp(cmd, "set input ", 10) == 0) {
        uint16_t voltage;
        if(!parseDecimal(cmd+11, &voltage)) {
            ptr = appendStr(txBuff, "Invalid Value: ");
            ptr = appendStr(ptr, cmd+11);
        } else {
            setInputVoltage(voltage);
            ptr = appendStr(txBuff, "input target = ");
            ptr = appendDecimal(ptr, getInputVoltage());
            ptr = appendStr(ptr, " V");
        }
        sendString((char *)txBuff);
        return;
    }

    if(strcmp(cmd, "get output step") == 0) {
        ptr = appendStr(txBuff, "output voltage step size = ");
        ptr = appendDecimal(ptr, getOutputVoltageStep());
        ptr = appendStr(ptr, " V");
        sendString((char *)txBuff);
        return;
    }

    if(strcmp(cmd, "get input step") == 0) {
        ptr = appendStr(txBuff, "input voltage step size = ");
        ptr = appendDecimal(ptr, getInputVoltageStep());
        ptr = appendStr(ptr, " V");
        sendString((char *)txBuff);
        return;
    }

    ptr = appendStr(txBuff, "invalid command: ");
    ptr = appendStr(ptr, cmd);
    sendString((char *)txBuff);
}

char *appendStr(char *buff, const char *str) {
    char c;
    while((c = *(str++)) != 0) {
        *(buff++) = c;
    }
    *buff = 0;
    return buff;
}

char *appendDecimal(char *buff, uint16_t value) {
    char temp[7];
    temp[6] = 0;
    temp[5] = (char) (value % 10); value /= 10; temp[5] += '0';
    temp[4] = (char) (value % 10); value /= 10; temp[4] += '0';
    temp[3] = '.';

    temp[2] = (char) (value % 10); value /= 10; temp[2] += '0';
    if(value == 0)
        return appendStr(buff, temp + 2);

    temp[1] = (char) (value % 10); value /= 10; temp[1] += '0';
    if(value == 0)
        return appendStr(buff, temp + 1);

    temp[0] = (char) value; temp[0] += '0';
    return appendStr(buff, temp);
}

bool parseDecimal(const char *str, uint16_t *value) {
    uint16_t result = 0;
    uint8_t place = 255;
    char c;

    while((c = *(str++)) != 0) {
        if(place != 255u && ++place > 2)
            continue;

        if(c == '.') {
            place = 0;
            continue;
        }

        if(c < '0' || c > '9')
            return 0;

        result *= 10u;
        result += c - '0';
    }

    if(place == 255)
        place = 0;
    while(place < 2) {
        ++place;
        result *= 10u;
    }

    *value = result;
    return true;
}
