//
// Created by robert on 11/16/20.
//

#include "cli.h"
#include <stdio.h>

static volatile char txBuff[256];

// from main .c
void sendString(char *msg);

void processCommand(const char *cmd) {
    sprintf((char *)txBuff, "%s\r\n", cmd);
    sendString((char *)txBuff);
}
