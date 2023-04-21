//
// Created by waxz on 4/19/23.
//

#include <stdio.h>
#include "controlcan.h"

#define MAX_CHANNELS  4
#define CHECK_POINT  200
#define RX_WAIT_TIME  100
#define RX_BUFF_SIZE  1000

unsigned gDevType = 0;
unsigned gDevIdx = 0;
unsigned gChMask = 0;
unsigned gBaud = 0;
unsigned gTxType = 0;
unsigned gTxSleep = 0;
unsigned gTxFrames = 0;
unsigned gTxCount = 0;


int main(int argc, char** argv){


    if (!VCI_OpenDevice(gDevType, gDevIdx, 0)) {
        printf("VCI_OpenDevice failed\n");
        return 0;
    }
    printf("VCI_OpenDevice succeeded\n");
    VCI_CloseDevice(gDevType, gDevIdx);
    printf("VCI_CloseDevice\n");
    return 0;
}