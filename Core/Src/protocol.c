#include "protocol.h"
#include "usart_comm.h"
#include "commands.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Zmienne globalne
uint8_t buffer[buf_RX_len];
Stan frameState = Waiting;
const char urz_add[4] = "STM\0";
char source_add[4];
uint16_t frame_len = 0;
int data_len;
int data_val = -1;
volatile Komenda performed_cmd = BRAK;

// Funkcje błędów
void ThrowBLEN(void) {
    char error[buf_TX_len];
    sprintf(error, ":%s%s004BLEN;", urz_add, source_add);
    Send(error);
    frameState = Waiting;
    frame_len = 0;
}

void ThrowBCMD(void) {
    char error[buf_TX_len];
    sprintf(error, ":%s%s004BCMD;", urz_add, source_add);
    Send(error);
    frameState = Waiting;
    frame_len = 0;
}

void ThrowBFRM(void) {
    char error[buf_TX_len];
    sprintf(error, ":%s%s004BFRM;", urz_add, source_add);
    Send(error);
    frameState = Waiting;
    frame_len = 0;
}

// Funkcje pomocnicze
char crc8(uint8_t *data, int len) {
    char crc = 0x00;
    char poly = 0x07;

    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

int VerifyChars(char *data, int len) {
    for (int i = 0; i < len; i++) {
        if (data[i] <= '0' && data[i] >= '9') {
            return 1;
        }
    }
    return 0;
}

void AnalizeFrame(void) {
    if (frame_len >= 16) {
        char dst_add[4];
        char data_size[4];

        memcpy(source_add, &buffer[0], 3);
        memcpy(dst_add, &buffer[3], 3);
        memcpy(data_size, &buffer[6], 3);
        source_add[3] = '\0';
        dst_add[3] = '\0';
        data_size[3] = '\0';

        int size = (sizeof(data_size) / sizeof(data_size[0]));
        if (VerifyChars(data_size, size) == 0) {
            data_len = strtol(data_size, NULL, 10);
            if (data_len > 252 || data_len < 0) {
                ThrowBLEN();
                return;
            }
        } else {
            ThrowBFRM();
            return;
        }

        char command[5];
        memcpy(command, &buffer[9], 4);
        command[4] = '\0';

        if (frame_len - 16 == data_len) {
            char data[data_len + 1];
            memcpy(data, &buffer[13], data_len);
            data[data_len] = '\0';

            size = (sizeof(data) / sizeof(data[0]));
            if (VerifyChars(data, size) == 0) {
                data_val = strtol(data, NULL, 10);
            }

            char frameCrc[4];
            memcpy(frameCrc, &buffer[13 + data_len], 3);
            frameCrc[3] = '\0';

            int frameCrcVal = 0;
            if (VerifyChars(frameCrc, 4) == 0) {
                frameCrcVal = strtol(frameCrc, NULL, 10);
            } else {
                ThrowBFRM();
                return;
            }

            if (frameCrcVal == crc8(buffer, frame_len - 3)) {
                if (data_val != -1) {
                    Check_cmd(command, data_val);
                }
            } else {
                frame_len = 0;
                frameState = Waiting;
                return;
            }
        } else {
            ThrowBLEN();
        }
    } else {
        ThrowBLEN();
    }
    data_val = -1;
}
