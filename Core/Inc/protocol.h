#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "main.h"

typedef enum {
    Waiting,
    Recieving,
    Encoding
} Stan;

typedef enum {
    BRAK,
    ZYRB,
    ACCB,
    GPOZ
} Komenda;

// Zmienne globalne protokołu (extern)
extern uint8_t buffer[buf_RX_len];
extern Stan frameState;
extern const char urz_add[4];
extern char source_add[4];
extern uint16_t frame_len;
extern int data_len;
extern int data_val;
extern volatile Komenda performed_cmd;

// Funkcje publiczne
void ThrowBLEN(void);
void ThrowBCMD(void);
void ThrowBFRM(void);
void AnalizeFrame(void);
char crc8(uint8_t *data, int len);

#endif // PROTOCOL_H
