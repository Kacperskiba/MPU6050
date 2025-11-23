#ifndef UART_COMM_H
#define UART_COMM_H

#include "main.h"

#define buf_RX_len 1024
#define buf_TX_len 2048

// Zmienne globalne bufora (extern)
extern volatile uint8_t buf_RX[buf_RX_len];
extern volatile uint16_t idx_RX_EMPTY;
extern volatile uint16_t idx_RX_BUSY;
extern volatile uint8_t buf_TX[buf_TX_len];
extern volatile uint16_t idx_TX_EMPTY;
extern volatile uint16_t idx_TX_BUSY;

// Funkcje publiczne
void Send(char *msgToSend, ...);
void UART_Init_Callbacks(void);

#endif // UART_COMM_H
