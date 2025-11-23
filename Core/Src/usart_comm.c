#include "usart_comm.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Zmienne globalne bufora
volatile uint8_t buf_RX[buf_RX_len];
volatile uint16_t idx_RX_EMPTY = 0;
volatile uint16_t idx_RX_BUSY = 0;

volatile uint8_t buf_TX[buf_TX_len];
volatile uint16_t idx_TX_EMPTY = 0;
volatile uint16_t idx_TX_BUSY = 0;

extern UART_HandleTypeDef huart2;

// Callback dla ODBIERANIA
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        idx_RX_EMPTY++;
        if (idx_RX_EMPTY >= buf_RX_len) {
            idx_RX_EMPTY = 0;
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&buf_RX[idx_RX_EMPTY], 1);
    }
}

// Callback dla TRANSMITOWANIA
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (idx_TX_BUSY != idx_TX_EMPTY) {
        uint8_t tmp = buf_TX[idx_TX_BUSY];
        idx_TX_BUSY++;
        if (idx_TX_BUSY >= buf_TX_len) {
            idx_TX_BUSY = 0;
        }
        HAL_UART_Transmit_IT(&huart2, &tmp, 1);
    }
}

// USART SEND
void Send(char *msgToSend, ...) {
    char dataToSend[100];
    int idx;
    va_list arglist;

    va_start(arglist, msgToSend);
    vsprintf(dataToSend, msgToSend, arglist);
    va_end(arglist);

    idx = idx_TX_EMPTY;

    for (int i = 0; i < strlen(dataToSend); i++) {
        buf_TX[idx] = dataToSend[i];
        idx++;
        if (idx >= buf_TX_len) {
            idx = 0;
        }
    }

    __disable_irq();

    if ((idx_TX_BUSY == idx_TX_EMPTY) &&
        (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {
        idx_TX_EMPTY = idx;
        HAL_UART_Transmit_IT(&huart2, (uint8_t*)&buf_TX[idx_TX_BUSY], 1);
        idx_TX_BUSY++;
        if (idx_TX_BUSY >= buf_TX_len) {
            idx_TX_BUSY = 0;
        }
    } else {
        idx_TX_EMPTY = idx;
    }

    __enable_irq();
}

void UART_Init_Callbacks(void) {
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&buf_RX[idx_RX_EMPTY], 1);
}
