#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

// Definicje rejestrów MPU6050
#define MPU6050_ADDR 0x68
#define FS_ACC_2G 0x00
#define FS_ACC_4G 0x08
#define FS_ACC_8G 0x10
#define FS_ACC_16G 0x18
#define FS_GYRO_250 0x00
#define FS_GYRO_500 0x08
#define FS_GYRO_1000 0x10
#define FS_GYRO_2000 0x18
#define REG_ACC_CONFIG 0x1C
#define REG_GYRO_CONFIG 0x1B
#define REG_USR_CTRL 0x6B
#define REG_DATA 0x3B

#define G 9.80665
#define buf_DATA_len 700

// Struktury danych
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} SensorData;

typedef struct {
    float x;
    float y;
    float z;
} GPOZData;

// Zmienne globalne (extern)
extern volatile SensorData buf_ACC[buf_DATA_len];
extern volatile uint16_t idx_acc;
extern volatile SensorData buf_GYR[buf_DATA_len];
extern volatile uint16_t idx_gyr;
extern volatile uint8_t MPU6050_asleep;
extern volatile float MPU6050_probkowanie;
extern float acc_setting;
extern float gyro_setting;

// Funkcje publiczne
void MPU6050_Init(void);
void MPU6050_ReadAccel(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az);
void MPU6050_GetAccelValues(float *acc_ax, float *acc_ay, float *acc_az);
void MPU6050_ReadGyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
void MPU6050_GetGyroValues(float *gyro_x, float *gyro_y, float *gyro_z);

#endif // MPU6050_H
