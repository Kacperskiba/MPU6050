
#include "mpu6050.h"
#include "usart_comm.h"

// Zmienne globalne modułu
volatile SensorData buf_ACC[buf_DATA_len];
volatile uint16_t idx_acc = 0;
volatile SensorData buf_GYR[buf_DATA_len];
volatile uint16_t idx_gyr = 0;
volatile uint8_t MPU6050_asleep = 1;
volatile float MPU6050_probkowanie = 0.1f;
float acc_setting = 16384.0f;
float gyro_setting = 131.0f;

extern I2C_HandleTypeDef hi2c1;

void MPU6050_Init(void) {
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, 0b1101000 << 1, 1, 100);
    if (ret == HAL_OK) {
        Send("The device is ready\n\r");
    } else {
        Send("The device not ready\n\r");
    }
}

void MPU6050_ReadAccel(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, 0x3B, 1, data, 6, 100);

    *raw_ax = (int16_t)(data[0] << 8 | data[1]);
    *raw_ay = (int16_t)(data[2] << 8 | data[3]);
    *raw_az = (int16_t)(data[4] << 8 | data[5]);
}

void MPU6050_GetAccelValues(float *acc_ax, float *acc_ay, float *acc_az) {
    int16_t raw_ax, raw_ay, raw_az;
    uint16_t idx_local_acc;

    __disable_irq();
    idx_local_acc = idx_acc;
    idx_local_acc = (idx_local_acc == 0) ? (buf_DATA_len - 1) : (idx_local_acc - 1);
    raw_ax = buf_ACC[idx_local_acc].x;
    raw_ay = buf_ACC[idx_local_acc].y;
    raw_az = buf_ACC[idx_local_acc].z;
    __enable_irq();

    *acc_ax = ((raw_ax + 0.0f) / acc_setting) * G;
    *acc_ay = ((raw_ay + 0.0f) / acc_setting) * G;
    *acc_az = ((raw_az + 0.0f) / acc_setting) * G;
}

void MPU6050_ReadGyro(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t gyro_data[6];

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, 0x43, 1, gyro_data, 6, 100);
    *gyro_x = (int16_t)(gyro_data[0] << 8 | gyro_data[1]);
    *gyro_y = (int16_t)(gyro_data[2] << 8 | gyro_data[3]);
    *gyro_z = (int16_t)(gyro_data[4] << 8 | gyro_data[5]);
}

void MPU6050_GetGyroValues(float *gyro_x, float *gyro_y, float *gyro_z) {
    const float PI = 3.14159265f;
    uint16_t idx_local_gyr;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

    __disable_irq();
    idx_local_gyr = idx_gyr;
    idx_local_gyr = (idx_local_gyr == 0) ? (buf_DATA_len - 1) : (idx_local_gyr - 1);
    gyro_x_raw = buf_GYR[idx_local_gyr].x;
    gyro_y_raw = buf_GYR[idx_local_gyr].y;
    gyro_z_raw = buf_GYR[idx_local_gyr].z;
    __enable_irq();

    *gyro_x = (gyro_x_raw / gyro_setting) * (PI / 180.0f);
    *gyro_y = (gyro_y_raw / gyro_setting) * (PI / 180.0f);
    *gyro_z = (gyro_z_raw / gyro_setting) * (PI / 180.0f);
}
