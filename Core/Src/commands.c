#include "commands.h"
#include "usart_comm.h"
#include "protocol.h"
#include "mpu6050.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// Zmienne globalne GPOZ
float vx = 0, vy = 0, vz = 0;
float xPos = 0, yPos = 0, zPos = 0;
GPOZData orientation = {0, 0, 0};

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

void DoTUSP(int cmd_parameter) {
    if (cmd_parameter == 0) {
        uint8_t temp_data = 0;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_USR_CTRL, 1,
                &temp_data, 1, HAL_MAX_DELAY);
        Send("Tryb uspienia wylaczono\n\r");
        MPU6050_asleep = 0;
    } else if (cmd_parameter == 1) {
        uint8_t temp_data = 1;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_USR_CTRL, 1,
                &temp_data, 1, HAL_MAX_DELAY);
        Send("Tryb uspienia wlaczono\n\r");
        MPU6050_asleep = 1;
    } else {
        ThrowBCMD();
        return;
    }
}

void DoGPOZ(void) {
    GPOZData gyro, acc;
    MPU6050_GetGyroValues(&gyro.x, &gyro.y, &gyro.z);
    MPU6050_GetAccelValues(&acc.x, &acc.y, &acc.z);

    GPOZData curr_orientation;

    curr_orientation.z = orientation.z + gyro.z * MPU6050_probkowanie;

    float xACC = atan2(acc.y, acc.z);
    float yACC = atan(-acc.x / sqrt(acc.y * acc.y + acc.z * acc.z));

    float alpha = 0.98f;
    curr_orientation.x = alpha * (orientation.x + gyro.x * MPU6050_probkowanie)
            + (1 - alpha) * xACC;
    curr_orientation.y = alpha * (orientation.y + gyro.y * MPU6050_probkowanie)
            + (1 - alpha) * yACC;

    orientation = curr_orientation;
    GPOZData gravity;
    gravity.x = -G * sin(orientation.y);
    gravity.y = G * sin(orientation.x) * cos(orientation.y);
    gravity.z = G * cos(orientation.x) * cos(orientation.y);

    GPOZData acc_net;
    acc_net.x = acc.x - gravity.x;
    acc_net.y = acc.y - gravity.y;
    acc_net.z = acc.z - gravity.z;

    vx += acc_net.x * MPU6050_probkowanie;
    vy += acc_net.y * MPU6050_probkowanie;
    vz += acc_net.z * MPU6050_probkowanie;

    xPos += vx * MPU6050_probkowanie;
    yPos += vy * MPU6050_probkowanie;
    zPos += vz * MPU6050_probkowanie;

    char msg[128];
    sprintf(msg, "Pozycja:[%.2f, %.2f, %.2f] m\n\r", xPos, yPos, zPos);
    Send(msg);
}

void DoZYRB(void) {
    const float PI = 3.14159265f;
    float gx, gy, gz;
    char msg[128];

    MPU6050_GetGyroValues(&gx, &gy, &gz);
    gx = gx * (180.0f / PI);
    gy = gy * (180.0f / PI);
    gz = gz * (180.0f / PI);

    sprintf(msg, "GX: %.2f deg/s, GY: %.2f deg/s, GZ: %.2f deg/s\n\r", gx, gy, gz);
    Send(msg);
}

void DoZYRA(int cmd_parameter) {
    if (cmd_parameter <= 700 && cmd_parameter > 0) {
        float gyro_x_dps, gyro_y_dps, gyro_z_dps;
        char msg[128];
        uint16_t idx_start;

        __disable_irq();
        idx_start = (idx_gyr + buf_DATA_len - cmd_parameter) % buf_DATA_len;
        __enable_irq();

        sprintf(msg, "Ostatnie %d danych z zyroskopu:\n\r", cmd_parameter);
        Send(msg);

        for (int i = 0; i < cmd_parameter; i++) {
            uint16_t idx = (idx_start + i) % buf_DATA_len;
            __disable_irq();
            gyro_x_dps = buf_GYR[idx].x / gyro_setting;
            gyro_y_dps = buf_GYR[idx].y / gyro_setting;
            gyro_z_dps = buf_GYR[idx].z / gyro_setting;
            __enable_irq();

            sprintf(msg, "%d. GX: %.2f deg/s, GY: %.2f deg/s, GZ: %.2f deg/s\n\r",
                    i + 1, gyro_x_dps, gyro_y_dps, gyro_z_dps);
            Send(msg);
        }
    } else {
        ThrowBCMD();
        return;
    }
}

void DoACCB(void) {
    float ax, ay, az;
    char msg[128];

    MPU6050_GetAccelValues(&ax, &ay, &az);
    sprintf(msg, "AX: %.2f (m/s2), AY: %.2f (m/s2), AZ: %.2f (m/s2)\n\r",
            ax, ay, az);
    Send(msg);
}

void DoACCA(int cmd_parameter) {
    if (cmd_parameter <= 700 && cmd_parameter > 0) {
        float acc_x, acc_y, acc_z;
        char msg[128];
        uint16_t idx_start;

        __disable_irq();
        idx_start = (idx_acc + buf_DATA_len - cmd_parameter) % buf_DATA_len;
        __enable_irq();

        sprintf(msg, "Ostatnie %d danych z akcelerometru:\n\r", cmd_parameter);
        Send(msg);

        for (int i = 0; i < cmd_parameter; i++) {
            uint16_t idx = (idx_start + i) % buf_DATA_len;
            __disable_irq();
            acc_x = (((buf_ACC[idx].x + 0.0f) * acc_setting) / 1000) * G;
            acc_y = (((buf_ACC[idx].y + 0.0f) * acc_setting) / 1000) * G;
            acc_z = (((buf_ACC[idx].z + 0.0f) * acc_setting) / 1000) * G;
            __enable_irq();

            sprintf(msg, "%d. AX: %.2f (m/s^2), AY: %.2f (m/s^2), AZ: %.2f (m/s^2)\n\r",
                    i + 1, acc_x, acc_y, acc_z);
            Send(msg);
        }
    } else {
        ThrowBCMD();
        return;
    }
}

void DoCACC(int cmd_parameter) {
    uint8_t temp;
    char msg[128];

    switch (cmd_parameter) {
    case 2:
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_ACC_CONFIG, 1,
                (uint8_t*)FS_ACC_2G, 1, 100);
        acc_setting = 16384.0f;
        break;
    case 4:
        temp = FS_ACC_4G;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_ACC_CONFIG, 1,
                &temp, 1, 100);
        acc_setting = 8192.0f;
        break;
    case 8:
        temp = FS_ACC_8G;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_ACC_CONFIG, 1,
                &temp, 1, 100);
        acc_setting = 4096.0f;
        break;
    case 16:
        temp = FS_ACC_16G;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_ACC_CONFIG, 1,
                &temp, 1, 100);
        acc_setting = 2048.0f;
        break;
    default:
        ThrowBCMD();
        return;
    }

    sprintf(msg, "Zmieniono czulosc akcelerometru na +/- %d (g)\r\n", cmd_parameter);
    Send(msg);
}

void DoCZYR(int cmd_parameter) {
    uint8_t temp;
    char msg[128];

    switch (cmd_parameter) {
    case 250:
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_GYRO_CONFIG, 1,
                (uint8_t*)FS_GYRO_250, 1, 100);
        gyro_setting = 131.0;
        break;
    case 500:
        temp = FS_GYRO_500;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_GYRO_CONFIG, 1,
                &temp, 1, 100);
        gyro_setting = 65.5;
        break;
    case 1000:
        temp = FS_GYRO_1000;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_GYRO_CONFIG, 1,
                &temp, 1, 100);
        gyro_setting = 32.8;
        break;
    case 2000:
        temp = FS_GYRO_2000;
        HAL_I2C_Mem_Write(&hi2c1, (MPU6050_ADDR << 1) + 0, REG_GYRO_CONFIG, 1,
                &temp, 1, 100);
        gyro_setting = 16.4;
        break;
    default:
        ThrowBCMD();
        return;
    }

    sprintf(msg, "Zmieniono czulosc zyroskopu na +/- %d (g)\r\n", cmd_parameter);
    Send(msg);
}

void DoPROB(int cmd_parameter) {
    if (cmd_parameter >= 1 && cmd_parameter <= 256) {
        HAL_TIM_Base_Stop_IT(&htim1);
        HAL_TIM_Base_Stop_IT(&htim2);
        htim1.Init.Period = cmd_parameter - 1;
        htim2.Init.Period = cmd_parameter - 1;
        MPU6050_probkowanie = cmd_parameter;
        HAL_TIM_Base_Start_IT(&htim1);
        HAL_TIM_Base_Start_IT(&htim2);
    }
}

void DoPOBR(int cmd_parameter) {
    if (cmd_parameter >= 1 && cmd_parameter <= 65535) {
        HAL_TIM_Base_Stop_IT(&htim6);
        uint32_t timerClock = HAL_RCC_GetPCLK1Freq();
        uint32_t prescaler = (timerClock / 1000) - 1;
        __HAL_TIM_SET_PRESCALER(&htim6, prescaler);
        __HAL_TIM_SET_AUTORELOAD(&htim6, cmd_parameter - 1);
        HAL_TIM_Base_Start_IT(&htim6);
    } else {
        ThrowBCMD();
        return;
    }
}

void Check_cmd(char *cmd, int cmd_parameter) {
    if (memcmp(cmd, "GPOZ", 4) == 0) {
        if (cmd_parameter == 1) {
            Send("Wlaczono wypisywanie pozycji w przestrzeni 3D\n\r");
            performed_cmd = GPOZ;
        } else if (cmd_parameter == 0) {
            performed_cmd = BRAK;
        } else {
            ThrowBCMD();
            return;
        }
    } else if (memcmp(cmd, "ZYRA", 4) == 0) {
        DoZYRA(cmd_parameter);
    } else if (memcmp(cmd, "ZYRB", 4) == 0) {
        if (cmd_parameter == 1) {
            Send("Wlaczono wypisywanie danych z zyroskopu\n\r");
            performed_cmd = ZYRB;
        } else if (cmd_parameter == 0) {
            Send("Wylaczono wypisywanie danych z zyroskopu\n\r");
            performed_cmd = BRAK;
        } else {
            ThrowBCMD();
            return;
        }
    } else if (memcmp(cmd, "ACCA", 4) == 0) {
        DoACCA(cmd_parameter);
    } else if (memcmp(cmd, "ACCB", 4) == 0) {
        if (cmd_parameter == 1) {
            Send("Wlaczono wypisywanie danych z akcelerometru\n\r");
            performed_cmd = ACCB;
        } else if (cmd_parameter == 0) {
            Send("Wylaczono wypisywanie danych z akcelerometru\n\r");
            performed_cmd = BRAK;
        } else {
            ThrowBCMD();
            return;
        }
    } else if (memcmp(cmd, "POBR", 4) == 0) {
        DoPOBR(cmd_parameter);
    } else if (memcmp(cmd, "TUSP", 4) == 0) {
        DoTUSP(cmd_parameter);
    } else if (memcmp(cmd, "CACC", 4) == 0) {
        DoCACC(cmd_parameter);
    } else if (memcmp(cmd, "CZYR", 4) == 0) {
        DoCZYR(cmd_parameter);
    } else if (memcmp(cmd, "PROB", 4) == 0) {
        DoPROB(cmd_parameter);
    } else {
        ThrowBCMD();
    }
}
