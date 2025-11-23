#ifndef COMMANDS_H
#define COMMANDS_H

#include "main.h"
#include "mpu6050.h"

// Zmienne globalne GPOZ (extern)
extern float vx, vy, vz;
extern float xPos, yPos, zPos;
extern GPOZData orientation;

// Funkcje publiczne
void DoTUSP(int cmd_parameter);
void DoGPOZ(void);
void DoZYRB(void);
void DoZYRA(int cmd_parameter);
void DoACCB(void);
void DoACCA(int cmd_parameter);
void DoCACC(int cmd_parameter);
void DoCZYR(int cmd_parameter);
void DoPROB(int cmd_parameter);
void DoPOBR(int cmd_parameter);
void Check_cmd(char *cmd, int cmd_parameter);

#endif // COMMANDS_H
