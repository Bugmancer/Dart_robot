#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "stm32f4xx_hal.h"
#include "can.h"

#define CHASSIS_TRANS_MOTOR_ID      0x1ff
#define CHASSIS_MOTOR1_RECEIVE_ID   0x205
#define CHASSIS_MOTOR2_RECEIVE_ID   0x206
#define TRANS_MOTOR1_RECEIVE_ID     0x207

#define SHOOT_MOTOR_ID              0x200
#define SHOOT_MOTOR1_RECEIVE_ID     0x201
#define SHOOT_MOTOR2_RECEIVE_ID     0x202
#define SHOOT_MOTOR3_RECEIVE_ID     0x203
#define SHOOT_MOTOR4_RECEIVE_ID     0x204

typedef struct
{
    uint16_t ecd;
    int16_t last_ecd;
    int16_t rpm;
    int16_t given_current;
    uint8_t temperate;
} Revice_Motro_Data;

void CAN_cmd_shoot(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void CAN_cmd_chassis(int16_t m1, int16_t m2, int16_t dev3, int16_t dev4);
void CAN_cmd_trans(int16_t dev1, int16_t dev2, int16_t m3, int16_t dev4);

void canfilter_init_start(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

Revice_Motro_Data *Back_OutsideShoot_Data(uint8_t i);
Revice_Motro_Data *Back_InsideShoot_Data(uint8_t i);
Revice_Motro_Data *Back_Chassis_yaw_Data(void);
Revice_Motro_Data *Back_Chassis_pitch_Data(void);
Revice_Motro_Data *Back_Trans_Data(void);

#endif
