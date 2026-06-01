#include "Can_receive.h"

/* 第一个 3508 拉绳电机的编码器跨圈判断参数。 */
#define MOTOR_ECD_WRAP_THRESHOLD    5000
#define MOTOR_ECD_FULL_RANGE        8191

/* 这些全局变量仍被 shoot.c 使用，因此保留原变量名。 */
int flag_CR;
int rou;
long length;

static CAN_TxHeaderTypeDef Header;

/* 各电机 ID 的最新反馈数据，仅通过 Back_xxx_Data() 对外返回指针。 */
static Revice_Motro_Data Revice_Chassis_yaw_Data;
static Revice_Motro_Data Revice_Chassis_pitch_Data;
static Revice_Motro_Data Revice_OutsideShoot_Data[2];
static Revice_Motro_Data Revice_InsideShoot_Data[2];
static Revice_Motro_Data Revice_Trans_Data;

/* DJI 电机电流控制帧格式：一个 8 字节帧中包含四个 int16 电流值。 */
static void set_motor_current_frame(uint8_t data[8], int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    data[0] = m1 >> 8;
    data[1] = m1;
    data[2] = m2 >> 8;
    data[3] = m2;
    data[4] = m3 >> 8;
    data[5] = m3;
    data[6] = m4 >> 8;
    data[7] = m4;
}

static void send_motor_current(uint16_t std_id, int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    uint32_t tx_mailbox;
    uint8_t data[8];

    Header.StdId = std_id;
    Header.IDE = CAN_ID_STD;
    Header.RTR = CAN_RTR_DATA;
    Header.DLC = 0x08;

    set_motor_current_frame(data, m1, m2, m3, m4);
    HAL_CAN_AddTxMessage(&hcan1, &Header, data, &tx_mailbox);
}

/* 解析一帧 DJI 电机反馈数据。 */
static void get_motor_data(uint8_t data[], Revice_Motro_Data *receive_data)
{
    receive_data->last_ecd = receive_data->ecd;
    receive_data->ecd = (data[0] << 8) | data[1];
    receive_data->rpm = (data[2] << 8) | data[3];
    receive_data->given_current = (data[4] << 8) | data[5];
    receive_data->temperate = data[6];
}

/*
 * 将 0-8191 的单圈编码器值展开为多圈累计行程。
 * flag_CR 用于防止编码器停留在边界附近时重复计入同一次跨圈。
 *
 */
static void update_outside_shoot_length(void)
{
    int16_t ecd_delta;

    ecd_delta = Revice_OutsideShoot_Data[0].ecd - Revice_OutsideShoot_Data[0].last_ecd;

    if (ecd_delta < -MOTOR_ECD_WRAP_THRESHOLD && flag_CR == 1)
    {
        rou++;
        flag_CR = 0;
    }
    else if (ecd_delta > MOTOR_ECD_WRAP_THRESHOLD && flag_CR == 1)
    {
        rou--;
        flag_CR = 0;
    }
    else if (ecd_delta >= -MOTOR_ECD_WRAP_THRESHOLD && ecd_delta <= MOTOR_ECD_WRAP_THRESHOLD && flag_CR == 0)
    {
        flag_CR = 1;
    }

    length = Revice_OutsideShoot_Data[0].ecd + MOTOR_ECD_FULL_RANGE * rou;
}

Revice_Motro_Data *Back_OutsideShoot_Data(uint8_t i)
{
    return &Revice_OutsideShoot_Data[i];
}

Revice_Motro_Data *Back_InsideShoot_Data(uint8_t i)
{
    return &Revice_InsideShoot_Data[i];
}

Revice_Motro_Data *Back_Chassis_yaw_Data(void)
{
    return &Revice_Chassis_yaw_Data;
}

Revice_Motro_Data *Back_Chassis_pitch_Data(void)
{
    return &Revice_Chassis_pitch_Data;
}

Revice_Motro_Data *Back_Trans_Data(void)
{
    return &Revice_Trans_Data;
}

/* CAN 接收回调：按反馈 ID 分发到对应的电机数据缓存。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CHASSIS_MOTOR1_RECEIVE_ID:
            get_motor_data(rx_data, &Revice_Chassis_pitch_Data);
            break;

        case CHASSIS_MOTOR2_RECEIVE_ID:
            get_motor_data(rx_data, &Revice_Chassis_yaw_Data);
            break;

        case SHOOT_MOTOR1_RECEIVE_ID:
            get_motor_data(rx_data, &Revice_OutsideShoot_Data[0]);
            update_outside_shoot_length();
            break;

        case SHOOT_MOTOR2_RECEIVE_ID:
            get_motor_data(rx_data, &Revice_OutsideShoot_Data[1]);
            break;

        case SHOOT_MOTOR3_RECEIVE_ID:
            get_motor_data(rx_data, &Revice_InsideShoot_Data[0]);
            break;

        case SHOOT_MOTOR4_RECEIVE_ID:
            get_motor_data(rx_data, &Revice_InsideShoot_Data[1]);
            break;

        case TRANS_MOTOR1_RECEIVE_ID:
            get_motor_data(rx_data, &Revice_Trans_Data);
            break;

        default:
            break;
    }
}

void CAN_cmd_shoot(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    send_motor_current(SHOOT_MOTOR_ID, m1, m2, m3, m4);
}

void CAN_cmd_chassis(int16_t m1, int16_t m2, int16_t dev3, int16_t dev4)
{
    send_motor_current(CHASSIS_TRANS_MOTOR_ID, m1, m2, dev3, dev4);
}

void CAN_cmd_trans(int16_t dev1, int16_t dev2, int16_t m3, int16_t dev4)
{
    send_motor_current(CHASSIS_TRANS_MOTOR_ID, dev1, dev2, m3, dev4);
}

/* 配置 CAN1/CAN2 接收所有标准帧，并放入 FIFO0。 */
void canfilter_init_start(void)
{
    CAN_FilterTypeDef can_filter_st;

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
