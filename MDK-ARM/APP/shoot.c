#include "shoot.h"
#include "main.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "Shoot_task.h"

/*
 * 3508 拉绳电机控制。
 *
 * 控制流程：
 *   step1：拉到前光电开关位置，并在该位置保持。
 *   step2：继续后拉，直到触发后光电开关。
 *   step3：按编码器累计行程回线，然后等待下一次复位。
 *
 * 本模块输出的是 DJI 电调电流值。FreeRTOS 任务会把该输出
 * 通过 CAN_cmd_shoot(out, 0, 0, 0) 发送出去，因此这里只控制 ID 0x201 的电机。
 */

/* 光电开关接线定义。GPIO_PIN_RESET 表示开关被触发。 */
#define FRONT_PHOTO_GPIO        GPIOA
#define FRONT_PHOTO_PIN         GPIO_PIN_2
#define REAR_PHOTO_GPIO         GPIOI
#define REAR_PHOTO_PIN          GPIO_PIN_0

/* 保留原控制代码中的单位换算系数。 */
#define MOTOR_RPM_TO_SPEED      0.000081799f
#define ECD_COUNT_TO_DISTANCE   0.000059911f

/* 编码器跨圈判断阈值和回线行程阈值。 */
#define RETURN_DISTANCE_COUNT   700000
#define ECD_HALF_RANGE          4096
#define ECD_FULL_RANGE          8192

/* 各机械阶段使用的目标速度。 */
#define SPEED_HOLD_FRONT        (-5550)
#define SPEED_PULL_BASE         (-12300)
#define SPEED_PULL_REAR         (-16000)
#define SPEED_RETURN            5700
#define STALL_RPM_THRESHOLD     4
#define STALL_SPEED_ADD_STEP    7

/* 遥控通道阈值。 */
#define RC_TRIGGER_ON           600
#define RC_TRIGGER_RESET        (-600)

extern int remote_init;
extern long length;
extern Shoot_move_t Shoot_move;

static PidTypeDef motor_pid_speed;
static PidTypeDef motor_pid_rmp;

static fp32 shoot_position_pid_param[3] = {21, 0, 0};
static fp32 shoot_speed_pid_param[3] = {6000.0f, 2.5f, 0.35f};

static const RC_ctrl_t *rc_ctrl_shoot;

static int shoot_out;
static int out_all;
static float Target_speed;

/* 当前状态。原代码使用三个标志位，这里保持这种形式。 */
static int step1;
static int step2;
static int step3;

static int flag_R;
static int destin;
int roll_flag;
static int flag_shoot;

static int allow_shoot;
int allow_flag = 1;
static int allow_flag1;
static int dart;
static int add;


/* step3 回线阶段使用的行程计数器。 */
static int ecd_flag_2;
static int count_ecd_2;
static int16_t ecd_start_2;
static int last_plus_2;
static int count_flag_2;
static int num_2;

static GPIO_PinState front_photo_state(void)
{
    return HAL_GPIO_ReadPin(FRONT_PHOTO_GPIO, FRONT_PHOTO_PIN);
}

static GPIO_PinState rear_photo_state(void)
{
    return HAL_GPIO_ReadPin(REAR_PHOTO_GPIO, REAR_PHOTO_PIN);
}

static uint8_t trigger_is_on(void)
{
    return rc_ctrl_shoot->rc.ch[4] > RC_TRIGGER_ON;
}

static void enter_step1(void)
{
    step1 = 1;
    step2 = 0;
    step3 = 0;
}

static void enter_step2(void)
{
    step1 = 0;
    step2 = 1;
    step3 = 0;
}

static void enter_step3(void)
{
    step1 = 0;
    step2 = 0;
    step3 = 1;
}

static void reset_return_distance_counter(void)
{
    ecd_flag_2 = 0;
    count_ecd_2 = 0;
    ecd_start_2 = 0;
    last_plus_2 = 0;
    count_flag_2 = 0;
    num_2 = 0;
}


/* 回线过程中累计编码器行程，并处理 0/8192 跨圈。 */
static void update_return_distance(const Revice_Motro_Data *shoot)
{
    if (ecd_flag_2 == 0)
    {
        ecd_start_2 = shoot->ecd;
        ecd_flag_2 = 1;
        last_plus_2 = shoot->ecd;
    }

    if (shoot->ecd - last_plus_2 > ECD_HALF_RANGE && count_flag_2 == 0)
    {
        count_ecd_2--;
        count_flag_2 = 1;
    }
    else if (shoot->ecd - last_plus_2 < -ECD_HALF_RANGE && count_flag_2 == 0)
    {
        count_ecd_2++;
        count_flag_2 = 1;
    }
    else
    {
        count_flag_2 = 0;
    }

    num_2 = shoot->ecd + count_ecd_2 * ECD_FULL_RANGE - ecd_start_2;
    num_2 = -num_2;
    last_plus_2 = shoot->ecd;
}

/* 复位 PID 和状态机标志，回到第一阶段。 */
void shootint(const Revice_Motro_Data *shootint)
{
    (void)shootint;

    PID_clear(&motor_pid_rmp);
    PID_clear(&motor_pid_speed);
    PID_Init(&motor_pid_speed, PID_POSITION, shoot_speed_pid_param, 16000, 5000);
    PID_Init(&motor_pid_rmp, PID_POSITION, shoot_position_pid_param, 16000, 5000);

    rc_ctrl_shoot = get_remote_control_point();

    shoot_out = 0;
    out_all = 0;
    flag_R = 0;
    allow_shoot = 0;

    reset_return_distance_counter();

    enter_step1();
    destin = 0;
}

/*
 * 废弃备选方案：位置环 + 电流环双环控制。
 *
 * 下面参数只是占位，后续如果启用需要重新标定：
 *   spring_current_k = 0.018f 电流/编码器计数
 *   spring_zero_ecd = 弹簧自然长度对应的编码器值
 *   最大电流输出 = +/-16000
 *
 * #define SPRING_CURRENT_K        0.018f
 * #define SPRING_ZERO_ECD         0
 * #define POSITION_TO_CURRENT_MAX 12000
 * #define CURRENT_LOOP_MAX        16000
 *
 * static PidTypeDef pull_position_pid;
 * static PidTypeDef pull_current_pid;
 * static fp32 pull_position_pid_param[3] = {8.0f, 0.0f, 0.2f};
 * static fp32 pull_current_pid_param[3] = {1.5f, 0.02f, 0.0f};
 *
 * static int calc_spring_feedforward_current(const Revice_Motro_Data *shoot)
 * {
 *     int32_t displacement;
 *     float feedforward;
 *
 *     displacement = (int32_t)shoot->ecd - SPRING_ZERO_ECD;
 *     feedforward = displacement * SPRING_CURRENT_K;
 *
 *     if (feedforward > CURRENT_LOOP_MAX)
 *     {
 *         feedforward = CURRENT_LOOP_MAX;
 *     }
 *     else if (feedforward < -CURRENT_LOOP_MAX)
 *     {
 *         feedforward = -CURRENT_LOOP_MAX;
 *     }
 *
 *     return (int)feedforward;
 * }
 *
 * static int position_current_ctrl(int32_t target_ecd, const Revice_Motro_Data *shoot)
 * {
 *     float target_current;
 *     float spring_ff_current;
 *     float current_pid_out;
 *
 *     target_current = PID_Calc(&pull_position_pid,
 *                               shoot->ecd,
 *                               target_ecd);
 *
 *     if (target_current > POSITION_TO_CURRENT_MAX)
 *     {
 *         target_current = POSITION_TO_CURRENT_MAX;
 *     }
 *     else if (target_current < -POSITION_TO_CURRENT_MAX)
 *     {
 *         target_current = -POSITION_TO_CURRENT_MAX;
 *     }
 *
 *     spring_ff_current = calc_spring_feedforward_current(shoot);
 *     current_pid_out = PID_Calc(&pull_current_pid,
 *                                shoot->given_current,
 *                                target_current);
 *
 *     return current_pid_out + spring_ff_current;
 * }
 */
/* 速度环。这里每次调用都清 PID，用来保持原有控制行为。 */
int setspeed_ctrl(int setspeed, const Revice_Motro_Data *shoot)
{
    double speed;

    PID_clear(&motor_pid_speed);

    speed = shoot->rpm * MOTOR_RPM_TO_SPEED;
    Target_speed = setspeed * MOTOR_RPM_TO_SPEED;
    PID_Calc(&motor_pid_speed, speed, -Target_speed);

    return motor_pid_speed.out;
}

/* step1：持续拉绳直到触发前光电，然后保持当前位置。 */
static int shoot_step1(const Revice_Motro_Data *shoot, int setspeed)
{
    (void)setspeed;

    flag_shoot = Shoot_move.shoot_flag;
    if (front_photo_state() == GPIO_PIN_RESET)
    {
        /* 飞镖和滑轨都已就绪，操作员拨杆触发后进入发射拉绳阶段。 */
        if (flag_shoot == 1 && allow_shoot == 1)
        {
            if (trigger_is_on() && roll_flag == 0)
            {
                enter_step2();
            }
        }
        else
        {
            /* 前光电已触发但暂不允许发射，电机给力保持拉绳位置。 */
            allow_shoot = 1;


            shoot_out = setspeed_ctrl(SPEED_HOLD_FRONT, shoot);

            if (allow_flag == 0)
            {
                dart++;
            }

            allow_flag = 1;
            add = 0;
            roll_flag = 0;
        }
    }
    else
    {
        /* 前光电尚未触发，继续后拉；低转速时通过 add 增加目标速度。 */
        shoot_out = setspeed_ctrl(SPEED_PULL_BASE - add, shoot);

        if (shoot->rpm < STALL_RPM_THRESHOLD && rc_ctrl_shoot->rc.s[0] == RC_SW_MID)
        {
            add += STALL_SPEED_ADD_STEP;
        }

        if (allow_flag1 == 0)
        {
            allow_flag = 0;
            allow_flag1 = 1;
        }
    }

    return shoot_out;
}

/* step2：发射拉绳阶段，触发后光电后结束。 */
static int shoot_step2(const Revice_Motro_Data *shoot, int setrmp)
{
    (void)setrmp;

    if (rear_photo_state() == GPIO_PIN_RESET)
    {
        enter_step3();
    }
    else
    {
        shoot_out = setspeed_ctrl(SPEED_PULL_REAR, shoot);
    }

    return shoot_out;
}

/* step3：按编码器行程回线，到位后零速保持。 */
static int shoot_step3(const Revice_Motro_Data *shoot, int setspeed)
{
    flag_shoot = Shoot_move.shoot_flag;

    update_return_distance(shoot);

    if (num_2 > RETURN_DISTANCE_COUNT && flag_R == 0)
    {
        flag_R = 1;
        roll_flag = 1;
    }

    if (flag_R == 0)
    {
        shoot_out = setspeed_ctrl(setspeed, shoot);
        allow_flag1 = 0;
    }

    if (flag_R == 1)
    {
        /* 供弹机构离开待发位置后，才允许进入下一发复位。 */
        if (flag_shoot == 0 && destin == 1 && trigger_is_on() && roll_flag == 1 && dart < 2)
        {
            shootint(shoot);
        }
        else
        {
            double speed;

            PID_clear(&motor_pid_rmp);
            speed = shoot->rpm * MOTOR_RPM_TO_SPEED;
            shoot_out = PID_Calc(&motor_pid_speed, speed, 0);
            destin = 1;
        }
    }

    return shoot_out;
}

/* 2 ms 周期任务入口：运行一次状态机，并应用安全输出限制。 */
int shoottask(const Revice_Motro_Data *shoot, int setspeed)
{
    (void)setspeed;

    if (step1 == 1)
    {
        out_all = shoot_step1(shoot, 0);
    }

    if (step2 == 1)
    {
        out_all = shoot_step2(shoot, -20000);
    }

    if (step3 == 1)
    {
        out_all = shoot_step3(shoot, SPEED_RETURN);
        add = 0;
    }

    /* 后光电优先级最高：一旦触发，强制进入回线阶段。 */
    if (rear_photo_state() == GPIO_PIN_RESET)
    {
        enter_step3();
    }

    if (rc_ctrl_shoot->rc.s[0] == RC_SW_DOWN)
    {
        out_all = 0;
    }

    if (remote_init == 0)
    {
        out_all = 0;
    }

    if (rc_ctrl_shoot->rc.ch[4] < RC_TRIGGER_RESET)
    {
        dart = 0;
    }

    return out_all;
}
