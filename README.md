# Dart_Project

这是一个基于 STM32F427 + FreeRTOS 的飞镖控制工程，主要使用 Keil MDK-ARM 构建。

## 目录结构

- `Core/`：STM32CubeMX 生成的 HAL、外设、中断和 FreeRTOS 入口代码。
- `MDK-ARM/`：Keil 工程文件、应用模块和硬件行为模块。
- `MDK-ARM/APP/`：主要应用任务，包括底盘、发射和辅助逻辑。
- `MDK-ARM/Behaviors/Can_receive/`：CAN 电机反馈解析和电流控制帧发送。
- `MDK-ARM/Hardware/`：PID、遥控器、电机数据等底层辅助模块。
- `Drivers/`、`Middlewares/`：STM32 HAL、CMSIS、FreeRTOS 依赖。

## 编译方式

使用 Keil uVision 打开：

```text
MDK-ARM/Dart_Project.uvprojx
```

然后编译目标 `Dart_Project`。

当前工程使用 ARMCC 5，例如：

```text
Compiler 'V5.06 update 5 (build 528)'
```

Keil 编译产物会生成在 `MDK-ARM/Dart_Project/` 下。这些文件已经通过 `.gitignore` 忽略；该目录中只保留 `Dart_Project.sct`，因为它是链接用的 scatter 文件。

## 3508 拉绳电机控制

3508 拉绳电机控制链路：

```text
CAN 反馈 0x201
  -> Back_OutsideShoot_Data(0)
  -> shoottask()
  -> CAN_cmd_shoot(out, 0, 0, 0)
  -> CAN 电流控制帧 0x200
```

相关文件：

- `MDK-ARM/APP/shoot.c`：3508 拉绳电机状态机。
- `Core/Src/freertos.c`：2 ms 周期任务，调用 `shoottask()` 并发送 CAN 电流。
- `MDK-ARM/Behaviors/Can_receive/Can_receive.c`：电机反馈解析和 CAN 电流帧打包。
- `MDK-ARM/APP/shoot_task/shoot_task.c`：供弹状态，以及拉绳逻辑需要共享的标志位。

状态机概要：

- `step1`：持续拉绳直到触发前光电，然后保持当前位置。
- `step2`：继续后拉，直到触发后光电。
- `step3`：根据编码器行程回线，到位后零速保持并等待下一次复位。

关键共享标志：

- `allow_flag`：`shoot_task.c` 会用它判断供弹行为。
- `roll_flag`：拉绳逻辑和供弹逻辑都会使用，不能改成 `static`。

## 清理规则

- Keil 编译产物不要提交。
- 新出现的 `.o`、`.d`、`.crf`、`.axf`、`.hex`、`.map` 等构建文件应保持忽略。
- `.uvguix.*` 是 Keil 用户会话文件，不应作为工程源码提交。
- 如果 `git status` 里出现新的编译产物，优先补充 `.gitignore`，不要直接提交产物。