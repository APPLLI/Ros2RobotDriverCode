## 阿克曼小车 ROS2 底层驱动与导航项目（2025 年 7 月版）

本仓库包含基于 **STM32F103C8** 的阿克曼转向小车底层驱动代码，以及部署在 **树莓派 4B + Ubuntu 22.04 + ROS2 Humble** 上的路径规划与多传感器融合导航示例代码。

- **底层 MCU**：STM32F103C8（Keil 工程，C 语言）
- **上位机**：树莓派 4B，Ubuntu 22.04，ROS2 Humble，Python 3
- **主要功能**：
  - 双路电机速度 PID 闭环控制
  - 舵机转向控制（阿克曼转向）
  - 编码器测速（TIM3 / TIM4 编码器模式）
  - MPU6050 DMP 姿态解算（Yaw 角）
  - 电池电压监测与低压保护
  - 自定义串口通信协议（STM32 ⇄ ROS2）
  - ROS2 侧 A* 网格路径规划与路径跟踪
  - 激光雷达 + IMU + 深度相机多传感器融合接口骨架

---

## 1. 版本与编码说明

- **当前代码版本**：2025-07
- **初始版本时间**：2024-07
- **文件编码**：统一使用 **UTF-8**（源代码与文档均建议为 UTF-8，无 BOM）

### 1.1 版本变更概览（2024 → 2025）

- **2024-07 初始版**
  - 实现 STM32F103C8 底层驱动：PWM、电机/舵机控制、编码器测速、MPU6050 DMP、ADC 电压检测等。
  - 实现与上位机之间的自定义串口二进制协议。
  - 双路电机增量式 PID 控制，以“快速响应”为主。

- **2025-07 升级版**
  - 调整 PID 参数（降低 Kp/Kd，引入小幅 Ki），改善中低速稳定性和稳态误差。
  - 串口发送频率从约 70Hz 调整为约 50Hz，降低上位机和串口负载。
  - Yaw 角编码放大系数由 50 调整为 100，提高角度分辨率，利于导航算法。
  - 在上位机侧新增 `ros2_nav/astar_nav_node.py`，提供 A* 路径规划 + 多传感器融合导航接口示例。
  - 在关键 C 文件（如 `USER/MBOT.c`、`mbot/PID/pid.c`）加入版本历史注释。

---

## 2. 仓库结构概览

主要目录与文件说明（只列出与开发相关的部分）：

- `USER/`
  - `MBOT.c`：STM32 主函数入口，完成底层外设初始化和与 ROS2 的串口数据交互。
- `mbot/`
  - `CONTROL/`
    - `control.c`：5ms 外部中断（MPU6050 INT）控制逻辑，实现一次控制周期：电压检测、编码器测速、PID 计算、舵机控制、PWM 更新等。
  - `PID/`
    - `pid.c` / `pid.h`：双路电机速度增量式 PID 控制模块（2025 调整版）。
  - `motor/`
    - `motor.c` / `motor.h`：左右电机与前轮舵机 PWM 控制、低压保护等。
  - `usart/`
    - `mbotLinuxUsart.c` / `mbotLinuxUsart.h`：自定义串口协议解析与封装。
  - `show/`
    - `show.c`：用于 PC 调试打印的显示函数（可选）。
- `Mbot_HARDWARE/`
  - `ADC/`：电池电压采样。
  - `ENCODER/`：TIM3/TIM4 编码器模式初始化与测速。
  - `MOTOR_PWM/`：TIM1/TIM2 PWM 初始化（电机+舵机）。
  - `MPU6050/`：MPU6050 + DMP 驱动与姿态解算。
  - `EXTI/`：外部中断（PA8 / EXTI8）初始化。
  - `USART1/`、`USART3/`：串口初始化。
- `SYSTEM/`、`STM32F10x_FWLib/`：芯片启动文件、时钟与标准外设库。
- `ros2_nav/`
  - `astar_nav_node.py`：ROS2 Humble 示例节点，A* 路径规划 + 多传感器融合接口 + 简单路径跟踪（Python）。

---

## 3. 底层 STM32 控制逻辑

### 3.1 主函数与主循环（`USER/MBOT.c`）

核心流程：

1. 关闭 JTAG、配置 NVIC 优先级。
2. 初始化 `delay`、`LED`、`BEEP`。
3. 初始化串口：`USART1`（对接 ROS2/树莓派）、`USART3`（调试）。
4. 初始化 I2C 与 MPU6050（含 DMP）。
5. 初始化编码器接口：`TIM3`（右轮）、`TIM4`（左轮）。
6. 初始化 ADC（电池电压）、PWM（TIM2 电机，TIM1 舵机）、PID、外部中断（MPU6050 INT 触发 5ms 控制周期）。
7. 主循环中周期性：
   - 按约 50Hz 通过 `USART1` 向 ROS2 发送当前左右轮速度与 Yaw 角。
   - 调用 `getAngle()` 读取并更新 Yaw。

主循环发送逻辑（简化）：

```c
// USER/MBOT.c 片段（2025-07 版）
while(1)
{
    // 约 20ms 发送一帧数据（~50Hz）
    if(sendCount==0)
    {
        // Yaw * 100 提升角度分辨率
        usartSendData(USART1,
                      (short)leftSpeedNow,
                      (short)rightSpeedNow,
                      (short)(yaw*100),
                      sendCtrlFlag);
        sendCount++;
    }
    else
    {
        sendCount++;
        if(sendCount==25)
            sendCount=0;
    }
    // 实时获取 Yaw 角（0~359°，并做漂移补偿）
    getAngle(&yaw,&yaw_acc_error);
}
```

> 注意：控制闭环（PID、电机/舵机更新）在 5ms 中断中完成，主循环主要用于上位机通信和数据刷新。

### 3.2 5ms 控制中断（`mbot/CONTROL/control.c`）

- 外部中断源：`PA8` 接 MPU6050 INT 引脚 → `EXTI9_5_IRQHandler`。
- 每 5ms 执行一次完整控制周期：

```c
void EXTI9_5_IRQHandler(void) 
{                                                         
    EXTI_ClearITPendingBit(EXTI_Line8);

    Led_Flash(200);                                  // 心跳灯，指示系统正常运行
    yaw_acc_error += FIVE_MS_ERROR;                  // 累计 Yaw 漂移误差
    Get_battery_volt_average(&Voltage,100);          // 电池电压平均值
    Get_Motor_Speed(&leftSpeedNow,&rightSpeedNow);   // 编码器测速

    // 更新 PID 设定值与反馈值
    pid_Task_Letf.speedSet  = leftSpeedSet;
    pid_Task_Right.speedSet = rightSpeedSet;
    pid_Task_Letf.speedNow  = leftSpeedNow;
    pid_Task_Right.speedNow = rightSpeedNow;

    Pid_Ctrl(&motorLeft,&motorRight);                // PID 调整左右轮 PWM 目标
    Steer_Ctrl(frontAngleSet,&motorFrontSteer);      // 计算舵机 PWM

    if(Turn_Off(Voltage) == 0)                       // 电压正常
    {	
        BEEP_Disable();
        Set_Pwm(motorLeft,motorRight,motorFrontSteer);
    } 	  
    else                                             // 电压异常，保护
    {
        BEEP_Flash(20);
    }
}
```

### 3.3 串口中断接收（`USART1_IRQHandler`）

- 接收来自树莓派 / ROS2 的控制指令，例如：
  - 左右轮速度设定值 `leftSpeedSet`、`rightSpeedSet`
  - 前轮转向角设定值 `frontAngleSet`
  - 控制标志位 `receCtrlFlag`

```c
void USART1_IRQHandler()
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
        usartReceiveOneData(USART1,
                            &leftSpeedSet,
                            &rightSpeedSet,
                            &frontAngleSet,
                            &receCtrlFlag);
    }
}
```

---

## 4. 底层关键模块概述

### 4.1 PID 速度控制（`mbot/PID/pid.c`）

- 使用 **增量式 PID**，以定点运算（×1024）实现高效控制。
- 2025-07 版本 PID 参数（左右轮相同）：
  - `Kp = 3.5`
  - `Ki = 0.02`
  - `Kd = 12.0`
  - `Ur = 3500`（输出限幅）

PID 初始化（简化）：

```c
void PID_Init(void)
{
    // 左轮
    pid_Task_Letf.Kp = 1024 * 3.5f;
    pid_Task_Letf.Ki = 1024 * 0.02f;
    pid_Task_Letf.Kd = 1024 * 12.0f;
    pid_Task_Letf.Ur = 1024 * 3500;
    ...
    // 右轮
    pid_Task_Right.Kp = 1024 * 3.5f;
    pid_Task_Right.Ki = 1024 * 0.02f;
    pid_Task_Right.Kd = 1024 * 12.0f;
    pid_Task_Right.Ur = 1024 * 3500;
}
```

### 4.2 电机与舵机 PWM（`mbot/motor/motor.c` + `Mbot_HARDWARE/MOTOR_PWM/pwm.c`）

- 电机 PWM：TIM2，4 路通道控制左右轮。
- 舵机 PWM：TIM1_CH2N，50Hz，SG90 等舵机。
- 舵机角度控制：限制在 `-50° ~ +50°`，中心 90°。

```c
// frontAngleSet : -50 ~ +50
void Steer_Ctrl(int frontAngleSet,int *motorFrontSteer)
{
    if(frontAngleSet > MAX_FRONT_ANGLE_SET)  frontAngleSet = MAX_FRONT_ANGLE_SET;
    if(frontAngleSet < -MAX_FRONT_ANGLE_SET) frontAngleSet = -MAX_FRONT_ANGLE_SET;

    if(frontAngleSet == 0) {
        *motorFrontSteer = 4500; // 1.5ms -> 90°
    } else if(frontAngleSet > 0) {  // left
        *motorFrontSteer = 4500 - (int)(myabs(frontAngleSet) * 33.3f + 0.5f);
    } else {                        // right
        *motorFrontSteer = 4500 + (int)(myabs(frontAngleSet) * 33.3f + 0.5f);
    }
}
```

### 4.3 编码器测速与电池监测

- 编码器：TIM3/TIM4 编码器模式，读取单位时间内计数变化，结合轮径与分辨率计算 mm/s 速度。
- ADC 电压：PA4 采样，经分压计算电池总电压，低于 11.1V 触发 `Turn_Off()` 关闭电机并报警。

---

## 5. 串口通信协议（STM32 ⇄ ROS2）

### 5.1 帧格式

通用格式：

```text
55 AA size [数据区 size 字节] crc8 0D 0A
```

- 帧头：`0x55 0xAA`（2 字节）
- 长度：`size`（1 字节），表示数据区长度
- 数据区（本项目中为 7 字节）：
  - 接收（ROS2 → STM32）：
    - 左速度设定值 `leftVelSet`（short，2 字节）
    - 右速度设定值 `rightVelSet`（short，2 字节）
    - 转向角设定值 `frontSteerAngleSet`（short，2 字节）
    - 控制标志 `ctrlFlag`（1 字节）
  - 发送（STM32 → ROS2）：
    - 左速度当前值（short，2 字节，单位 mm/s）
    - 右速度当前值（short，2 字节，单位 mm/s）
    - Yaw 角（short，2 字节，值为 `yaw * 100`）
    - 控制标志（1 字节）
- CRC8：`getCrc8()` 计算（1 字节）
- 帧尾：`0x0D 0x0A`（2 字节）

### 5.2 CRC8 算法

```c
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc = 0;
    unsigned char i;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc & 0x01)
                crc = (crc >> 1) ^ 0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
```

---

## 6. ROS2 路径规划与多传感器融合导航（`ros2_nav/astar_nav_node.py`）

在 ROS2 Humble 环境中，本项目提供了一个示例节点：

- **节点名**：`multi_sensor_astar_nav_node`
- **编写语言**：Python 3（rclpy）
- **主要功能**：
  - 订阅全局栅格地图 `/map`（`nav_msgs/OccupancyGrid`），使用 A* 进行路径规划。
  - 订阅 `/odom`（`nav_msgs/Odometry`）获取里程计位姿。
  - 订阅 `/imu/data`（`sensor_msgs/Imu`）、`/scan`（`sensor_msgs/LaserScan`）、`/camera/depth/points`（`sensor_msgs/PointCloud2`）作为多传感器融合接口。
  - 订阅目标位姿 `/goal_pose`（`geometry_msgs/PoseStamped`）触发规划。
  - 发布路径 `/planned_path`（`nav_msgs/Path`）与控制命令 `/cmd_vel`（`geometry_msgs/Twist`）。

### 6.1 A* 网格规划器结构

类 `AStarPlanner` 负责对 `OccupancyGrid` 做 8 邻域 A* 搜索：

- 支持世界坐标 ↔ 栅格坐标转换。
- 支持障碍阈值设置（栅格值 > 阈值视为障碍）。
- 规划成功返回一系列栅格索引，可进一步转换为 Path。

### 6.2 多传感器融合接口（示例骨架）

`MultiSensorAstarNavNode` 中预留了以下回调：

- `odom_callback`：更新当前位姿估计（简单采用 /odom）。
- `imu_callback`：IMU 姿态数据（可用于 EKF/UKF 融合）。
- `scan_callback`：激光数据（可构造局部成本地图）。
- `depth_callback`：深度点云（可用于 3D 障碍检测）。

> 当前示例 **未内置复杂 EKF**，仅提供接口与注释，方便你根据实际需求接入 `robot_localization` 等通用包。

### 6.3 简单路径跟踪控制

- 在控制定时器（默认 50Hz）中：
  - 从 `current_path` 中找到距离当前车位姿最近的点。
  - 选择更远一点的“前瞻点”（look-ahead）。
  - 使用纯追踪思想计算目标航向与当前 yaw 的差值，生成线速度与角速度：
    - `linear.x`：0 ~ 0.5m/s，随偏航误差减小而增大。
    - `angular.z`：`1.5 * yaw_error`。
  - 发布到 `/cmd_vel`，再经中间桥接节点/串口发送到 STM32。

---

## 7. 开发与使用建议

### 7.1 编码与工具链

- **STM32 部分**：
  - 建议使用 Keil uVision / MDK-ARM，确保工程设置为 UTF-8（或至少源码用 UTF-8 保存）。
  - 若从其他编辑器（VS Code ）修改文件，保存格式请选择 UTF-8。

- **ROS2 / Python 部分**：
  - 推荐使用 VS Code，Python 文件统一为 UTF-8 编码，首行已注明：
    - `# -*- coding: utf-8 -*-`

### 7.2 ROS2 端快速运行示例（简要）

1. 在树莓派上创建工作空间并拷贝 `ros2_nav` 目录到某个 ROS2 包中（如 `ackermann_nav`）：
2. 在 `ackermann_nav` 的 `setup.py` 或 `package.xml` 中注册该节点（略）。
3. 启动基础地图与定位（示例）：

```bash
ros2 launch slam_toolbox online_async_launch.py    # 或使用已有 map_server + amcl
```

4. 启动导航节点：

```bash
ros2 run ackermann_nav astar_nav_node.py
```

5. 发布目标点（可通过 RViz2 “2D Goal Pose” 或命令行发布 `/goal_pose`）。

> 实际部署时，还需要一个**桥接节点**，在 `/cmd_vel` 与串口协议之间互相转换（例如使用 Python 或 C++ 订阅 `/cmd_vel`，按协议封装后发送给 STM32）。

---

## 8. 后续可扩展方向

- 在 STM32 侧增加“速度设定斜率限制”和“转向角速度限制”，进一步提升行驶平顺性与安全性。
- 将当前 ROS2 导航节点与 `nav2` 框架整合，使用 A* 作为全局规划插件。
- 使用 `robot_localization` 构建 IMU + 里程计 + LiDAR / 深度相机融合定位。
- 增加 GitHub Actions，用于自动构建与静态检查（clang-tidy / flake8 / mypy 等）。

---

欢迎基于本项目进行二次开发和教学使用，建议在派生项目中保留版本注释与主要技术文档，方便后续维护与交流。  
如需进一步的配置示例（例如 ROS2 包的 `setup.py`、`launch` 文件等），可以在 Issue 中提出或自行扩展本仓库。


