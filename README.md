# STM32F407 + FreeRTOS 智能电机控制与状态监测系统

基于 STM32F407VGT6、FreeRTOS、BTS7960/IBT-2 和带霍尔编码器的 MG310 直流减速电机，实现电机 PWM 驱动、编码器测速、PI 速度闭环控制、多传感器状态采集、故障保护、UART/CAN 通信、Flash 参数保存以及 OLED 本地显示。

项目从底层硬件驱动开始搭建，并逐步完成电机控制、速度闭环、状态监测、故障管理、通信和 FreeRTOS 系统监控。

---

## 1. 项目简介

本项目设计了一套基于 STM32F407VGT6 的智能直流电机控制与状态监测系统。

系统采用 FreeRTOS 进行任务划分，将电机控制、传感器采集、通信、故障检测、显示和按键处理划分为独立任务。

主要实现：

- TIM1 PWM 电机驱动
- TIM3 编码器测速
- PI 速度闭环控制
- ADC + DMA 多通道数据采集
- 电机电流检测
- 电机供电电压检测
- NTC 温度检测
- 过流、过压、欠压、过温保护
- 堵转检测
- 速度异常检测
- 故障自动恢复与 LOCKED 锁定
- UART 命令控制
- CAN 总线通信
- Flash 参数掉电保存
- CRC 数据校验
- SSD1306 OLED 状态显示
- 按键控制
- 蜂鸣器故障/操作提示
- FreeRTOS 任务状态监控
- FreeRTOS CPU 占用率监控
- DWT CYCCNT 高精度运行时间统计

---

## 2. 系统架构

```text
                         ┌─────────────────────┐
                         │    STM32F407VGT6    │
                         │                     │
                         │      FreeRTOS       │
                         └──────────┬──────────┘
                                    │
          ┌─────────────────────────┼─────────────────────────┐
          │                         │                         │
          ▼                         ▼                         ▼
   Motor Control              Sensor Manager             Communication
          │                         │                         │
     TIM1 PWM                 ADC + DMA                 USART2 / CAN1
          │                         │                         │
          ▼                    ┌────┼────┐             ┌─────┴─────┐
       BTS7960                 │    │    │             │           │
          │                  Current Voltage Temp      UART        CAN
          ▼
       MG310 Motor
          │
          ▼
     Hall Encoder
          │
          ▼
      TIM3 Encoder
          │
          ▼
       RPM计算
          │
          ▼
     PI Speed Control
          │
          └─────────────── Feedback ────────────────┐
                                                    │
                                                    ▼
                                               Motor Control

FreeRTOS 软件架构

系统主要任务：

Task  	  Priority	 功能
MotorTask	  40	     PI速度控制、电机控制
FaultTask	  40	     故障检测与恢复
CommTask	  24	     UART/CAN通信
SensorTask	24	   ADC及传感器数据更新
DisplayTask	8	     OLED显示
KeyTask	-	         按键扫描及蜂鸣器反馈
