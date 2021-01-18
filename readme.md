## Project for Microprocessor technology 2

## Phone controlled car

## Table of contents
* [Design assumptions](#design-assumptions)
* [Technologies](#technologies)
* [Used hardware](#used-hardware)
* [Hardware setup](#hardware-setup)
* [Authors](#authors)

## Design assumptions:
- Processor used: STM32F103 or STM32F411Re
- USB On-The-Go controlled car using UART and phone with android system.
- Powerbank as power supply
- Car construction placed on 2WD chassis
- MCP23S08 port expander controlled with SPI 
- L293D half-H driver used to control motors
- PWM used to change speed of dc motors, and LED brightness

## Technologies
Project is created with:
* Keil uVision5
* STM32CubeIDE 1.5.0

## Used hardware:
- STM32 NUCLEO-F103RB - STM32F103RBT6 ARM Cortex M3

![NUCLEO-F103RB](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/nucleo_f103.jpeg)

 or STM32 NUCLEO-F411RE - STM32F411RE ARM Cortex M4
 
![NUCLEO-F411RE](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/nucleo.jpeg)

- MCP23S08 expander 

![MCP23S08](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/mcp23s06_expander.jpeg)

- L293D half-H driver

![L293D](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/L293D_h_bridge.jpeg)

- DC motors

![DC_motors](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/dc_motor.jpg)

- Buzzer

![Buzzer](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/buzzer.jpeg)

![Buzzer2](https://github.com/JakMir98/Phone-controlled-car-stm32/blob/main/Images/buzzer2.jpg)

- 2WD chassis

![2WD](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/2WD.png)

## Hardware setup

![Car view](https://github.com/JakMir98/Phone-controlled-car-stm32/blob/main/Images/completeCar.jpg)

## Authors
Andrzej Kasica, Jakub Mirota 
