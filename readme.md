## Project for Microprocessor technology 2

## Remote controlled car

## Table of contents
* [Design assumptions](#design-assumptions)
* [Technologies](#technologies)
* [Used hardware](#used-hardware)
* [Authors](#authors)

## Design assumptions:
- Processor used: STM32F103 or STM32F411Re
- Remote controlled car using bluetooth module and application on phone with android system.
- Powerbank as power supply
- Car construction placed on 2WD chassis
- Bluetooth module controlled with UART
- MCP23S08 port expander controlled with SPI 
- L293D half-H driver used to control motors
- PWM used to change speed of dc motors, and LED brightness

## Technologies
Project is created with:
* Keil uVision5
* STM32CubeIDE 1.5.0

## Used hardware:
- STM32 NUCLEO-F103RB - STM32F103RBT6 ARM Cortex M3 or STM32 NUCLEO-F411RE - STM32F411RE ARM Cortex M4
![NUCLEO-F103RB](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/nucleo_f103.jpeg)
![NUCLEO-F411RE](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/nucleo.jpeg)

- MCP23S08 expander 

![MCP23S08](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/mcp23s06_expander.jpeg)

- L293D half-H driver

![L293D](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/L293D_h_bridge.jpeg)

- DC motors

![DC_motors](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/dc_motor.jpg)

- Buzzer or speaker

![Buzzer](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/buzzer.jpeg)
![Speaker](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/speaker.jpg)

- 2WD chassis

![2WD](https://github.com/JakMir98/Remote-controlled-car-stm32/blob/main/Images/2WD.png)

## Authors
Andrzej Kasica, Jakub Mirota 
