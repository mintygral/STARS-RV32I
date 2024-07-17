# STARS 2024 Design Final Project

- [Table of Contents](#stars-2024-design-final-project)
  - [CPU Team 5 Members](#cpu-team-5)
  - [Demo Video](#demo-video)
  - [Project Type](#project-type)
  - [Supporting Equipment](#supporting-equipment)
      - [4x4 keypad](#4x4-keypad)
      - [16x2 GDM1602K LCD screen](#16x2-gdm1602k-lcd-screen)
  - [RTL Diagrams](#rtl-diagrams)
    - [Top level RTLs](#top-level-rtls)
      - [General](#general)
      - [Detailed](#detailed)
    - [Sub-Modules](#sub-modules)
      - [Control unit](#control-unit)
      - [Register file](#register-file)
      - [Program counter (PC)](#program-counter-pc)
      - [Arithmetic logic unit (ALU)](#arithmetic-logic-unit-alu)
      - [Memory control (request unit)](#memory-control-request-unit)

## CPU TEAM 5
* Berin Celik
* Shrienidhi Gopalakrishnan
* Medha Shinde
* Ainsley Strothkamp
* Alex Tauriainen
* Peer Mentor: Pranav Wadhwa

## Demo Video
Click on the thumbnail to go to the youtube video of our demo!

[![YouTube](http://i.ytimg.com/vi/anySU9C3ncY/hqdefault.jpg)](https://www.youtube.com/watch?v=anySU9C3ncY)

## Project Type
Our project was to create a 32-bit single-cycle RISC-V CPU. We integrated this with a 4x4 keypad for inputs and a 16x2 LCD screen for outputs. 

## Supporting Equipment

#### 4x4 keypad
For our calculator program:

* 0-9 for entering numbers
* "*" for confirming input
* "#" for clearing input
* A for addition
* B for subtraction
* C for multiplication
* D for division
  
![keypad](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/io_components/keypad.jpeg)

#### 16x2 GDM1602K LCD screen
![lcd_screen](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/io_components/lcd_screen.jpg)

## RTL Diagrams

### Top level RTLs

*Note*: These top-level RTLs include a temperature sensor input that is not included in our final demo video. This is because we faced space constraints due to the FPGA so we were not able to fully integrate the temperature sensor as we were planning to.

#### General
![top_general](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/top_level/top_general.png)

#### Detailed
![top_detail](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/top_level/top_detail.jpg)

### Sub-Modules

#### Control unit
![control_unit](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/control_unit.png)

![tb_control_unit](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/tb_sub_modules/tb_control_unit.png)

#### Register file
![reg_file](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/reg_file.png)

![tb_reg_file](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/tb_sub_modules/tb_reg_file.png)

#### Program counter (PC)
![pc_1](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/pc_1.png)

![pc_2](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/pc_2.png)

![tb_pc](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/tb_sub_modules/tb_pc.png)

#### Arithmetic logic unit (ALU)
![alu](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/alu.png)

![tb_alu](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/tb_sub_modules/tb_alu.png)

#### Memory control (request unit)
![mem_control](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/mem_control.png)

![tb_mem_control](https://github.com/STARS-Design-Track-2024/nebula-ii-team-05/blob/main/docs/team_05/sub_modules/tb_sub_modules/tb_memcontrol.png)


