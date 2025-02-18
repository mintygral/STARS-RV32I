# STARS 2024 Design Final Project

- [STARS 2024 Design Final Project](#stars-2024-design-final-project)
  - [CPU Team 5](#cpu-team-5)
  - [Demo Video](#demo-video)
  - [Project Type](#project-type)
  - [Supporting Equipment](#supporting-equipment)
      - [4x4 keypad](#4x4-keypad)
      - [16x2 GDM1602K LCD screen](#16x2-gdm1602k-lcd-screen)
  - [Repository Tree](#repository-tree)
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


## CPU Team 5
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

#### GDSII Image
![image](https://github.com/user-attachments/assets/c9b98c74-36cc-4ddd-bc61-5db633397994)

## Repository Tree
```
📦 
├─ .gitattributes
├─ README.md
├─ build
│  ├─ template
│  ├─ template.asc
│  ├─ template.bin
│  ├─ template.json
│  └─ template.v
├─ docs
│  ├─ io_components
│  │  ├─ keypad.jpeg
│  │  └─ lcd_screen.jpg
│  ├─ sub_modules
│  │  ├─ alu.png
│  │  ├─ control_unit.png
│  │  ├─ mem_control.png
│  │  ├─ memcontrol_std.png
│  │  ├─ pc_1.png
│  │  ├─ pc_2.png
│  │  ├─ reg_file.png
│  │  └─ tb_sub_modules
│  │     ├─ tb_alu.png
│  │     ├─ tb_control_unit.png
│  │     ├─ tb_memcontrol.png
│  │     ├─ tb_pc.png
│  │     └─ tb_reg_file.png
│  └─ top_level
│     ├─ top_detail.jpg
│     └─ top_general.png
├─ pinmap.pcf
└─ source
   ├─ build
   │  ├─ 32bitcpu.asc
   │  ├─ 32bitcpu.bin
   │  └─ 32bitcpu.json
   ├─ ice40hx8k.sv
   ├─ pinmap.pcf
   ├─ programs
   │  └─ find_max.asm
   ├─ small_integration
   │  ├─ sv
   │  │  ├─ ALU_reg.sv
   │  │  ├─ PC_ALU_integration.sv
   │  │  ├─ control_reg.sv
   │  │  ├─ dmem_ALU.sv
   │  │  ├─ dmem_reg.sv
   │  │  └─ register_ALU_integration.sv
   │  └─ tb
   │     ├─ tb_ALU_reg.sv
   │     ├─ tb_PC_ALU_integration.sv
   │     ├─ tb_control_reg.sv
   │     ├─ tb_dmem_ALU.sv
   │     └─ tb_dmem_reg.sv
   ├─ sub_modules
   │  ├─ sv
   │  │  ├─ ALU.sv
   │  │  ├─ bin_to_LCD.sv
   │  │  ├─ control_unit.sv
   │  │  ├─ data_memory.sv
   │  │  ├─ display.sv
   │  │  ├─ edge_detector.sv
   │  │  ├─ instruction_fetch.sv
   │  │  ├─ instruction_memory.sv
   │  │  ├─ key_translate.sv
   │  │  ├─ keypad.sv
   │  │  ├─ keypad_interface.sv
   │  │  ├─ lcd_controller.sv
   │  │  ├─ memcontrol.sv
   │  │  ├─ pc.sv
   │  │  ├─ ram.sv
   │  │  ├─ register_file.sv
   │  │  ├─ request_unit_stars.sv
   │  │  ├─ ssdec.sv
   │  │  ├─ synckey.sv
   │  │  └─ temp_sensor.sv
   │  └─ tb
   │     ├─ tb_ALU.sv
   │     ├─ tb_control_unit.sv
   │     ├─ tb_instruction_fetch.sv
   │     ├─ tb_memcontrol.sv
   │     ├─ tb_pc.sv
   │     ├─ tb_register_file.sv
   │     └─ tb_temp_sensor.sv
   ├─ support
   │  ├─ cells_map_timing.v
   │  ├─ cells_sim_timing.v
   │  ├─ lock_bb_top.sv
   │  ├─ uart.v
   │  ├─ uart_rx.v
   │  └─ uart_tx.v
   └─ top
      ├─ sv
      │  ├─ Makefile
      │  ├─ cpu.mem
      │  ├─ cpu_core.sv
      │  ├─ temp_interface.sv
      │  ├─ top.sv
      │  ├─ top_ram.sv
      │  └─ top_real.sv
      └─ tb
         ├─ tb.sv
         └─ tb_cpu_core.sv
```
©generated by [Project Tree Generator](https://woochanleee.github.io/project-tree-generator)

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


