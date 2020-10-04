#!/bin/bash

openocd -c "tcl_port disabled" -c "telnet_port disabled" -c "source [find board/st_nucleo_f4.cfg]" -c "stm32f4x.cpu configure -rtos chibios" -c "init" -c "reset halt" -c "flash write_image erase unlock ./build/asservNucleo.elf" -c "verify_image ./build/asservNucleo.elf" -c "reset run" -c "shutdown"