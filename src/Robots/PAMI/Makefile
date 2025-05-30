##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -Og -g -ggdb -fomit-frame-pointer -falign-functions=16 -lm 
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -fno-exceptions -std=c++17
endif

# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = -lstdc++
endif

# Enable this if you want link time optimizations (LTO).
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x1400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x800
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, target, sources and paths
#

# Define project name here
PROJECT = asservNucleo

# Target settings.
MCU  = cortex-m4

# Imported source files and paths.
CHIBIOS  := ../../../ChibiOS/
CHIBIOS_CONTRIB = ../../../ChibiOS-Contrib/
CONFDIR  := ./cfg_chibios
BUILDDIR := ./build
DEPDIR   := ./.dep
SRCDIR   := ../../../src

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32g4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS_CONTRIB)/os/hal/hal.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/STM32/STM32G4xx/platform.mk
include $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_G431RB/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
# Other files (optional).
include $(CHIBIOS)/test/lib/test.mk
include $(CHIBIOS)/test/rt/rt_test.mk
include $(CHIBIOS)/test/oslib/oslib_test.mk
include $(CHIBIOS)/os/various/shell/shell.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk


# Define linker script file here
LDSCRIPT= $(STARTUPLD)/STM32G431xB.ld

# Custom part: Depending on a input variable ROBOT, the used main.cpp is different

## Check if the specified ROBOT exist
#AVAILABLE_ROBOTS=$(shell find src/Robots -mindepth 1 -type d -exec basename {} \;)
#ifeq (,$(wildcard $(SRCDIR)/Robots/$(ROBOT)/main.cpp))
#$(error Unknown ROBOT specified! Knowns are : $(AVAILABLE_ROBOTS))
#
#endif


ifneq ($(STAR),)  # enable the shell
	PAMI_STAR_DEFINE = -DSTAR
endif

ifneq ($(SHELL_ENABLE),)  # enable the shell
	SHELL_MODE_DEFINE = -DENABLE_SHELL
endif

SHELL_MODE_DEFINE = -DENABLE_SHELL
#$(error ALLSRC : >$(ALLCSRC)<)

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC)\
       $(SRCDIR)/usbcfg.c \
       $(SRCDIR)/util/exceptionVectors.c \
       $(TESTSRC) 


# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC) \
       $(wildcard $(SRCDIR)/Robots/PAMI/*.cpp ) \
       $(SRCDIR)/sampleStream/USBStream.cpp \
       $(SRCDIR)/Encoders/QuadratureEncoder.cpp \
       $(SRCDIR)/AsservMain.cpp \
       $(SRCDIR)/SpeedController/SpeedController.cpp \
       $(SRCDIR)/SpeedController/AdaptativeSpeedController.cpp \
       $(SRCDIR)/Pll.cpp \
       $(SRCDIR)/Regulator.cpp \
       $(SRCDIR)/Odometry.cpp \
       $(SRCDIR)/commandManager/CommandManager.cpp \
       $(SRCDIR)/commandManager/CommandList.cpp \
       $(SRCDIR)/commandManager/Commands/StraitLine.cpp \
       $(SRCDIR)/commandManager/Commands/Turn.cpp \
       $(SRCDIR)/commandManager/Commands/Goto.cpp \
       $(SRCDIR)/commandManager/Commands/GotoAngle.cpp \
       $(SRCDIR)/commandManager/Commands/GotoNoStop.cpp \
       $(SRCDIR)/commandManager/Commands/WheelSpeed.cpp \
       $(SRCDIR)/commandManager/Commands/OrbitalTurn.cpp \
       $(SRCDIR)/motorController/Mp6550.cpp \
       $(SRCDIR)/util/chibiOsAllocatorWrapper.cpp  \
       $(SRCDIR)/AccelerationLimiter/AbstractAccelerationLimiter.cpp \
       $(SRCDIR)/AccelerationLimiter/SimpleAccelerationLimiter.cpp \
       $(SRCDIR)/AccelerationLimiter/AdvancedAccelerationLimiter.cpp \
       $(SRCDIR)/AccelerationLimiter/AccelerationDecelerationLimiter.cpp \
       $(SRCDIR)/blockingDetector/OldSchoolBlockingDetector.cpp  \
       $(SRCDIR)/Communication/SerialIO.cpp \
       $(SRCDIR)/Communication/RaspIO.cpp 

    

# List ASM source files here.
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

# Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) $(TESTINC) $(CHIBIOS_CONTRIB)/os/various $(SRCDIR)

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef

#
# Project, target, sources and paths
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = $(SHELL_MODE_DEFINE) $(PAMI_STAR_DEFINE)

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user section
##############################################################################

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

#
# Common rules
##############################################################################

##############################################################################
# Custom rules
#
help :
	@echo "make                                 :   build with default robot config"
	@echo "make ROBOT=PAMI                      :   build using the robot config (PAMI here)"
	@echo "make ROBOT=myRobot STAR=true         :   build using the PAMI Robot config and set it in STAR mode "
	@echo "make flash                           :   load the generated elf to the board"
	@echo "make debug                           :   load the generated elf to the board & wait for a debugger to connect (with arm-none-eabi-gdb build/asservNucleo.elf -ex \"target remote :3333\" )"
	@echo "make robots                          :   print the knows robots"
	
robots :
	@echo "Available robots :  $(AVAILABLE_ROBOTS)" 
	
flash : all
	openocd -c "tcl_port disabled" -c "telnet_port disabled" -c "source [find board/st_nucleo_g4.cfg]" -c "stm32g4x.cpu configure -rtos chibios" -c "init" -c "reset halt" -c "flash write_image erase unlock $(BUILDDIR)/$(PROJECT).elf" -c "verify_image $(BUILDDIR)/$(PROJECT).elf" -c "reset run" -c "shutdown"

debug : all
	openocd -c "tcl_port disabled" -c "telnet_port disabled" -c "source [find board/st_nucleo_g4.cfg]" -c "stm32g4x.cpu configure -rtos chibios" -c "init" -c "reset halt" -c "flash write_image erase unlock $(BUILDDIR)/$(PROJECT).elf" -c "verify_image $(BUILDDIR)/$(PROJECT).elf" -c "reset halt"

	

#
# Custom rules
##############################################################################
