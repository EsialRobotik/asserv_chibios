# Check if the specified ROBOT exist
AVAILABLE_ROBOTS=$(shell find src/Robots -mindepth 1 -type d -exec basename {} \;)
ifeq (,$(wildcard $(SRCDIR)/Robots/$(ROBOT)/main.cpp))
$(error Unknown ROBOT specified! Knowns are : $(AVAILABLE_ROBOTS))
endif

SRCDIR   := ./src
make -c $(SRCDIR)/Robots/$(ROBOT)/ 

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(wildcard $(SRCDIR)/Robots/$(ROBOT)/*.c ) \
       $(SRCDIR)/usbcfg.c \
       $(SRCDIR)/util/exceptionVectors.c \
       $(TESTSRC) 

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC) \
       $(wildcard $(SRCDIR)/Robots/$(ROBOT)/*.cpp ) \
       $(SRCDIR)/motorController/Vnh5019.cpp \
       $(SRCDIR)/motorController/Md22.cpp \
       $(SRCDIR)/sampleStream/USBStream.cpp \
       $(SRCDIR)/Encoders/QuadratureEncoder.cpp \
       $(SRCDIR)/Encoders/ams_as5048b.cpp \
       $(SRCDIR)/Encoders/MagEncoders.cpp \
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
       $(SRCDIR)/util/chibiOsAllocatorWrapper.cpp  \
       $(SRCDIR)/AccelerationLimiter/AbstractAccelerationLimiter.cpp \
       $(SRCDIR)/AccelerationLimiter/SimpleAccelerationLimiter.cpp \
       $(SRCDIR)/AccelerationLimiter/AdvancedAccelerationLimiter.cpp \
       $(SRCDIR)/AccelerationLimiter/AccelerationDecelerationLimiter.cpp \
       $(SRCDIR)/blockingDetector/SpeedErrorBlockingDetector.cpp \
       $(SRCDIR)/blockingDetector/OldSchoolBlockingDetector.cpp \

    

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
UDEFS = $(SHELL_MODE_DEFINE)

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
	@echo "make ROBOT=myRobot                   :   build using the myRobot config. A myRobot dir must be present in src/Robots"
	@echo "make ROBOT=myRobot SHELL_ENABLE=true :   build using the myRobot config and enable the shell (Ie: ENABLE_SHELL will be defined and must be handled in src/Robots/myRobot/main.cpp ! ) "
	@echo "make flash                           :   load the generated elf to the board"
	@echo "make debug                           :   load the generated elf to the board & wait for a debugger to connect (with arm-none-eabi-gdb build/asservNucleo.elf -ex \"target remote :3333\" )"
	@echo "make robots                          :   print the knows robots"
	
robots :
	@echo "Available robots :  $(AVAILABLE_ROBOTS)" 
	
flash : all
	openocd -c "tcl_port disabled" -c "telnet_port disabled" -c "source [find board/st_nucleo_f4.cfg]" -c "stm32f4x.cpu configure -rtos chibios" -c "init" -c "reset halt" -c "flash write_image erase unlock $(BUILDDIR)/$(PROJECT).elf" -c "verify_image $(BUILDDIR)/$(PROJECT).elf" -c "reset run" -c "shutdown"

debug : all
	openocd -c "tcl_port disabled" -c "telnet_port disabled" -c "source [find board/st_nucleo_f4.cfg]" -c "stm32f4x.cpu configure -rtos chibios" -c "init" -c "reset halt" -c "flash write_image erase unlock $(BUILDDIR)/$(PROJECT).elf" -c "verify_image $(BUILDDIR)/$(PROJECT).elf" -c "reset halt"

	

#
# Custom rules
##############################################################################
