##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -Og -g -ggdb  -lm
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT =  -std=c++17
endif

# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = -lstdc++
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
PROJECT = Simulation

# Target settings.
#MCU  = cortex-m4

# Imported source files and paths.
BUILDDIR := ./build
DEPDIR   := ./.dep
SRCDIR   := ../../../src

          
# Find all the C and C++ files we want to compile
# Note the single quotes around the * expressions. The shell will incorrectly expand these otherwise, but we want to send the * directly to the find command.
# SRCS := $(shell find $(SRC_DIRS) -name '*.cpp' -or -name '*.c' -or -name '*.s')

CPPSRC=$(wildcard $(SRCDIR)/Robots/Simulation/*.cpp ) \
       $(wildcard $(SRCDIR)/robotStub/*.cpp ) \
       $(wildcard $(SRCDIR)/Robots/Simulation/chibiosStub/*.cpp ) \
       $(SRCDIR)/sampleStream/msgpack11.cpp \
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
       $(SRCDIR)/AccelerationLimiter/AbstractAccelerationLimiter.cpp \
       $(SRCDIR)/AccelerationLimiter/SimpleAccelerationLimiter.cpp \
       $(SRCDIR)/AccelerationLimiter/AdvancedAccelerationLimiter.cpp \
       $(SRCDIR)/AccelerationLimiter/AccelerationDecelerationLimiter.cpp \
       $(SRCDIR)/blockingDetector/OldSchoolBlockingDetector.cpp 
   
   # Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) $(TESTINC) $(CHIBIOS_CONTRIB)/os/various $(SRCDIR) \
			$(shell find . -type d)  \
			$(SRCDIR)/Encoders \
			$(SRCDIR)/motorController \
			$(SRCDIR)/sampleStream

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef
   
   
   
include native.mk
include rules.mk
   
  