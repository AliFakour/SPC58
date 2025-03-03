##############################################################################
# This file is automatically generated and can be overwritten, do no change
# this file manually.
##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

ifeq ($(OS),Windows_NT)
WINDOWS := yes
PATH_SEPARATOR=;
else
WINDOWS := no
PATH_SEPARATOR=:
endif

COMPILER=freegcc


# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2  -gdwarf-2 -fomit-frame-pointer -falign-functions=16 -fno-gcse -std=gnu99
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -fno-exceptions
endif

# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Enable this if you want to use the C++ linker.
ifeq ($(USE_CPLUSPLUS_LINKER),)
  USE_CPLUSPLUS_LINKER = no
endif

# Linker options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = -lm,-lc,-z,max-page-size=32768
endif

# If enabled, this option allows to compile the application in VLE mode.
ifeq ($(USE_VLE),)
  USE_VLE = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

ifeq ($(USE_VERBOSE_COMPILE),yes)
  AT = 
else
  AT = @
  .SILENT:
endif

# Enable this if you want to create static library.
ifeq ($(CREATE_LIB),)
  CREATE_LIB = false
endif

#
# Build global options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = out

# Imported source files
include components/components.mak

# Checks if there is a user mak file in the project directory.
ifneq ($(wildcard user.mak),)
    include user.mak
endif

# Default linker script file
LDSCRIPT = application.ld

# Override with user.ld if it does exist
ifneq ($(wildcard user.ld),)
LDSCRIPT = user.ld
endif

# Override with compiler specific linker script if it does exist
# COMPILER = freegcc, hightec, ghs
ifneq ($(wildcard user_$(COMPILER).ld),)
LDSCRIPT = user_$(COMPILER).ld
endif

# Source files located under ./source are automatically added.
rwildcard = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
AUTO_C_SRC = $(call rwildcard,./source/,*.c)
AUTO_CPP_SRC = $(call rwildcard,./source/,*.cpp)
AUTO_ASM_SRC = $(call rwildcard,./source/,*.s)
AUTO_INCLUDES = $(sort $(dir $(call rwildcard,./source/,*)))

# C sources here.
CSRC =      $(LIB_C_SRC) \
            $(APP_C_SRC) \
            $(U_C_SRC) \
            $(AUTO_C_SRC) \
            ./components/components.c

# C++ sources here.
CPPSRC =    $(LIB_CPP_SRC) \
            $(APP_CPP_SRC) \
            $(U_CPP_SRC) \
            $(AUTO_CPP_SRC)

# List ASM source files here.
ASMSRC =    $(LIB_ASM_SRC) \
            $(APP_ASM_SRC) \
            $(U_ASM_SRC) \
            $(AUTO_ASM_SRC)

# Inclusion paths.
INCDIR =    $(LIB_INCLUDES) \
            $(APP_INCLUDES) \
            $(AUTO_INCLUDES) \
            ./components

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

export PATH := C:\SPC5Studio-6.0\eclipse\plugins\com.st.tools.spc5.tools.gnu.gcc.ppcvle.win32_4.9.4.20200908161514\toolchain\bin\$(PATH_SEPARATOR)${PATH}


MCU = -mcpu=e200z4 -mvle
MCU += -meabi -msdata=none -mregnames

TRGT = ppc-freevle-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
ifeq ($(USE_CPLUSPLUS_LINKER),yes)
# C++ Linker
LD   = $(TRGT)g++
else
# Standard C Linker
LD   = $(TRGT)gcc
endif

CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar cr
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
MOT  = $(CP) -O srec
BIN  = $(CP) -O binary

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

include ./components/spc58ecxx_platform_component_rla/lib/rsc/rules.mk


