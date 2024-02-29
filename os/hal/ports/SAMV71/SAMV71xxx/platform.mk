# Required source files
PLATFORMSRC := ${CHIBIOS}/os/hal/ports/common/ARMCMx/nvic.c \
	       ${CHIBIOS_CONTRIB}/os/hal/ports/SAMV71/SAMV71xxx/hal_lld.c \
	       ${CHIBIOS_CONTRIB}/os/hal/ports/SAMV71/SAMV71xxx/hal_matrix_lld.c \

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/common/ARMCMx \
              ${CHIBIOS_CONTRIB}/os/hal/ports/SAMV71/SAMV71xxx

# List of all the template platform files.
ifeq ($(USE_SMART_BUILD),yes)

# Configuration files directory
ifeq ($(HALCONFDIR),)
  ifeq ($(CONFDIR),)
    HALCONFDIR = .
  else
    HALCONFDIR := $(CONFDIR)
  endif
endif

HALCONF := $(strip $(shell cat $(CONFDIR)/halconf.h | egrep -e "\#define"))

else
endif

# Drivers compatible with the platform.
include $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/SYSTICKv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/PMCv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/GPIOv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/USARTv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/GPTv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/MCANv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/XDMACv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
