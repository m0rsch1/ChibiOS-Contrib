# List of all the board related files.
BOARDSRC = $(CHIBIOS_CONTRIB)/os/hal/boards/SAMV71_XPLAINED_ULTRA/board.c \
	   $(CHIBIOS_CONTRIB)/os/hal/boards/SAMV71_XPLAINED_ULTRA/system_samv71.c

# Required include directories
BOARDINC = $(CHIBIOS_CONTRIB)/os/hal/boards/SAMV71_XPLAINED_ULTRA/

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
