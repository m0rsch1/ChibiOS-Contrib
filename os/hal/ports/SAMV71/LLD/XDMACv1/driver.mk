ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_XDMAC TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/XDMACv1/samv71_xdmac.c
endif
else
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/XDMACv1/samv71_xdmac.c
endif

PLATFORMINC += $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/XDMACv1
