ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_EFL TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/EEFCv1/hal_efl_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/EEFCv1/hal_efl_lld.c
endif

PLATFORMINC += $(CHIBIOS_CONTRIB)/os/hal/ports/SAMV71/LLD/EEFCv1
