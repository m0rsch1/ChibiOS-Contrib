# SAMV71 Xplained Ultra Port to ChibiOS

This demo sets up an minimal image for the SAMV71 Xplained Ultra evaluation board.
ChibiOS shall spawn up the main() thread and let the two leds blink.

## ToDo

* Implement HAL for the SAMV71 and add boardfile for the SAMV71 Xplained Ultra board

## Porting Steps

* In `os/common/startup/ARMCMx/` the following changes/additions were made
  - In `compilers/GCC/mk` a **startup_samv71xxx.mk** file has been added
  - In `compilers/GCC/ld` a **SAMV71Q21B.ld** linkerscript has been added
  - In `devices` a folder named **SAMV71xxx** containing a **cmparams.h** has been created
* In `os/common/ext/CMSIS/` a folder named **SAMV71** has been added based on [Atmel Software Framework](https://github.com/avrxml/asf.git)
