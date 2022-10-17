# SAMV71 Xplained Ultra Port to ChibiOS

This demo sets up an minimal image for the SAMV71 Xplained Ultra evaluation board.
ChibiOS shall spawn up the main() thread and let the two leds blink.

## Porting Steps

* In `os/common/startup/ARMCMx/` the following changes/additions were made
  - In `compilers/GCC/mk` a **startup_samv71xxx.mk** file has been added
  - In `compilers/GCC/ld` a **SAMV71Q21B.ld** linkerscript has been added
  - In `devices` a folder named **SAMV71xxx** containing a **cmparams.h** has been created
* In `os/common/ext/CMSIS/` a folder named **SAMV71** has been added based on [Atmel Software Framework](https://github.com/avrxml/asf.git)
* In `os/hal/ports/SAMV71` all the integrated drivers can be found
  - In `LLD/PMCv1` a driver to access the Power Management Controller can be found. Currently it only supports enabling/diabling the clock gates of the peripherals.
  - In `LLD/SYSTICKv1` is a driver to setup the systick for the timing management/scheduling
  - In `LLD/GPIOv1` is a driver to implement the PAL driver of ChibiOS for GPIO setup and usage
  - In `LLD/USARTv1` is a driver to implement the SD driver of ChibiOS for serial communication
  - The driver in `SAMV71xxx` uses the previously listed drivers to setup the SAMV71 specific driver package.
* In `demos/SAMV71/RT-SAMV71Q21-XPLAINED-ULTRA/` is a demo which blinks an LED whenever there is a character sent to the device which will be send back.
