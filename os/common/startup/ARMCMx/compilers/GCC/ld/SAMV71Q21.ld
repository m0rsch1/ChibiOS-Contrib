/**
 * \file
 *
 * Copyright (c) 2015 Microchip Technology Inc. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/*------------------------------------------------------------------------------
 *      Linker script for running in internal FLASH on the ATSAMV71Q21
 *----------------------------------------------------------------------------*/

/*
 * 2022-09-28 Moritz Schilling, DFKI, Bremen, Germany
 * Adapted for ChibiOS
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
SEARCH_DIR(.)

/* Memory Spaces Definitions */
MEMORY
{
  flash0 (RX) : org = 0x00400000, len = 2M        /* Flash as AXIM (writable) */
  flash1 (X)  : org = 0x00800000, len = 2M        /* Flash as ROM: Contains SA-BOOT code etc. length is unknown actually*/
  flash2 (RX) : org = 0x00000000, len = 0
  flash3 (RX) : org = 0x00000000, len = 0
  flash4 (RX) : org = 0x00000000, len = 0
  flash5 (RX) : org = 0x00000000, len = 0
  flash6 (RX) : org = 0x00000000, len = 0
  flash7 (RX) : org = 0x00000000, len = 0
  /* The following RAM setup assumes a default setup. Setting GPNVM Bits[8:7] to 0-3 allows distributing SRAM to DTCM and ITCM*/
  ram0   (wx) : org = 0x20400000, len = 384k      /* SRAM1 + SRAM2 */
  ram1   (wx) : org = 0x20400000, len = 320k      /* SRAM1: Data, Heap*/
  ram2   (wx) : org = 0x20450000, len = 64k       /* SRAM2: Main Stack, Process Stack, BSS, NOCACHE, ETH */
  ram3   (wx) : org = 0x20000000, len = 0         /* DTCM-RAM */
  ram4   (wx) : org = 0x00000000, len = 0         /* ITCM-RAM */
  ram5   (wx) : org = 0x40074000, len = 1k        /* BCKP SRAM */
  ram6   (wx) : org = 0x00000000, len = 0
  ram7   (wx) : org = 0x00000000, len = 0
}

/* For each data/text section two region are defined, a virtual region
   and a load region (_LMA suffix).*/

/* Flash region to be used for exception vectors.*/
REGION_ALIAS("VECTORS_FLASH", flash0);
REGION_ALIAS("VECTORS_FLASH_LMA", flash0);

/* Flash region to be used for constructors and destructors.*/
REGION_ALIAS("XTORS_FLASH", flash0);
REGION_ALIAS("XTORS_FLASH_LMA", flash0);

/* Flash region to be used for code text.*/
REGION_ALIAS("TEXT_FLASH", flash0);
REGION_ALIAS("TEXT_FLASH_LMA", flash0);

/* Flash region to be used for read only data.*/
REGION_ALIAS("RODATA_FLASH", flash0);
REGION_ALIAS("RODATA_FLASH_LMA", flash0);

/* Flash region to be used for various.*/
REGION_ALIAS("VARIOUS_FLASH", flash0);
REGION_ALIAS("VARIOUS_FLASH_LMA", flash0);

/* Flash region to be used for RAM(n) initialization data.*/
REGION_ALIAS("RAM_INIT_FLASH_LMA", flash0);

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts.*/
REGION_ALIAS("MAIN_STACK_RAM", ram2);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram2);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram1);
REGION_ALIAS("DATA_RAM_LMA", flash0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram2);

/* RAM region to be used for the default heap.*/
REGION_ALIAS("HEAP_RAM", ram1);

/* Stack rules inclusion.*/
INCLUDE rules_stacks.ld

/* Code rules inclusion.*/
INCLUDE rules_code.ld

/* Data rules inclusion.*/
INCLUDE rules_data.ld

/* Memory rules inclusion.*/
INCLUDE rules_memory.ld
