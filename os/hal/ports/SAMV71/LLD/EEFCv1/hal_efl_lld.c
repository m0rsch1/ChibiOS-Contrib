/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_efl_lld.c
 * @brief   SAMV71 Embedded Flash subsystem low level driver source.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#include "hal.h"

#include <string.h>

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   EFL1 driver identifier.
 */
#if (SAMV71_EFL_USE_EFL1 == TRUE) || defined(__DOXYGEN__)
EFlashDriver EFLD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/


  /*
   * embedded flash:
   *
   * sectors of 128kB each
   * first sector is 3 sub-sectors: 8kB, 8kB, 112kB (all of this is sector 0)
   * pages of 512bytes each
   *
   * erase options:
   * 8kB block
   * 128kB sector
   * 512 byte page within small sectors(8kB)
   * Chip Erase
   */
static flash_sector_descriptor_t const efl_lld_sectors[18] = {
  {131072*0+8192*0, 8192},
  {131072*0+8192*1, 8192},
  {131072*0+8192*2, 131072-8192*2},
  {131072*1, 131072},
  {131072*2, 131072},
  {131072*3, 131072},
  {131072*4, 131072},
  {131072*5, 131072},
  {131072*6, 131072},
  {131072*7, 131072},
  {131072*8, 131072},
  {131072*9, 131072},
  {131072*10, 131072},
  {131072*11, 131072},
  {131072*12, 131072},
  {131072*13, 131072},
  {131072*14, 131072},
  {131072*15, 131072}
};

static flash_descriptor_t efl_lld_descriptor = {
 .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                      FLASH_ATTR_MEMORY_MAPPED |
                      FLASH_ATTR_ECC_CAPABLE   |
                      FLASH_ATTR_ECC_ZERO_LINE_CAPABLE,
 .page_size         = 0,
 .sectors_count     = 0,
 .sectors           = efl_lld_sectors,
 .sectors_size      = 0,
 .address           = (uint8_t *)0x00400000,
 .size              = 0
};

#if SAMV71_EFL_FROM_FLASH
static uint32_t elf_lld_last_FSR = EEFC_FSR_FRDY;;
#endif

typedef uint32_t SAM_BA_flash_helper_t(uint32_t dont_care, uint32_t val_REG_EFC_FCR);

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint32_t efl_lld_commit_command_and_wait_for_RDY(uint32_t FCR_command) {
  /*
   * there is an IAP code snippet that resides in the SAM-BA rom code
   * that can be used to avoid loading code to memory
   * (SAM-BA cannot flash by itself)
   *
   * uint32_t (*func)(uint_32_t dont_care, uint32_t val_REG_EFC_FCR) = *(uint32_t*)(0x00800008);
   * return value is from REG_EFC_FSR
   *
   * The function:
   *
   * uint32_t flash_helper(uint32_t dont_care, uint32_t val_REG_EFC_FCR) {
   *   uint32_t res;
   *
   *   REG_EFC_FCR = val_REG_EFC_FCR;
   *   do {
   *     res = REG_EFC_FSR;
   *   } while ((res & EEFC_FSR_FRDY) == 0);
   *   return res;
   * }
   *
   *
   */

  SAM_BA_flash_helper_t *func = *(SAM_BA_flash_helper_t**)(0x00800008);
  chSysLock();
  uint32_t res = func(0, FCR_command);
  chSysUnlock();
  return res;
}

static __attribute__((unused)) uint32_t efl_lld_commit_command_and_wait_for_RDY_FromISR(uint32_t FCR_command) {
  SAM_BA_flash_helper_t *func = *(SAM_BA_flash_helper_t**)(0x00800008);
  chSysLockFromISR();
  uint32_t res = func(0, FCR_command);
  chSysUnlockFromISR();
  return res;
}

static void efl_lld_program_full_page(size_t page, const uint8_t *pp) {
  uint8_t *flash_addr = efl_lld_descriptor.address + efl_lld_descriptor.page_size * page;

  /* write the latch buffer */
  memcpy(flash_addr, pp, efl_lld_descriptor.page_size);

#if SAMV71_EFL_FROM_FLASH
  elf_lld_last_FSR = efl_lld_commit_command_and_wait_for_RDY(
                       EEFC_FCR_FKEY_PASSWD |
                       EEFC_FCR_FCMD_WP |
                       EEFC_FCR_FARG(page));
#else
  REG_EFC_FCR = EEFC_FCR_FKEY_PASSWD |
                EEFC_FCR_FCMD_EA |
                EEFC_FCR_FCMD_WP |
                EEFC_FCR_FARG(page);
#endif
}

static void efl_lld_program_partial_page(size_t page, size_t offset, size_t n, const uint8_t *pp) {
  osalDbgCheck(offset % 16 == 0 && n % 16 == 0);

  uint8_t *flash_addr = efl_lld_descriptor.address + efl_lld_descriptor.page_size * page + offset;

  /* write the latch buffer */
  memcpy(flash_addr, pp, n);

#if SAMV71_EFL_FROM_FLASH
  elf_lld_last_FSR = efl_lld_commit_command_and_wait_for_RDY(
                       EEFC_FCR_FKEY_PASSWD |
                       EEFC_FCR_FCMD_WP |
                       EEFC_FCR_FARG(page));
#else
  REG_EFC_FCR = EEFC_FCR_FKEY_PASSWD |
                EEFC_FCR_FCMD_EA |
                EEFC_FCR_FCMD_WP |
                EEFC_FCR_FARG(page);
#endif
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level Embedded Flash driver initialization.
 *
 * @notapi
 */
void efl_lld_init(void) {

#if SAMV71_EFL_USE_EFL1 == TRUE
  /* Driver initialization.*/
  eflObjectInit(&EFLD1);

  /* the GETD command is documented to not freeze flash access */
#if SAMV71_EFL_FROM_FLASH && FALSE
  efl_lld_commit_command_and_wait_for_RDY(EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_GETD);
#else
  REG_EFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_GETD;
  while (!(REG_EFC_FSR & EEFC_FSR_FRDY)) {
  }
#endif

  uint32_t fl_id = REG_EFC_FRR;
  uint32_t fl_size = REG_EFC_FRR;
  uint32_t fl_page_size = REG_EFC_FRR;
  uint32_t fl_nb_plane = REG_EFC_FRR;
  uint32_t fl_plane[2];
  uint32_t plane_no = 0;
  while(plane_no < fl_nb_plane && plane_no < 2) {
    fl_plane[plane_no] = REG_EFC_FRR;
    plane_no++;
  }
  while(plane_no < fl_nb_plane) {
    (void)REG_EFC_FRR;
    plane_no++;
  }
  uint32_t fl_nb_lock = REG_EFC_FRR;
  uint32_t lock_no = 0;
  uint32_t fl_lock[32];
  while(lock_no < fl_nb_lock && lock_no < 32) {
    fl_lock[lock_no] = REG_EFC_FRR;
    lock_no++;
  }
  while(lock_no < fl_nb_lock) {
    (void)REG_EFC_FRR;
    lock_no++;
  }

  (void)fl_id;
  (void)fl_plane;
  (void)plane_no;
  (void)fl_nb_lock;
  (void)lock_no;
  (void)fl_lock;

  //my device has:
  //a single plane of 2MB
  //128 lock blocks of 16kB each
  //sector information is not part of the flash descriptor, apparently.

  efl_lld_descriptor.page_size = fl_page_size;
  efl_lld_descriptor.size = fl_size;

  //when erasing, the first two erase units at 8192 bytes, followed
  //by the remainder to 128k, then 128k units.
  efl_lld_descriptor.sectors_count = fl_size / 131072+2;
#endif
}

/**
 * @brief   Configures and activates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_start(EFlashDriver *eflp) {

  if (eflp->state == FLASH_STOP) {
    /* Enables the peripheral.*/
#if SAMV71_EFL_USE_EFL1 == TRUE
    if (&EFLD1 == eflp) {

    }
#endif
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Deactivates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_stop(EFlashDriver *eflp) {

  if (eflp->state == FLASH_READY) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if SAMV71_EFL_USE_EFL1 == TRUE
    if (&EFLD1 == eflp) {

    }
#endif
  }
}

/**
 * @brief   Gets the flash descriptor structure.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @return                          A flash device descriptor.
 *
 * @notapi
 */
const flash_descriptor_t *efl_lld_get_descriptor(void *instance) {
  (void)instance;

  return &efl_lld_descriptor;
}

/**
 * @brief   Read operation.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[in] offset                flash offset
 * @param[in] n                     number of bytes to be read
 * @param[out] rp                   pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_READ         if the read operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_read(void *instance, flash_offset_t offset,
                           size_t n, uint8_t *rp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (rp != NULL) && (n > 0U));
  osalDbgCheck(((size_t)offset + n) <= (size_t)efl_lld_descriptor.size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No reading while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READY state while the operation is performed.*/
  devp->state = FLASH_READ;

  memcpy(rp, efl_lld_descriptor.address + offset, n);

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;

}
/**
 * @brief   Program operation.
 * @note    The device supports ECC, it is only possible to write erased
 *          pages once except when writing all zeroes.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[in] offset                flash offset
 * @param[in] n                     number of bytes to be programmed
 * @param[in] pp                    pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_PROGRAM      if the program operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                              size_t n, const uint8_t *pp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  osalDbgCheck(((size_t)offset + n) <= (size_t)efl_lld_descriptor.size);

  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No programming while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  if(offset % 16 != 0 || n % 16 != 0) {
    return FLASH_ERROR_PROGRAM;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_PGM;

  size_t page_size = efl_lld_descriptor.page_size;
  size_t page = offset / page_size;

  if (offset % page_size != 0) {
    size_t this_n = n;
    if(offset % page_size + this_n > page_size) {
      this_n = page_size - offset % page_size;
    }
    efl_lld_program_partial_page(page, offset % page_size, this_n, pp);
    pp += this_n;
    n -= this_n;
    page++;
  }

  while(n >= page_size) {
    efl_lld_program_full_page(page, pp);
    pp += page_size;
    n -= page_size;
    page++;
  }

  if (n != 0) {
    efl_lld_program_partial_page(page, 0, n, pp);
    pp += n;
    n -= n;
    page++;
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

/**
 * @brief   Starts a whole-device erase operation.
 * @note    This function only erases bank 2 if it is present. Bank 1 is not
 *          touched because it is where the program is running on.
 *          Pages on bank 1 can be individually erased.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_all(void *instance) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_ERASE;

#if SAMV71_EFL_FROM_FLASH
  /* Note, this does not make much sense to do, the code will be wiped
     afterwards. */
  elf_lld_last_FSR = efl_lld_commit_command_and_wait_for_RDY(
                       EEFC_FCR_FKEY_PASSWD |
                       EEFC_FCR_FCMD_EA);
#else
  REG_EFC_FCR = EEFC_FCR_FKEY_PASSWD |
                EEFC_FCR_FCMD_EA;
#endif

  return FLASH_NO_ERROR;
}

/**
 * @brief   Starts an sector erase operation.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be erased
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_sector(void *instance,
                                         flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < efl_lld_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  uint32_t offset = flashGetSectorOffset(getBaseFlash(devp), sector);
  uint32_t page = offset / efl_lld_descriptor.page_size;

#if SAMV71_EFL_FROM_FLASH
  elf_lld_last_FSR = efl_lld_commit_command_and_wait_for_RDY(
                       EEFC_FCR_FKEY_PASSWD |
                       EEFC_FCR_FCMD_ES |
                       EEFC_FCR_FARG(page));
#else
  REG_EFC_FCR = EEFC_FCR_FKEY_PASSWD |
                EEFC_FCR_FCMD_ES |
                EEFC_FCR_FARG(page);
#endif

  return FLASH_NO_ERROR;
}

/**
 * @brief   Queries the driver for erase operation progress.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[out] msec                 recommended time, in milliseconds, that
 *                                  should be spent before calling this
 *                                  function again, can be @p NULL
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_ERASE        if the erase operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
flash_error_t efl_lld_query_erase(void *instance, uint32_t *msec) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  (void)msec;

  /* If there is an erase in progress then the device must be checked.*/
  if (devp->state == FLASH_ERASE) {
#if SAMV71_EFL_FROM_FLASH
    if (elf_lld_last_FSR & EEFC_FSR_FLOCKE) {
      err = FLASH_ERROR_ERASE;
    } else {
      err =  FLASH_NO_ERROR;
    }
    elf_lld_last_FSR = EEFC_FSR_FRDY;
    devp->state = FLASH_READY;
#else
    uint32_t fsr = REG_EFC_FSR;
    if ((fsr & EEFC_FSR_FRDY) == 0) {
      err = FLASH_BUSY_ERASING;
    } else {
      devp->state = FLASH_READY;
      if(fsr & EEFC_FSR_FLOCKE) {
        err = FLASH_ERROR_ERASE;
      } else {
        err = FLASH_NO_ERROR;
      }
    }
#endif
  }

  return err;
}

/**
 * @brief   Returns the erase state of a sector.
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be verified
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if the sector is erased.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_VERIFY       if the verify operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < efl_lld_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No verifying while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Just read the thing and check if it is all ones */
  uint64_t *begin =
    (uint64_t *)(efl_lld_descriptor.address + sector * efl_lld_descriptor.sectors_size);
  uint64_t *end =
    (uint64_t *)(efl_lld_descriptor.address + (sector+1) * efl_lld_descriptor.sectors_size);
  for(uint64_t *pos = begin; pos != end; pos++) {
    if(*pos != ~0ULL) {
      err = FLASH_ERROR_VERIFY;
      break;
    }
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

/* The unique identifier is mapped instead of the main flash when trying to
 * read it, so all accesses to flash must be stopped and the data copied while
 * running from RAM.
 */
static flash_error_t efl_lld_read_unique_identifier_helper(flash_offset_t offset,
                              size_t n, uint8_t *pp) __attribute__((section(".ramtext"),flatten,noinline));
static flash_error_t efl_lld_read_unique_identifier_helper(flash_offset_t offset,
                              size_t n, uint8_t *pp) {
  //this cannot use any library functions and similar.
  REG_EFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_STUI;
  while (REG_EFC_FSR & EEFC_FSR_FRDY) {
  }

  uint8_t *addr = (uint8_t *)0x00400000+offset;

  DCACHE_INVALIDATE_FOR_READ(addr, n);

  for(unsigned int count = 0; count < n; count++) {
    pp[count] = addr[count];
  }

  REG_EFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FCMD_SPUI;
  while (!(REG_EFC_FSR & EEFC_FSR_FRDY)) {
  }

  DCACHE_INVALIDATE_FOR_READ(addr, n);
  return FLASH_NO_ERROR;
}


/**
 * @brief   Read unique identifier
 *
 * @param[in] instance              pointer to a @p EFlashDriver instance
 * @param[in] offset                flash offset
 * @param[in] n                     number of bytes to be programmed
 * @param[in] pp                    pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t flashReadUniqueIdentifier(void *instance, flash_offset_t offset,
                              size_t n, uint8_t *pp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  osalDbgCheck(((size_t)offset + n) <= (size_t)efl_lld_descriptor.size);

  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No reading while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  chSysLock();
  err = efl_lld_read_unique_identifier_helper(offset, n, pp);
  chSysUnlock();

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

#endif /* HAL_USE_EFL == TRUE */

/** @} */
