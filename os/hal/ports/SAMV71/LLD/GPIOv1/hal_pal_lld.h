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
 * @file    hal_pal_lld.h
 * @brief   SAMV71 PAL subsystem low level driver header.
 *
 * @addtogroup PAL
 * @{
 */

#ifndef HAL_PAL_LLD_H
#define HAL_PAL_LLD_H

#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

#include "samv71.h"

/*===========================================================================*/
/* Unsupported modes and specific modes                                      */
/*===========================================================================*/

/* Specifies palInit() without parameter, required until all platforms will
   be updated to the new style.*/
#define PAL_NEW_INIT

/**
 * @name    Additional PAL event modes
 * @{
 */
#define PAL_EVENT_MODE_LEVEL_MASK   4U  /**< @brief Mask for level select  */
#define PAL_EVENT_MODE_HIGH_LEVEL   5U  /**< @brief High level callback.  */
#define PAL_EVENT_MODE_LOW_LEVEL    6U  /**< @brief Low level callback.   */
/** @} */

/*
 * The following modes are specific for the SAMV71
 *
 * NOTE: The built in modes stop at 7U
 *
 * So 0-15U are reserved for the standard modes while the upper 16-31U are reserved for extra modes which can be ORed.
 */

/* Mark the pad to be controlled by a peripheral */
#define PAL_MODE_PERIPHERAL_CONTROLLED (1U << 16)
/* Set the input to be an input with debouncing filter active. Can be ored with PAL_MODE_INPUT* modes */
#define PAL_MODE_INPUT_DEBOUNCE (1U << 17)
/* Set the input to be an input with deglitch filter active. Can be ored with PAL_MODE_INPUT* modes */
#define PAL_MODE_INPUT_DEGLITCH (1U << 18)
/* Enable the internal pullup resistor. Can be ORed */
#define PAL_MODE_PULLUP (1U << 19)
/* Enable the internal pulldown resistor. Can be ORed. For opendrain outputs this will have no effect. */
#define PAL_MODE_PULLDOWN (1U << 20)

/* Redefine some built in defines */
#undef PAL_MODE_INPUT_PULLUP
#define PAL_MODE_INPUT_PULLUP (PAL_MODE_INPUT | PAL_MODE_PULLUP)
#undef PAL_MODE_INPUT_PULLDOWN
#define PAL_MODE_INPUT_PULLDOWN (PAL_MODE_INPUT | PAL_MODE_PULLDOWN)

/* Set the pad to be controlled by a peripheral */
#define PAL_MODE_PERIPHERAL(n) (PAL_MODE_PERIPHERAL_CONTROLLED | (n & 0x3U))

#define PAL_PERIPHERAL_A 0
#define PAL_PERIPHERAL_B 1
#define PAL_PERIPHERAL_C 2
#define PAL_PERIPHERAL_D 3

#ifdef ID_PIOA
#define PIOA_NVIC_NUMBER PIOA_IRQn
#define PIOA_HANDLER Vector68
#endif
#ifdef ID_PIOB
#define PIOB_NVIC_NUMBER PIOB_IRQn
#define PIOB_HANDLER Vector6C
#endif
#ifdef ID_PIOC
#define PIOC_NVIC_NUMBER PIOC_IRQn
#define PIOC_HANDLER Vector70
#endif
#ifdef ID_PIOD
#define PIOD_NVIC_NUMBER PIOD_IRQn
#define PIOD_HANDLER Vector80
#endif
#ifdef ID_PIOE
#define PIOE_NVIC_NUMBER PIOE_IRQn
#define PIOE_HANDLER Vector84
#endif

/*===========================================================================*/
/* I/O Ports Types and constants.                                            */
/*===========================================================================*/

/**
 * @name    Port related definitions
 * @{
 */
/**
 * @brief   Width, in bits, of an I/O port.
 */
#define PAL_IOPORTS_WIDTH           32U

/**
 * @brief   Whole port mask.
 * @details This macro specifies all the valid bits into a port.
 */
#define PAL_WHOLE_PORT              ((ioportmask_t)0xFFFFFFFFU)
/** @} */

/**
 * @name    Line handling macros
 * @{
 */
/**
 * @brief   Forms a line identifier.
 * @details A port/pad pair are encoded into an @p ioline_t type. The encoding
 *          of this type is platform-dependent.
 * @note    In this driver the pad number is encoded in the lower 5 bits of
 *          the GPIO address which are guaranteed to be zero.
 */
#define PAL_LINE(port, pad)                                                 \
  ((ioline_t)(((ioline_t)((uintptr_t)port)) | ((ioline_t)(pad))))

/**
 * @brief   Decodes a port identifier from a line identifier.
 */
#define PAL_PORT(line)                                                      \
  ((ioportid_t)((uintptr_t)((((ioline_t)(line))) & 0xFFFFFFE0U)))

/**
 * @brief   Decodes a pad identifier from a line identifier.
 */
#define PAL_PAD(line)                                                       \
  ((iopadid_t)(((ioline_t)(line)) & 0x1FU))

/**
 * @brief   Value identifying an invalid line.
 */
#define PAL_NOLINE                      0U
/** @} */

#if defined(ID_PIOE)
#define PAL_PORTNO(port) (((port) == PIOE)?4:    \
                          ((port) == PIOD)?3:    \
                          ((port) == PIOC)?2:    \
                          ((port) == PIOB)?1:0)
#elif defined(ID_PIOD)
#define PAL_PORTNO(port) (((port) == PIOD)?3:    \
                          ((port) == PIOC)?2:    \
                          ((port) == PIOB)?1:0)
#elif defined(ID_PIOC)
#define PAL_PORTNO(port) (((port) == PIOC)?2:    \
                          ((port) == PIOB)?1:0)
#elif defined(ID_PIOB)
#define PAL_PORTNO(port) (((port) == PIOB)?1:0)
#else
#define PAL_PORTNO(port) 0
#endif


/**
 * @brief   Generic I/O ports static initializer.
 * @details An instance of this structure must be passed to @p palInit() at
 *          system startup time in order to initialized the digital I/O
 *          subsystem. This represents only the initial setup, specific pads
 *          or whole ports can be reprogrammed at later time.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {

} PALConfig;

/**
 * @brief   Digital I/O port sized unsigned type.
 */
typedef uint32_t ioportmask_t;

/**
 * @brief   Digital I/O modes.
 */
typedef uint32_t iomode_t;

/**
 * @brief   Type of an I/O line.
 */
typedef uint32_t ioline_t;

/**
 * @brief   Type of an event mode.
 */
typedef uint32_t ioeventmode_t;

/**
 * @brief   Type of a port Identifier.
 * @details This type can be a scalar or some kind of pointer, do not make
 *          any assumption about it, use the provided macros when populating
 *          variables of this type.
 */
typedef Pio* ioportid_t;

/**
 * @brief   Type of a pad identifier.
 */
typedef uint32_t iopadid_t;

/*===========================================================================*/
/* I/O Ports Identifiers.                                                    */
/*===========================================================================*/

/**
 * @brief   First I/O port identifier.
 * @details Low level drivers can define multiple ports, it is suggested to
 *          use this naming convention.
 */
#if defined(ID_PIOA)
#define IOPORT1         PIOA
#endif

#if defined(ID_PIOB)
#define IOPORT2         PIOB
#endif

#if defined(ID_PIOC)
#define IOPORT3         PIOC
#endif

#if defined(ID_PIOD)
#define IOPORT4         PIOD
#endif

#if defined(ID_PIOE)
#define IOPORT5         PIOE
#endif

/*===========================================================================*/
/* Implementation, some of the following macros could be implemented as      */
/* functions, if so please put them in pal_lld.c.                            */
/*===========================================================================*/

/**
 * @brief   Low level PAL subsystem initialization.
 *
 * @notapi
 */
#define pal_lld_init() _pal_lld_init()

/**
 * @brief   Reads the physical I/O port states.
 *
 * @param[in] port      port identifier
 * @return              The port bits.
 *
 * @notapi
 */
#define pal_lld_readport(port) ((ioportmask_t)((port)->PIO_PDSR))

/**
 * @brief   Reads the output latch.
 * @details The purpose of this function is to read back the latched output
 *          value.
 *
 * @param[in] port      port identifier
 * @return              The latched logical states.
 *
 * @notapi
 */
#define pal_lld_readlatch(port) ((ioportmask_t)(port)->PIO_ODSR)

/**
 * @brief   Writes a bits mask on a I/O port.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be written on the specified port
 *
 * @notapi
 */
#define pal_lld_writeport(port, bits) ((port)->PIO_ODSR = (ioportmask_t)(bits))

/**
 * @brief   Sets a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be ORed on the specified port
 *
 * @notapi
 */
#define pal_lld_setport(port, bits) ((port)->PIO_SODR = (ioportmask_t)(bits))

/**
 * @brief   Clears a bits mask on a I/O port.
 * @note    The @ref PAL provides a default software implementation of this
 *          functionality, implement this function if can optimize it by using
 *          special hardware functionalities or special coding.
 *
 * @param[in] port      port identifier
 * @param[in] bits      bits to be cleared on the specified port
 *
 * @notapi
 */
#define pal_lld_clearport(port, bits) ((port)->PIO_CODR = (ioportmask_t)(bits))

/**
 * @brief   Pads group mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] mask      group mask
 * @param[in] offset    group bit offset within the port
 * @param[in] mode      group mode
 *
 * @notapi
 */
#define pal_lld_setgroupmode(port, mask, offset, mode)                      \
  _pal_lld_setgroupmode((port), (mask) << (offset), (mode))

/**
 * @brief   Pad event enable.
 * @note    Programming an unknown or unsupported mode is silently ignored.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 * @param[in] mode      pad event mode
 *
 * @notapi
 */
#define pal_lld_enablepadevent(port, pad, mode)                             \
  _pal_lld_enablepadevent(port, pad, mode)

/**
 * @brief   Pad event disable.
 * @details This function disables previously programmed event callbacks.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_disablepadevent(port, pad)                                  \
  _pal_lld_disablepadevent(port, pad)

/**
 * @brief   Returns a PAL event structure associated to a pad.
 *
 * @param[in] port      port identifier
 * @param[in] pad       pad number within the port
 *
 * @notapi
 */
#define pal_lld_get_pad_event(port, pad)                                    \
  &_pal_events[PAL_PORTNO(port)*32 + pad]

/**
 * @brief   Returns a PAL event structure associated to a line.
 *
 * @param[in] line      line identifier
 *
 * @notapi
 */
#define pal_lld_get_line_event(line)                                        \
  pal_lld_get_pad_event(PAL_PORT(line), PAL_PAD(line))

#if !defined(__DOXYGEN__)
#if (PAL_USE_WAIT == TRUE) || (PAL_USE_CALLBACKS == TRUE)
#if defined(ID_PIOE)
extern palevent_t _pal_events[32*5];
#elif defined(ID_PIOD)
extern palevent_t _pal_events[32*4];
#elif defined(ID_PIOC)
extern palevent_t _pal_events[32*3];
#elif defined(ID_PIOB)
extern palevent_t _pal_events[32*2];
#elif defined(ID_PIOA)
extern palevent_t _pal_events[32*1];
#else
extern palevent_t _pal_events[1];
#endif
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void _pal_lld_init(void);
  void _pal_lld_setgroupmode(ioportid_t port,
                             ioportmask_t mask,
                             iomode_t mode);
  void _pal_lld_setgroupPUPD(ioportid_t port,
                             ioportmask_t mask,
                             iomode_t mode);
  void _pal_lld_enablepadevent(ioportid_t port,
                               iopadid_t pad,
                               ioeventmode_t mode);
  void _pal_lld_disablepadevent(ioportid_t port, iopadid_t pad);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PAL == TRUE */

#endif /* HAL_PAL_LLD_H */

/** @} */
