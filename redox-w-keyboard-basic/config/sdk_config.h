/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef SDK_CONFIG_H
#define SDK_CONFIG_H

#ifdef USE_APP_CONFIG
#include "app_config.h"
#endif

/**
 * Provide a non-zero value here in applications that need to use several
 * peripherals with the same ID that are sharing certain resources
 * (for example, SPI0 and TWI0). Obviously, such peripherals cannot be used
 * simultaneously. Therefore, this definition allows to initialize the driver
 * for another peripheral from a given group only after the previously used one
 * is uninitialized. Normally, this is not possible, because interrupt handlers
 * are implemented in individual drivers.
 * This functionality requires a more complicated interrupt handling and driver
 * initialization, hence it is not always desirable to use it.
 */
#define PERIPHERAL_RESOURCE_SHARING_ENABLED  0

/* CLOCK */
#define CLOCK_ENABLED 1

#if (CLOCK_ENABLED == 1)
#define CLOCK_CONFIG_XTAL_FREQ          NRF_CLOCK_XTALFREQ_Default
#define CLOCK_CONFIG_LF_SRC             NRF_CLOCK_LFCLK_Xtal
#define CLOCK_CONFIG_IRQ_PRIORITY       3
#endif

/* RTC */
#define RTC_ENABLED  1
#define RTC0_ENABLED 1
#define RTC1_ENABLED 0
#define RTC2_ENABLED 0

#define NRF_MAXIMUM_LATENCY_US 2000

/* RNG */
#define RNG_ENABLED 0

/* UART */
#define UART_ENABLED     1
#define UART0_ENABLED    1
#define UART1_ENABLED    0

#define APP_FIFO_ENABLED 1
#define APP_UART_ENABLED 1
#define APP_UART_DRIVER_INSTANCE  0
#define RETARGET_ENABLED 1

#define UART_DEFAULT_CONFIG_HWFC NRF_UART_HWFC_DISABLED
#define UART_DEFAULT_CONFIG_PARITY NRF_UART_PARITY_EXCLUDED
#define UART_DEFAULT_CONFIG_BAUDRATE NRF_UART_BAUDRATE_115200
#define UART_DEFAULT_CONFIG_IRQ_PRIORITY 3

#endif // SDK_CONFIG_H
