/*
 * Copyright (c) 2022 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_STM32L4_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_STM32L4_CLOCK_H_

#include "stm32_common_clocks.h"

/** Bus clocks */
#define STM32_CLOCK_BUS_AHB1    0x048
#define STM32_CLOCK_BUS_AHB2    0x04c
#define STM32_CLOCK_BUS_AHB3    0x050
#define STM32_CLOCK_BUS_APB1    0x058
#define STM32_CLOCK_BUS_APB1_2  0x05c
#define STM32_CLOCK_BUS_APB2    0x060

#define STM32_PERIPH_BUS_MIN	STM32_CLOCK_BUS_AHB1
#define STM32_PERIPH_BUS_MAX	STM32_CLOCK_BUS_APB2

/** Domain clocks */
/* RM0351/RM0432/RM0438, § Clock configuration register (RCC_CCIPRx) */

/** System clock */
/* defined in stm32_common_clocks.h */
/** Fixed clocks  */
/* Low speed clocks defined in stm32_common_clocks.h */
#define STM32_SRC_HSI		(STM32_SRC_LSI + 1)
#define STM32_SRC_HSI48		(STM32_SRC_HSI + 1)
#define STM32_SRC_MSI		(STM32_SRC_HSI48 + 1)
/** Bus clock */
#define STM32_SRC_PCLK		(STM32_SRC_MSI + 1)
/** PLL clock outputs */
#define STM32_SRC_PLL_P		(STM32_SRC_PCLK + 1)
#define STM32_SRC_PLL_Q		(STM32_SRC_PLL_P + 1)
#define STM32_SRC_PLL_R		(STM32_SRC_PLL_Q + 1)
/* TODO: PLLSAI clocks */

/** @brief RCC_CCIPR register offset */
#define CCIPR_REG		0x88
#define CCIPR2_REG		0x9C

/** @brief RCC_BDCR register offset */
#define BDCR_REG		0x90

/** @brief RCC_CFGRx register offset */
#define CFGR_REG                0x08

/** @brief Device domain clocks selection helpers */
/** CCIPR devices */
#define USART1_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 0, CCIPR_REG)
#define USART2_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 2, CCIPR_REG)
#define USART3_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 4, CCIPR_REG)
#define UART4_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 6, CCIPR_REG)
#define UART5_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 8, CCIPR_REG)
#define LPUART1_SEL(val)	STM32_DT_CLOCK_SELECT((val), 3, 10, CCIPR_REG)
#define I2C1_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 12, CCIPR_REG)
#define I2C2_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 14, CCIPR_REG)
#define I2C3_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 16, CCIPR_REG)
#define LPTIM1_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 18, CCIPR_REG)
#define LPTIM2_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 20, CCIPR_REG)
#define SAI1_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 22, CCIPR_REG)
#define SAI2_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 24, CCIPR_REG)
#define CLK48_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 26, CCIPR_REG)
#define ADC_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 28, CCIPR_REG)
#define SWPMI1_SEL(val)		STM32_DT_CLOCK_SELECT((val), 1, 30, CCIPR_REG)
#define DFSDM1_SEL(val)		STM32_DT_CLOCK_SELECT((val), 1, 31, CCIPR_REG)
/** CCIPR2 devices */
#define I2C4_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 0, CCIPR2_REG)
#define DFSDM_SEL(val)		STM32_DT_CLOCK_SELECT((val), 1, 2, CCIPR2_REG)
#define ADFSDM_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 3, CCIPR2_REG)
#define DSI_SEL(val)		STM32_DT_CLOCK_SELECT((val), 1, 12, CCIPR2_REG)
#define SDMMC_SEL(val)		STM32_DT_CLOCK_SELECT((val), 1, 14, CCIPR2_REG)
#define OSPI_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 20, CCIPR2_REG)
/** BDCR devices */
#define RTC_SEL(val)		STM32_DT_CLOCK_SELECT((val), 3, 8, BDCR_REG)
/** CFGR devices */
#define MCO1_SEL(val)           STM32_DT_CLOCK_SELECT((val), 0xF, 24, CFGR_REG)
#define MCO1_PRE(val)           STM32_DT_CLOCK_SELECT((val), 0x7, 28, CFGR_REG)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_STM32L4_CLOCK_H_ */
