/*
 * Copyright (c) 2018-2020 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULES_LVGL_LV_CONF_H_
#define ZEPHYR_MODULES_LVGL_LV_CONF_H_

#include <zephyr/toolchain.h>
#include <string.h>
#include <stdint.h>

/* Memory manager settings */

#define LV_USE_STDLIB_MALLOC  LV_STDLIB_CUSTOM
#define LV_USE_STDLIB_STRING  LV_STDLIB_CLIB
#define LV_USE_STDLIB_SPRINTF LV_STDLIB_CLIB

#if defined(CONFIG_LV_Z_MEM_POOL_HEAP_LIB_C)
#define LV_STDLIB_INCLUDE "stdlib.h"
#define lv_malloc_core    malloc
#define lv_realloc_core   realloc
#define lv_free_core      free
#else
#define LV_STDLIB_INCLUDE "lvgl_mem.h"
#define lv_malloc_core    lvgl_malloc
#define lv_realloc_core   lvgl_realloc
#define lv_free_core      lvgl_free
#endif

#define LV_ASSERT_HANDLER         __ASSERT_NO_MSG(false);
#define LV_ASSERT_HANDLER_INCLUDE "zephyr/sys/__assert.h"

/* Provide definition to align LVGL buffers */
#define LV_ATTRIBUTE_MEM_ALIGN __aligned(CONFIG_LV_ATTRIBUTE_MEM_ALIGN_SIZE)

#ifdef CONFIG_LV_COLOR_16_SWAP
#define LV_COLOR_16_SWAP 1
#endif /* CONFIG_LV_COLOR_16_SWAP */

#ifdef CONFIG_LV_Z_USE_OSAL
#define LV_USE_OS            LV_OS_CUSTOM
#define LV_OS_CUSTOM_INCLUDE "lvgl_zephyr_osal.h"
#endif /* CONFIG_LV_Z_USE_OSAL */

#ifdef CONFIG_LV_Z_NEMA_GFX
#if defined(CONFIG_CPU_CORTEX_M33)
#define LV_USE_NEMA_LIB	LV_NEMA_LIB_M33_REVC
#elif defined(CONFIG_CPU_CORTEX_M55)
#define LV_USE_NEMA_LIB	LV_NEMA_LIB_M55
#elif defined(CONFIG_CPU_CORTEX_M7)
#define LV_USE_NEMA_LIB	LV_NEMA_LIB_M7
#else
#define LV_USE_NEMA_LIB	LV_NEMA_LIB_NONE
#endif
#endif /* CONFIG_LV_Z_NEMA_GFX */

/*
 * Needed because of a workaround for a GCC bug,
 * see https://github.com/lvgl/lvgl/issues/3078
 */
#define LV_CONF_SUPPRESS_DEFINE_CHECK 1

#endif /* ZEPHYR_MODULES_LVGL_LV_CONF_H_ */
