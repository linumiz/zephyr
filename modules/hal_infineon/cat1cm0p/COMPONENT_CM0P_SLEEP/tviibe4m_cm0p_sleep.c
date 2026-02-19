/*
 * Copyright (c) 2022 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include "cy_device_headers.h"

#if defined(CY_DEVICE_TVIIBE4M)

#if defined(__APPLE__) && defined(__clang__)
__attribute__ ((__section__("__CY_M0P_IMAGE,__cy_m0p_image"), used))
#elif defined(__GNUC__) || defined(__ARMCC_VERSION)
__attribute__ ((__section__(".cy_m0p_image"), used))
#elif defined(__ICCARM__)
#pragma  location=".cy_m0p_image"
#else
#error "An unsupported toolchain"
#endif
const uint8_t cy_m0p_image[] = {
    #include <tviibe4m_cm0p_sleep.bin.inc>
};

#endif /* defined(CY_DEVICE_TVIIBE4M) */