/*
 * Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Definitions for boot code
 */

#ifndef _BOOT_H_
#define _BOOT_H_

#ifndef _ASMLANGUAGE

extern void *_vector_table[];
extern void __start(void);

#endif /* _ASMLANGUAGE */
#endif /* _BOOT_H_ */
