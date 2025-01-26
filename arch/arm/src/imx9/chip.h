/****************************************************************************
 * arch/arm/src/imx9/chip.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMX9_CHIP_H
#define __ARCH_ARM_SRC_IMX9_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#  include <arch/irq.h>
#  include <arch/imx9/chip.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache line sizes (in bytes) for the i.MX 9 M7 Core */

#define ARMV7M_DCACHE_LINESIZE 32  /* 32 bytes (8 words) */
#define ARMV7M_ICACHE_LINESIZE 32  /* 32 bytes (8 words) */

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX9_CHIP_H */
