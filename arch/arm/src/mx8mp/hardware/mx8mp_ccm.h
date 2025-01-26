/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_ccm.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* Reference:
 *   "i.MX 8M Plus Applications Processor Reference Manual",
 *   Document Number: IMX8MPRM Rev. 1, 06/2021. NXP
 */

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_CCM_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/mx8mp_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CCM Register Addresses ***************************************************/

#define CCM_GPR0                     (MX8M_CCM + 0)
#define CCM_PLL_BASE                 (MX8M_CCM + 0x800)
#define CCM_CCGR_BASE                (MX8M_CCM + 0x4000)
#define CCM_CLK_ROOT_BASE            (MX8M_CCM + 0x8000)

/* CCM Common Register Offsets **********************************************/

#define CCM_SET                      0x4
#define CCM_CLR                      0x8
#define CCM_TOG                      0xC

/* CCM Clock Gating definitions *********************************************/

/* Chapter 5.1.2 Clock Root Selects */

#define CCM_DVFS_CLK_GATE            0
#define CCM_ANAMIX_CLK_GATE          1
#define CCM_CPU_CLK_GATE             2
#define CCM_CSU_CLK_GATE             3
#define CCM_DEBUG_CLK_GATE           4
#define CCM_DDRAM1_CLK_GATE          5
#define CCM_ECSPI1_CLK_GATE          7
#define CCM_ECSPI2_CLK_GATE          8
#define CCM_ECSPI3_CLK_GATE          9
#define CCM_ENET1_CLK_GATE           10
#define CCM_GPIO1_CLK_GATE           11
#define CCM_GPIO2_CLK_GATE           12
#define CCM_GPIO3_CLK_GATE           13
#define CCM_GPIO4_CLK_GATE           14
#define CCM_GPIO5_CLK_GATE           15
#define CCM_GPT1_CLK_GATE            16
#define CCM_GPT2_CLK_GATE            17
#define CCM_GPT3_CLK_GATE            18
#define CCM_GPT4_CLK_GATE            19
#define CCM_GPT5_CLK_GATE            20
#define CCM_GPT6_CLK_GATE            21
#define CCM_HS_CLK_GATE              22
#define CCM_I2C1_CLK_GATE            23
#define CCM_I2C2_CLK_GATE            24
#define CCM_I2C3_CLK_GATE            25
#define CCM_I2C4_CLK_GATE            26
#define CCM_IOMUX_CLK_GATE           27
#define CCM_IPMUX1_CLK_GATE          28
#define CCM_IPMUX2_CLK_GATE          29
#define CCM_IPMUX3_CLK_GATE          30
#define CCM_MU_CLK_GATE              33
#define CCM_OCOTP_CLK_GATE           34
#define CCM_OCRAM_CLK_GATE           35
#define CCM_OCRAM_S_CLK_GATE         36
#define CCM_PCIE_CLK_GATE            37
#define CCM_PERFMON1_CLK_GATE        38
#define CCM_PERFMON2_CLK_GATE        39
#define CCM_PWM1_CLK_GATE            40
#define CCM_PWM2_CLK_GATE            41
#define CCM_PWM3_CLK_GATE            42
#define CCM_PWM4_CLK_GATE            43
#define CCM_QOS_CLK_GATE             44
#define CCM_QOS_ETHENET_CLK_GATE     46
#define CCM_FLEXSPI_CLK_GATE         47
#define CCM_RAWNAND_CLK_GATE         48
#define CCM_RDC_CLK_GATE             49
#define CCM_ROM_CLK_GATE             50
#define CCM_I2C5_CLK_GATE            51
#define CCM_I2C6_CLK_GATE            52
#define CCM_CAN1_CLK_GATE            53
#define CCM_CAN2_CLK_GATE            54
#define CCM_SCTR_CLK_GATE            57
#define CCM_SDMA1_CLK_GATE           58
#define CCM_ENET_QOS_CLK_GATE        59
#define CCM_SEC_DEBUG_CLK_GATE       60
#define CCM_SEMA1_CLK_GATE           61
#define CCM_SEMA2_CLK_GATE           62
#define CCM_IRQ_STEER_CLK_GATE       63
#define CCM_SIM_ENET_CLK_GATE        64
#define CCM_SIM_M_CLK_GATE           65
#define CCM_SIM_MAIN_CLK_GATE        66
#define CCM_SIM_S_CLK_GATE           67
#define CCM_SIM_WAKEUP_CLK_GATE      68
#define CCM_GPU2D_CLK_GATE           69
#define CCM_GPU3D_CLK_GATE           70
#define CCM_SNVS_CLK_GATE            71
#define CCM_TRACE_CLK_GATE           72
#define CCM_UART1_CLK_GATE           73
#define CCM_UART2_CLK_GATE           74
#define CCM_UART3_CLK_GATE           75
#define CCM_UART4_CLK_GATE           76
#define CCM_USB_CLK_GATE             77
#define CCM_USB_PHY_CLK_GATE         79
#define CCM_USDHC1_CLK_GATE          81
#define CCM_USDHC2_CLK_GATE          82
#define CCM_WDOG1_CLK_GATE           83
#define CCM_WDOG2_CLK_GATE           84
#define CCM_WDOG3_CLK_GATE           85
#define CCM_VPU_G1_CLK_GATE          86
#define CCM_GPU_CLK_GATE             87
#define CCM_NOC_WRAPPER_CLK_GATE     88
#define CCM_VPU_VC8KE_CLK_GATE       89
#define CCM_VPUG2_CLK_GATE           90
#define CCM_NPU_CLK_GATE             91
#define CCM_HSIO_CLK_GATE            92
#define CCM_MEDIA_CLK_GATE           93
#define CCM_USDHC3_CLK_GATE          94
#define CCM_HDMI_CLK_GATE            95
#define CCM_XTAL_CLK_GATE            96
#define CCM_PLL_CLK_GATE             97
#define CCM_TSENSOR_CLK_GATE         98
#define CCM_VPU_CLK_GATE             99
#define CCM_MRPR_CLK_GATE            100
#define CCM_AUDIO_CLK_GATE           101

/* CCM Clock Gating Bit Definitions *****************************************/

/* Note: only bits of our domain will be applied - cf. 5.1.6.3 */
#define CLK_NOT_NEEDED               0x0000
#define CLK_RUN_NEEDED               0x1111
#define CLK_RUN_WAIT_NEEDED          0x2222
#define CLK_ALWAYS_NEEDED            0x3333

/* Clock root select definitions ********************************************/

/* Chapter 5.1.2 Clock Root Selects */
#define ARM_A53_CLK_ROOT             0
#define ARM_M7_CLK_ROOT              1
#define ML_CLK_ROOT                  2
#define GPU3D_CORE_CLK_ROOT          3
#define GPU3D_SHADER_CLK_ROOT        4
#define GPU2D_CLK_ROOT               5
#define AUDIO_AXI_CLK_ROOT           6
#define HSIO_AXI_CLK_ROOT            7
#define MEDIA_ISP_CLK_ROOT           8
#define MAIN_AXI_CLK_ROOT            16
#define ENET_AXI_CLK_ROOT            17
#define NAND_USDHC_BUS_CLK_ROOT      18
#define VPU_BUS_CLK_ROOT             19
#define MEDIA_AXI_CLK_ROOT           20
#define MEDIA_APB_CLK_ROOT           21
#define HDMI_APB_CLK_ROOT            22
#define HDMI_AXI_CLK_ROOT            23
#define GPU_AXI_CLK_ROOT             24
#define GPU_AHB_CLK_ROOT             25
#define NOC_CLK_ROOT                 26
#define NOC_IO_CLK_ROOT              27
#define ML_AXI_CLK_ROOT              28
#define ML_AHB_CLK_ROOT              29
#define AHB_CLK_ROOT                 32
#define IPG_CLK_ROOT                 33
#define AUDIO_AHB_CLK_ROOT           34
#define MEDIA_DISP2_CLK_ROOT         38
#define DRAM_SEL_CFG                 48
#define ARM_A53_CLK_ROOT_SEL         49
#define DRAM_ALT_CLK_ROOT            64
#define DRAM_APB_CLK_ROOT            65
#define VPU_G1_CLK_ROOT              66
#define VPU_G2_CLK_ROOT              67
#define CAN1_CLK_ROOT                68
#define CAN2_CLK_ROOT                69
#define MEMREPAIR_CLK_ROOT           70
#define PCIE_PHY_CLK_ROOT            71
#define PCIE_AUX_CLK_ROOT            72
#define I2C5_CLK_ROOT                73
#define I2C6_CLK_ROOT                74
#define SAI1_CLK_ROOT                75
#define SAI2_CLK_ROOT                76
#define SAI3_CLK_ROOT                77
#define SAI5_CLK_ROOT                79
#define SAI6_CLK_ROOT                80
#define ENET_QOS_CLK_ROOT            81
#define ENET_QOS_TIMER_CLK_ROOT      82
#define ENET_REF_CLK_ROOT            83
#define ENET_TIMER_CLK_ROOT          84
#define ENET_PHY_REF_CLK_ROOT        85
#define NAND_CLK_ROOT                86
#define QSPI_CLK_ROOT                87
#define USDHC1_CLK_ROOT              88
#define USDHC2_CLK_ROOT              89
#define I2C1_CLK_ROOT                90
#define I2C2_CLK_ROOT                91
#define I2C3_CLK_ROOT                92
#define I2C4_CLK_ROOT                93
#define UART1_CLK_ROOT               94
#define UART2_CLK_ROOT               95
#define UART3_CLK_ROOT               96
#define UART4_CLK_ROOT               97
#define GIC_CLK_ROOT                 100
#define ECSPI1_CLK_ROOT              101
#define ECSPI2_CLK_ROOT              102
#define PWM1_CLK_ROOT                103
#define PWM2_CLK_ROOT                104
#define PWM3_CLK_ROOT                105
#define PWM4_CLK_ROOT                106
#define GPT1_CLK_ROOT                107
#define GPT2_CLK_ROOT                108
#define GPT3_CLK_ROOT                109
#define GPT4_CLK_ROOT                110
#define GPT5_CLK_ROOT                111
#define GPT6_CLK_ROOT                112
#define TRACE_CLK_ROOT               113
#define WDOG_CLK_ROOT                114
#define WRCLK_CLK_ROOT               115
#define IPP_DO_CLKO1                 116
#define IPP_DO_CLKO2                 117
#define HDMI_FDCC_TST_CLK_ROOT       118
#define HDMI_27M_CLK_ROOT            119
#define HDMI_REF_266M_CLK_ROOT       120
#define USDHC3_CLK_ROOT              121
#define MEDIA_CAM1_PIX_CLK_ROOT      122
#define MEDIA_MIPI_PHY1_REF_CLK_ROOT 123
#define MEDIA_DISP1_PIX_CLK_ROOT     124
#define MEDIA_CAM2_PIX_CLK_ROOT      125
#define MEDIA_LDB_CLK_ROOT           126
#define MEDIA_MIPI_TEST_BYTE_CLK     130
#define ECSPI3_CLK_ROOT              131
#define PDM_CLK_ROOT                 132
#define VPU_VC8000E_CLK_ROOT         133
#define SAI7_CLK_ROOT                134
#define CLOCK_ROOT_MAP_SIZE          (SAI7_CLK_ROOT + 1)

/* CLK_ROOT Register Bit Definitions ****************************************/

#define CCM_CLK_ROOT_ENABLE          (1 << 28)
#define CCM_CLK_ROOT_MUX_SHIFT       24
#define CCM_CLK_ROOT_MUX_MASK        (7 << CCM_CLK_ROOT_MUX_SHIFT)
#define CCM_CLK_ROOT_PRE_PODF_SHIFT  16
#define CCM_CLK_ROOT_PRE_PODF_MASK   (7 << CCM_CLK_ROOT_PRE_PODF_SHIFT)
#define CCM_CLK_ROOT_POST_PODF_SHIFT 0
#define CCM_CLK_ROOT_POST_PODF_MASK  (0x3f << CCM_CLK_ROOT_POST_PODF_SHIFT)

/* Analog PLL Register Addresses ********************************************/

#define CCM_ANALOG                   (MX8M_CCM_ANALOG + 0)

#define CCM_ANALOG_AUDIO_PLL1        (MX8M_CCM_ANALOG + 0x000)
#define CCM_ANALOG_AUDIO_PLL2        (MX8M_CCM_ANALOG + 0x014)
#define CCM_ANALOG_VIDEO_PLL1        (MX8M_CCM_ANALOG + 0x028)
#define CCM_ANALOG_DRAM_PLL          (MX8M_CCM_ANALOG + 0x050)
#define CCM_ANALOG_GPU_PLL           (MX8M_CCM_ANALOG + 0x064)
#define CCM_ANALOG_VPU_PLL           (MX8M_CCM_ANALOG + 0x074)
#define CCM_ANALOG_ARM_PLL           (MX8M_CCM_ANALOG + 0x084)
#define CCM_ANALOG_SYSTEM_PLL1       (MX8M_CCM_ANALOG + 0x094)
#define CCM_ANALOG_SYSTEM_PLL2       (MX8M_CCM_ANALOG + 0x104)
#define CCM_ANALOG_SYSTEM_PLL3       (MX8M_CCM_ANALOG + 0x114)

/* Analog PLL Register Offsets **********************************************/

#define CCM_ANALOG_GEN               0
#define CCM_ANALOG_FDIV0             4
#define CCM_ANALOG_FDIV1             8

/* CLK_ROOT Register Bit Definitions ****************************************/

/* General Function Control Register */

#define CCM_PLL_LOCK                 (1 << 31)
#define CCM_PLL_EXT_BYPASS           (1 << 16)
#define CCM_PLL_CLKE                 (1 << 13)
#define CCM_PLL_CLKE_OVERRIDE        (1 << 12)
#define CCM_PLL_RST                  (1 << 9)
#define CCM_PLL_RST_OVERRIDE         (1 << 8)
#define CCM_PLL_BYPASS               (1 << 4)
#define CCM_PAD_CLK_SEL_SHIFT        2
#define CCM_PAD_CLK_SEL_MASK         (3 << CCM_PAD_CLK_SEL_SHIFT)
#define CCM_PLL_REF_CLK_SEL_SHIFT    0
#define CCM_PLL_REF_CLK_SEL_MASK     (3 << CCM_PLL_REF_CLK_SEL_SHIFT)

/* Divide and Fraction Data Control 0 Register */

#define CCM_FDIV0_MAIN_DIV_SHIFT     12
#define CCM_FDIV0_MAIN_DIV_MASK      (0x3ff << CCM_FDIV0_MAIN_DIV_SHIFT)
#define CCM_FDIV0_PRE_DIV_SHIFT      4
#define CCM_FDIV0_PRE_DIV_MASK       (0x3f << CCM_FDIV0_PRE_DIV_SHIFT)
#define CCM_FDIV0_POST_DIV_SHIFT     0
#define CCM_FDIV0_POST_DIV_MASK      (7 << CCM_FDIV0_POST_DIV_SHIFT)

/* Divide and Fraction Data Control 1 Register */

#define CCM_FDIV1_DSM_SHIFT          0
#define CCM_FDIV1_DSM_MASK           (0xffff << CCM_FDIV1_DSM_SHIFT)

/* Input clocks definitions *************************************************/

#define ARM_PLL_CLK                  12
#define GPU_PLL_CLK                  13
#define VPU_PLL_CLK                  14
#define DRAM_PLL1_CLK                15
#define SYSTEM_PLL1_CLK              16
#define SYSTEM_PLL1_DIV2_CLK         17
#define SYSTEM_PLL1_DIV3_CLK         18
#define SYSTEM_PLL1_DIV4_CLK         19
#define SYSTEM_PLL1_DIV5_CLK         20
#define SYSTEM_PLL1_DIV6_CLK         21
#define SYSTEM_PLL1_DIV8_CLK         22
#define SYSTEM_PLL1_DIV10_CLK        23
#define SYSTEM_PLL1_DIV20_CLK        24
#define SYSTEM_PLL2_CLK              25
#define SYSTEM_PLL2_DIV2_CLK         26
#define SYSTEM_PLL2_DIV3_CLK         27
#define SYSTEM_PLL2_DIV4_CLK         28
#define SYSTEM_PLL2_DIV5_CLK         29
#define SYSTEM_PLL2_DIV6_CLK         30
#define SYSTEM_PLL2_DIV8_CLK         31
#define SYSTEM_PLL2_DIV10_CLK        32
#define SYSTEM_PLL2_DIV20_CLK        33
#define SYSTEM_PLL3_CLK              34
#define AUDIO_PLL1_CLK               35
#define AUDIO_PLL2_CLK               36
#define VIDEO_PLL_CLK                37

/* Theses definitions values are arbitrary: previous definitions
 * are used in both PLL_CTRL and clock tree mapping while
 * theses definitions have no meaning for PLL_CTR (since they are
 * external clock source) but are still use in clock tree mapping.
 */

#define OSC_32K_REF_CLK                     40
#define OSC_24M_REF_CLK                     41
#define EXT_CLK_1                           42
#define EXT_CLK_2                           43
#define EXT_CLK_3                           44
#define EXT_CLK_4                           45
#define CLK_ROOT_SRC_UNDEFINED              46

/* Clock muxing definitions *************************************************/

#define ARM_A53_CLK_MUX       { OSC_24M_REF_CLK, ARM_PLL_CLK, SYSTEM_PLL2_DIV2_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL1_DIV2_CLK, AUDIO_PLL1_CLK, SYSTEM_PLL3_CLK }
#define ARM_M7_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL2_DIV4_CLK, VPU_PLL_CLK, SYSTEM_PLL1_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, SYSTEM_PLL3_CLK   }
#define ML_CLK_MUX            { OSC_24M_REF_CLK, GPU_PLL_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define GPU3D_CORE_CLK_MUX    { OSC_24M_REF_CLK, GPU_PLL_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define GPU3D_SHADER_CLK_MUX  { OSC_24M_REF_CLK, GPU_PLL_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define GPU2D_CLK_MUX         { OSC_24M_REF_CLK, GPU_PLL_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define AUDIO_AXI_CLK_MUX     { OSC_24M_REF_CLK, GPU_PLL_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define HSIO_AXI_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV2_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL2_DIV5_CLK, EXT_CLK_2, EXT_CLK_4, AUDIO_PLL2_CLK }
#define MEDIA_ISP_CLK_MUX     { OSC_24M_REF_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV20_CLK, AUDIO_PLL2_CLK, EXT_CLK_1, SYSTEM_PLL2_DIV2_CLK }

#define MAIN_AXI_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV3_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_DIV4_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV8_CLK }
#define ENET_AXI_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV3_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_DIV4_CLK, SYSTEM_PLL2_DIV5_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, SYSTEM_PLL3_CLK }
#define NAND_USDHC_BUS_CLK_MUX { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV3_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL1_DIV6_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV4_CLK, AUDIO_PLL1_CLK }
#define VPU_BUS_CLK_MUX       { OSC_24M_REF_CLK, SYSTEM_PLL1_CLK, VPU_PLL_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL1_DIV8_CLK }
#define MEDIA_AXI_CLK_MUX     { OSC_24M_REF_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV20_CLK, AUDIO_PLL2_CLK, EXT_CLK_1, SYSTEM_PLL2_DIV2_CLK }
#define MEDIA_APB_CLK_MUX     { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV20_CLK, AUDIO_PLL2_CLK, EXT_CLK_1, SYSTEM_PLL1_DIV6_CLK }
#define HDMI_APB_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV20_CLK, AUDIO_PLL2_CLK, EXT_CLK_1, SYSTEM_PLL1_DIV6_CLK }
#define HDMI_AXI_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV20_CLK, AUDIO_PLL2_CLK, EXT_CLK_1, SYSTEM_PLL2_DIV2_CLK }
#define GPU_AXI_CLK_MUX       { OSC_24M_REF_CLK, SYSTEM_PLL1_CLK, GPU_PLL_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define GPU_AHB_CLK_MUX       { OSC_24M_REF_CLK, SYSTEM_PLL1_CLK, GPU_PLL_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define NOC_CLK_MUX           { OSC_24M_REF_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL2_DIV2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define NOC_IO_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL2_DIV2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define ML_AXI_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL1_CLK, GPU_PLL_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define ML_AHB_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL1_CLK, GPU_PLL_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }

#define AHB_CLK_MUX           { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV6_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK }
#define IPG_CLK_MUX           { OSC_24M_REF_CLK, }
#define AUDIO_AHB_CLK_MUX     { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV2_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL2_DIV6_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK }

#define MEDIA_DISP2_CLK_MUX   { OSC_24M_REF_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK, AUDIO_PLL1_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL3_CLK, EXT_CLK_4 }

#define DRAM_SEL_CFG_MUX          { DRAM_PLL1_CLK, }
#define ARM_A53_CLK_ROOT_SEL_MUX  { ARM_PLL_CLK, }

#define DRAM_ALT_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL1_DIV8_CLK, SYSTEM_PLL2_DIV2_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, SYSTEM_PLL1_DIV3_CLK }
#define DRAM_APB_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV4_CLK, AUDIO_PLL2_CLK }
#define VPU_G1_CLK_MUX        { OSC_24M_REF_CLK, VPU_PLL_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL1_DIV8_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK }
#define VPU_G2_CLK_MUX        { OSC_24M_REF_CLK, VPU_PLL_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL1_DIV8_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK }
#define CAN1_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV4_CLK, AUDIO_PLL2_CLK }
#define CAN2_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV4_CLK, AUDIO_PLL2_CLK }
#define MEMREPAIR_CLK_MUX     { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV6_CLK }
#define PCIE_PHY_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL2_DIV2_CLK, EXT_CLK_1, EXT_CLK_2, EXT_CLK_3, EXT_CLK_4, SYSTEM_PLL1_DIV2_CLK }
#define PCIE_AUX_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV10_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_DIV4_CLK }
#define I2C5_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV6_CLK}
#define I2C6_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV6_CLK}
#define SAI1_CLK_MUX          { OSC_24M_REF_CLK, AUDIO_PLL1_CLK, AUDIO_PLL2_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV6_CLK, CLK_ROOT_SRC_UNDEFINED, EXT_CLK_1, EXT_CLK_2}
#define SAI2_CLK_MUX          { OSC_24M_REF_CLK, AUDIO_PLL1_CLK, AUDIO_PLL2_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV6_CLK, CLK_ROOT_SRC_UNDEFINED, EXT_CLK_2, EXT_CLK_3}
#define SAI3_CLK_MUX          { OSC_24M_REF_CLK, AUDIO_PLL1_CLK, AUDIO_PLL2_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV6_CLK, CLK_ROOT_SRC_UNDEFINED, EXT_CLK_3, EXT_CLK_4}

#define SAI5_CLK_MUX          { OSC_24M_REF_CLK, AUDIO_PLL1_CLK, AUDIO_PLL2_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV6_CLK, CLK_ROOT_SRC_UNDEFINED, EXT_CLK_2, EXT_CLK_3}
#define SAI6_CLK_MUX          { OSC_24M_REF_CLK, AUDIO_PLL1_CLK, AUDIO_PLL2_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV6_CLK, CLK_ROOT_SRC_UNDEFINED, EXT_CLK_3, EXT_CLK_4}
#define ENET_QOS_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV5_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, EXT_CLK_4 }
#define ENET_QOS_TIMER_CLK_MUX { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, AUDIO_PLL1_CLK, EXT_CLK_1, EXT_CLK_2, EXT_CLK_3, EXT_CLK_4, VIDEO_PLL_CLK }
#define ENET_REF_CLK_MUX      { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV5_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, EXT_CLK_4 }
#define ENET_TIMER_CLK_MUX    { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, AUDIO_PLL1_CLK, EXT_CLK_1, EXT_CLK_2, EXT_CLK_3, EXT_CLK_4, VIDEO_PLL_CLK }
#define ENET_PHY_REF_CLK_MUX  { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL2_DIV2_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK }
#define NAND_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV2_CLK, AUDIO_PLL1_CLK, SYSTEM_PLL1_DIV2_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV4_CLK, VIDEO_PLL_CLK }
#define QSPI_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL2_DIV3_CLK, SYSTEM_PLL2_DIV2_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV3_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV8_CLK }
#define USDHC1_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_DIV2_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV3_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV8_CLK }
#define USDHC2_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_DIV2_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV3_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV8_CLK }
#define I2C1_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV6_CLK }
#define I2C2_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV6_CLK }
#define I2C3_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV6_CLK }
#define I2C4_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK, VIDEO_PLL_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV6_CLK }
#define UART1_CLK_MUX         { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV10_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL3_CLK, EXT_CLK_2, EXT_CLK_4, AUDIO_PLL2_CLK }
#define UART2_CLK_MUX         { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV10_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL3_CLK, EXT_CLK_2, EXT_CLK_3, AUDIO_PLL2_CLK }
#define UART3_CLK_MUX         { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV10_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL3_CLK, EXT_CLK_2, EXT_CLK_4, AUDIO_PLL2_CLK }
#define UART4_CLK_MUX         { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV10_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL3_CLK, EXT_CLK_2, EXT_CLK_3, AUDIO_PLL2_CLK }

#define GIC_CLK_MUX           { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_DIV2_CLK, EXT_CLK_4, AUDIO_PLL2_CLK }
#define ECSPI1_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV4_CLK, AUDIO_PLL2_CLK }
#define ECSPI2_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV4_CLK, AUDIO_PLL2_CLK }
#define PWM1_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL3_CLK, EXT_CLK_1, SYSTEM_PLL1_DIV10_CLK, VIDEO_PLL_CLK }
#define PWM2_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL3_CLK, EXT_CLK_1, SYSTEM_PLL1_DIV10_CLK, VIDEO_PLL_CLK }
#define PWM3_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL3_CLK, EXT_CLK_2, SYSTEM_PLL1_DIV10_CLK, VIDEO_PLL_CLK }
#define PWM4_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL3_CLK, EXT_CLK_2, SYSTEM_PLL1_DIV10_CLK, VIDEO_PLL_CLK }
#define GPT1_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL1_DIV20_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV10_CLK, AUDIO_PLL1_CLK, EXT_CLK_1 }
#define GPT2_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL1_DIV20_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV10_CLK, AUDIO_PLL1_CLK, EXT_CLK_2 }
#define GPT3_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL1_DIV20_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV10_CLK, AUDIO_PLL1_CLK, EXT_CLK_3 }
#define GPT4_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL1_DIV20_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV10_CLK, AUDIO_PLL1_CLK, EXT_CLK_1 }
#define GPT5_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL1_DIV20_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV10_CLK, AUDIO_PLL1_CLK, EXT_CLK_2 }
#define GPT6_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL1_DIV20_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV10_CLK, AUDIO_PLL1_CLK, EXT_CLK_3 }
#define TRACE_CLK_MUX         { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV6_CLK, SYSTEM_PLL1_DIV5_CLK, VPU_PLL_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL3_CLK, EXT_CLK_1, EXT_CLK_3 }
#define WDOG_CLK_MUX          { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV6_CLK, SYSTEM_PLL1_DIV5_CLK, VPU_PLL_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV10_CLK, SYSTEM_PLL2_DIV6_CLK }
#define WRCLK_CLK_MUX         { OSC_24M_REF_CLK, }
#define IPP_DO_CLKO1_MUX      { OSC_24M_REF_CLK, }
#define IPP_DO_CLKO2_MUX      { OSC_24M_REF_CLK, }
#define HDMI_FDCC_TST_CLK_MUX { OSC_24M_REF_CLK, }
#define HDMI_27M_CLK_MUX      { OSC_24M_REF_CLK, }
#define HDMI_REF_266M_CLK_MUX { OSC_24M_REF_CLK, }
#define USDHC3_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL1_DIV2_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_DIV2_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL1_DIV3_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL1_DIV8_CLK }
#define MEDIA_CAM1_PIX_CLK_MUX { OSC_24M_REF_CLK, }
#define MEDIA_MIPI_PHY1_REF_CLK_MUX  { OSC_24M_REF_CLK, }
#define MEDIA_DISP1_PIX_CLK_MUX      { OSC_24M_REF_CLK, }
#define MEDIA_CAM2_PIX_CLK_MUX       { OSC_24M_REF_CLK, }
#define MEDIA_LDB_CLK_MUX     { OSC_24M_REF_CLK, }

#define MEDIA_MIPI_TEST_BYTE_CLK_MUX  { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL2_DIV20_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV10_CLK, SYSTEM_PLL1_DIV10_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_DIV4_CLK }
#define ECSPI3_CLK_MUX        { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV5_CLK, SYSTEM_PLL1_DIV20_CLK, SYSTEM_PLL1_DIV5_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL3_CLK, SYSTEM_PLL2_DIV4_CLK, AUDIO_PLL2_CLK }
#define PDM_CLK_MUX           { OSC_24M_REF_CLK, SYSTEM_PLL2_DIV10_CLK, AUDIO_PLL1_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_CLK, SYSTEM_PLL3_CLK, EXT_CLK_3, AUDIO_PLL2_CLK }
#define VPU_VC8000E_CLK_MUX   { OSC_24M_REF_CLK, VPU_PLL_CLK, SYSTEM_PLL1_CLK, SYSTEM_PLL2_CLK, AUDIO_PLL2_CLK, SYSTEM_PLL2_DIV8_CLK, SYSTEM_PLL3_CLK, AUDIO_PLL1_CLK }
#define SAI7_CLK_MUX          { OSC_24M_REF_CLK, AUDIO_PLL1_CLK, AUDIO_PLL2_CLK, VIDEO_PLL_CLK, SYSTEM_PLL1_DIV6_CLK, CLK_ROOT_SRC_UNDEFINED, EXT_CLK_3, EXT_CLK_4 }

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_CCM_H */
