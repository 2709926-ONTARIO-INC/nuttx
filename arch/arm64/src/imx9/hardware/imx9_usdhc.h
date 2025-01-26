/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_usdhc.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_USDHC_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_USDHC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMX9_USDHC_DSADDR_OFFSET        0x0000 /* DMA System Address Register */
#define IMX9_USDHC_BLKATTR_OFFSET       0x0004 /* Block Attributes Register */
#define IMX9_USDHC_CMDARG_OFFSET        0x0008 /* Command Argument Register */
#define IMX9_USDHC_XFERTYP_OFFSET       0x000c /* Transfer Type Register */
#define IMX9_USDHC_CMDRSP0_OFFSET       0x0010 /* Command Response 0 */
#define IMX9_USDHC_CMDRSP1_OFFSET       0x0014 /* Command Response 1 */
#define IMX9_USDHC_CMDRSP2_OFFSET       0x0018 /* Command Response 2 */
#define IMX9_USDHC_CMDRSP3_OFFSET       0x001c /* Command Response 3 */
#define IMX9_USDHC_DATAPORT_OFFSET      0x0020 /* Buffer Data Port Register */
#define IMX9_USDHC_PRSSTAT_OFFSET       0x0024 /* Present State Register */
#define IMX9_USDHC_PROCTL_OFFSET        0x0028 /* Protocol Control Register */
#define IMX9_USDHC_SYSCTL_OFFSET        0x002c /* System Control Register */
#define IMX9_USDHC_IRQSTAT_OFFSET       0x0030 /* Interrupt Status Register */
#define IMX9_USDHC_IRQSTATEN_OFFSET     0x0034 /* Interrupt Status Enable Register */
#define IMX9_USDHC_IRQSIGEN_OFFSET      0x0038 /* Interrupt Signal Enable Register */
#define IMX9_USDHC_AC12ERR_OFFSET       0x003c /* Auto CMD12 Error Status Register */
#define IMX9_USDHC_HTCAPBLT_OFFSET      0x0040 /* Host Controller Capabilities */
#define IMX9_USDHC_WML_OFFSET           0x0044 /* Watermark Level Register */
#define IMX9_USDHC_MIX_OFFSET           0x0048 /* Mixer Control Register */
#define IMX9_USDHC_FEVT_OFFSET          0x0050 /* Force Event Register */
#define IMX9_USDHC_ADMAES_OFFSET        0x0054 /* ADMA Error Status Register */
#define IMX9_USDHC_ADSADDR_OFFSET       0x0058 /* ADMA System Address Register */
#define IMX9_USDHC_DLL_CONTROL_OFFSET   0x0060 /* DLL Control Register */
#define IMX9_USDHC_DLL_STATUS_OFFSET    0x0064 /* DLL Status Register */
#define IMX9_USDHC_CLK_TUNE_CTRL_OFFSET 0x0068 /* Clock turing control Register */
#define IMX9_USDHC_VENDOR_OFFSET        0x00c0 /* Vendor Specific Register */
#define IMX9_USDHC_MMCBOOT_OFFSET       0x00c4 /* MMC Boot Register */
#define IMX9_USDHC_VENDOR2_OFFSET       0x00c8 /* Vendor 2 Register */
#define IMX9_USDHC_TC_OFFSET            0x00cc /* Tuning Control Register */
#define IMX9_USDHC_CQVER_OFFSET         0x0100 /* Command Queuing Version */
#define IMX9_USDHC_CQCAP_OFFSET         0x0104 /* Command Queuing Capabilities */
#define IMX9_USDHC_CQCFG_OFFSET         0x0108 /* Command Queuing Configuration */
#define IMX9_USDHC_CQCTL_OFFSET         0x010c /* Command Queuing Control */
#define IMX9_USDHC_CQIS_OFFSET          0x0110 /* Command Queuing Interrupt Status */
#define IMX9_USDHC_CQISTE_OFFSET        0x0114 /* Command Queuing Interrupt Status Enable */
#define IMX9_USDHC_CQISGE_OFFSET        0x0118 /* Command Queuing Interrupt Signal Enable */
#define IMX9_USDHC_CQIC_OFFSET          0x011c /* Command Queuing Interrupt Coalescing */
#define IMX9_USDHC_CQTDLBA_OFFSET       0x0120 /* Command Queuing Task Descriptor List Base Address */
#define IMX9_USDHC_CQTDLBAU_OFFSET      0x0124 /* Command Queuing Task Descriptor List Base Address Upper 32 Bits */
#define IMX9_USDHC_CQTDBR_OFFSET        0x0128 /* Command Queuing Task Doorbell */
#define IMX9_USDHC_CQTCN_OFFSET         0x012c /* Command Queuing Task Completion Notification */
#define IMX9_USDHC_CQDQS_OFFSET         0x0130 /* Command Queuing Device Queue Status */
#define IMX9_USDHC_CQDPT_OFFSET         0x0134 /* Command Queuing Device Pending Tasks */
#define IMX9_USDHC_CQTCLR_OFFSET        0x0138 /* Command Queuing Task Clear */
#define IMX9_USDHC_CQSSC1_OFFSET        0x0140 /* Command Queuing Send Status Configuration 1 */
#define IMX9_USDHC_CQSSC2_OFFSET        0x0144 /* Command Queuing Send Status Configuration 2 */
#define IMX9_USDHC_CQCRDCT_OFFSET       0x0148 /* Command Queuing Command Response for Direct-Command Task */
#define IMX9_USDHC_CQRMEM_OFFSET        0x0150 /* Command Queuing Response Mode Error Mask */
#define IMX9_USDHC_CQTERRI_OFFSET       0x0154 /* Command Queuing Task Error Information */
#define IMX9_USDHC_CQCRI_OFFSET         0x0158 /* Command Queuing Command Response Index */
#define IMX9_USDHC_CQCRA_OFFSET         0x015c /* Command Queuing Command Response Argument */

/* Register Addresses *******************************************************/

/* For USDHC1 ... */

#define IMX9_USDHC1_DSADDR              (IMX9_USDHC1_BASE + IMX9_USDHC_DSADDR_OFFSET)
#define IMX9_USDHC1_BLKATTR             (IMX9_USDHC1_BASE + IMX9_USDHC_BLKATTR_OFFSET)
#define IMX9_USDHC1_CMDARG              (IMX9_USDHC1_BASE + IMX9_USDHC_CMDARG_OFFSET)
#define IMX9_USDHC1_XFERTYP             (IMX9_USDHC1_BASE + IMX9_USDHC_XFERTYP_OFFSET)
#define IMX9_USDHC1_CMDRSP0             (IMX9_USDHC1_BASE + IMX9_USDHC_CMDRSP0_OFFSET)
#define IMX9_USDHC1_CMDRSP1             (IMX9_USDHC1_BASE + IMX9_USDHC_CMDRSP1_OFFSET)
#define IMX9_USDHC1_CMDRSP2             (IMX9_USDHC1_BASE + IMX9_USDHC_CMDRSP2_OFFSET)
#define IMX9_USDHC1_CMDRSP3             (IMX9_USDHC1_BASE + IMX9_USDHC_CMDRSP3_OFFSET)
#define IMX9_USDHC1_DATAPORT            (IMX9_USDHC1_BASE + IMX9_USDHC_DATAPORT_OFFSET)
#define IMX9_USDHC1_PRSSTAT             (IMX9_USDHC1_BASE + IMX9_USDHC_PRSSTAT_OFFSET)
#define IMX9_USDHC1_PROCTL              (IMX9_USDHC1_BASE + IMX9_USDHC_PROCTL_OFFSET)
#define IMX9_USDHC1_SYSCTL              (IMX9_USDHC1_BASE + IMX9_USDHC_SYSCTL_OFFSET)
#define IMX9_USDHC1_IRQSTAT             (IMX9_USDHC1_BASE + IMX9_USDHC_IRQSTAT_OFFSET)
#define IMX9_USDHC1_IRQSTATEN           (IMX9_USDHC1_BASE + IMX9_USDHC_IRQSTATEN_OFFSET)
#define IMX9_USDHC1_IRQSIGEN            (IMX9_USDHC1_BASE + IMX9_USDHC_IRQSIGEN_OFFSET)
#define IMX9_USDHC1_AC12ERR             (IMX9_USDHC1_BASE + IMX9_USDHC_AC12ERR_OFFSET)
#define IMX9_USDHC1_HTCAPBLT            (IMX9_USDHC1_BASE + IMX9_USDHC_HTCAPBLT_OFFSET)
#define IMX9_USDHC1_WML                 (IMX9_USDHC1_BASE + IMX9_USDHC_WML_OFFSET)
#define IMX9_USDHC1_MIX                 (IMX9_USDHC1_BASE + IMX9_USDHC_MIX_OFFSET)
#define IMX9_USDHC1_FEVT                (IMX9_USDHC1_BASE + IMX9_USDHC_FEVT_OFFSET)
#define IMX9_USDHC1_ADMAES              (IMX9_USDHC1_BASE + IMX9_USDHC_ADMAES_OFFSET)
#define IMX9_USDHC1_ADSADDR             (IMX9_USDHC1_BASE + IMX9_USDHC_ADSADDR_OFFSET)
#define IMX9_USDHC1_DLL_CONTROL         (IMX9_USDHC1_BASE + IMX9_USDHC_DLL_CONTROL_OFFSET)
#define IMX9_USDHC1_DLL_STATUS          (IMX9_USDHC1_BASE + IMX9_USDHC_DLL_STATUS)
#define IMX9_USDHC1_CLK_TUNE_CTRL       (IMX9_USDHC1_BASE + IMX9_USDHC_CLK_TUNE_CTRL)
#define IMX9_USDHC1_VENDOR              (IMX9_USDHC1_BASE + IMX9_USDHC_VENDOR_OFFSET)
#define IMX9_USDHC1_MMCBOOT             (IMX9_USDHC1_BASE + IMX9_USDHC_MMCBOOT_OFFSET)
#define IMX9_USDHC1_VENDOR2             (IMX9_USDHC1_BASE + IMX9_USDHC_VENDOR2_OFFSET)
#define IMX9_USDHC1_TC                  (IMX9_USDHC1_BASE + IMX9_USDHC_TC_OFFSET)
#define IMX9_USDHC1_CQVER               (IMX9_USDHC1_BASE + IMX9_USDHC_CQVER_OFFSET)
#define IMX9_USDHC1_CQCAP               (IMX9_USDHC1_BASE + IMX9_USDHC_CQCAP_OFFSET)
#define IMX9_USDHC1_CQCFG               (IMX9_USDHC1_BASE + IMX9_USDHC_CQCFG_OFFSET)
#define IMX9_USDHC1_CQCTL               (IMX9_USDHC1_BASE + IMX9_USDHC_CQCTL_OFFSET)
#define IMX9_USDHC1_CQIS                (IMX9_USDHC1_BASE + IMX9_USDHC_CQIS_OFFSET)
#define IMX9_USDHC1_CQISTE              (IMX9_USDHC1_BASE + IMX9_USDHC_CQISTE_OFFSET)
#define IMX9_USDHC1_CQISGE              (IMX9_USDHC1_BASE + IMX9_USDHC_CQISGE_OFFSET)
#define IMX9_USDHC1_CQIC                (IMX9_USDHC1_BASE + IMX9_USDHC_CQIC_OFFSET)
#define IMX9_USDHC1_CQTDLBA             (IMX9_USDHC1_BASE + IMX9_USDHC_CQTDLBA_OFFSET)
#define IMX9_USDHC1_CQTDLBAU            (IMX9_USDHC1_BASE + IMX9_USDHC_CQTDLBAU_OFFSET)
#define IMX9_USDHC1_CQTDBR              (IMX9_USDHC1_BASE + IMX9_USDHC_CQTDBR_OFFSET)
#define IMX9_USDHC1_CQTCN               (IMX9_USDHC1_BASE + IMX9_USDHC_CQTCN_OFFSET)
#define IMX9_USDHC1_CQDQS               (IMX9_USDHC1_BASE + IMX9_USDHC_CQDQS_OFFSET)
#define IMX9_USDHC1_CQDPT               (IMX9_USDHC1_BASE + IMX9_USDHC_CQDPT_OFFSET)
#define IMX9_USDHC1_CQTCL               (IMX9_USDHC1_BASE + IMX9_USDHC_CQTCLR_OFFSET)
#define IMX9_USDHC1_CQSSC1              (IMX9_USDHC1_BASE + IMX9_USDHC_CQSSC1_OFFSET)
#define IMX9_USDHC1_CQSSC2              (IMX9_USDHC1_BASE + IMX9_USDHC_CQSSC2_OFFSET)
#define IMX9_USDHC1_CQCRDCT             (IMX9_USDHC1_BASE + IMX9_USDHC_CQCRDCT_OFFSET)
#define IMX9_USDHC1_CQRMEM              (IMX9_USDHC1_BASE + IMX9_USDHC_CQRMEM_OFFSET)
#define IMX9_USDHC1_CQTERRI             (IMX9_USDHC1_BASE + IMX9_USDHC_CQTERRI_OFFSET)
#define IMX9_USDHC1_CQCRI               (IMX9_USDHC1_BASE + IMX9_USDHC_CQCRI_OFFSET)
#define IMX9_USDHC1_CQCRA               (IMX9_USDHC1_BASE + IMX9_USDHC_CQCRA_OFFSET)

/* For USDHC2 ... */

#define IMX9_USDHC2_DSADDR              (IMX9_USDHC2_BASE + IMX9_USDHC_DSADDR_OFFSET)
#define IMX9_USDHC2_BLKATTR             (IMX9_USDHC2_BASE + IMX9_USDHC_BLKATTR_OFFSET)
#define IMX9_USDHC2_CMDARG              (IMX9_USDHC2_BASE + IMX9_USDHC_CMDARG_OFFSET)
#define IMX9_USDHC2_XFERTYP             (IMX9_USDHC2_BASE + IMX9_USDHC_XFERTYP_OFFSET)
#define IMX9_USDHC2_CMDRSP0             (IMX9_USDHC2_BASE + IMX9_USDHC_CMDRSP0_OFFSET)
#define IMX9_USDHC2_CMDRSP1             (IMX9_USDHC2_BASE + IMX9_USDHC_CMDRSP1_OFFSET)
#define IMX9_USDHC2_CMDRSP2             (IMX9_USDHC2_BASE + IMX9_USDHC_CMDRSP2_OFFSET)
#define IMX9_USDHC2_CMDRSP3             (IMX9_USDHC2_BASE + IMX9_USDHC_CMDRSP3_OFFSET)
#define IMX9_USDHC2_DATAPORT            (IMX9_USDHC2_BASE + IMX9_USDHC_DATPORT_OFFSET)
#define IMX9_USDHC2_PRSSTAT             (IMX9_USDHC2_BASE + IMX9_USDHC_PRSSTAT_OFFSET)
#define IMX9_USDHC2_PROCTL              (IMX9_USDHC2_BASE + IMX9_USDHC_PROCTL_OFFSET)
#define IMX9_USDHC2_SYSCTL              (IMX9_USDHC2_BASE + IMX9_USDHC_SYSCTL_OFFSET)
#define IMX9_USDHC2_IRQSTAT             (IMX9_USDHC2_BASE + IMX9_USDHC_IRQSTAT_OFFSET)
#define IMX9_USDHC2_IRQSTATEN           (IMX9_USDHC2_BASE + IMX9_USDHC_IRQSTATEN_OFFSET)
#define IMX9_USDHC2_IRQSIGEN            (IMX9_USDHC2_BASE + IMX9_USDHC_IRQSIGEN_OFFSET)
#define IMX9_USDHC2_AC12ERR             (IMX9_USDHC2_BASE + IMX9_USDHC_AC12ERR_OFFSET)
#define IMX9_USDHC2_HTCAPBLT            (IMX9_USDHC2_BASE + IMX9_USDHC_HTCAPBLT_OFFSET)
#define IMX9_USDHC2_WML                 (IMX9_USDHC2_BASE + IMX9_USDHC_WML_OFFSET)
#define IMX9_USDHC2_MIX                 (IMX9_USDHC2_BASE + IMX9_USDHC_MIX_OFFSET)
#define IMX9_USDHC2_FEVT                (IMX9_USDHC2_BASE + IMX9_USDHC_FEVT_OFFSET)
#define IMX9_USDHC2_ADMAES              (IMX9_USDHC2_BASE + IMX9_USDHC_ADMAES_OFFSET)
#define IMX9_USDHC2_ADSADDR             (IMX9_USDHC2_BASE + IMX9_USDHC_ADSADDR_OFFSET)
#define IMX9_USDHC2_DLL_CONTROL         (IMX9_USDHC2_BASE + IMX9_USDHC_DLL_CONTROL_OFFSET)
#define IMX9_USDHC2_DLL_STATUS          (IMX9_USDHC2_BASE + IMX9_USDHC_DLL_STATUS)
#define IMX9_USDHC2_CLK_TUNE_CTRL       (IMX9_USDHC2_BASE + IMX9_USDHC_CLK_TUNE_CTRL)
#define IMX9_USDHC2_VENDOR              (IMX9_USDHC2_BASE + IMX9_USDHC_VENDOR_OFFSET)
#define IMX9_USDHC2_MMCBOOT             (IMX9_USDHC2_BASE + IMX9_USDHC_MMCBOOT_OFFSET)
#define IMX9_USDHC2_VENDOR2             (IMX9_USDHC2_BASE + IMX9_USDHC_VENDOR2_OFFSET)
#define IMX9_USDHC2_TC                  (IMX9_USDHC2_BASE + IMX9_USDHC_TC_OFFSET)
#define IMX9_USDHC2_CQVER               (IMX9_USDHC2_BASE + IMX9_USDHC_CQVER_OFFSET)
#define IMX9_USDHC2_CQCAP               (IMX9_USDHC2_BASE + IMX9_USDHC_CQCAP_OFFSET)
#define IMX9_USDHC2_CQCFG               (IMX9_USDHC2_BASE + IMX9_USDHC_CQCFG_OFFSET)
#define IMX9_USDHC2_CQCTL               (IMX9_USDHC2_BASE + IMX9_USDHC_CQCTL_OFFSET)
#define IMX9_USDHC2_CQIS                (IMX9_USDHC2_BASE + IMX9_USDHC_CQIS_OFFSET)
#define IMX9_USDHC2_CQISTE              (IMX9_USDHC2_BASE + IMX9_USDHC_CQISTE_OFFSET)
#define IMX9_USDHC2_CQISGE              (IMX9_USDHC2_BASE + IMX9_USDHC_CQISGE_OFFSET)
#define IMX9_USDHC2_CQIC                (IMX9_USDHC2_BASE + IMX9_USDHC_CQIC_OFFSET)
#define IMX9_USDHC2_CQTDLBA             (IMX9_USDHC2_BASE + IMX9_USDHC_CQTDLBA_OFFSET)
#define IMX9_USDHC2_CQTDLBAU            (IMX9_USDHC2_BASE + IMX9_USDHC_CQTDLBAU_OFFSET)
#define IMX9_USDHC2_CQTDBR              (IMX9_USDHC2_BASE + IMX9_USDHC_CQTDBR_OFFSET)
#define IMX9_USDHC2_CQTCN               (IMX9_USDHC2_BASE + IMX9_USDHC_CQTCN_OFFSET)
#define IMX9_USDHC2_CQDQS               (IMX9_USDHC2_BASE + IMX9_USDHC_CQDQS_OFFSET)
#define IMX9_USDHC2_CQDPT               (IMX9_USDHC2_BASE + IMX9_USDHC_CQDPT_OFFSET)
#define IMX9_USDHC2_CQTCL               (IMX9_USDHC2_BASE + IMX9_USDHC_CQTCLR_OFFSET)
#define IMX9_USDHC2_CQSSC1              (IMX9_USDHC2_BASE + IMX9_USDHC_CQSSC1_OFFSET)
#define IMX9_USDHC2_CQSSC2              (IMX9_USDHC2_BASE + IMX9_USDHC_CQSSC2_OFFSET)
#define IMX9_USDHC2_CQCRDCT             (IMX9_USDHC2_BASE + IMX9_USDHC_CQCRDCT_OFFSET)
#define IMX9_USDHC2_CQRMEM              (IMX9_USDHC2_BASE + IMX9_USDHC_CQRMEM_OFFSET)
#define IMX9_USDHC2_CQTERRI             (IMX9_USDHC2_BASE + IMX9_USDHC_CQTERRI_OFFSET)
#define IMX9_USDHC2_CQCRI               (IMX9_USDHC2_BASE + IMX9_USDHC_CQCRI_OFFSET)
#define IMX9_USDHC2_CQCRA               (IMX9_USDHC2_BASE + IMX9_USDHC_CQCRA_OFFSET)

/* Register Bit Definitions *************************************************/

/* DMA System Address Register */

#define USDHC_DSADDR_SHIFT               (0)           /* Bits 2-31: DMA System Address */
#define USDHC_DSADDR_MASK                (0xfffffffc)  /* Bits 0-1: 32 bit aligned, low bits Reserved */

/* Block Attributes Register */

#define USDHC_BLKATTR_SIZE_SHIFT         (0)       /* Bits 0-12: Transfer Block Size */
#define USDHC_BLKATTR_SIZE_MASK          (0x1fff << USDHC_BLKATTR_SIZE_SHIFT)
#  define USDHC_BLKATTR_SIZE(n)          ((n) << USDHC_BLKATTR_SIZE_SHIFT)
                                                   /* Bits 13-15: Reserved */
#define USDHC_BLKATTR_CNT_SHIFT          (16)      /* Bits 16-31: Blocks Count For Current Transfer */
#define USDHC_BLKATTR_CNT_MASK           (0xffff << USDHC_BLKATTR_CNT_SHIFT)
#  define USDHC_BLKATTR_CNT(n)           ((n) << USDHC_BLKATTR_CNT_SHIFT)

/* Command Argument Register (32-bit cmd/arg data) */

/* Transfer Type Register */

#define USDHC_XFERTYP_DMAEN              (1 << 0)  /* Bit 0: Enable DMA functionality */
#define USDHC_XFERTYP_BCEN               (1 << 1)  /* Bit 1: Enable the Block Count register */
#define USDHC_XFERTYP_AC12EN             (1 << 2)  /* Bit 2: Enable automatic CMD12 */
#define USDHC_XFERTYP_DDREN              (1 << 3)  /* Bit 3: Dual data rate mode selection */
#define USDHC_XFERTYP_DTDSEL             (1 << 4)  /* Bit 4: The direction of DATA line data transfers */
#define USDHC_XFERTYP_MSBSEL             (1 << 5)  /* Bit 5: Enable multiple block DATA line data transfers */
#define USDHC_XFERTYP_NIBBLE_POS         (1 << 6)  /* Bit 6: The nibble position */
#define USDHC_XFERTYP_AC23EN             (1 << 7)  /* Bit 7: Enable automatic CMD23 */
                                                   /* Bits 8-15: Reserved */
#define USDHC_XFERTYP_RSPTYP_SHIFT       (16)      /* Bits 16-17: Response Type Select */
#define USDHC_XFERTYP_RSPTYP_MASK        (3 << USDHC_XFERTYP_RSPTYP_SHIFT)
#  define USDHC_XFERTYP_RSPTYP_NONE      (0 << USDHC_XFERTYP_RSPTYP_SHIFT) /* No response */
#  define USDHC_XFERTYP_RSPTYP_LEN136    (1 << USDHC_XFERTYP_RSPTYP_SHIFT) /* Response length 136 */
#  define USDHC_XFERTYP_RSPTYP_LEN48     (2 << USDHC_XFERTYP_RSPTYP_SHIFT) /* Response length 48 */
#  define USDHC_XFERTYP_RSPTYP_LEN48BSY  (3 << USDHC_XFERTYP_RSPTYP_SHIFT) /* Response length 48, check busy */

                                                   /* Bit 18: Reserved */
#define USDHC_XFERTYP_CCCEN              (1 << 19) /* Bit 19: Command CRC Check Enable */
#define USDHC_XFERTYP_CICEN              (1 << 20) /* Bit 20: Command Index Check Enable */
#define USDHC_XFERTYP_DPSEL              (1 << 21) /* Bit 21: Data Present Select */
#define USDHC_XFERTYP_CMDTYP_SHIFT       (22)      /* Bits 22-23: Command Type */
#define USDHC_XFERTYP_CMDTYP_MASK        (3 << USDHC_XFERTYP_CMDTYP_SHIFT)
#  define USDHC_XFERTYP_CMDTYP_NORMAL    (0 << USDHC_XFERTYP_CMDTYP_SHIFT) /* Normal other commands */
#  define USDHC_XFERTYP_CMDTYP_SUSPEND   (1 << USDHC_XFERTYP_CMDTYP_SHIFT) /* Suspend CMD52 for writing bus suspend in CCCR */
#  define USDHC_XFERTYP_CMDTYP_RESUME    (2 << USDHC_XFERTYP_CMDTYP_SHIFT) /* Resume CMD52 for writing function select in CCCR */
#  define USDHC_XFERTYP_CMDTYP_ABORT     (3 << USDHC_XFERTYP_CMDTYP_SHIFT) /* Abort CMD12, CMD52 for writing I/O abort in CCCR */

#define USDHC_XFERTYP_CMDINX_SHIFT       (24)      /* Bits 24-29: Command Index */
#define USDHC_XFERTYP_CMDINX_MASK        (0x3f << USDHC_XFERTYP_CMDINX_SHIFT)
                                                   /* Bits 30-31: Reserved */

/* Command Response 0-3 (32-bit response data) */

/* Buffer Data Port Register (32-bit data content) */

/* Present State Register */

#define USDHC_PRSSTAT_CIHB               (1 << 0)  /* Bit 0:  Command Inhibit (CMD) */
#define USDHC_PRSSTAT_CDIHB              (1 << 1)  /* Bit 1:  Command Inhibit (DAT) */
#define USDHC_PRSSTAT_DLA                (1 << 2)  /* Bit 2:  Data Line Active */
#define USDHC_PRSSTAT_SDSTB              (1 << 3)  /* Bit 3:  SD Clock Stable */
                                                   /* Bits 4-7: Reserved */
#define USDHC_PRSSTAT_WTA                (1 << 8)  /* Bit 8:  Write Transfer Active */
#define USDHC_PRSSTAT_RTA                (1 << 9)  /* Bit 9:  Read Transfer Active */
#define USDHC_PRSSTAT_BWEN               (1 << 10) /* Bit 10: Buffer Write Enable */
#define USDHC_PRSSTAT_BREN               (1 << 11) /* Bit 11: Buffer Read Enable */
#define USDHC_PRSSTAT_RTR                (1 << 12) /* Bit 12: Retuning request */
                                                   /* Bits 13-14: Reserved */
#define USDHC_PRSSTAT_TSCD               (1 << 15) /* Bit 15: Tape Select Change Done */
#define USDHC_PRSSTAT_CINS               (1 << 16) /* Bit 16: Card Inserted */
                                                   /* Bit 17: Reserved */
#define USDHC_PRSSTAT_CDPL               (1 << 18) /* Bit 18: Card Detect Pin Level */
#define USDHC_PRSSTAT_WPSPL              (1 << 19) /* Bit 19: Write Protect Switch Pin Level */
                                                   /* Bits 20-22: Reserved */
#define USDHC_PRSSTAT_CLSL               (1 << 23) /* Bit 23: CMD Line Signal Level */
#define USDHC_PRSSTAT_DLSL_SHIFT         (24)      /* Bits 24-31: DAT Line Signal Level */
#define USDHC_PRSSTAT_DLSL_MASK          (0xff << USDHC_PRSSTAT_DLSL_SHIFT)
#  define USDHC_PRSSTAT_DLSL_DAT0        (0x01 << USDHC_PRSSTAT_DLSL_SHIFT)
#  define USDHC_PRSSTAT_DLSL_DAT1        (0x02 << USDHC_PRSSTAT_DLSL_SHIFT)
#  define USDHC_PRSSTAT_DLSL_DAT2        (0x04 << USDHC_PRSSTAT_DLSL_SHIFT)
#  define USDHC_PRSSTAT_DLSL_DAT3        (0x08 << USDHC_PRSSTAT_DLSL_SHIFT)
#  define USDHC_PRSSTAT_DLSL_DAT4        (0x10 << USDHC_PRSSTAT_DLSL_SHIFT)
#  define USDHC_PRSSTAT_DLSL_DAT5        (0x20 << USDHC_PRSSTAT_DLSL_SHIFT)
#  define USDHC_PRSSTAT_DLSL_DAT6        (0x40 << USDHC_PRSSTAT_DLSL_SHIFT)
#  define USDHC_PRSSTAT_DLSL_DAT7        (0x80 << USDHC_PRSSTAT_DLSL_SHIFT)

/* Protocol Control Register */

                                                   /* Bit 0: Reserved */
#define USDHC_PROCTL_DTW_SHIFT           (1)       /* Bits 1-2: Data Transfer Width */
#define USDHC_PROCTL_DTW_MASK            (3 << USDHC_PROCTL_DTW_SHIFT)
#  define USDHC_PROCTL_DTW_1BIT          (0 << USDHC_PROCTL_DTW_SHIFT)  /* 1-bit mode */
#  define USDHC_PROCTL_DTW_4BIT          (1 << USDHC_PROCTL_DTW_SHIFT)  /* 4-bit mode */
#  define USDHC_PROCTL_DTW_8BIT          (2 << USDHC_PROCTL_DTW_SHIFT)  /* 8-bit mode */

#define USDHC_PROCTL_D3CD                (1 << 3)  /* Bit 3: DAT3 as Card Detection Pin */
#define USDHC_PROCTL_EMODE_SHIFT         (4)       /* Bits 4-5: Endian mode */
#define USDHC_PROCTL_EMODE_MASK          (3 << USDHC_PROCTL_EMODE_SHIFT)
#  define USDHC_PROCTL_EMODE_BE          (0 << USDHC_PROCTL_EMODE_SHIFT)/* Big endian mode */
#  define USDHC_PROCTL_EMODE_HWBE        (1 << USDHC_PROCTL_EMODE_SHIFT)/* Half word big endian mode */
#  define USDHC_PROCTL_EMODE_LE          (2 << USDHC_PROCTL_EMODE_SHIFT)/* Little endian mode */
                                                                        /* Bits 6-7: Reserved */
#define USDHC_PROCTL_DMAS_SHIFT          (8)                            /* Bits 8-9: DMA Select */
#define USDHC_PROCTL_DMAS_MASK           (3 << USDHC_PROCTL_DMAS_SHIFT)
#  define USDHC_PROCTL_DMAS_NODMA        (0 << USDHC_PROCTL_DMAS_SHIFT) /* No DMA or simple DMA is selected */
#  define USDHC_PROCTL_DMAS_ADMA1        (1 << USDHC_PROCTL_DMAS_SHIFT) /* ADMA1 is selected */
#  define USDHC_PROCTL_DMAS_ADMA2        (2 << USDHC_PROCTL_DMAS_SHIFT) /* ADMA2 is selected */

                                                   /* Bits 10-25: Reserved */
#define USDHC_PROCTL_SABGREQ             (1 << 16) /* Bit 16: Stop At Block Gap Request */
#define USDHC_PROCTL_CREQ                (1 << 17) /* Bit 17: Continue Request */
#define USDHC_PROCTL_RWCTL               (1 << 18) /* Bit 18: Read Wait Control */
#define USDHC_PROCTL_IABG                (1 << 19) /* Bit 19: Interrupt At Block Gap */
#define USDHC_PROCTL_RDDONENO8CLK        (1 << 20) /* Bit 20: Read done to 8 clock */
#define USDHC_PROCTL_RESV2023            (4 << 21) /* Bits 21-23: Reserved, write as 0x100 */
#define USDHC_PROCTL_WECINT              (1 << 24) /* Bit 24: Wakeup Event Enable On Card Interrupt */
#define USDHC_PROCTL_WECINS              (1 << 25) /* Bit 25: Wakeup Event Enable On SD Card Insertion */
#define USDHC_PROCTL_WECRM               (1 << 26) /* Bit 26: Wakeup Event Enable On SD Card Removal */
                                                   /* Bits 27-29: Reserved */
#define USDHC_PROTCTL_NEBLKRD            (1 << 30) /* Bit 30: Non-exect block read */
                                                   /* Bit 31: Reserved */

/* System Control Register */

#define USDHC_SYSCTL_RES0                (0x0F << 0)  /* Bit 0-3:  Reserved, set to 1 */
#define USDHC_SYSCTL_DVS_SHIFT           (4)          /* Bits 4-7: Divisor */
#define USDHC_SYSCTL_DVS_MASK            (0x0f << USDHC_SYSCTL_DVS_SHIFT)
#  define USDHC_SYSCTL_DVS_DIV(n)        (((n) - 1) << USDHC_SYSCTL_DVS_SHIFT) /* Divide by n, n=1,2,15,16 */

#define USDHC_SYSCTL_SDCLKFS_SHIFT       (8)       /* Bits 8-15: SDCLK Frequency Select */
#define USDHC_SYSCTL_SDCLKFS_MASK        (0xff << USDHC_SYSCTL_SDCLKFS_SHIFT)
#  define USDHC_SYSCTL_SDCLKFS_BYPASS    (0x00 << USDHC_SYSCTL_SDCLKFS_SHIFT) /* Bypass the prescaler */
#  define USDHC_SYSCTL_SDCLKFS_DIV2      (0x01 << USDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 2 */
#  define USDHC_SYSCTL_SDCLKFS_DIV4      (0x02 << USDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 4 */
#  define USDHC_SYSCTL_SDCLKFS_DIV8      (0x04 << USDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 8 */
#  define USDHC_SYSCTL_SDCLKFS_DIV16     (0x08 << USDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 16 */
#  define USDHC_SYSCTL_SDCLKFS_DIV32     (0x10 << USDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 32 */
#  define USDHC_SYSCTL_SDCLKFS_DIV64     (0x20 << USDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 64 */
#  define USDHC_SYSCTL_SDCLKFS_DIV128    (0x40 << USDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 128 */
#  define USDHC_SYSCTL_SDCLKFS_DIV256    (0x80 << USDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 256 */

#define USDHC_SYSCTL_DTOCV_SHIFT         (16)      /* Bits 16-19: Data Timeout Counter Value */
#define USDHC_SYSCTL_DTOCV_MASK          (0xf << USDHC_SYSCTL_DTOCV_SHIFT)
#  define USDHC_SYSCTL_DTOCV_PWR32       (0x0 << USDHC_SYSCTL_DTOCV_SHIFT) /* SDCLK x 2^32 */
#  define USDHC_SYSCTL_DTOCV_PWR33       (0x1 << USDHC_SYSCTL_DTOCV_SHIFT) /* SDCLK x 2^33 */
#  define USDHC_SYSCTL_DTOCV_PWR18       (0x2 << USDHC_SYSCTL_DTOCV_SHIFT) /* SDCLK x 2^18 */
#  define USDHC_SYSCTL_DTOCV_PWR19       (0x3 << USDHC_SYSCTL_DTOCV_SHIFT) /* SDCLK x 2^19 */
#  define USDHC_SYSCTL_DTOCV_PWR29       (0xd << USDHC_SYSCTL_DTOCV_SHIFT) /* SDCLK x 2^29 */
#  define USDHC_SYSCTL_DTOCV_PWR30       (0xe << USDHC_SYSCTL_DTOCV_SHIFT) /* SDCLK x 2^30 */
#  define USDHC_SYSCTL_DTOCV_PWR31       (0xf << USDHC_SYSCTL_DTOCV_SHIFT) /* SDCLK x 2^31 */

                                                   /* Bits 20-21: Reserved */
#define USDHC_SYSCTL_RSTF                (1 << 22) /* Bit 24: Reset the async FIFO */
#define USDHC_SYSCTL_IPPRSTN             (1 << 23) /* Bit 23: Card /reset (default 1) */
#define USDHC_SYSCTL_RSTA                (1 << 24) /* Bit 24: Software Reset For ALL */
#define USDHC_SYSCTL_RSTC                (1 << 25) /* Bit 25: Software Reset For CMD Line */
#define USDHC_SYSCTL_RSTD                (1 << 26) /* Bit 26: Software Reset For DAT Line */
#define USDHC_SYSCTL_INITA               (1 << 27) /* Bit 27: Initialization Active */
#define USDHC_SYSCTL_RSTT                (1 << 28) /* Bit 28: Reset tuning */
                                                   /* Bits 29-31: Reserved */

/* Interrupt Status Register, Interrupt Status Enable Register and
 * Interrupt Signal Enable Register
 * Common interrupt bit definitions
 */

#define USDHC_INT_CC                     (1 << 0)     /* Bit 0:  Command Complete */
#define USDHC_INT_TC                     (1 << 1)     /* Bit 1:  Transfer Complete */
#define USDHC_INT_BGE                    (1 << 2)     /* Bit 2:  Block Gap Event */
#define USDHC_INT_DINT                   (1 << 3)     /* Bit 3:  DMA Interrupt */
#define USDHC_INT_BWR                    (1 << 4)     /* Bit 4:  Buffer Write Ready */
#define USDHC_INT_BRR                    (1 << 5)     /* Bit 5:  Buffer Read Ready */
#define USDHC_INT_CINS                   (1 << 6)     /* Bit 6:  Card Insertion */
#define USDHC_INT_CRM                    (1 << 7)     /* Bit 7:  Card Removal */
#define USDHC_INT_CINT                   (1 << 8)     /* Bit 8:  Card Interrupt */
                                                      /* Bits 9-11: Reserved */
#define USDHC_INT_RTR                    (1 << 12)    /* Bit 12: Re-tuning event */
#define USDHC_INT_TP                     (1 << 13)    /* Bit 13: Tuning pass */
#define USDHC_INT_CQI                    (1 << 14)    /* Bit 14: Command queuing interrupt */
#define USDHC_INT_STATUS                 (1 << 15)    /* Bit 15: Error Interrupt Status / Reserved */
#define USDHC_INT_CTOE                   (1 << 16)    /* Bit 16: Command Timeout Error */
#define USDHC_INT_CCE                    (1 << 17)    /* Bit 17: Command CRC Error */
#define USDHC_INT_CEBE                   (1 << 18)    /* Bit 18: Command End Bit Error */
#define USDHC_INT_CIE                    (1 << 19)    /* Bit 19: Command Index Error */
#define USDHC_INT_DTOE                   (1 << 20)    /* Bit 20: Data Timeout Error */
#define USDHC_INT_DCE                    (1 << 21)    /* Bit 21: Data CRC Error */
#define USDHC_INT_DEBE                   (1 << 22)    /* Bit 22: Data End Bit Error */
                                                      /* Bit 23: Reserved */
#define USDHC_INT_AC12E                  (1 << 24)    /* Bit 24: Auto CMD12 Error */
                                                      /* Bit 25: Reserved */
#define USDHC_INT_TNE                    (1 << 26)    /* Bit 26: Tuning error */
                                                      /* Bit 27: Reserved */
#define USDHC_INT_DMAE                   (1 << 28)    /* Bit 28: DMA Error */
                                                      /* Bits 29-31: Reserved */
#define USDHC_INT_ALL                    0x117f01ff

/* Auto CMD12 Error Status Register */

#define USDHC_AC12ERR_NE                 (1 << 0)     /* Bit 0:  Auto CMD12 Not Executed */
#define USDHC_AC12ERR_TOE                (1 << 1)     /* Bit 1:  Auto CMD12/23 Timeout Error */
#define USDHC_AC12ERR_CE                 (1 << 2)     /* Bit 2:  Auto CMD12/23 CRC Error */
#define USDHC_AC12ERR_EBE                (1 << 3)     /* Bit 3:  Auto CMD12/23 End Bit Error */
#define USDHC_AC12ERR_IE                 (1 << 4)     /* Bit 4:  Auto CMD12/23 Index Error */
                                                      /* Bits 5-6: Reserved */
#define USDHC_AC12ERR_CNI                (1 << 7)     /* Bit 7: Command Not Issued By Auto CMD12 Error */
                                                      /* Bits 8-21: Reserved */
#define USDHC_AC12ERR_EXECUTE_TUNING     (1 << 22)    /* Bit 22: Execute Tuning */
#define USDHC_AC12ERR_SMP_CLK_SEL        (1 << 23)    /* Bit 23: Sample clock sel */
                                                      /* Bits 24-31: Reserved */

/* Host Controller Capabilities */

#define USDHC_HTCAPBLT_SDR50             (1 << 0)     /* Bit 0: SDR50 support indication */
#define USDHC_HTCAPBLT_SDR104            (1 << 1)     /* Bit 1: SDR104 support indication */
#define USDHC_HTCAPBLT_DDR50             (1 << 2)     /* Bit 2: DDR50 support indication */
                                                      /* Bits 3-7: Reserved */
                                                      /* Bits 8-12: Reserved */
#define USDHC_HTCAPBLT_USE_TUNING_SDR50  (1 << 13)    /* Bit 13: Use tuning for SDR50 */
#define USDHC_HTCAPBLT_MBL_SHIFT         (16)         /* Bits 16-18: Max Block Length */
#define USDHC_HTCAPBLT_MBL_MASK          (7 << USDHC_HTCAPBLT_MBL_SHIFT)
#  define USDHC_HTCAPBLT_MBL_512BYTES    (0 << USDHC_HTCAPBLT_MBL_SHIFT)
#  define USDHC_HTCAPBLT_MBL_1KB         (1 << USDHC_HTCAPBLT_MBL_SHIFT)
#  define USDHC_HTCAPBLT_MBL_2KB         (2 << USDHC_HTCAPBLT_MBL_SHIFT)
#  define USDHC_HTCAPBLT_MBL_4KB         (3 << USDHC_HTCAPBLT_MBL_SHIFT)
                                                      /* Bit 19: Reserved */
#define USDHC_HTCAPBLT_ADMAS             (1 << 20)    /* Bit 20: ADMA Support */
#define USDHC_HTCAPBLT_HSS               (1 << 21)    /* Bit 21: High Speed Support */
#define USDHC_HTCAPBLT_DMAS              (1 << 22)    /* Bit 22: DMA Support */
#define USDHC_HTCAPBLT_SRS               (1 << 23)    /* Bit 23: Suspend/Resume Support */
#define USDHC_HTCAPBLT_VS33              (1 << 24)    /* Bit 24: Voltage Support 3.3 V */
#define USDHC_HTCAPBLT_VS30              (1 << 25)    /* Bit 25: Voltage Support 3.0 V */
                                                      /* Bits 26-31: Reserved */

/* Watermark Level Register */

#define USDHC_WML_RD_SHIFT               (0)          /* Bits 0-7: Read Watermark Level */
#define USDHC_WML_RD_MASK                (0xff << USDHC_WML_RDWML_SHIFT)
#  define USDHC_WML_RD(n)                ((n) << SDHC_WML_RDWML_SHIFT)
                                                      /* Bits 8-15: Reserved */
#define USDHC_WML_WR_SHIFT               (16)         /* Bits 16-23: Write Watermark Level */
#define USDHC_WML_WR_MASK                (0xff << USDHC_WML_WRWML_SHIFT)
#  define USDHC_WML_WR(n)                ((n) << SDHC_WML_WRWML_SHIFT)
                                                      /* Bits 24-31: Reserved */

/* Mixer Control Register */

#define USDHC_MC_DEFAULTVAL              (0x80000000) /* Bit 31 is always set */
#define USDHC_MC_DMAEN                   (1 << 0)     /* Bit 0: DMA Enable */
#define USDHC_MC_BCEN                    (1 << 1)     /* Bit 1: Block Count Enable */
#define USDHC_MC_AC12EN                  (1 << 2)     /* Bit 2: Auto CMD12 Enable */
#define USDHC_MC_DDR_EN                  (1 << 3)     /* Bit 3: DDR mode enable */
#define USDHC_MC_DTDSEL                  (1 << 4)     /* Bit 4: Data Transfer direction select */
#define USDHC_MC_MSBSEL                  (1 << 5)     /* Bit 5: Multi/single block select */
#define USDHC_MC_NIBBLE_POS              (1 << 6)     /* Bit 6: Nibble position for DDR 4 bit */
#define USDHC_MC_AC23EN                  (1 << 7)     /* Bit 7: Auto CMD23 Enable */
                                                      /* Bits 8-21: Reserved */
#define USDHC_MC_EXE_TUNE                (1 << 22)    /* Bit 22: Execute Tuning */
#define USDHC_MC_SMP_CLK_SEL             (1 << 23)    /* Bit 23: SMP Clock Sel */
#define USDHC_MC_AUTO_TUNE_EN            (1 << 24)    /* Bit 24: Auto tune enable */
#define USDHC_MC_FBCLK_SEL               (1 << 25)    /* Bit 25: Feedback clock source selection */
#define USDHC_MC_HS400EN                 (1 << 26)    /* Bit 26: Enable HS400 mode */
#define USDHC_MC_EHS400EN                (1 << 27)    /* Bit 27: Enable enhance HS400 mode */
                                                      /* Bits 28-31: reserved */

/* Force Event Register */

#define USDHC_FEVT_AC12NE                (1 << 0)     /* Bit 0:  Force Event Auto Command 12 Not Executed */
#define USDHC_FEVT_AC12TOE               (1 << 1)     /* Bit 1:  Force Event Auto Command 12 Time Out Error */
#define USDHC_FEVT_AC12CE                (1 << 2)     /* Bit 2:  Force Event Auto Command 12 CRC Error */
#define USDHC_FEVT_AC12EBE               (1 << 3)     /* Bit 3:  Force Event Auto Command 12 End Bit Error */
#define USDHC_FEVT_AC12IE                (1 << 4)     /* Bit 4:  Force Event Auto Command 12 Index Error */
                                                      /* Bits 5-6: Reserved */
#define USDHC_FEVT_CNIBAC12E             (1 << 7)     /* Bit 7:  Force Event Command Not Executed By Auto Command 12 Error */
                                                      /* Bits 8-15: Reserved */
#define USDHC_FEVT_CTOE                  (1 << 16)    /* Bit 16: Force Event Command Time Out Error */
#define USDHC_FEVT_CCE                   (1 << 17)    /* Bit 17: Force Event Command CRC Error */
#define USDHC_FEVT_CEBE                  (1 << 18)    /* Bit 18: Force Event Command End Bit Error */
#define USDHC_FEVT_CIE                   (1 << 19)    /* Bit 19: Force Event Command Index Error */
#define USDHC_FEVT_DTOE                  (1 << 20)    /* Bit 20: Force Event Data Time Out Error */
#define USDHC_FEVT_DCE                   (1 << 21)    /* Bit 21: Force Event Data CRC Error */
#define USDHC_FEVT_DEBE                  (1 << 22)    /* Bit 22: Force Event Data End Bit Error */
                                                      /* Bit 23: Reserved */
#define USDHC_FEVT_AC12E                 (1 << 24)    /* Bit 24: Force Event Auto Command 12 Error */
                                                      /* Bit 25: Reserved */
#define USDHC_FEVT_TTNE                  (1 << 26)    /* Bit 26: Force tuning error */
                                                      /* Bit 27: reserved */
#define USDHC_FEVT_DMAE                  (1 << 28)    /* Bit 28: Force Event DMA Error */
                                                      /* Bits 29-30: Reserved */
#define USDHC_FEVT_CINT                  (1 << 31)    /* Bit 31: Force Event Card Interrupt */

/* ADMA Error Status Register */

#define USDHC_ADMAES_SHIFT               (0)          /* Bits 0-1: ADMA Error State (when ADMA Error is occurred) */
#define USDHC_ADMAES_MASK                (3 << USDHC_ADMAES_ADMAES_SHIFT)
#  define USDHC_ADMAES_STOP              (0 << USDHC_ADMAES_ADMAES_SHIFT) /* Stop DMA */
#  define USDHC_ADMAES_FDS               (1 << USDHC_ADMAES_ADMAES_SHIFT) /* Fetch descriptor */
#  define USDHC_ADMAES_CADR              (2 << USDHC_ADMAES_ADMAES_SHIFT) /* Change address */
#  define USDHC_ADMAES_TFR               (3 << USDHC_ADMAES_ADMAES_SHIFT) /* Transfer data */

#define USDHC_ADMAES_LME                 (1 << 2)     /* Bit 2:  ADMA Length Mismatch Error */
#define USDHC_ADMAES_DCE                 (1 << 3)     /* Bit 3:  ADMA Descriptor Error */
                                                      /* Bits 4-31: Reserved */

/* ADMA System Address Register */

#define USDHC_ADSADDR_SHIFT              (0)          /* Bits 1-31: ADMA System Address */
#define USDHC_ADSADDR_MASK               (0xfffffffc) /* Bits 0-1: Reserved */

/* Delay Line Control */

#define USDHC_DL_CTRL_EN                 (1 << 0)     /* Bit 0: Delay Line enable */
#define USDHC_DL_CTRL_RST                (1 << 1)     /* Bit 1: Delay line reset */
#define USDHC_DL_CTRL_SLV_FORCE_UP       (1 << 2)     /* Bit 2: SLV Force update */
#define USDHC_DL_SLV_DLY_TGT0_SHIFT      (3)          /* Bits 3-6: Delay Target 0 */
#define USDHC_DL_SLV_DLY_TGT0_MASK       (0xf << USDHC_DL_SLV_DLY_TGT0_SHIFT)
#  define USDHC_DL_SLV_DLY_TGT0(n)       ((n) << USDHC_DL_SLV_DLY_TGT0_SHIFT)
#define USDHC_DL_CTRL_SLV_UPD            (1 << 7)     /* Bit 7: Delay Control Gate update */
#define USDHC_DL_CTRL_SLV_OVR            (1 << 8)     /* Bit 8: Delay Control Gate override */
#define USDHC_DL_CTRL_OVR_VAL_SHIFT      (9)          /* Bits 9-15: Override Value */
#define USDHC_DL_CTRL_OVR_VAL_MASK       (0x7f << USDHC_DL_CTRL_OVR_VAL_SHIFT)
#  define USDHC_DL_CTRL_OVR_VAL(n)       ((n) << USDHC_DL_CTRL_OVR_VAL_SHIFT)
#define USDHC_DL_SLV_DLY_TGT1_SHIFT      (16)         /* Bits 16-18: Delay Target 1 */
#define USDHC_DL_SLV_DLY_TGT1_MASK       (0x7 << USDHC_DL_SLV_DLY_TGT1_SHIFT)
#  define USDHC_DL_SLV_DLY_TGT1(n)       ((n) << USDHC_DL_SLV_DLY_TGT1_SHIFT)
                                                      /* Bit 19: Reserved */
#define USDHC_DL_CTRL_SLV_UPDINT_SHIFT   (20)         /* Bits 20-27: DLL Control SLV Update Interval */
#define USDHC_DL_CTRL_SLV_UPDINT_MASK    (0xff << USDHC_DL_CTRL_SLV_UPDINT_SHIFT)
#  define USDHC_DL_CTRL_SLV_UPDINT(n)    ((n) << USDHC_DL_CTRL_SLV_UPDINT_SHIFT)
#define USDHC_DL_CTRL_REF_UPDINT_SHIFT   (28)         /* Bits 28-31: DLL Control Reference Update Interval */
#define USDHC_DL_CTRL_REF_UPDINT_MASK    (0xf << USDHC_DL_CTRL_REF_UPDINT_SHIFT)
#  define USDHC_DL_CTRL_REF_UPDINT(n)    ((n)<< USDHC_DL_CTRL_REF_UPDINT_SHIFT)

/* Delay Line Status */

#define USDHC_DL_STAT_SLV_LOCK           (1 << 0)     /* Bit 0: Slave delay-line lock status */
#define USDHC_DL_STAT_REF_LOCK           (1 << 1)     /* Bit 1: Reference delay-line lock status */
#define USDHC_DL_STAT_SLV_SEL_SHIFT      (2)          /* Bits 2-8: Slave delay line select status */
#define USDHC_DL_STAT_SLV_SEL_MASK       (0x7f << USDHC_DL_STAT_SLV_SEL_SHIFT)
#  define USDHC_DL_STAT_SLV_SEL(n)       ((n) << USDHC_DL_STAT_SLV_SEL_SHIFT)
#define USDHC_DL_STAT_REF_SEL_SHIFT      (9)          /* Bits 9-15: Reference delay line select taps */
#define USDHC_DL_STAT_REF_SEL_MASK       (0x7f << USDHC_DL_STAT_REF_SEL_SHIFT)
#  define USDHC_DL_STAT_REF_SEL(n)       ((n) << USDHC_DL_STAT_REF_SEL_SHIFT)
                                                      /* Bits 16-31: Reserved */

/* Clk tuning control and status */

#define USDHC_CLKTUNE_CS_DCS_POST_SHIFT  (0)          /* Bits 0-3: Set delay cells between CLK_OUT and CLK_POST*/
#define USDHC_CLKTUNE_CS_DCS_POST_MASK   (0xf << USDHC_CLKTUNE_CS_DCS_POST_SHIFT)
#  define USDHC_CLKTUNE_CS_DCS_POST(n)   ((n) << USDHC_CLKTUNE_CS_DCS_POST_SHIFT)
#define USDHC_CLKTUNE_CS_DCS_OUT_SHIFT   (4)          /* Bits 4-7: Set delay cells between CLK_PRE and CLK_OUT */
#define USDHC_CLKTUNE_CS_DCS_OUT_MASK    (0xf << USDHC_CLKTUNE_CS_DCS_OUT_SHIFT)
#  define USDHC_CLKTUNE_CS_DCS_OUT(n)    ((n) << USDHC_CLKTUNE_CS_DCS_OUT_SHIFT)
#define USDHC_CLKTUNE_CS_DCS_PRE_SHIFT   (8)          /* Bits 8-14: Set celay cells between the feedback clock and CLK_PRE */
#define USDHC_CLKTUNE_CS_DCS_PRE_MASK    (0x7f << USDHC_CLKTUNE_CS_DCS_PRE_SHIFT)
#  define USDHC_CLKTUNE_CS_DCS_PRE(n)    ((n) << USDHC_CLKTUNE_CS_DCS_PRE_SHIFT)
#define USDHC_CLKTUNE_CS_NXT_ERR         (1 << 15)    /* Bit 15: NXT error */
#define USDHC_CLKTUNE_CS_TS_POST_SHIFT   (16)         /* Bits 16-19: Delay cells between CLK_OUT and CLK_POST */
#define USDHC_CLKTUNE_CS_TS_POST_MASK    (0xf << USDHC_CLKTUNE_CS_TS_POST_SHIFT)
#  define USDHC_CLKTUNE_CS_TS_POST(n)    ((n) << USDHC_CLKTUNE_CS_TS_POST_SHIFT)
#define USDHC_CLKTUNE_CS_TS_OUT_SHIFT    (20)         /* Bits 20-23: Delay cells between CLK_PRE and CLK_OUT */
#define USDHC_CLKTUNE_CS_TS_OUT_MASK     (0xf << USDHC_CLKTUNE_CS_TS_OUT_SHIFT)
#  define USDHC_CLKTUNE_CS_TS_OUT(n)     ((n) << USDHC_CLKTUNE_CS_TS_OUT_SHIFT)
#define USDHC_CLKTUNE_CS_TS_PRE_SHIFT    (24)         /* Bits 24-30: Delay cells between the feedback clock and CLK_PRE */
#define USDHC_CLKTUNE_CS_TS_PRE_MASK     (0x7f << USDHC_CLKTUNE_CS_TS_PRE_SHIFT)
#  define USDHC_CLKTUNE_CS_TS_PRE_OUT(n) ((n) << USDHC_CLKTUNE_CS_TS_PRE_SHIFT)
#define USDHC_CLKTUNE_CS_PRE_ERR         (1 << 31)    /* Bit 31: PRE error */

/* Strobe DLL control */

/* Strobe DLL status */

/* Vendor Specific Register */

                                                      /* Bit 0: Reserved */
#define USHDC_VENDOR_VSELECT             (1 << 1)     /* Bit 1: Voltage selection */
                                                      /* Bit 2: Reserved */
#define USDHC_VENDOR_CHKBUSY_ON          (1 << 3)     /* Bit 3: Enable Check busy */
                                                      /* Bit 4-7: Reserved */
#define USDHC_VENDOR_FRC_SDCLK_ON        (1 << 8)     /* Bit 8: Force clock active */
                                                      /* Bits 9-14: Reserved */
#define USDHC_VENDOR_CRC_CHECK_OFF       (1 << 15)    /* Bit 15: Switch off CRC checking */
                                                      /* Bits 16:30 Reserved */
#define USHDC_VENDOR_DEFAULTVAL          (0x20000000) /* Bit 29 is always set */
#define USDHC_VENDOR_CMDBYTEACC_ON       (1 << 31)    /* Bit 31: Enable command byte access */

/* MMC Boot Register */

#define USDHC_MMCBOOT_DTOCVACK_SHIFT     (0)          /* Bits 0-3: Boot ACK time out counter value */
#define USDHC_MMCBOOT_DTOCVACK_MASK      (0xf << USDHC_MMCBOOT_DTOCVACK_SHIFT)
#  define USDHC_MMCBOOT_DTOCVACK_PWR32   (0x0 << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^32 */
#  define USDHC_MMCBOOT_DTOCVACK_PWR33   (0x1 << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^33 */
#  define USDHC_MMCBOOT_DTOCVACK_PWR18   (0x2 << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^18 */
#  define USDHC_MMCBOOT_DTOCVACK_PWR19   (0x3 << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^19 */
#  define USDHC_MMCBOOT_DTOCVACK_PWR20   (0x4 << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^20 */
#  define USDHC_MMCBOOT_DTOCVACK_PWR21   (0x5 << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^21 */
#  define USDHC_MMCBOOT_DTOCVACK_PWR22   (0x6 << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^22 */
#  define USDHC_MMCBOOT_DTOCVACK_PWR23   (0x7 << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^23 */
#  define USDHC_MMCBOOT_DTOCVACK_PWR30   (0xe << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^30 */
#  define USDHC_MMCBOOT_DTOCVACK_PWR31   (0xf << USDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^31 */

#define USDHC_MMCBOOT_BOOTACK            (1 << 4)     /* Bit 4:  Boot ack mode select */
#define USDHC_MMCBOOT_BOOTMODE           (1 << 5)     /* Bit 5:  Boot mode select */
#define USDHC_MMCBOOT_BOOTEN             (1 << 6)     /* Bit 6:  Boot mode enable */
#define USDHC_MMCBOOT_AUTOSABGEN         (1 << 7)     /* Bit 7:  Enable auto stop at block gap function */
#define USDHC_MMCBOOT_DISABLETO          (1 << 8)     /* Bit 8:  Timeout enable */
                                                      /* Bits 9-15: Reserved */
#define USDHC_MMCBOOT_BOOTBLKCNT_SHIFT   (16)         /* Bits 16-31: Stop at block gap value of automatic mode */
#define USDHC_MMCBOOT_BOOTBLKCNT_MASK    (0xffff << USDHC_MMCBOOT_BOOTBLKCNT_SHIFT)
#define USDHC_MMCBOOT_BOOTBLKCNT(n)      ((n) << USDHC_MMCBOOT_BOOTBLKCNT_SHIFT)

/* Vendor specific register 2 */

                                                      /* Bits 0-2: Reserved */
#define USDHC_VS2_CARDINTD3              (1 << 3)     /* Bit 3: Card interrupt detection test */
#define USDHC_VS2_TUNINGBITEN_SHIFT      (4)          /* Bits 4-5: Tuning bit enable */
#define USDHC_VS2_TUNINGBITEN_MASK       (0x3 << USDHC_VS2_TUNINGBITEN_SHIFT)
#  define USDHC_VS2_TUNINGBITEN_30       (0 << USDHC_VS2_TUNINGBITEN_SHIFT) /* Enable Tuning circuit for DATA[3:0] */
#  define USDHC_VS2_TUNINGBITEN_70       (1 << USDHC_VS2_TUNINGBITEN_SHIFT) /* Enable Tuning circuit for DATA[7:0] */
#  define USDHC_VS2_TUNINGBITEN_0        (2 << USDHC_VS2_TUNINGBITEN_SHIFT) /* Enable Tuning circuit for DATA[0] */

#define USDHC_VS2_TUNINGCMDEN            (1 << 6)     /* Bit 6: Tuning CMD enable */
                                                      /* Bits 7-9: Reserved */
#define USDHC_VS2_HS400WRCLKSEN          (1 << 10)    /* Bit 10: HS400 write clock stop enable */
#define USDHC_VS2_HS400RDCLKSEN          (1 << 11)    /* Bit 11: HS400 read clock stop enable */
#define USDHC_VS2_ACMD23ARGU2            (1 << 12)    /* Bit 12: Argument 2 register enable for ACMD23 */
                                                      /* Bit 13: Reserved */
#define USDHC_VS2_BUSRESET               (1 << 14)    /* Bit 14: Bus reset (undocumented!) */
#define USDHC_VS2_32KCLKEN               (1 << 15)    /* Bit 15: Use low power clock for card detection */
#define USDHC_VS2_FBCLK_TAP_SEL_SHIFT    (16)         /* Bits 16-31: Enable extra delay on internal feedback clock */
#define USDHC_VS2_FBCLK_TAP_SEL_MASK     (0xffff << USDHC_VS2_FBCLK_TAP_SEL_SHIFT)
#define USDHC_VS2_FBCLK_TAP_SEL(n)       ((n) << USDHC_VS2_FBCLK_TAP_SEL_SHIFT)

/* Tuning Control Register */

#define USDHC_TC_STARTTAP_SHIFT          (0)          /* Bits 0-6: Start TAP for CMD19 tuning */
#define USDHC_TC_STARTTAP_MASK           (0x7f << USDHC_TC_STARTTAP_SHIFT)
#  define USDHC_TC_STARTTAP(n)           ((n) << USDHC_TC_STARTTAP_SHIFT)
#define USDHC_TC_DIS_CMD                 (1 << 7)     /* Bit 7: Disable command check for standard tuning */
#define USDHC_TC_COUNT_SHIFT             (8)          /* Bits 8-15: Count for CMD19 tuning */
#define USDHC_TC_COUNT_MASK              (0xff << USDHC_TC_COUNT_SHIFT)
#  define USDHC_TC_COUNT(n)              ((n) << USDHC_TC_COUNT_SHIFT)
                                                      /* Bit 19: Reserved */
#define USDHC_TC_WINDOW_SHIFT            (20)         /* Bits 20-22: Tuning window */
#define USDHC_TC_WINDOW_MASK             (0x7 << USDHC_TC_WINDOWS_SHIFT)
#  define USDHC_TC_WINDOW(n)             ((n) << USDHC_TC_WINDOW_SHIFT)
                                                      /* Bit 23: Reserved */
#define USDHC_TC_TUNINGEN                (1 << 24)    /* Bit 24: Tuning enable */
                                                      /* Bits 25-31: Reserved */

/* Command Queuing Version */

/* Command Queuing Capabilities */

/* Command Queuing Configuration */

/* Command Queuing Control */

/* Command Queuing Interrupt Status */

/* Command Queuing Interrupt Status Enable */

/* Command Queuing Interrupt Signal Enable */

/* Command Queuing Interrupt Coalescing */

/* Command Queuing Task Descriptor List Base Address */

/* Command Queuing Task Descriptor List Base Address Upper 32 Bits */

/* Command Queuing Task Doorbell */

/* Command Queuing Task Completion Notification */

/* Command Queuing Device Queue Status */

/* Command Queuing Device Pending Tasks */

/* Command Queuing Task Clear */

/* Command Queuing Send Status Configuration 1 */

/* Command Queuing Send Status Configuration 2 */

/* Command Queuing Command Response for Direct-Command Task */

/* Command Queuing Response Mode Error Mask */

/* Command Queuing Task Error Information */

/* Command Queuing Command Response Index */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_USDHC_H */
