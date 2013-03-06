/****************************************************************************
 * arch/arm/src/armv6-m/up_svcall.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/sched.h>

#ifdef CONFIG_NUTTX_KERNEL
#  include <syscall.h>
#endif

#include "svcall.h"
#include "exc_return.h"
#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Debug ********************************************************************/
/* Debug output from this file may interfere with context switching!  To get
 * debug output you must enabled the following in your NuttX configuration:
 *
 * CONFIG_DEBUG and CONFIG_DEBUG_SYSCALL
 */

#ifdef CONFIG_DEBUG_SYSCALL
# define svcdbg(format, arg...) lldbg(format, ##arg)
#else
# define svcdbg(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dispatch_syscall
 *
 * Description:
 *   Call the stub function corresponding to the system call.
 *
 ****************************************************************************/

#ifdef CONFIG_NUTTX_KERNEL
static void dispatch_syscall(void) naked_function;
static void dispatch_syscall(void)
{
  __asm__ __volatile__
  (
    " push {r4-r6}\n"             /* Save R4, R5 and R6 */
    " mov r6, r14\n"              /* Save LR in R6 */
    " ldr r4, =g_stublookup\n"    /* Get the base of the stub lookup table */
    " lsl r3, r0, #2\n"           /* Get the offset of the stub for this syscall */
    " ldr r3, [r4, r3]\n"         /* Load the entry of the stub for this syscall */
    " blx r3\n"                   /* Call the stub */
    " mov r14, r6\n"              /* Restore R14 */
    " pop {r4-r6}\n"              /* Restore R4, R5, and R6 */
    " mov r2, r0\n"               /* Save the return value in R0 in R2 for now */
    " mov r0, #3\n"               /* R0=SYS_syscall_return */
    " svc 0"                      /* Return from the syscall */
    :::
  );
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

int up_svcall(int irq, FAR void *context)
{
  uint32_t *regs = (uint32_t*)context;

  DEBUGASSERT(regs && regs == current_regs);

  /* The SVCall software interrupt is called with R0 = system call command
   * and R1..R7 =  variable number of arguments depending on the system call.
   */

  svcdbg("SVCALL Entry: regs: %p cmd: %d\n", regs, regs[REG_R0]);
  svcdbg("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
         regs[REG_R0],  regs[REG_R1],  regs[REG_R2],  regs[REG_R3],
         regs[REG_R4],  regs[REG_R5],  regs[REG_R6],  regs[REG_R7]);
  svcdbg("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
         regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
         regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
#ifdef CONFIG_NUTTX_KERNEL
  svcdbg("xPSR: %08x BASEPRI: %08x EXEC_RETURN: %08x\n",
         regs[REG_XPSR], regs[REG_BASEPRI], regs[REG_EXC_RETURN]);
#else
  svcdbg("xPSR: %08x BASEPRI: %08x\n",
         regs[REG_XPSR], regs[REG_BASEPRI]);
#endif

  /* Handle the SVCall according to the command in R0 */

  switch (regs[REG_R0])
    {
      /* R0=SYS_save_context:  This is a save context command:
       *
       *   int up_saveusercontext(uint32_t *saveregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_save_context
       *   R1 = saveregs
       *
       * In this case, we simply need to copy the current regsters to the
       * save register space references in the saved R1 and return.
       */

      case SYS_save_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0);
          memcpy((uint32_t*)regs[REG_R1], regs, XCPTCONTEXT_SIZE);
        }
        break;

      /* R0=SYS_restore_context: This a restore context command:
       *
       *   void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_restore_context
       *   R1 = restoreregs
       *
       * In this case, we simply need to set current_regs to restore register
       * area referenced in the saved R1. context == current_regs is the normal
       * exception return.  By setting current_regs = context[R1], we force
       * the return to the saved context referenced in R1.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0);
          current_regs = (uint32_t*)regs[REG_R1];
        }
        break;

      /* R0=SYS_switch_context: This a switch context command:
       *
       *   void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_switch_context
       *   R1 = saveregs
       *   R2 = restoreregs
       *
       * In this case, we do both: We save the context registers to the save
       * register area reference by the saved contents of R1 and then set
       * current_regs to to the save register area referenced by the saved
       * contents of R2.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_R1] != 0 && regs[REG_R2] != 0);
          memcpy((uint32_t*)regs[REG_R1], regs, XCPTCONTEXT_SIZE);
          current_regs = (uint32_t*)regs[REG_R2];
        }
        break;

      /* R0=SYS_syscall_return: This a switch context command:
       *
       *   void up_sycall_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_syscall_return
       *
       * We need to restore the saved return address and return in
       * unprivileged thread mode.
       */

#ifdef CONFIG_NUTTX_KERNEL
      case SYS_syscall_return:
        {
          struct tcb_s *rtcb = sched_self();

          /* Make sure that we got here from a privileged thread and
           * that there is a saved syscall return address.
           */

          DEBUGASSERT(rtcb->xcp.sysreturn != NULL &&
                      regs[REG_EXC_RETURN] == EXC_RETURN_PRIVTHR);

          /* Setup to return to the saved syscall return address in
           * unprivileged mode.
           */

          current_regs[REG_PC]         = rtcb->xcp.sysreturn;
          current_regs[REG_EXC_RETURN] = EXC_RETURN_UNPRIVTHR;
          rtcb->sysreturn              = NULL;

          /* The return value must be in R0-R1.  dispatch_syscall() temporarily
           * moved the value to R2.
           */

          current_regs[REG_R0]         = current_regs[REG_R2];
        }
        break;
#endif

      /* This is not an architecture-specific system call.  If NuttX is built
       * as a standalone kernel with a system call interface, then all of the
       * additional system calls must be handled as in the default case.
       */

      default:
        {
#ifdef CONFIG_NUTTX_KERNEL
          FAR struct tcb_s *rtcb = sched_self();

          /* Verify the the SYS call number is within range */

          DEBUGASSERT(current_regs[REG_R0] < SYS_maxsyscall);

          /* Make sure that we got here from an unprivileged thread and that
           * there is a no saved syscall return address.
           */

          DEBUGASSERT(rtcb->xcp.sysreturn == NULL &&
                      regs[REG_EXC_RETURN] == EXC_RETURN_UNPRIVTHR);

          /* Setup to return to dispatch_syscall in privileged mode. */

          rtcb->sysreturn              = regs[REG_PC]
          regs[REG_PC]                 = (uint32_t)dispatch_syscall;
          current_regs[REG_EXC_RETURN] = EXC_RETURN_PRIVTHR;

          /* Offset R0 to account for the reserved values */

          current_regs[REG_R0]        -= CONFIG_SYS_RESERVED;
#else
          slldbg("ERROR: Bad SYS call: %d\n", regs[REG_R0]);
#endif
        }
        break;
    }

  /* Report what happened.  That might difficult in the case of a context switch */

  if (regs != current_regs)
    {
      svcdbg("SVCall Return: Context switch!\n");
      svcdbg("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             current_regs[REG_R0],  current_regs[REG_R1],  current_regs[REG_R2],  current_regs[REG_R3],
             current_regs[REG_R4],  current_regs[REG_R5],  current_regs[REG_R6],  current_regs[REG_R7]);
      svcdbg("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             current_regs[REG_R8],  current_regs[REG_R9],  current_regs[REG_R10], current_regs[REG_R11],
             current_regs[REG_R12], current_regs[REG_R13], current_regs[REG_R14], current_regs[REG_R15]);
#ifdef CONFIG_NUTTX_KERNEL
      svcdbg("xPSR: %08x BASEPRI: %08x EXEC_RETURN: %08x\n",
             current_regs[REG_XPSR], current_regs[REG_BASEPRI], current_regs[REG_EXC_RETURN]);
#else
      svcdbg("xPSR: %08x BASEPRI: %08x\n",
             current_regs[REG_XPSR], current_regs[REG_BASEPRI]);
#endif
    }
  else
    {
      svcdbg("SVCall Return: %d\n", regs[REG_R0]);
    }

  return OK;
}
