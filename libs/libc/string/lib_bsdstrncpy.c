/****************************************************************************
 * libs/libc/string/lib_bsdstrncpy.c
 *
 * Copyright (c) 1994-2009  Red Hat, Inc. All rights reserved.
 *
 * This copyrighted material is made available to anyone wishing to use,
 * modify, copy, or redistribute it subject to the terms and conditions
 * of the BSD License.   This program is distributed in the hope that
 * it will be useful, but WITHOUT ANY WARRANTY expressed or implied,
 * including the implied warranties of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  A copy of this license is available at
 * http://www.opensource.org/licenses. Any Red Hat trademarks that are
 * incorporated in the source code or documentation are not subject to
 * the BSD License and may only be used or replicated with the express
 * permission of Red Hat, Inc.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LBLOCKSIZE (sizeof(long))

/* Nonzero if either x or y is not aligned on a "long" boundary. */

#define UNALIGNED(x, y) \
  (((long)(uintptr_t)(x) & (sizeof(long) - 1)) | ((long)(uintptr_t)(y) & (sizeof(long) - 1)))

/* Macros for detecting endchar */

#if LONG_MAX == 2147483647
#  define DETECTNULL(x) (((x) - 0x01010101) & ~(x) & 0x80808080)
#elif LONG_MAX == 9223372036854775807
/* Nonzero if x (a long int) contains a NULL byte. */

#  define DETECTNULL(x) (((x) - 0x0101010101010101) & ~(x) & 0x8080808080808080)
#endif

#define TOO_SMALL(len) ((len) < sizeof(long))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strncpy
 *
 * Description:
 *   Copies the string pointed to by 'src' (including the terminating NUL
 *   character) into the array pointed to by 'dest'.  strncpy() will not
 *   copy more than 'n' bytes from 'src' to 'dest' array (including the
 *   NUL terminator).
 *
 *   If the array pointed to by 'src' is a string that is shorter than 'n'
 *   bytes, NUL characters will be appended to the copy in the array
 *   pointed to by 'dest', until 'n' bytes in all are written.
 *
 *   If copying takes place between objects that overlap, the behavior is
 *   undefined.
 *
 * Returned Value:
 *   The strncpy() function returns the pointer to 'dest'
 *
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_STRNCPY) && defined(LIBC_BUILD_STRNCPY)
#undef strncpy /* See mm/README.txt */
nosanitize_address
FAR char *strncpy(FAR char *dest, FAR const char *src, size_t n)
{
  FAR char *dst0 = dest;
  FAR const char *src0 = src;
  FAR long *aligned_dst;
  FAR const long *aligned_src;

  /* If src and dest is aligned and n large enough, then copy words. */

  if (!UNALIGNED(src0, dst0) && !TOO_SMALL(n))
    {
      aligned_dst = (FAR long *)dst0;
      aligned_src = (FAR long *)src0;

      /* src and dest are both "long int" aligned, try to do "long int"
       * sized copies.
       */

      while (n >= LBLOCKSIZE && !DETECTNULL(*aligned_src))
        {
          n -= LBLOCKSIZE;
          *aligned_dst++ = *aligned_src++;
        }

      dst0 = (FAR char *)aligned_dst;
      src0 = (FAR char *)aligned_src;
    }

  while (n > 0)
    {
      --n;
      if ((*dst0++ = *src0++) == '\0')
        {
          break;
        }
    }

  while (n-- > 0)
    {
      *dst0++ = '\0';
    }

  return dest;
}
#endif
