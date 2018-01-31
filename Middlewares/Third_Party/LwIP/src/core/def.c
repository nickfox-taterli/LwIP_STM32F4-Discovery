/**
 * @file
 * Common functions used throughout the stack.
 *
 * These are reference implementations of the byte swapping functions.
 * Again with the aim of being simple, correct and fully portable.
 * Byte swapping is the second thing you would want to optimize. You will
 * need to port it to your architecture and in your cc.h:
 *
 * \#define lwip_htons(x) your_htons
 * \#define lwip_htonl(x) your_htonl
 *
 * Note lwip_ntohs() and lwip_ntohl() are merely references to the htonx counterparts.
 * 
 * If you \#define them to htons() and htonl(), you should
 * \#define LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS to prevent lwIP from
 * defining htonx/ntohx compatibility macros.

 * @defgroup sys_nonstandard Non-standard functions
 * @ingroup sys_layer
 * lwIP provides default implementations for non-standard functions.
 * These can be mapped to OS functions to reduce code footprint if desired.
 * All defines related to this section must not be placed in lwipopts.h,
 * but in arch/cc.h!
 * These options cannot be \#defined in lwipopts.h since they are not options
 * of lwIP itself, but options of the lwIP port to your system.
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Simon Goldschmidt
 *
 */

#include "lwip/opt.h"
#include "lwip/def.h"

#include <string.h>

#if BYTE_ORDER == LITTLE_ENDIAN

#if !defined(lwip_htons)
/**
 * Convert an u16_t from host- to network byte order.
 *
 * @param n u16_t in host byte order
 * @return n in network byte order
 */
u16_t
lwip_htons(u16_t n)
{
  return (u16_t)PP_HTONS(n);
}
#endif /* lwip_htons */

#if !defined(lwip_htonl)
/**
 * Convert an u32_t from host- to network byte order.
 *
 * @param n u32_t in host byte order
 * @return n in network byte order
 */
u32_t
lwip_htonl(u32_t n)
{
  return (u32_t)PP_HTONL(n);
}
#endif /* lwip_htonl */

#endif /* BYTE_ORDER == LITTLE_ENDIAN */

#ifndef lwip_strnstr
/**
 * @ingroup sys_nonstandard
 * lwIP default implementation for strnstr() non-standard function.
 * This can be \#defined to strnstr() depending on your platform port.
 */
char*
lwip_strnstr(const char* buffer, const char* token, size_t n)
{
  const char* p;
  size_t tokenlen = strlen(token);
  if (tokenlen == 0) { /* 如果比较目标是0长度,肯定不符合. */
    return LWIP_CONST_CAST(char *, buffer);
  }
  for (p = buffer; *p && (p + tokenlen <= buffer + n); p++) {
    if ((*p == *token) && (strncmp(p, token, tokenlen) == 0)) { /* 如果用strncmp比较相等的字符串,那么他们相等.如果不等,就p指针继续移动继续比较,直到比较出p出现在token位置. */
      return LWIP_CONST_CAST(char *, p);
    }
  }
  return NULL;
}
#endif

#ifndef lwip_stricmp
/**
 * @ingroup sys_nonstandard
 * lwIP default implementation for stricmp() non-standard function.
 * This can be \#defined to stricmp() depending on your platform port.
 */
int
lwip_stricmp(const char* str1, const char* str2)
{
  char c1, c2;

  do {
    c1 = *str1++;
    c2 = *str2++;
    if (c1 != c2) {
      char c1_upc = c1 | 0x20; /* 转为小写 */
      if ((c1_upc >= 'a') && (c1_upc <= 'z')) { /* 在字母范围. */
        /* characters are not equal an one is in the alphabet range:
        downcase both chars and check again */
        char c2_upc = c2 | 0x20; /* 转为小写 */
        if (c1_upc != c2_upc) { /* 两字母不等(已经都是小写,所以等于忽略了大小写) */
          /* still not equal */
          /* don't care for < or > */
          return 1;
        }
      } else {
        /* characters are not equal but none is in the alphabet range */
        return 1; /* 本来都不是字母. */
      }
    } 
  } while (c1 != 0);
  return 0;
}
#endif

#ifndef lwip_strnicmp
/**
 * @ingroup sys_nonstandard
 * lwIP default implementation for strnicmp() non-standard function.
 * This can be \#defined to strnicmp() depending on your platform port.
 */
int
lwip_strnicmp(const char* str1, const char* str2, size_t len)
{
  char c1, c2;

  do {
    c1 = *str1++;
    c2 = *str2++;
    if (c1 != c2) { /* 与lwip_stricmp一样逻辑. */
      char c1_upc = c1 | 0x20;
      if ((c1_upc >= 'a') && (c1_upc <= 'z')) {
        /* characters are not equal an one is in the alphabet range:
        downcase both chars and check again */
        char c2_upc = c2 | 0x20;
        if (c1_upc != c2_upc) {
          /* still not equal */
          /* don't care for < or > */
          return 1;
        }
      } else {
        /* characters are not equal but none is in the alphabet range */
        return 1;
      }
    }
	} while (len-- && c1 != 0); /* 多了一个条件,就是len,因为只比较len长度的内存,超过的不管. */
  return 0;
}
#endif

#ifndef lwip_itoa
/**
 * @ingroup sys_nonstandard
 * lwIP default implementation for itoa() non-standard function.
 * This can be \#defined to itoa() or snprintf(result, bufsize, "%d", number) depending on your platform port.
 */
void
lwip_itoa(char* result, size_t bufsize, int number)
{
  const int base = 10;
  char* ptr = result, *ptr1 = result, tmp_char;
  int tmp_value;
  LWIP_UNUSED_ARG(bufsize);

  do {
    tmp_value = number; 
    number /= base; /* 比如传入123,那么等下数组偏移是(35 + 123 - 120) = 38,也就是char型的3.这个数组是不是可以不要字母,为什么要字母? */
    *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35 + (tmp_value - number * base)]; /* 35是26个字母 + 10个数字的元素偏移,元素从0开始,所以是35. */
  } while(number);

   /* Apply negative sign */
  if (tmp_value < 0) {
     *ptr++ = '-';
  }
  *ptr-- = '\0';
  while(ptr1 < ptr) {
    tmp_char = *ptr;
    *ptr--= *ptr1;
    *ptr1++ = tmp_char;
  }
}
#endif
