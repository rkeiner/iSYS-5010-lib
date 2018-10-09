/**************************************************************************************

      II    N     N     N     N      OOOO      SSSSS     EEEEE    N     N    TTTTTTT
     II    NNN   N     NNN   N    OO    OO    S         E        NNN   N       T
    II    N NN  N     N NN  N    OO    OO    SSSSS     EEE      N NN  N       T
   II    N  NN N     N  NN N    OO    OO        S     E        N  NN N       T
  II    N    NN     N    NN      OOOO      SSSSS     EEEEE    N    NN       T
                         copyright (c) 2011, InnoSenT GmbH
                                 all rights reserved

***************************************************************************************

    filename:			isys_api_basicTypes.h
    brief:				guidelines for data types used in isys_api
    creation:			05.05.2015
    author:				Benjamin Grünewald

    version:			v1.0
    last edit:          05.05.2015
    last editor:        Benjamin Grünewald
    change:
    compile switches:
***************************************************************************************
*/


#ifndef ISYS_API_BASICTYPES_H
#define ISYS_API_BASICTYPES_H

#ifndef INCLUSION_GUARDS_INNOSENT_GUIDELINES_H
/**************************************************************************************
  includes
**************************************************************************************/
#include <limits.h>

 
#if !defined(bool_t)
typedef /*@concrete@*/ unsigned short		bool_t;		/* boolean 1 bit */
#endif
#if !defined(float32_t)
typedef /*@concrete@*/ float				float32_t;	/* 32 bit floating point */
#endif

/**************************************************************************************
  define 8 bit signed/unsigned types & constants
**************************************************************************************/

#if SCHAR_MAX == 127 || SCHAR_MAX == 32767
/** 8bit signed type */
#if !defined(sint8_t)
typedef /*@concrete@*/ signed char sint8_t;
#endif
/** minimum signed value */
#if !defined(MIN8_8)
#define MIN8_8     (sint8_t)SCHAR_MIN
#endif
/** maximum signed value */
#if !defined(MAX8_8)
#define MAX8_8     (sint8_t)SCHAR_MAX
#endif
/** 8bit unsigned type */
#if !defined(uint8_t)
typedef /*@concrete@*/ unsigned char uint8_t;
#endif
/** minimum unsigned value */
#if !defined(UMIN8_8)
#define UMIN8_8    (uint8_t)0
#endif
/** maximum unsigned value */
#if !defined(UMAX8_8)
#define UMAX8_8    (uint8_t)UCHAR_MAX
#endif
#endif 


/**************************************************************************************
  define 16 bit signed/unsigned types & constants
**************************************************************************************/

#if INT_MAX == 32767
/** 16bit signed type; the system type in this documentation might not
    reflect the definition on another target device; please see \ref
    basictypes and the source code */
#if !defined(sint16_t)
typedef /*@concrete@*/ int sint16_t;
#endif
/** minimum signed value */
#if !defined(MIN16_16)
#define MIN16_16     (sint16_t)INT_MIN
#endif
/** maximum signed value */
#if !defined(MAX16_16)
#define MAX16_16     (sint16_t)INT_MAX
#endif
#ifndef _STDINT_H
/** 16bit unsigned type; the system type in this documentation might
    not reflect the definition on another target device; please see
    \ref basictypes and the source code */
#if !defined(uint16_t)
typedef /*@concrete@*/ unsigned int uint16_t;
#endif
/** minimum unsigned value */
#if !defined(UMIN16_16)
#define UMIN16_16    (uint16_t)0
#endif
/** maximum unsigned value */
#if !defined(UMAX16_16)
#define UMAX16_16    (uint16_t)UINT_MAX
#endif
#endif

#elif SHRT_MAX == 32767
#if !defined(sint16_t)
typedef /*@concrete@*/ short sint16_t;
#endif
#if !defined(MIN16_16)
#define MIN16_16     (sint16_t)SHRT_MIN
#endif
#if !defined(MAX16_16)
#define MAX16_16     (sint16_t)SHRT_MAX
#endif
#ifndef _STDINT_H
#if !defined(uint16_t)
typedef /*@concrete@*/ unsigned short uint16_t;
#endif
#if !defined(UMIN16_16)
#define UMIN16_16    (uint16_t)0
#endif
#if !defined(UMAX16_16)
#define UMAX16_16    (uint16_t)USHRT_MAX
#endif
#endif

#elif SHRT_MAX == 8388607
#if !defined(sint16_t)
typedef /*@concrete@*/ short sint16_t;
#endif
#if !defined(MIN16_16)
#define MIN16_16     (sint16_t)-32768
#endif
#if !defined(MAX16_16)
#define MAX16_16     (sint16_t)32767
#endif
#ifndef _STDINT_H
#if !defined(uint16_t)
typedef /*@concrete@*/ unsigned short uint16_t;
#endif
#if !defined(UMIN16_16)
#define UMIN16_16    (uint16_t)0
#endif
#if !defined(UMAX16_16)
#define UMAX16_16    (uint16_t)65535
#endif
#endif

#else
#error cannot find 16-bit type
#endif

/**************************************************************************************
  define 32 bit signed/unsigned types & constants
**************************************************************************************/

#if INT_MAX == 2147483647
/** 32bit signed type; the system type in this documentation might not
    reflect the definition on another target device; please see \ref
    basictypes and the source code */
#if !defined(sint32_t)
typedef /*@concrete@*/ int sint32_t;
#endif
/** minimum signed value */
#if !defined(MIN32_32)
#define MIN32_32     (sint32_t)INT_MIN
#endif
/** maximum signed value */
#if !defined(MAX32_32)
#define MAX32_32     (sint32_t)INT_MAX
#endif
#ifndef _STDINT_H
/** 32bit unsigned type; the system type in this documentation might not
    reflect the definition on another target device; please see \ref
    basictypes and the source code */
#if !defined(uint32_t)
typedef /*@concrete@*/ unsigned int uint32_t;
#endif
/** minimum unsigned value */
#if !defined(UMIN32_32)
#define UMIN32_32    (uint32_t)0
#endif
/** maximum unsigned value */
#if !defined(UMAX32_32)
#define UMAX32_32    (uint32_t)UINT_MAX
#endif
#endif

#elif LONG_MAX == 2147483647
#if !defined(sint32_t)
typedef /*@concrete@*/ long sint32_t;
#endif
#if !defined(MIN32_32)
#define MIN32_32     (sint32_t)LONG_MIN
#endif
#if !defined(MAX32_32)
#define MAX32_32     (sint32_t)LONG_MAX
#endif
#ifndef _STDINT_H
#if !defined(uint32_t)
typedef /*@concrete@*/ unsigned long uint32_t;
#endif
#if !defined(UMIN32_32)
#define UMIN32_32    (uint32_t)0
#endif
#if !defined(UMAX32_32)
#define UMAX32_32    (uint32_t)ULONG_MAX
#endif
#endif

#else
#error cannot find 32-bit type
#endif

/**************************************************************************************
  float constants
**************************************************************************************/
#define FLOAT32_T_MAX (FLT_MAX)
#define FLOAT32_T_MIN (FLT_MIN)

/**************************************************************************************
   data type definitions with size and sign
**************************************************************************************/

typedef /*@concrete@*/ char					char_t;		/* plain 8 bit character */
typedef /*@concrete@*/ unsigned short		bool_t;		/* boolean 1 bit */
typedef /*@concrete@*/ unsigned long long	uint64_t;	/* unsigned 64 bit integer */
typedef /*@concrete@*/ long long			sint64_t;	/* signed 64 bit integer */
typedef /*@concrete@*/ float				float32_t;	/* 32 bit floating point */

#ifdef S_SPLINT_S
typedef /*@concrete@*/ long double			float64_t;	/* 64 bit floating point */
#else
#ifndef ARM_MATH_CM4
typedef /*@concrete@*/ long double			float64_t;	/* 64 bit floating point */
#endif
#endif

#endif
#endif // ISYS_API_BASICTYPES_H
