/**********************************************************************
 *
 * Filename:    crc.h
 * 
 * Description: A header file describing the various CRC standards.
 *
 * See: http://www.barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 *
 * 
 * Copyright (c) 2000 by Michael Barr.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 **********************************************************************/

#ifndef _crc_h
#define _crc_h

#include <stdint.h>

#define FALSE	0
#define TRUE	!FALSE

/*
 * Select the CRC standard from the list that follows.
 */
#define CRC_CCITT


#if defined(CRC_CCITT)

typedef uint16_t  crc_t;

#define CRC_NAME			"CRC-CCITT"
#define POLYNOMIAL			0x1021
#define INITIAL_REMAINDER	0xFFFF
#define FINAL_XOR_VALUE		0x0000
#define CHECK_VALUE			0x29B1

#elif defined(CRC16)

typedef uint16_t  crc_t;

#define CRC_NAME			"CRC-16"
#define POLYNOMIAL			0x8005
#define INITIAL_REMAINDER	0x0000
#define FINAL_XOR_VALUE		0x0000
#define IS_REFLECT_DATA
#define IS_REFLECT_REMAINDER
#define CHECK_VALUE			0xBB3D

#elif defined(CRC32)

typedef uint32_t crc_t;

#define CRC_NAME			"CRC-32"
#define POLYNOMIAL			0x04C11DB7
#define INITIAL_REMAINDER	0xFFFFFFFF
#define FINAL_XOR_VALUE		0xFFFFFFFF
#define IS_REFLECT_DATA
#define IS_REFLECT_REMAINDER
#define CHECK_VALUE			0xCBF43926

#else

#error "One of CRC_CCITT, CRC16, or CRC32 must be #define'd."

#endif

/**
 * This function must be called before crcFast() is used.
 * If called more than once, each next call returns without
 * re-initialization, so it is better to call this function
 * from several modules that need it than to forget to call it
 * from one module.
 */
void crcInit(void);
crc_t crcSlow(unsigned char const message[], int nBytes);
crc_t crcFast(unsigned char const message[], int nBytes);


#endif /* _crc_h */
