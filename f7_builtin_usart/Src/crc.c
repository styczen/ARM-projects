/**
 * Slow and fast implementations of the CRC standards.
 *
 * The parameters for each supported CRC standard are
 *				defined in the header file crc.h.  The implementations
 *				here should stand up to further additions to that list.
 *
 * 
 * @copy Copyright (c) 2000 by Michael Barr.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 **********************************************************************/
 
#include "crc.h"


/*
 * Derive parameters from the standard-specific parameters in crc.h.
 */
#define WIDTH    (8 * sizeof(crc_t))
#define TOPBIT   (1 << (WIDTH - 1))

#ifdef IS_REFLECT_DATA
    #define REFLECT_DATA(X)			((uint8_t) reflect((X), 8))
#else
    #define REFLECT_DATA(X)			(X)
#endif

#ifdef IS_REFLECT_REMAINDER
    #define REFLECT_REMAINDER(X)	((crc_t) reflect((X), WIDTH))
#else
    #define REFLECT_REMAINDER(X)	(X)
#endif

#if defined(IS_REFLECT_DATA)  ||  defined(IS_REFLECT_REMAINDER)
/**
 * Reorder the bits of a binary sequence, by reflecting
 *				them about the middle position.
 *
 * No checking is done that nBits <= 32.
 *
 * @return The reflection of the original data.
 */
static uint32_t reflect(unsigned long data, unsigned char nBits)
{
	uint32_t reflection = 0x00000000;
	unsigned char  bit;

	/*
	 * Reflect the data about the center bit.
	 */
	for (bit = 0; bit < nBits; ++bit)
	{
		/*
		 * If the LSB bit is set, set the reflection of it.
		 */
		if (data & 0x01)
		{
			reflection |= (1 << ((nBits - 1) - bit));
		}

		data = (data >> 1);
	}

	return (reflection);
}
#endif

/**
 * Computes the CRC of a given message.
 * 
 * @return The CRC of the message.
 *
 */
crc_t crcSlow(unsigned char const message[], int nBytes)
{
    crc_t remainder = INITIAL_REMAINDER;
	int byte;
	unsigned char bit;


    /*
     * Perform modulo-2 division, a byte at a time.
     */
    for (byte = 0; byte < nBytes; ++byte)
    {
        /*
         * Bring the next byte into the remainder.
         */
        remainder ^= (REFLECT_DATA(message[byte]) << (WIDTH - 8));

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (bit = 8; bit > 0; --bit)
        {
            /*
             * Try to divide the current data bit.
             */
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }

    /*
     * The final remainder is the CRC result.
     */
    return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);

}   /* crcSlow() */


crc_t crcTable[256];
uint8_t s_isCRCInitialized = 0;

/**
 * Populate the partial CRC lookup table.
 *
 * This function must be rerun any time the CRC standard
 *				is changed. If desired, it can be run "offline" and
 *				the table results stored in an embedded system's ROM.
 */
void crcInit(void)
{
	// make redundant calls to this function cheap
	if (s_isCRCInitialized) {
		return;
	}

	s_isCRCInitialized = 1;

    crc_t remainder;
	int dividend;
	unsigned char bit;

    // Compute the remainder of each possible dividend.
    for (dividend = 0; dividend < 256; ++dividend)
    {
        // Start with the dividend followed by zeros.
        remainder = dividend << (WIDTH - 8);

        // Perform modulo-2 division, a bit at a time.
        for (bit = 8; bit > 0; --bit)
        {
            // Try to divide the current data bit.
            if (remainder & TOPBIT) {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            } else {
                remainder = (remainder << 1);
            }
        }

        // Store the result into the table.
        crcTable[dividend] = remainder;
    }

}


/**
 * Compute the CRC of a given message.
 *
 * crcInit() must be called first.
 *
 * @return The CRC of the message.
 */
crc_t crcFast(unsigned char const message[], int nBytes)
{
    crc_t remainder = INITIAL_REMAINDER;

    // Divide the message by the polynomial, a byte at a time.
    for (int byte = 0; byte < nBytes; ++byte)
    {
        uint8_t data = REFLECT_DATA(message[byte]) ^ (remainder >> (WIDTH - 8));
  		remainder = crcTable[data] ^ (remainder << 8);
    }

    // The final remainder is the CRC.
    return (REFLECT_REMAINDER(remainder) ^ FINAL_XOR_VALUE);
}
