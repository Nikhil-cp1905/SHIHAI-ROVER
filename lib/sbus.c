#include <stdint.h>

// Parsing an 11-bit channel from an SBUS-like protocol
uint16_t *parse_buffer(uint8_t buff[]) { 
		
	// Array to store 16 channels, each 11 bits wide
	static uint16_t channel[16];

	// Mask to extract 11-bit data (0x07FF = 2047 in decimal)
	const uint16_t mask = 0x07FF;

	// Extracting 11-bit values for each channel
	channel[0]  = (buff[0]      | (buff[1] << 8)) & mask;
	channel[1]  = ((buff[1] >> 3) | (buff[2] << 5)) & mask;
	channel[2]  = ((buff[2] >> 6) | (buff[3] << 2) | (buff[4] << 10)) & mask;
	channel[3]  = ((buff[4] >> 1) | (buff[5] << 7)) & mask;
	channel[4]  = ((buff[5] >> 4) | (buff[6] << 4)) & mask;
	channel[5]  = ((buff[6] >> 7) | (buff[7] << 1) | (buff[8] << 9)) & mask;
	channel[6]  = ((buff[8] >> 2) | (buff[9] << 6)) & mask;
	channel[7]  = ((buff[9] >> 5) | (buff[10] << 3)) & mask;
	channel[8]  = (buff[11]      | (buff[12] << 8)) & mask;
	channel[9]  = ((buff[12] >> 3) | (buff[13] << 5)) & mask;
	channel[10] = ((buff[13] >> 6) | (buff[14] << 2) | (buff[15] << 10)) & mask;
	channel[11] = ((buff[15] >> 1) | (buff[16] << 7)) & mask;
	channel[12] = ((buff[16] >> 4) | (buff[17] << 4)) & mask;
	channel[13] = ((buff[17] >> 7) | (buff[18] << 1) | (buff[19] << 9)) & mask;
	channel[14] = ((buff[19] >> 2) | (buff[20] << 6)) & mask;
	channel[15] = ((buff[20] >> 5) | (buff[21] << 3)) & mask;

	return channel;
}

