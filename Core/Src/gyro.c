// Functions to manage the gyro sensor

//Basic functions for MPU6050 were written with the use of the chip datasheet and HAL libraries for STM32
//DMP functions are not documented  in datasheet
//DMP Functions are ported from Arduino code written by Jeff Rowberg
//There is STM32 library for MPU6050 by Jeff Rowberg but it lacks DMP functionality


#include "gyro.h"

//DMP image FW for MPU6050 needs to be writen to chip on startup for DMP to work
const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] = {
	/* bank # 0 */
	0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
	0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
	0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
	0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCB, 0x47, 0xA2, 0x20, 0x00, 0x00, 0x00,
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
	0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
	0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
	0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
	0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,
	/* bank # 1 */
	0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
	0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
	0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x09, 0x23, 0xA1, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
	0x80, 0x00, 0xFF, 0xFF, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00,
	0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,
	/* bank # 2 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x01, 0x00, 0x05, 0x8B, 0xC1, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* bank # 3 */
	0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
	0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
	0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
	0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
	0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
	0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
	0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
	0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0x4C, 0xCD, 0x6C, 0xA9, 0x0C,
	0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
	0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
	0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
	0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
	0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
	0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
	0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
	0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,
	/* bank # 4 */
	0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
	0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
	0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
	0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
	0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
	0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
	0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
	0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
	0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
	0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
	0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
	0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
	0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
	0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
	0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
	0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,
	/* bank # 5 */
	0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
	0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
	0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
	0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
	0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
	0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
	0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
	0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
	0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
	0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
	0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
	0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
	0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
	0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
	0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
	0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,
	/* bank # 6 */
	0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
	0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
	0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
	0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
	0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
	0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
	0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
	0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
	0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
	0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
	0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
	0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
	0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
	0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
	0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
	0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,
	/* bank # 7 */
	0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
	0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
	0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
	0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
	0xDD, 0xF1, 0x20, 0x28, 0x30, 0x38, 0x9A, 0xF1, 0x28, 0x30, 0x38, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
	0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
	0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0x28, 0x30, 0x38,
	0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0x30, 0xDC,
	0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xFE, 0xD8, 0xFF,

};


void MPU6050_Write_Single_Bit(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddress, uint8_t RegisterAddress, uint8_t BitPosition, uint8_t BitValue)
{
	uint8_t data;
	HAL_I2C_Mem_Read (I2Cx, DeviceAddress, RegisterAddress, 1, &data, 1, 1000);//read whole Register

	if(BitValue==1)			data|= 1<<BitPosition; 		 //force 1 to bit in BitPosition
	else if(BitValue==0) 	data&=~(1<<BitPosition); 	 //force 0 to bit in BitPosition

	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, RegisterAddress, 1,&data, 1, 1000); //write corected data (1 bit only) back to register
}

void MPU6050_Write_Few_Bits(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddress, uint8_t RegisterAddress, uint8_t BitStart, uint8_t Length, uint8_t data)
{
	uint8_t mask;
	uint8_t temp;

	HAL_I2C_Mem_Read (I2Cx, DeviceAddress, RegisterAddress, 1, &temp, 1, 1000);//read whole Register

    mask = ((1 << Length) - 1) << (BitStart - Length + 1);
    data <<= (BitStart - Length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    temp &= ~(mask); // zero all important bits in existing word
    temp |= data; // combine data with existing word

	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, RegisterAddress, 1,&temp, 1, 1000); //write corected data (1 bit only) back to register
}

uint8_t MPU6050_Read_Single_Bit(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddress, uint8_t RegisterAddress, uint8_t BitPosition)
{
	uint8_t data;
	HAL_I2C_Mem_Read (I2Cx, DeviceAddress, RegisterAddress, 1, &data, 1, 1000);//read whole Register

	//filter bit data
	data&=(1<<BitPosition);
	//shift bit
	data=data>>BitPosition;

	return data;
}

MPU6050_Result MPU6050_check(I2C_HandleTypeDef* I2Cx)
{
	//I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t data;

	HAL_I2C_Mem_Read (I2Cx, MPU6050_ADDRESS,MPU6050_WHO_AM_I,1, &data, 1, 1000);

	if(data==MPU6050_I_AM_VAL) return MPU6050_DETECTED;
	else return MPU6050_NOTDETECTED;

}

void MPU6050_accread(I2C_HandleTypeDef* I2Cx, MPU6050str* DataStruct)
{
	uint8_t data[6];
	HAL_I2C_Mem_Read (I2Cx, MPU6050_ADDRESS, MPU6050_ACCEL_RA_XOUT_H, 1, data, 6, 1000);

	DataStruct->Accelerometer_X_RAW = (int16_t)(data[0] << 8 | data [1]);
	DataStruct->Accelerometer_Y_RAW = (int16_t)(data[2] << 8 | data [3]);
	DataStruct->Accelerometer_Z_RAW = (int16_t)(data[4] << 8 | data [5]);

}

void MPU6050_gyroread(I2C_HandleTypeDef* I2Cx, MPU6050str* DataStruct)
{
	uint8_t data[6];
	HAL_I2C_Mem_Read (I2Cx, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, data, 6, 1000);

	DataStruct->Gyroscope_X_RAW = (int16_t)(data[0] << 8 | data [1]);
	DataStruct->Gyroscope_Y_RAW = (int16_t)(data[2] << 8 | data [3]);
	DataStruct->Gyroscope_Z_RAW = (int16_t)(data[4] << 8 | data [5]);
}

void MPU6050_init(I2C_HandleTypeDef* I2Cx)
{
	//reset Device
	MPU6050_Reset(I2Cx,MPU6050_ADDRESS);
	HAL_Delay(100);

	MPU6050_SetSleepEnabled(I2Cx,MPU6050_ADDRESS,0); //get from sleep mode
	MPU6050_Set_CLK_Source(I2Cx,MPU6050_ADDRESS,MPU6050_CLOCK_PLL_ZGYRO);

	HAL_Delay(50);//PLL settling time

	MPU6050_SetRate(I2Cx,MPU6050_ADDRESS, 0);// Sample Rate equals Gyro sample range

	HAL_Delay(50);

	MPU6050_SetGyroRange(I2Cx,MPU6050_ADDRESS, MPU6050_GYRO_FS_2000);
	MPU6050_SetAccelRange(I2Cx,MPU6050_ADDRESS, MPU6050_ACCEL_FS_8);
	SetDLPFMode(I2Cx,MPU6050_ADDRESS, MPU6050_DLPF_BW_256); //gyro fast sample rate
}

void MPU6050_DMP_Enable(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddress, uint8_t enable)
{
	MPU6050_Write_Single_Bit(I2Cx,DeviceAddress, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enable);
}

void MPU6050_DMP_Reset(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddres)
{
	MPU6050_Write_Single_Bit(I2Cx,DeviceAddres, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, 1);
}

uint8_t  MPU6050_DMP_Get_Enable(I2C_HandleTypeDef* I2Cx)
{
	uint8_t data;
	data=MPU6050_Read_Single_Bit(I2Cx, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT);
	return data;
}

void MPU6050_Set_Memory_Bank(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank)
{
	bank &= 0x1F;
	if (userBank) bank |= 0x20;
	if (prefetchEnabled) bank |= 0x40;

	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_BANK_SEL, 1,&bank, 1, 1000);
}

void MPU6050_Set_Memory_Start_Address(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t address)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_MEM_START_ADDR, 1, &address, 1, 1000);
}

uint8_t MPU6050_Read_Memory_Bank(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress)
{
	uint8_t data;
	HAL_I2C_Mem_Read (I2Cx, DeviceAddress, MPU6050_RA_MEM_R_W, 1, &data, 1, 1000);
	return data;
}

uint8_t MPU6050_getOTPBankValid(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress)
{
	uint8_t data;
	data=MPU6050_Read_Single_Bit(I2Cx, DeviceAddress, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT);
	return data;
}

void MPU6050_Set_SlaveAddress(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t num, uint8_t address)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_I2C_SLV0_ADDR + num*3, 1, &address, 1, 1000);
}

void MPU6050_Set_MasterModeEnable(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t enable)
{
	MPU6050_Write_Single_Bit(I2Cx,DeviceAddress, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT ,enable);
}

void MPU6050_Reset_I2CMaster(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress)
{
	MPU6050_Write_Single_Bit(I2Cx,DeviceAddress, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT ,1);
}

void MPU6050_Set_CLK_Source(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t source)
{
	MPU6050_Write_Few_Bits(I2Cx,DeviceAddress, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH ,source);
}

void MPU6050_Reset(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress)
{
	MPU6050_Write_Single_Bit(I2Cx,MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT ,1);
}

void MPU6050_SetSleepEnabled(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t enable)
{
	MPU6050_Write_Single_Bit(I2Cx,MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT ,enable);
}

void MPU6050_SetIntEnabled(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t enable)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_INT_ENABLE, 1, &enable, 1, 1000);
}

void MPU6050_SetRate(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t rate)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_SMPLRT_DIV, 1, &rate, 1, 1000);
}

void MPU6050_SetGyroRange(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t range)
{
	MPU6050_Write_Few_Bits(I2Cx, DeviceAddress, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH,range);
}

void MPU6050_SetAccelRange(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t range)
{
	MPU6050_Write_Few_Bits(I2Cx, DeviceAddress, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH,range);
}

void SetExternalFrameSync(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t sync)
{
	MPU6050_Write_Few_Bits(I2Cx, DeviceAddress, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH,sync);
}

void SetDLPFMode(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t mode)
{
	MPU6050_Write_Few_Bits(I2Cx, DeviceAddress, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH,mode);
}

void MPU6050_DMPConfig1(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t config)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_DMP_CFG_1, 1, &config, 1, 1000);
}

void MPU6050_DMPConfig2(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t config)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_DMP_CFG_2, 1, &config, 1, 1000);
}

void MPU6050_SetOTPBankValid(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t enable)
{
	MPU6050_Write_Single_Bit(I2Cx,MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT ,enable);
}

void MPU6050_SetMotionDetectionThreshold(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t threshold)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_MOT_THR, 1, &threshold, 1, 1000);
}

void MPU6050_SetZeroMotionDetectionThreshold(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t threshold)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_ZRMOT_THR, 1, &threshold, 1, 1000);
}

void MPU6050_SetMotionDetectionDuration(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t duration)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_MOT_DUR, 1, &duration, 1, 1000);
}

void MPU6050_SetZeroMotionDetectionDuration(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t duration)
{
	HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_ZRMOT_DUR, 1, &duration, 1, 1000);
}

void MPU6050_SetFIFOenabled(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t enable)
{
	MPU6050_Write_Single_Bit(I2Cx,MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT ,enable);
}

void MPU6050_ResetFIFO(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress)
{
	MPU6050_Write_Single_Bit(I2Cx,MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT ,1);
}

uint16_t MPU6050_GetFifoCount(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress)
{
	uint8_t data[2];
	HAL_I2C_Mem_Read (I2Cx, DeviceAddress, MPU6050_RA_FIFO_COUNTH, 1, data, 2, 1000);
	return (((uint16_t)data[0]) << 8) | data[1];
}

uint8_t MPU6050_FifoOvreflowStatus(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress)
{
	uint8_t data;
	data=MPU6050_Read_Single_Bit(I2Cx, DeviceAddress, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_FIFO_OFLOW_BIT);
	return data;
}

void MPU6050_GetFifoBytes(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t *data, uint8_t length)
{
    if(length > 0)
    {
    	HAL_I2C_Mem_Read (I2Cx, DeviceAddress, MPU6050_RA_FIFO_R_W, 1 , data, length, 1000);
    }

    else
    {
    	*data = 0;
    }
}

uint8_t MPU6050_GetFIFOEnableStatus(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress)
{
	uint8_t data;
	data=MPU6050_Read_Single_Bit(I2Cx, DeviceAddress, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT);
	return data;
}

uint8_t MPU6050_GetIntStatus(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress)
{
	uint8_t data;
	HAL_I2C_Mem_Read (I2Cx, DeviceAddress, MPU6050_RA_INT_STATUS, 1, &data, 1, 1000);
	return data;
}

uint32_t MPU6050_GetCurrentFIFOPacket(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t *data,uint8_t length)
{
    int16_t fifoC;

    fifoC = MPU6050_GetFifoCount(I2Cx, DeviceAddress);

    if(!fifoC)//No data in FIFO
    {
    	return 0;
    }

    if(fifoC > length)//reset Buffer, more than expected 42 packets clr buffer read in next cycle
    {
    	MPU6050_ResetFIFO(I2Cx,MPU6050_ADDRESS);
    	return 0;
    }

    else //read expected 42 bytes
    {
    	MPU6050_GetFifoBytes(I2Cx, DeviceAddress,data, length);
    	return 1;
    }
}

uint8_t MPU6050_WriteMemoryBlock(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address,uint8_t verify, uint8_t useProgMem)
{
	MPU6050_Set_Memory_Bank(I2Cx,DeviceAddress,bank,0,0);
	MPU6050_Set_Memory_Start_Address(I2Cx,DeviceAddress,address);

    uint8_t chunkSize;
    uint8_t *verifyBuffer=0;
    uint8_t *progBuffer=0;
    uint16_t i;
    uint8_t j;
    if (verify) 	verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);

    for (i = 0; i < dataSize;)
    {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        if (useProgMem) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        HAL_I2C_Mem_Write(I2Cx, DeviceAddress, MPU6050_RA_MEM_R_W, 1, progBuffer, chunkSize, 1000);

        // verify data if needed
        if (verify && verifyBuffer)
        {
        	MPU6050_Set_Memory_Bank(I2Cx,DeviceAddress,bank,0,0);
        	MPU6050_Set_Memory_Start_Address(I2Cx,DeviceAddress,address);

        	HAL_I2C_Mem_Read (I2Cx, DeviceAddress, MPU6050_RA_MEM_R_W, 1, verifyBuffer, chunkSize, 1000);

            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0)
            {
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return 0; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize)
        {
            if (address == 0) bank++;
        	MPU6050_Set_Memory_Bank(I2Cx,DeviceAddress,bank,0,0);
        	MPU6050_Set_Memory_Start_Address(I2Cx,DeviceAddress,address);
        }
    }
    if (verify) free(verifyBuffer);
    if (useProgMem) free(progBuffer);
    return 1;
}

uint8_t MPU6050_DMP_Init(I2C_HandleTypeDef* I2Cx)
{

	//reset Device
	MPU6050_Reset(I2Cx,MPU6050_ADDRESS);

	HAL_Delay(30);

	//Sleep False
	MPU6050_SetSleepEnabled(I2Cx,MPU6050_ADDRESS,0);

	MPU6050_Set_Memory_Bank(I2Cx,MPU6050_ADDRESS, 0x10, 1, 1);
	MPU6050_Set_Memory_Start_Address(I2Cx,MPU6050_ADDRESS,0x06);
	MPU6050_Read_Memory_Bank(I2Cx,MPU6050_ADDRESS);
	MPU6050_Set_Memory_Bank(I2Cx,MPU6050_ADDRESS, 0, 0, 0);
	MPU6050_getOTPBankValid(I2Cx,MPU6050_ADDRESS);

	//Set Slave Stuff
	MPU6050_Set_SlaveAddress(I2Cx, MPU6050_ADDRESS, 0, 0x7F);	//slave 0 to address 0x7F
	MPU6050_Set_MasterModeEnable(I2Cx, MPU6050_ADDRESS,0); 		//disable master mode
	MPU6050_Set_SlaveAddress(I2Cx, MPU6050_ADDRESS, 0, 0x68);	//slave 0 to address 0x68
	MPU6050_Reset_I2CMaster(I2Cx,MPU6050_ADDRESS);

	HAL_Delay(20);

	MPU6050_Set_CLK_Source(I2Cx,MPU6050_ADDRESS,MPU6050_CLOCK_PLL_ZGYRO);
	MPU6050_SetIntEnabled(I2Cx,MPU6050_ADDRESS,1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT|1<<MPU6050_INTERRUPT_DMP_INT_BIT);

	MPU6050_SetRate(I2Cx,MPU6050_ADDRESS, 4);// if GYRO clk 1khz (depends on BW setting) 1Khz / (1+4)=200hZ  ,, else 8kHz /(39+1)=200hZ

	SetExternalFrameSync(I2Cx,MPU6050_ADDRESS, MPU6050_EXT_SYNC_TEMP_OUT_L);

	SetDLPFMode(I2Cx,MPU6050_ADDRESS, MPU6050_DLPF_BW_42);

	MPU6050_SetGyroRange(I2Cx,MPU6050_ADDRESS, MPU6050_GYRO_FS_2000); //must be full range or DMP overcompensates on fast movements !!!!

	//MPU6050_SetAccelRange(I2Cx,MPU6050_ADDRESS, MPU6050_ACCEL_FS_2);//don't change leave default seems DMP needs specific range to work properly

	//Load DMP Code in Memory Bank
	MPU6050_WriteMemoryBlock(I2Cx,MPU6050_ADDRESS,dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0,1,0);

	// Set the FIFO Rate Divisor int the DMP Firmware Memory
	unsigned char dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
	MPU6050_WriteMemoryBlock(I2Cx,MPU6050_ADDRESS,dmpUpdate, 0x02, 0x02, 0x16,1,0); // Lets write the dmpUpdate data to the Firmware image, we have 2 bytes to write in bank 0x02 with the Offset 0x16

	//write start address MSB into register
	MPU6050_DMPConfig1(I2Cx,MPU6050_ADDRESS, 0x03);

	//write start address LSB into register
	MPU6050_DMPConfig2(I2Cx,MPU6050_ADDRESS, 0x00);

	MPU6050_SetOTPBankValid(I2Cx,MPU6050_ADDRESS,0);// Clear OTP flag

	MPU6050_SetMotionDetectionThreshold(I2Cx,MPU6050_ADDRESS,2);
	MPU6050_SetZeroMotionDetectionThreshold(I2Cx,MPU6050_ADDRESS,156);
	MPU6050_SetMotionDetectionDuration(I2Cx,MPU6050_ADDRESS,80);
	MPU6050_SetZeroMotionDetectionDuration(I2Cx,MPU6050_ADDRESS,0);

	MPU6050_SetFIFOenabled(I2Cx,MPU6050_ADDRESS,1);//enable FIFO

	MPU6050_DMP_Reset(I2Cx,MPU6050_ADDRESS);

	MPU6050_DMP_Enable(I2Cx,MPU6050_ADDRESS,0);//disable DMP

	MPU6050_ResetFIFO(I2Cx,MPU6050_ADDRESS); //reset FIFO

	MPU6050_GetIntStatus(I2Cx,MPU6050_ADDRESS);
}


void CalculateQuaternions(struct Quaternions *q, uint8_t *fifo_data)
{

	int32_t q1,q2,q3,q4;

	q1=((int32_t)fifo_data[0] << 24) | ((int32_t)fifo_data[1] << 16) | ((int32_t)fifo_data[2] << 8) | fifo_data[3];
	q2=((int32_t)fifo_data[4] << 24) | ((int32_t)fifo_data[5] << 16) | ((int32_t)fifo_data[6] << 8) | fifo_data[7];
	q3=((int32_t)fifo_data[8] << 24) | ((int32_t)fifo_data[9] << 16) | ((int32_t)fifo_data[10] << 8) | fifo_data[11];
	q4=((int32_t)fifo_data[12] << 24) | ((int32_t)fifo_data[13] << 16) | ((int32_t)fifo_data[14] << 8) | fifo_data[15];

	q->w=(float)(q1>>16) / ACCELCONSTANT; //Depends on ACCEL RANGE!
	q->x=(float)(q2>>16) / ACCELCONSTANT; //Depends on ACCEL RANGE!
	q->y=(float)(q3>>16) / ACCELCONSTANT; //Depends on ACCEL RANGE!
	q->z=(float)(q4>>16) / ACCELCONSTANT; //Depends on ACCEL RANGE!
}

void CalculateGravityVector(struct Quaternions *q, struct GravityVector *v)
{
	v -> x = 2 * (q -> x * q -> z - q -> w * q -> y);
	v -> y = 2 * (q -> w * q -> x + q -> y * q -> z);
	v -> z = q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z;
}

void CalculateYawPitchRoll(struct Quaternions *q, struct GravityVector *v, struct Angles *ang)
{
	//X,Y,Z angles in radians calculated from Quaternions
	float zRad;
	float yRad;
	float xRad;

	//(about Z axis)
	zRad = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);

	//(about Y axis)
	yRad = atan2(v -> x , sqrt(v -> y*v -> y + v -> z*v -> z));

	//(about X axis)
	xRad = atan2(v -> y , v -> z);
	if (v -> z < 0)
	{
		if(xRad > 0)
		{
			xRad = PI - xRad;
		}
		else
		{
			xRad = -PI - xRad;
		}
	}

	//MPU6050 position on Drone--> X direction (+ drone right)-Rotation around x Pitch, Y direction (+ drone front) -Rotation around Y Roll
	ang->yaw=zRad*RADIANSTODEGREES;
	ang->yaw+=YAWDMPOFFSET;
	if ( ang->yaw < -180 ) ang->yaw += 360;

	ang->pitch=xRad*RADIANSTODEGREES;
	ang->pitch+=PITCHDMPOFFSET; //Add manual offset
	if ( ang->pitch < -180 ) ang->pitch += 360;

	ang->roll=yRad*RADIANSTODEGREES;
	ang->roll+=ROLLDMPOFFSET; //Add manual offset
	if ( ang->roll < -180 ) ang->roll += 360;
	ang->roll=-ang->roll; //positive angle drone tilt to right
}

void MPU6050_CalculateFromRAWData(MPU6050str* d,float timedelta)
{

	float AccelVectorPitch;
	float AccelVectorRoll;
	float p,q,r;
	float X,Y,Z;

	//Offset RAW gyro values with calibrated offsets
	d->Gyroscope_X_Cal = (float)(d->Gyroscope_X_RAW) - d->Offset_Gyro_X;
	d->Gyroscope_Y_Cal = (float)(d->Gyroscope_Y_RAW) - d->Offset_Gyro_Y;
	d->Gyroscope_Z_Cal = (float)(d->Gyroscope_Z_RAW) - d->Offset_Gyro_Z;


	//GYRO AND ACCEL DATA in STANDARD X,Y,Z directions Roll (nose), Pitch(right wing), Yaw (down)
	//Sensor MPU 6050 axis position X (right wing), Y (nose), Z (up)
	d->Gyro_X = d->Gyroscope_Y_Cal;
	d->Gyro_Y = d->Gyroscope_X_Cal;
	d->Gyro_Z = -d->Gyroscope_Z_Cal;

	d->Accel_X = d->Accelerometer_Y_RAW;
	d->Accel_Y = d->Accelerometer_X_RAW;
	d->Accel_Z = -d->Accelerometer_Z_RAW;


	//Accelerometer angles-----------------------------------------------------------------
	AccelVectorRoll =  sqrt( (d->Accel_X * d->Accel_X) + (d->Accel_Z * d->Accel_Z) );
	AccelVectorPitch = sqrt( (d->Accel_Y * d->Accel_Y) + (d->Accel_Z * d->Accel_Z) );

	d->Angle_Accel_Roll  = -atan(d->Accel_Y/AccelVectorRoll) * RADIANSTODEGREES;
	d->Angle_Accel_Pitch = atan(d->Accel_X/AccelVectorPitch) * RADIANSTODEGREES;

	//Compensate offset with spirit level manual offset
	d->Angle_Accel_Pitch-=ACCELPITCHMANUALOFFSET;
	d->Angle_Accel_Roll-=ACCELROLLMANUALOFFSET;
	//Save angles in Radians
	d->Angle_Accel_Pitch_Rad=d->Angle_Accel_Pitch*DEGREESTORADIANS;
	d->Angle_Accel_Roll_Rad=d->Angle_Accel_Roll*DEGREESTORADIANS;

	//Calculate angular gyro velocities----------------------------------------------------
	d->AngleSpeed_Gyro_X = d->Gyro_X / GYROCONSTANT;
	d->AngleSpeed_Gyro_Y = d->Gyro_Y / GYROCONSTANT;
	d->AngleSpeed_Gyro_Z = d->Gyro_Z / GYROCONSTANT;

	//convert angular velocity to radians/s
	p = d->AngleSpeed_Gyro_X * DEGREESTORADIANS;
	q = d->AngleSpeed_Gyro_Y * DEGREESTORADIANS;
	r = d->AngleSpeed_Gyro_Z * DEGREESTORADIANS;

	//Save Angles in radians from previous STEP
	X = d->Roll_Rad;
	Y = d->Pitch_Rad;
	Z = d->Angle_Gyro_Yaw_Rad ;

	//TRANSFORM gyro data to Euler Angles with complementary filter with accelerometer
	d->Roll_Rad   = 0.999 * (X + timedelta * (p  +  q*sin(X)*tan(Y) + r*cos(X)*tan(Y) ) ) + 0.001*d->Angle_Accel_Roll_Rad;
	d->Pitch_Rad  = 0.999 * (Y + timedelta * (q * cos(X) -  r * sin(X) ) 			    ) + 0.001*d->Angle_Accel_Pitch_Rad;
	d->Angle_Gyro_Yaw_Rad = Z + timedelta * (q*sin(X)/cos(Y) + r*cos(X)/cos(Y) ); //Only Gyro Angle will drift

	//Convert to Degrees
	d->Roll   = d->Roll_Rad * RADIANSTODEGREES;
	d->Pitch  = d->Pitch_Rad * RADIANSTODEGREES;
	d->Angle_Gyro_Yaw   = d->Angle_Gyro_Yaw_Rad * RADIANSTODEGREES;
}

void GetGyroOffset(I2C_HandleTypeDef* I2Cx, MPU6050str* d, int32_t Loops, uint32_t Delayms)
{
	int32_t SUMGyroX,SUMGyroY,SUMGyroZ;
	uint32_t i;

	SUMGyroX=0;
	SUMGyroY=0;
	SUMGyroZ=0;

	for(i=0;i<Loops;i++)
	{
		  //MPU6050_gyroread(&hi2c2,&mpu6050DataStr);

		  SUMGyroX+=d->Gyroscope_X_RAW;
		  SUMGyroY+=d->Gyroscope_Y_RAW;
		  SUMGyroZ+=d->Gyroscope_Z_RAW;

		  HAL_Delay(Delayms);

	}

	d->Offset_Gyro_X=(float)(SUMGyroX) / (float)(Loops);
	d->Offset_Gyro_Y=(float)(SUMGyroY) / (float)(Loops);
	d->Offset_Gyro_Z=(float)(SUMGyroZ) / (float)(Loops);


	//MPU6050_accread(&hi2c2,&mpu6050DataStr);

	MPU6050_CalculateFromRAWData(&mpu6050DataStr,0); //Gyro angles don't matter

	//Transfer accelerometer angles to Gyro
	d->Pitch = d->Angle_Accel_Pitch;
	d->Roll =  d->Angle_Accel_Roll;

	d->Angle_Gyro_Pitch_Rad = d->Angle_Accel_Pitch_Rad;
	d->Angle_Gyro_Roll_Rad = d->Angle_Accel_Roll_Rad;

	d->Angle_Gyro_Yaw = 0;
	d->Angle_Gyro_Yaw_Rad = 0;

}

