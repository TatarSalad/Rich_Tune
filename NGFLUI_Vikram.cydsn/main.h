/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <project.h>

// The data expected to come in is LedMatrixData, its bits should match the table.
enum
{
   LedMatrixData_NumberOfBytes = 2
};

typedef struct
{
   uint8_t bytes[LedMatrixData_NumberOfBytes];
} LedMatrixData_t;


enum
{
   NumberOfMatrixScans = 6,
   Scan0Mask = 0x0001,
   Scan1Mask = 0x0002,
   Scan2Mask = 0x0004,
   Scan3Mask = 0x0008,
   Scan4Mask = 0x0010,
   Scan5Mask = 0x0020,
   Sink0 = 6,
   Sink1 = 7,
   Sink2 = 15,
   Sink3 = 14,
   Sink4 = 13,
   Sink5 = 12,
   Sink6 = 11,
   Sink7 = 10,
   Sink8 = 9,
   Sink9 = 8
};

enum {
	Z201,
	Z202,
	Z203,
	Z204,
	Z205,
	Z206,
	Z207,
	Z208,
	Z209,
	Z210,
	Z211,
	Z212,
	Z213,
	Z214,
	Z216,
	Z217,
	LedMatrixMax
};

typedef uint8 Scan_t;

typedef struct
{
   Scan_t scan;
   uint8_t pin;
} LedMatrixScanAndPins_t;


const static LedMatrixScanAndPins_t ledMatrixScanAndPins[LedMatrixMax] = 
{
	{4,Sink0}, // Z201
	{4,Sink1}, // Z202
	{4,Sink2}, // Z203
	{4,Sink3}, // Z204
	{4,Sink4}, // Z205
	{4,Sink5}, // Z206
	{4,Sink6}, // Z207
	{4,Sink7}, // Z208
	{5,Sink0}, // Z209
	{5,Sink1}, // Z210
	{5,Sink2}, // Z211
	{5,Sink3}, // Z212
	{5,Sink0}, // Z213
	{5,Sink1}, // Z214
	{5,Sink8}, // Z216
	{5,Sink9}  // Z217
};


/* [] END OF FILE */
