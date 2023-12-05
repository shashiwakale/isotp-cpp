/*
 * isotp_defines.h
 *
 *  Created on: 03-Dec-2023
 *      Author: shash
 */

#pragma once

/******************************************************************************
 * OS includes
 *****************************************************************************/
#include <vector>
/******************************************************************************
 * Local includes
 *****************************************************************************/

/******************************************************************************
 * Defines
 *****************************************************************************/

/******************************************************************************
 * Global variables
 *****************************************************************************/

/******************************************************************************
 * Namespaces
 *****************************************************************************/
namespace isotp
{
/*Separation time*/
constexpr unsigned char DEFAULT_SEPARATION_TIME = 20;
constexpr unsigned long WAIT_TIME               = 3000;
constexpr unsigned char SEND_ALL_BLKS_WO_INT    = 0;
/*masking*/
constexpr unsigned char FIRST_NIBBLE_MASK       = 0xF0;
constexpr unsigned char SECOND_NIBBLE_MASK      = 0x0F;
constexpr unsigned char BYTE_MASK               = 0xFF;
/*error codes*/
constexpr int INVALID_LENGTH                    = -121;
constexpr int SEND_ERROR                        = -122;
constexpr int RECEIVE_ERROR                     = -123;
constexpr int INVALID_DATA                      = -124;
constexpr int SUCCESS                           = 0;
/*isotp lengths*/
constexpr int SINGLE_FRAME_MAX_LENGTH           = 7;
constexpr int FIRST_FRAME_MAX_LENGTH            = 6;
constexpr int CONSECUTIVE_FRAME_MAX_LENGTH      = 7;
constexpr int ISOTP_MAX_DATA_LENGTH             = 4096;
/*isotp frames*/
constexpr unsigned char SINGLE_FRAME            = 0x00;
constexpr unsigned char FIRST_FRAME             = 0x10;
constexpr unsigned char CONSECUTIVE_FRAME       = 0x20;
constexpr unsigned char FLOW_CONTROL_FRAME      = 0x30;
/*isotp flow control flags*/
constexpr unsigned char CTS                     = 0;
constexpr unsigned char WAIT                    = 1;
constexpr unsigned char OVFLW                   = 2;
constexpr unsigned char RES                     = 3;
/*SID*/
constexpr unsigned char NEGATIVE_RESPONSE_FRAME = 0x7F;
/*NRC*/
constexpr unsigned char NRC_RESPONSE_PENDING    = 0x78;

/*isotp message*/
struct isotp_message
{
    int responseCode;
    unsigned long identifier;
    std::vector<unsigned char> data;
};

}
