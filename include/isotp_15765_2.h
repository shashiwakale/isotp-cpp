/*
 * isotp_15765_2.h
 *
 *  Created on: 25-Nov-2023
 *      Author: shashi
 */

#pragma once
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "socket_can.h"

namespace isotp
{
/*Separation time*/
constexpr unsigned char DEFAULT_SEPARATION_TIME = 10;
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

/*isotp message*/
struct isotp_message
{
    int responseCode;
    unsigned long identifier;
    std::vector<unsigned char> data;
};

class isotp15765
{
public:
    isotp15765(socketcan::SocketCAN* socketCAN, unsigned long srcAddr, unsigned long destAddr);
    ~isotp15765();
    isotp_message isotp_send(const std::vector<unsigned char>& data);

private:
    std::mutex m;
    socketcan::SocketCAN* canHandle = nullptr;
    unsigned long srcID;
    unsigned long destID;
    bool exitThread = false;
    std::condition_variable cv;
    std::thread canReceiveThread;
    std::queue<struct can_frame> messageQueue;

    void ReceiveThreadFunction(void);
    std::vector<unsigned char> GetCANMessage(void);
    int SendConsecutiveData(std::vector<unsigned char>& data);
    int SendSingleFrame(const std::vector<unsigned char>& data);
    int SendFirstFrame(const std::vector<unsigned char>& data);
    int SendConsecutiveFrame(const std::vector<unsigned char>& data, unsigned char sequenceNumber);
    int SendFlowControl(unsigned char flag, unsigned char blockSize, unsigned char separationTime);
};
}
