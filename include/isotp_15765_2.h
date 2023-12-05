/*
 * isotp_15765_2.h
 *
 *  Created on: 25-Nov-2023
 *      Author: shashi
 */

#pragma once

/******************************************************************************
 * OS includes
 *****************************************************************************/
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
/******************************************************************************
 * Local includes
 *****************************************************************************/
#include "socket_can.h"
#include "isotp_defines.h"
#include "Iisotp_15765.h"
/******************************************************************************
 * Defines
 *****************************************************************************/

/******************************************************************************
 * Global variables
 *****************************************************************************/

/******************************************************************************
 * Classes
 *****************************************************************************/
namespace isotp
{

class isotp15765 : public Iisotp15765
{
public:
    isotp15765(socketcan::SocketCAN* socketCAN, unsigned int srcAddr, unsigned int destAddr, bool ext = false);
    ~isotp15765();
    isotp_message isotp_send(const std::vector<unsigned char>& data, bool wait = true) override;
    isotp_message isotp_receive(void) override;

private:
    std::mutex m;
    std::mutex receiveFuncProtection;
    socketcan::SocketCAN* canHandle = nullptr;
    unsigned int srcID;
    unsigned int destID;
    bool extendedID = false;
    bool exitThread = false;
    std::condition_variable cv;
    std::thread canReceiveThread;
    std::queue<struct can_frame> messageQueue;

    void ReceiveThreadFunction(void);
    std::vector<unsigned char> GetCANMessage(const long waitDuration=WAIT_TIME);
    int SendConsecutiveData(std::vector<unsigned char>& data);
    int SendSingleFrame(const std::vector<unsigned char>& data);
    int SendFirstFrame(const std::vector<unsigned char>& data);
    int SendConsecutiveFrame(const std::vector<unsigned char>& data, unsigned char sequenceNumber);
    int SendFlowControl(unsigned char flag, unsigned char blockSize, unsigned char separationTime);
};
}
