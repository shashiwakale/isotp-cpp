/*
 * isotp_wrapper.h
 *
 *  Created on: 06-Dec-2023
 *      Author: shash
 */

#pragma once

#include "Iisotp.h"

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
#include "isotp.h"
#include "Iisotp.h"
#include "isotp_wrapper_defines.h"
#include "spdlog/spdlog.h"
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
    isotp_message send(const std::vector<uint8_t>& data, bool wait = true) override;
    isotp_message receive(void) override;
    static socketcan::SocketCAN* canHandle;

private:
    std::mutex m;
    unsigned int srcID;
    unsigned int destID;
    bool exitThread = false;
    std::condition_variable cv;
    std::thread canReceiveThread;
    std::thread pollingThreadThread;
    std::queue<std::vector<unsigned char>> messageQueue;

    void poolingFunction()
    {
        spdlog::debug("starting polling thread...");
        while(1)
        {
            /* Poll link to handle multiple frame transmition */
            isotp_poll(&g_link);
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }
    }

    /*isotp defines*/
    /* Alloc IsoTpLink statically in RAM */
    IsoTpLink g_link;

    /* Alloc send and receive buffer statically in RAM */
    uint8_t g_isotpRecvBuf[ISOTP_BUFSIZE];
    uint8_t g_isotpSendBuf[ISOTP_BUFSIZE];
    uint8_t receivedPayload[ISOTP_BUFSIZE];
    uint16_t receivedPayloadSize = 0;

    void ReceiveThreadFunction(void);
    std::vector<unsigned char> GetCANMessage(const long waitDuration=1000);
};
}
