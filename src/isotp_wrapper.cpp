/******************************************************************************
 * Name        : isotp_wrapper.cpp
 * Author      : shashiwakale
 * Version     : 1.0.0
 * Copyright   : Copyright (c) 2024 shashiwakale
 * Description : ISOTP Wrapper Implementation
*****************************************************************************/

/******************************************************************************
 * OS Includes
 *****************************************************************************/
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sys/time.h>

/******************************************************************************
 * Local Includes
 *****************************************************************************/
#include "isotp_wrapper.h"

/******************************************************************************
 * Defines
 *****************************************************************************/

/******************************************************************************
 * Global Variables
 *****************************************************************************/
socketcan::SocketCAN* isotp::isotp15765::canHandle = nullptr;

/******************************************************************************
 * ISOTP Library Specific User Implemented Functions
 *****************************************************************************/
/* user implemented, print debug message */
void isotp_user_debug(const char *message, ...)
{
    spdlog::debug("{}", message);
}

/* user implemented, send can message. should return ISOTP_RET_OK when success.
 */
int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t *data, const uint8_t size)
{
    spdlog::debug("Sending can message with ID 0x{0:x}", arbitration_id);

    if(nullptr != isotp::isotp15765::canHandle)
    {
        isotp::isotp15765::canHandle->SendFrame(arbitration_id, data, size);
    }
    else
    {
        spdlog::error("CAN handle is null..");
        return ISOTP_RET_ERROR;
    }

    return ISOTP_RET_OK;
}

/* user implemented, get millisecond */
uint32_t isotp_user_get_ms(void)
{
    struct timeval cateq_v;
    gettimeofday(&cateq_v, nullptr);
    return (uint32_t)(((1000.0 * (double)(cateq_v.tv_sec)) + (double)(cateq_v.tv_usec)) / 1000.0);
}

/******************************************************************************
 * ISOTP Implementation
 *****************************************************************************/
namespace isotp
{

const std::string VERSION       = "1.0.0";
const std::string AUTHOR        = "Shashikant Wakale";
const std::string COPY_RIGHT    = "Copyright (c) 2024 shashiwakale";
const std::string LICENSE       = "MIT License";

/*
 * Function Name            : isotp15765
 * Function ID              : -
 * Description              : isotp constructor
 * Author                   : shashiwakale
 * Date                     : 25-11-2023
 * Global Arguments Modified: srcID, destID, canHandle, pollingThreadThread, canReceiveThread
 * Global Arguments Refer   : -
 * Arguments                : socketCAN - Socket CAN Pointer
 *                            srcAddr - client CAN identifier
 *                            destAddr - server CAN identifier
 *                            ext - extended CAN id flag (true if extended)
 * Return Value             : -
 */
isotp15765::isotp15765(socketcan::SocketCAN *socketCAN, unsigned int srcAddr, unsigned int destAddr, bool ext) :
         srcID(srcAddr), destID(destAddr)
{
    spdlog::info("Starting ISOTP 15765_2 v{}", VERSION);
    spdlog::info("Author: {}", AUTHOR);
    spdlog::info("{}", LICENSE);
    spdlog::info("{}", COPY_RIGHT);

    this->canHandle = socketCAN;

    if(ext)
    {
        srcID = srcAddr | CAN_EFF_FLAG;
        destID = destAddr | CAN_EFF_FLAG;
        spdlog::info("Source Address: 0x{0:x}", srcID);
        spdlog::info("Destination Address: 0x{0:x} ", destID);
    }

    /* Initialize link, srcID is the CAN ID you send with */
    isotp_init_link(&g_link, srcID, g_isotpSendBuf, sizeof(g_isotpSendBuf), g_isotpRecvBuf, sizeof(g_isotpRecvBuf));

    pollingThreadThread = std::thread(&isotp15765::poolingFunction,this);
    canReceiveThread = std::thread(&isotp15765::ReceiveThreadFunction, this);

}

/*
 * Function Name            : ~isotp15765
 * Function ID              : -
 * Description              : Destructor
 * Author                   : shashiwakale
 * Date                     : 25-11-2023
 * Global Arguments Modified: exitThread
 * Global Arguments Refer   : canReceiveThread
 * Arguments                : -
 * Return Value             : int
 */
isotp15765::~isotp15765()
{
    exitThread = true;
    if(canReceiveThread.joinable())
        canReceiveThread.join();
}

/*
 * Function Name            : ReceiveThreadFunction
 * Function ID              : -
 * Description              : Function to receive CAN frames
 * Author                   : shashiwakale
 * Date                     : 25-11-2023
 * Global Arguments Modified: -
 * Global Arguments Refer   : exitThread, canHandle, destID, g_link,
 *                            receivedPayload, receivedPayloadSize, messageQueue, cv
 * Arguments                : -
 * Return Value             : -
 */
void isotp15765::ReceiveThreadFunction(void)
{
    struct can_frame rxFrame;
    while(!exitThread)
    {
        if(nullptr != canHandle && canHandle->ReceiveFrame(rxFrame) < 0)
        {
            std::cerr<<"CAN receive error..\n";
        }
        else
        {
            if(destID == rxFrame.can_id)
            {
                /*Sleep to avoid quick reply*/
                std::this_thread::sleep_for (std::chrono::microseconds(100));
                /*Pass received CAN frame to TP*/
                isotp_on_can_message (&g_link, rxFrame.data, rxFrame.can_dlc);

                /*
                 * You can receive message with isotp_receive.payload is upper layer message buffer, usually UDS;
                 * payload_size is payload buffer size;
                 * out_size is the actual read size;
                 */
                int ret = isotp_receive (&g_link, receivedPayload, ISOTP_BUFSIZE, &receivedPayloadSize);

                if(ISOTP_RET_OK == ret && receivedPayload[0] == 0x7F && receivedPayload[2] == 0x78)
                {
                    /*Server is busy wait for next frame*/
                }
                else if(ISOTP_RET_OK == ret)
                {
                    std::vector<unsigned char> data;
                    /* Handle received message */
                    for(int i = 0; i < receivedPayloadSize; i++)
                    {
                        data.push_back(receivedPayload[i]);
                    }
                    std::unique_lock<std::mutex> lk(m);
                    messageQueue.push(data);
                    cv.notify_one();
                }
            }
        }
    }
}

/*
 * Function Name            : GetCANMessage
 * Function ID              : -
 * Description              : Function to get isotp message synchronously
 * Author                   : shashiwakale
 * Date                     : 25-11-2023
 * Global Arguments Modified: messageQueue
 * Global Arguments Refer   : m, cv
 * Arguments                : waitDuration - duration to wait for message
 * Return Value             : vector<int> receivedData
 */
std::vector<unsigned char> isotp15765::GetCANMessage(const long waitDuration)
{
    std::vector<unsigned char> receivedData;

    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk,std::chrono::milliseconds (waitDuration), [this]{ return (!messageQueue.empty()); });

    if(!messageQueue.empty())
    {
        receivedData = messageQueue.front ();
        messageQueue.pop ();
    }
    else
    {
        /*Timeout*/
        spdlog::warn("ISOTP Timeout..");
    }

    return receivedData;
}

/*
 * Function Name            : send
 * Function ID              : -
 * Description              : Function to send isotp message
 * Author                   : shashiwakale
 * Date                     : 25-11-2023
 * Global Arguments Modified: messageQueue
 * Global Arguments Refer   : m, cv
 * Arguments                : -
 * Return Value             : isotp_message
 */
isotp_message isotp15765::send(const std::vector<uint8_t>& data, bool wait)
{
    isotp_message responseMessage;
    /* And send message with isotp_send */
    int ret = isotp_send(&g_link, data.data(), data.size());
    if (ISOTP_RET_OK == ret) {
        /* Send ok */
        spdlog::info("Send Ok");
    } else {
        /* An error occured */
        spdlog::error("Send error");
    }

    if(wait)
        responseMessage.data = GetCANMessage(1000);

    return responseMessage;
}

/*
 * Function Name            : receive
 * Function ID              : -
 * Description              : Function to receive isotp message asynchronously
 * Author                   : shashiwakale
 * Date                     : 25-11-2023
 * Global Arguments Modified: -
 * Global Arguments Refer   : -
 * Arguments                : -
 * Return Value             : isotp_message
 */
isotp_message isotp15765::receive()
{
    isotp_message responseMessage;
    responseMessage.data = GetCANMessage(1000);
    return responseMessage;
}

}

