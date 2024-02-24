/*
 * isotp_15765_2.cpp
 *
 *  Created on: 25-Nov-2023
 *      Author: shashi
 */

#include <iostream>
#include <chrono>
#include <iomanip>
#include <sys/time.h>

#include "isotp_wrapper.h"

socketcan::SocketCAN* isotp::isotp15765::canHandle = nullptr;

/* user implemented, print debug message */
void isotp_user_debug(const char *message, ...)
{
    std::cout<< message<<std::endl;
}

/* user implemented, send can message. should return ISOTP_RET_OK when success.
 */
int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t *data, const uint8_t size)
{
//    std::cout<<"Sending can message with ID "<<std::hex<<arbitration_id<<std::endl;
//    for(int i = 0; i < size; i++)
//    {
//        std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<(int)data[i]<<" ";
//    }
//    std::cout<<std::endl;

    if(nullptr != isotp::isotp15765::canHandle)
    {
        isotp::isotp15765::canHandle->SendFrame(arbitration_id, data, size);
    }
    else
    {
        std::cerr<<"CAN handle is null..\n";
    }

    return 0;
}

/* user implemented, get millisecond */
uint32_t isotp_user_get_ms(void)
{
    struct timeval cateq_v;
    gettimeofday(&cateq_v, nullptr);
    return (uint32_t)(1000.0 * (double)(cateq_v.tv_sec) + (double)(cateq_v.tv_usec) / 1000.0);
}

namespace isotp
{

const std::string VERSION       = "1.0.0";
const std::string AUTHOR        = "Shashikant Wakale";
const std::string COPY_RIGHT    = "Copyright (c) 2023 shashiwakale";
const std::string LICENSE       = "MIT License";
const std::string LICENSE_INFO = "THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR\n\
         IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,\n\
         FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE\n\
         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER\n\
         LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,\n\
         OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE\n\
         SOFTWARE.";


isotp15765::isotp15765(socketcan::SocketCAN *socketCAN, unsigned int srcAddr, unsigned int destAddr, bool ext) :
         srcID(srcAddr), destID(destAddr)
{
    std::cout<<"[isotp]: Starting ISOTP 15765_2 v"<<VERSION<<std::endl;
    std::cout<<"[isotp]: Author: "<<AUTHOR<<std::endl;
    std::cout<<"[isotp]: "<<LICENSE<<std::endl;
    std::cout<<"[isotp]: "<<COPY_RIGHT<<std::endl;
    std::cout<<"[isotp]: "<<LICENSE_INFO<<std::endl;

    this->canHandle = socketCAN;

    if(ext)
    {
        srcID = srcAddr | CAN_EFF_FLAG;
        destID = destAddr | CAN_EFF_FLAG;
        extendedID = true;
    }

    /* Initialize link, srcID is the CAN ID you send with */
    isotp_init_link(&g_link, srcID, g_isotpSendBuf, sizeof(g_isotpSendBuf), g_isotpRecvBuf, sizeof(g_isotpRecvBuf));

    pollingThreadThread = std::thread(&isotp15765::poolingFunction,this);
    canReceiveThread = std::thread(&isotp15765::ReceiveThreadFunction, this);

}

isotp15765::~isotp15765()
{
    exitThread = true;
    if(canReceiveThread.joinable())
        canReceiveThread.join();
}

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
//            if(extendedID)
//            {
//                /*To get the ID only without flags*/
//                rxFrame.can_id = rxFrame.can_id & CAN_EFF_MASK;
//            }
//            std::cout<<"[Rx]"<<std::hex<<rxFrame.can_id<<"-> "<<destID<< " ";
//            for(int i = 0; i < rxFrame.can_dlc; i++)
//            {
//                std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<(int)rxFrame.data[i]<<" ";
//            }
//            std::cout<<std::endl;

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
        std::cerr<< "[isotp]: ISOTP Timeout..\n";
    }

    return receivedData;
}

isotp_message isotp15765::send(const std::vector<uint8_t>& data, bool wait)
{
    isotp_message responseMessage;
    /* And send message with isotp_send */
    int ret = isotp_send(&g_link, data.data(), data.size());
    if (ISOTP_RET_OK == ret) {
        /* Send ok */
        std::cout<<"Send Ok\n";
    } else {
        /* An error occured */
        std::cout<<"Send error\n";
    }

    if(wait)
        responseMessage.data = GetCANMessage(1000);

    return responseMessage;
}

isotp_message isotp15765::receive()
{
    isotp_message responseMessage;
    responseMessage.data = GetCANMessage(1000);
    return responseMessage;
}

}

