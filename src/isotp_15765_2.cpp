/*
 * isotp_15765_2.cpp
 *
 *  Created on: 25-Nov-2023
 *      Author: shashi
 */

#include <iostream>
#include <chrono>

#include "isotp_15765_2.h"

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
        canHandle (socketCAN), srcID(srcAddr), destID(destAddr)
{
    std::cout<<"[isotp]: Starting ISOTP 15765_2 v"<<VERSION<<std::endl;
    std::cout<<"[isotp]: Author: "<<AUTHOR<<std::endl;
    std::cout<<"[isotp]: "<<LICENSE<<std::endl;
    std::cout<<"[isotp]: "<<COPY_RIGHT<<std::endl;
    std::cout<<"[isotp]: "<<LICENSE_INFO<<std::endl;

    if(ext)
    {
        srcID = srcID | CAN_EFF_FLAG;
        destAddr = destAddr | CAN_EFF_FLAG;
        extendedID = true;
    }

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
            if(extendedID)
            {
                /*To get the ID only without flags*/
                rxFrame.can_id = rxFrame.can_id & CAN_EFF_MASK;
            }

            if(destID == rxFrame.can_id)
            {
                /*Check CAN Frame*/
                std::unique_lock<std::mutex> lk(m);
                messageQueue.push(rxFrame);
                cv.notify_one();
            }
        }
    }
}

std::vector<unsigned char> isotp15765::GetCANMessage(const long waitDuration)
{
    std::vector<unsigned char> receivedData;
    bool stopReception = false;
    int bytesToBeReceived = 0;
    int frameSize = 8;
    int numberOfBytesReceived = FIRST_FRAME_MAX_LENGTH;

    while(!stopReception)
    {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk,std::chrono::milliseconds (waitDuration), [this]{ return (!messageQueue.empty()); });

        if(!messageQueue.empty())
        {
            struct can_frame message = messageQueue.front ();
            messageQueue.pop ();

            unsigned char frameType = message.data[0] & FIRST_NIBBLE_MASK;

            switch(frameType)
            {
                case SINGLE_FRAME:
                    bytesToBeReceived = (message.data[0] & SECOND_NIBBLE_MASK)+1;

                    if(message.data[1] == NEGATIVE_RESPONSE_FRAME && message.data[3] == NRC_RESPONSE_PENDING)
                    {
                        /*Wait for response*/
                    }
                    else
                    {
                        for(int i = 1; i < bytesToBeReceived; i++)
                        {
                            receivedData.push_back (message.data[i]);
                        }
                        stopReception = true;
                    }
                    break;
                case FIRST_FRAME:
                    /*Get number of bytes to be received*/
                    bytesToBeReceived = ((message.data[0] & SECOND_NIBBLE_MASK) | message.data[1]);
                    for(int i = 2; i < 8; i++)
                    {
                        receivedData.push_back (message.data[i]);
                    }
                    /*Send control frame*/
                    SendFlowControl(CTS, SEND_ALL_BLKS_WO_INT, DEFAULT_SEPARATION_TIME);
                    break;
                case CONSECUTIVE_FRAME:
                    /*Check frame counter*/
                    if(numberOfBytesReceived < bytesToBeReceived)
                    {
                        int remainingBytes = (bytesToBeReceived-numberOfBytesReceived);
                        if(remainingBytes < 8)
                        {
                            frameSize = remainingBytes + 1;
                            stopReception = true;
                        }

                        for(int i = 1; i < frameSize; i++)
                        {
                            receivedData.push_back (message.data[i]);
                            numberOfBytesReceived++;
                        }
                    }
                    else
                    {
                        numberOfBytesReceived = 0;
                        bytesToBeReceived = 0;
                        stopReception = true;
                    }
                    break;
                case FLOW_CONTROL_FRAME:
                    /*Send consecutive frames*/
                    for(int i = 0; i < 8; i++)
                    {
                        receivedData.push_back (message.data[i]);
                    }
                    stopReception = true;
                    break;
                default:
                    stopReception = true;
                    break;
            }
        }
        else
        {
            /*Timeout*/
            std::cerr<< "[isotp]: ISOTP Timeout..\n";
            stopReception = true;
        }
    }

    return receivedData;
}

isotp_message isotp15765::isotp_send(const std::vector<unsigned char>& data, bool wait)
{
    isotp_message responseMessage;

    std::unique_lock<std::mutex> protect(receiveFuncProtection);

    if(!wait)
        protect.unlock();

    if(data.size () > ISOTP_MAX_DATA_LENGTH)
    {
        responseMessage.responseCode = INVALID_LENGTH;
    }
    else if(data.size () > SINGLE_FRAME_MAX_LENGTH)
    {
        /*Send First Frame*/
        SendFirstFrame(data);
        std::vector<unsigned char> localData = data;

        if(SUCCESS == SendConsecutiveData(localData))
        {
            if(wait)
                responseMessage.data = GetCANMessage();
        }
        else
        {
            responseMessage.responseCode = SEND_ERROR;
        }
    }
    else
    {
        /*Send Single Frame*/
        SendSingleFrame(data);

        if(wait)
            /*Block Here for response*/
            responseMessage.data = GetCANMessage();
    }

    if(!responseMessage.data.empty())
         responseMessage.responseCode = SUCCESS;
     else
         responseMessage.responseCode = SEND_ERROR;

    if(!wait)
        responseMessage.responseCode = SUCCESS;

    return responseMessage;
}

isotp_message isotp15765::isotp_receive(void)
{
    isotp_message responseMessage;
    std::unique_lock<std::mutex> protect(receiveFuncProtection);
    /*Block Here for response*/
    responseMessage.data = GetCANMessage();
    responseMessage.responseCode = SUCCESS;
    return responseMessage;
}

int isotp15765::SendConsecutiveData(std::vector<unsigned char>& data)
{
    int returnCode = SUCCESS;
    bool stopSending = false;
    unsigned int totalBlockSent = 0;
    unsigned char flag = 10;
    unsigned int blockToBeSent = (data.size () / CONSECUTIVE_FRAME_MAX_LENGTH);

    /*Erase first 6 Bytes*/
    data.erase(data.begin(), data.begin()+FIRST_FRAME_MAX_LENGTH);

    /*Lambda function to send consecutive blocks*/
    auto SendConsecutiveBlock = [&data, &totalBlockSent, this](int totalBlocks, unsigned char separationTime)
    {
        unsigned char sequenceNumber = 1;

        for(int blockNumber = 0; blockNumber < totalBlocks; blockNumber++)
        {
            SendConsecutiveFrame (data, sequenceNumber);

            if(data.size () > CONSECUTIVE_FRAME_MAX_LENGTH)
            {
                data.erase (data.begin (), data.begin () + CONSECUTIVE_FRAME_MAX_LENGTH);
            }
            else
            {
                data.erase (data.begin (), data.end ());
            }

            /*Reset sequence number*/
            if(sequenceNumber >= 0x0F)
                sequenceNumber = 0;
            else
                sequenceNumber++;

            /*Separation Time*/
            std::this_thread::sleep_for (std::chrono::milliseconds (separationTime));
            totalBlockSent++;
        }
        return totalBlockSent;
    };

    while(!stopSending)
    {
        /*Block Here for Flow Control*/
        std::vector<unsigned char> response = GetCANMessage();

        try
        {
            if(response.empty())
            {
                stopSending = true;
                returnCode = SEND_ERROR;
            }
            else if((response.at(0) & FIRST_NIBBLE_MASK) != FLOW_CONTROL_FRAME)
            {
                flag = 11;
                returnCode = SEND_ERROR;
            }
            else
            {
                flag = response.at (0) & SECOND_NIBBLE_MASK;
            }
        }
        catch(const std::out_of_range& err){
            std::cout<<"[isotp]: exception, out of range response: "<<err.what()<<std::endl;
        }

        if(CTS == flag)
        {
            unsigned char blockSize = 0;
            unsigned char separationTime = DEFAULT_SEPARATION_TIME;

            try{
                blockSize = response.at (1);
                separationTime = response.at (2);
            }
            catch(const std::out_of_range& err){
                std::cout<<"[isotp]: exception, out of range response: "<<err.what()<<std::endl;
            }

            unsigned int totalBlocks = (blockToBeSent - totalBlockSent);

            if(blockSize != SEND_ALL_BLKS_WO_INT)
            {
                totalBlocks = blockSize;
            }

            totalBlockSent = SendConsecutiveBlock(totalBlocks, separationTime);

            if(totalBlockSent >= blockToBeSent)
            {
                stopSending = true;
            }
        }
        else if(WAIT == flag)
        {
            /*Do nothing wait for next flow control*/
        }
        else if(OVFLW == flag)
        {
            /*Overflow, abort the operation*/
            returnCode = SEND_ERROR;
            stopSending = true;
        }
        else if(11 == flag)
        {
            std::cout<<"Unwanted frame received..";
        }
        else
        {
            /*Reserve flags*/
            returnCode = SEND_ERROR;
            stopSending = true;
        }
    }

    return returnCode;
}

int isotp15765::SendSingleFrame(const std::vector<unsigned char>& data)
{
    unsigned char dataBytes[8] = {0};
    dataBytes[0] = SINGLE_FRAME | (unsigned char)data.size();
    /*Copy Data*/
    memcpy(&dataBytes[1], data.data(), data.size());

    if(canHandle->SendFrame(srcID, dataBytes, sizeof(dataBytes)))
    {
        return SEND_ERROR;
    }
    else
    {
        return SUCCESS;
    }
}

int isotp15765::SendFirstFrame(const std::vector<unsigned char>& data)
{
    unsigned char dataBytes[8] = {0};
    dataBytes[0] = FIRST_FRAME | ((data.size()>>8) & SECOND_NIBBLE_MASK);
    dataBytes[1] = data.size() & BYTE_MASK;
    /*Copy Data*/
    memcpy(&dataBytes[2], data.data(), FIRST_FRAME_MAX_LENGTH);

    if(canHandle->SendFrame(srcID, dataBytes, sizeof(dataBytes)))
    {
        return SEND_ERROR;
    }
    else
    {
        return SUCCESS;
    }
}

int isotp15765::SendConsecutiveFrame(const std::vector<unsigned char>& data, unsigned char sequenceNumber)
{
    unsigned char dataBytes[8] = {0};
    int frameSize = CONSECUTIVE_FRAME_MAX_LENGTH;
    if(data.size()< CONSECUTIVE_FRAME_MAX_LENGTH)
        frameSize = data.size();

    dataBytes[0] = CONSECUTIVE_FRAME | (sequenceNumber & SECOND_NIBBLE_MASK);
    /*Copy Data*/
    memcpy(&dataBytes[1], data.data(), frameSize);

    if(canHandle->SendFrame(srcID, dataBytes, sizeof(dataBytes)))
    {
        return SEND_ERROR;
    }
    else
    {
        return SUCCESS;
    }
    return SUCCESS;
}

int isotp15765::SendFlowControl(unsigned char flag, unsigned char blockSize, unsigned char separationTime)
{
    unsigned char dataBytes[8] = {0};
    dataBytes[0] = FLOW_CONTROL_FRAME | (flag & SECOND_NIBBLE_MASK);
    dataBytes[1] = blockSize & BYTE_MASK;
    dataBytes[2] = separationTime & BYTE_MASK;

    if(canHandle->SendFrame(srcID, dataBytes, sizeof(dataBytes)))
    {
        return SEND_ERROR;
    }
    else
    {
        return SUCCESS;
    }
    return SUCCESS;
}

}

