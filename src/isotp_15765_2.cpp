/*
 * isotp_15765_2.cpp
 *
 *  Created on: 25-Nov-2023
 *      Author: shashi
 */

#include <iostream>
#include <chrono>

#include "isotp_15765_2.h"
//#include "spdlog/spdlog.h"

namespace isotp
{

isotp15765::isotp15765(socketcan::SocketCAN *socketCAN, unsigned long srcAddr, unsigned long destAddr) :
        canHandle (socketCAN), srcID(srcAddr), destID(destAddr)
{
    //spdlog::set_level(spdlog::level::debug);
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
            //spdlog::error("CAN receive error..");
        }
        else
        {
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

std::vector<unsigned char> isotp15765::GetCANMessage(void)
{
    std::vector<unsigned char> receivedData;
    bool stopReception = false;
    int bytesToBeReceived = 0;
    int frameSize = 8;
    int numberOfBytesReceived = FIRST_FRAME_MAX_LENGTH;

    while(!stopReception)
    {
        std::unique_lock<std::mutex> lk(m);
        cv.wait(lk, [this]{ return (!messageQueue.empty()); });

        if(!messageQueue.empty())
        {
            struct can_frame message = messageQueue.front ();
            messageQueue.pop ();

            unsigned char frameType = message.data[0] & FIRST_NIBBLE_MASK;

            switch(frameType)
            {
                case SINGLE_FRAME:
                    bytesToBeReceived = (message.data[0] & SECOND_NIBBLE_MASK)+1;
                    for(int i = 1; i < bytesToBeReceived; i++)
                    {
                        receivedData.push_back (message.data[i]);
                    }
                    stopReception = true;
                    bytesToBeReceived = 0;
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
        }
    }

    return receivedData;
}

isotp_message isotp15765::isotp_send(const std::vector<unsigned char>& data)
{
    isotp_message responseMessage;

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
            responseMessage.data = GetCANMessage();
            responseMessage.responseCode = SUCCESS;
        }
    }
    else
    {
        /*Send Single Frame*/
        SendSingleFrame(data);

        /*Block Here for response*/
        responseMessage.data = GetCANMessage();
        responseMessage.responseCode = SUCCESS;
    }

    return responseMessage;
}

int isotp15765::SendConsecutiveData(std::vector<unsigned char>& data)
{
    int returnCode = SUCCESS;
    bool stopSending = false;
    unsigned int totalBlockSent = 0;
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
        unsigned char flag = response.at (0) & SECOND_NIBBLE_MASK;
        if(CTS == flag)
        {
            unsigned char blockSize = response.at (1);
            unsigned char separationTime = response.at (2);
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
    //spdlog::debug("Sending single frame..");
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
    //spdlog::debug("Sending first frame..");
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
    //spdlog::debug("Sending consecutive frame..");
    unsigned char dataBytes[8] = {0};
    int frameSize = CONSECUTIVE_FRAME_MAX_LENGTH;
    if(data.size()< CONSECUTIVE_FRAME_MAX_LENGTH)
        frameSize = data.size();

    dataBytes[0] = CONSECUTIVE_FRAME | (sequenceNumber & SECOND_NIBBLE_MASK);
    //spdlog::debug("Sending first frame..");
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
    //spdlog::debug("Sending flow control frame..");
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

