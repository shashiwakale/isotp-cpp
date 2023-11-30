/*
 * socket_can.cpp
 *
 *  Description  :
 *  Created on   : 30-Apr-2023
 *      Author   : shashi
 *  Organization : 
 */

#include "../include/socket_can.h"

namespace socketcan
{

SocketCAN* SocketCAN::socketCAN = nullptr;
std::mutex SocketCAN::mutex;

SocketCAN::SocketCAN(const std::string& p_ref_strInterface, long p_lnBaudRate)
{
    if ((sc = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Socket");
    }
    else
    {
        strcpy(ifr.ifr_name, p_ref_strInterface.c_str());
        ioctl(sc, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sc, (struct sockaddr*) &addr, sizeof(addr)) < 0)
        {
            perror("Bind");
        }
    }

}

SocketCAN::~SocketCAN()
{
    if (close(sc) < 0)
    {
        perror("Close");
    }
}

SocketCAN* SocketCAN::OpenChannel(const std::string& interface, long baudRate)
{
    std::lock_guard<std::mutex> lock(mutex);
    if(nullptr == socketCAN)
        socketCAN = new SocketCAN(interface, baudRate);

    return socketCAN;
}

int SocketCAN::SendFrame(int identifier, unsigned char* data, int length)
{
    if(8 < length)
    {
        perror("Frame length is greater than 8 bytes");
        return 1;
    }

    txFrame.can_id = identifier;
    txFrame.can_dlc = length;
    memcpy((char*)txFrame.data, data, length);

    if (write(sc, &txFrame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return 1;
    }

    return 0;
}

int SocketCAN::ReceiveFrame(struct can_frame& rxFrame)
{
    int nbytes = read(sc, &rxFrame, sizeof(struct can_frame));

    if (nbytes < 0)
    {
        perror("Read");
        return 1;
    }

    return nbytes;
}

} /* namespace socketcan */
