/*
 * socket_can.h
 *
 *  Description  :
 *  Created on   : 30-Apr-2023
 *      Author   : shashi
 *  Organization : 
 */

#ifndef INCLUDE_SOCKET_CAN_H_
#define INCLUDE_SOCKET_CAN_H_

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <mutex>

namespace socketcan
{

class SocketCAN
{
public:
    virtual ~SocketCAN();
    SocketCAN(SocketCAN &other) = delete;
    void operator=(const SocketCAN &) = delete;
    static SocketCAN* OpenChannel(const std::string& interface, long baudRate);
    int SendFrame(int identifier, const uint8_t* data, int length);
    int ReceiveFrame(struct can_frame& rxFrame);

protected:
    SocketCAN(const std::string& p_ref_strInterface, long p_lnBaudRate);

private:
    int sc;
    struct ifreq ifr;
    struct sockaddr_can addr;
    struct can_frame txFrame;
    static SocketCAN* socketCAN;
    static std::mutex mutex;
};

} /* namespace socketcan */

#endif /* INCLUDE_SOCKET_CAN_H_ */
