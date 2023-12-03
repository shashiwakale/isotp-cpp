#include <stdio.h>
#include <string.h>
#include <chrono>
#include <iostream>
#include "socket_can.h"

#include "isotp_15765_2.h"
/*
 * sudo ip link add vcan0 type vcan
 * sudo ip link set vcan0 up
 */

int main(int, char**)
{
    socketcan::SocketCAN *sock = socketcan::SocketCAN::OpenChannel ("vcan0", 500000);

    isotp::Iisotp15765 *isotp = new isotp::isotp15765(sock, 0x7FF, 0x123);

    std::vector<unsigned char> data =
        { 0, 1, 2, 3, 4, 5, 6 };
    std::vector<unsigned char> data1 =
        { 0, 1, 2, 3 };
    std::vector<unsigned char> data2 (512, 0xFF);

    isotp::isotp_message responseMessage;
    responseMessage = isotp->isotp_send (data);
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }
    std::cout << "\n";

    responseMessage = isotp->isotp_send (data1);
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }
    std::cout << "\n";

    responseMessage = isotp->isotp_send (data2);
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }
    std::cout << "\n";

    return 0;
}