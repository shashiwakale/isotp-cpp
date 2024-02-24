#include <stdio.h>
#include <string.h>
#include <chrono>
#include <iostream>
#include "socket_can.h"

#include "isotp_wrapper.h"
/*
 * sudo ip link add vcan0 type vcan
 * sudo ip link set vcan0 up
 */

int main(int, char**)
{
    socketcan::SocketCAN *sock = socketcan::SocketCAN::OpenChannel ("vcan0", 500000);

    //isotp::Iisotp15765 *isotp = new isotp::isotp15765(sock, 0x18DA00FB, 0x18DAFB00, true);
    isotp::Iisotp15765 *isotp = new isotp::isotp15765(sock, 0x18DAFB00, 0x18DA00FB, true);

    std::vector<unsigned char> data =
        { 0, 1, 2, 3, 4, 5, 6 };
    std::vector<unsigned char> data1 =
        { 0, 1, 2, 3 };
    std::vector<unsigned char> data2 (512, 0xFF);

    isotp::isotp_message responseMessage;
    responseMessage = isotp->send (data);
    std::cout<<"Message: ";
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }
    std::cout << "\n";

    responseMessage = isotp->send (data1);
    std::cout<<"Message: ";
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }
    std::cout << "\n";

    responseMessage = isotp->send (data2);
    std::cout<<"Message: ";
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }
    std::cout << "\n";

    while(1)
    {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));;
    }

    return 0;
}
