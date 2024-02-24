#include <stdio.h>
#include <string.h>
#include <chrono>
#include <iostream>

#include "spdlog/spdlog.h"
#include "socket_can.h"

#include "isotp_wrapper.h"
/*
 * sudo ip link add vcan0 type vcan
 * sudo ip link set vcan0 up
 */

int main(int, char**)
{

    spdlog::set_pattern("*** %H:%M:%S.%e %l ***: %v");

    spdlog::set_level(spdlog::level::debug);

    spdlog::debug("isotp-main in debug mode...");

    socketcan::SocketCAN *sock = socketcan::SocketCAN::OpenChannel ("vcan0", 500000);

    isotp::Iisotp15765 *isotp = new isotp::isotp15765(sock, 0x18DA00FA, 0x18DAFA00, true);

    std::vector<unsigned char> data =
        { 0xba, 1, 2, 3, 4, 5, 6 };
    std::vector<unsigned char> data1 =
        { 0xbb, 1, 2, 3 };
    std::vector<unsigned char> data2 (512, 0xFF);

    isotp::isotp_message responseMessage;
    responseMessage = isotp->send (data);

    spdlog::info("Message: ");
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }

    responseMessage = isotp->send (data1);

    spdlog::info("Message: ");
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }

    responseMessage = isotp->send (data2);
    spdlog::info("Message: ");
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }

    while(1)
    {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));;
    }

    return 0;
}
