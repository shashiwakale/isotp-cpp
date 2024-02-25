/******************************************************************************
 * Name        : main.cpp
 * Author      : shashiwakale
 * Version     : 1.0.0
 * Copyright   : Copyright (c) 2024 shashiwakale
 * Description : ISOTP Sample Application
*****************************************************************************/

/******************************************************************************
 * OS Includes
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <iostream>

/******************************************************************************
 * Local Includes
 *****************************************************************************/
#include "spdlog/spdlog.h"
#include "socket_can.h"
#include "isotp_wrapper.h"

/******************************************************************************
 * Defines
 *****************************************************************************/

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/*
 * Linux command to add virtual CAN interface
 * sudo ip link add vcan0 type vcan
 * sudo ip link set vcan0 up
 */

/*
 * Function Name            : main
 * Function ID              : -
 * Description              : Entry Function
 * Author                   : shashiwakale
 * Date                     : 02-25-2024
 * Global Arguments Modified: -
 * Global Arguments Refer   : -
 * Arguments                : -
 * Return Value             : int
 */
int main(int, char**)
{
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
    std::cout<<std::endl;

    responseMessage = isotp->send (data1);

    spdlog::info("Message: ");
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }
    std::cout<<std::endl;

    responseMessage = isotp->send (data2);
    spdlog::info("Message: ");
    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }
    std::cout<<std::endl;

    while(1)
    {
        responseMessage  = isotp->receive();

        for(auto byte : responseMessage.data)
        {
            std::cout << std::hex << (int) byte << " ";
        }
        std::cout<<std::endl;
        std::this_thread::sleep_for (std::chrono::milliseconds (100));;
    }

    delete isotp;
    delete sock;

    return 0;
}
