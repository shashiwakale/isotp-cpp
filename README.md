# isotp-cpp

#### This project is inspired by [lishen2/isotp-c](https://github.com/lishen2/isotp-c) and implementes CPP wrapper on it.

## ![test status](https://github.com/shashiwakale/isotp-cpp/actions/workflows/c-cpp.yml/badge.svg)

## Build Instruction

```bash
$ git clone https://github.com/shashiwakale/socket-can-wrapper-cpp.git
$ cd socket-can-wrapper-cpp
$ make
```

## Add Virtual CAN Interface in Linux

```bash
$ sudo ip link add vcan0 type vcan
$ sudo ip link set vcan0 up
```

## Run the binary

```bash
$ ./isotp_sample
```
## API Usage

### Send isotp frame
```cpp
#include "socket_can.h"
#include "isotp_wrapper.h"

int main(int argc, char** argv)
{
    /*Open CAN Channel*/
    socketcan::SocketCAN* sock = socketcan::SocketCAN::OpenChannel("vcan0", 500000);

    /*
     * Create isotp object
     * sock - socket can handler
     * 0x18DA00FA - Source address
     * 0x18DAFA00 - Destination address
     * true - Extended CAN ID
     */
    isotp::Iisotp15765* isotp = new isotp::isotp15765(sock, 0x18DA00FA, 0x18DAFA00, true);

    /*Data*/
    std::vector<unsigned char> data = { 0, 1, 2, 3, 4, 5, 6 };

    isotp::isotp_message responseMessage;

    /*Send Data*/
    responseMessage = isotp->send(data);

    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }

    /*Send data more than 8 bytes*/
    std::vector<unsigned char> data2 (512, 0xFF);
    responseMessage = isotp->send (data2);

    for(auto byte : responseMessage.data)
    {
        std::cout << std::hex << (int) byte << " ";
    }

    delete isotp;
    delete sock;
}
```

### Receive isotp frame asynchronously
```cpp
#include "socket_can.h"
#include "isotp_wrapper.h"

int main(int argc, char** argv)
{
    /*Open CAN Channel*/
    socketcan::SocketCAN* sock = socketcan::SocketCAN::OpenChannel("vcan0", 500000);

    /*
     * Create isotp object
     * sock - socket can handler
     * 0x18DA00FA - Source address
     * 0x18DAFA00 - Destination address
     * true - Extended CAN ID
     */
    isotp::Iisotp15765* isotp = new isotp::isotp15765(sock, 0x18DA00FA, 0x18DAFA00, true);

    isotp::isotp_message responseMessage;

    while(1)
    {
        /*Receive isotp message asynchronoulsy*/
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
}
```

## License
[MIT](https://github.com/shashiwakale/isotp-cpp/blob/main/LICENSE)