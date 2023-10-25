//
// Created by waxz on 10/24/23.
//


#include "SocketManager.h"
#include "common/signals.h"
#include "common/functions.h"

#include <thread>
#include <iostream>
#include <atomic>

int main(int argc,char** argv){


    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    set_signal_handler(my_handler);


    sockpp::initialize();

    std::string host = "localhost";
    int port = 12345;

    const sockpp::tcp_connector::addr_t remote_addr{host, (in_port_t)port};

    SocketClient client;
    client.connect(remote_addr);

//    client.set_non_blocking();


    // send data
    char buf[512];
    for(int i = 0 ; i < 512;i++){
        buf[i] = i%100;
    }

    // loop
    while (program_run){
        std::this_thread::sleep_for(std::chrono::milliseconds (100));
        std::cout << "send\n";

        client.send(buf,512);
        std::cout << "recv\n";

        ssize_t  ret = client.recv();
        std::cout << "reconnect\n";

        client.reconnect();
        if(ret > 0 ){
            std::cout << "recv buffer: " << ret << "bytes\n";

#if 0
            std::cout << "recv buffer: " << ret << "bytes\n";
            for(int i = 0 ; i < ret ; i++){
                std::cout << client.recv_buffer[i]<<",";
            }
            std::cout<<std::endl;
#endif

        }
    }
    client.close();
    std::cout << "Exit" << std::endl;

    return 0;

}