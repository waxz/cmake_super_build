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

#if 0
    std::mutex mtx;


    std::vector<int> v;
    int cnt = 0;
    int ccc = 0;

    std::thread t1([&]{
        common::Suspend s;
        while (program_run){
            ccc++;
            std::lock_guard<std::mutex> lockGuard(mtx);
            v.push_back(cnt);
            cnt++;
//            std::cout << "[" << cnt << "]";

            s.sleep(3);
        }
    });
    common::Suspend s;

    while (program_run){
        s.sleep(3);

        std::lock_guard<std::mutex> lockGuard(mtx);
        std::cout << "1 v.size " << v.size() << std::endl;

        if(v.size() > 100){
            v.resize(50);
        }
        std::cout << "cnt " << cnt<< std::endl;
        std::cout << "ccc " << ccc<< std::endl;

        std::cout << "2 v.size " << v.size() << std::endl;

    }


    if(t1.joinable()) t1.join();
    return 0;
#endif








    sockpp::initialize();

    int port = 12345;

    SocketServer server(port) ;
//    sockpp::tcp_acceptor server(port) ;

    server.bind(sockpp::tcp_connector::addr_t{(in_port_t)port});

//    server.set_non_blocking();
    std::cout << "Awaiting connections on port " << port << "..." <<  std::endl;

    server.start_loop();


    common::TaskManager taskManager;


    common::Suspend suspend;
    std::cout << "Start user loop on " << port << "..." <<  std::endl;

    while (program_run) {

        std::cerr<< "server.connect_socks.size(): " <<server.connect_socks.size() << std::endl;

        {

            std::unique_lock<std::mutex> lockGuard(server.connect_socks_mtx);

            for(auto& sock : server.connect_socks){
                ssize_t ret = sock.recv();
                std::cerr<< "recv from " << sock.sock_ptr->peer_address()<< ", ret: " << ret  <<  " bytes" << std::endl;

                sock.send( sock.recv_buffer, sock.recv_buffer_size);
            }
        }

        if(server.connect_socks.size() > 10){
            server.clean_dead();
        }

        suspend.sleep(10);

    }
    server.stop_loop();
    std::cout << "Exit"<< std::endl;

    return 0;


}