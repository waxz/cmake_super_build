//
// Created by waxz on 10/24/23.
//

#include "SocketManager.h"

ssize_t SocketClient::send(void *buffer, size_t buffer_len) {
    if (is_fault) {
        std::cerr << "Error need reset" << std::endl;
        return -1;
    }
    ssize_t ret = write_n(buffer, buffer_len);
    if (ret != (ssize_t) buffer_len) {
        std::cerr << "Error writing to the UNIX stream" << std::endl;
        is_fault = true;
    }
    return ret;
}

ssize_t SocketClient::recv() {
    if (is_fault) {
        std::cerr << "Error need reset" << std::endl;
        return -1;
    }

    ssize_t ret = base::read(recv_buffer, MAX_BUFFER_SIZE);

    recv_buffer_size = ret > 0 ? ret : 0;
    if (ret < 0) {
        std::cerr << "Error reading from UNIX stream" << std::endl;
        is_fault = true;
    }
    return ret;
}


void SocketClient::reconnect() {
    if (is_fault) {
        std::cerr << "Reconnect" << std::endl;

        base::connect(remote_addr);
        if (last_error() == 0) {
            std::cerr << "Reconnect done" << std::endl;
            is_fault = false;
        }
    }


}

bool SocketClient::connect(const addr_t &addr) {
    remote_addr = addr;
    return base::connect(addr);
}

//=================================

bool SocketServer::bind(const sockpp::inet_address &addr) {
    bind_addr = addr;
    return base::bind(addr);
}

bool SocketServer::stop_loop() {
    std::cout << "stop_loop" << std::endl;
    listen_thread_run = false;
    // connect to server to break accept blocking
    SocketClient client;
    client.connect(bind_addr);
    return true;
}
bool SocketServer::start_loop() {
    listen_thread_run = true;

    listen_thread = std::make_shared<std::thread>([&]{

        while (listen_thread_run){
            sockpp::inet_address peer;
            // Accept a new client connection
            sockpp::tcp_socket sock = base::accept(&peer);
            if(!listen_thread_run){
              break;
            }
            if (!sock) {
                std::cerr << "Error accepting incoming connection: " << base::last_error_str() << std::endl;
            } else {
                std::cerr << "Received a connection request from " << peer << std::endl;

                // Create a thread and transfer the new stream to it.
                std::shared_ptr<sockpp::tcp_socket> sock_ptr = std::make_shared<sockpp::tcp_socket>(std::move(sock));
                Message msg;
                msg.valid = true;
                msg.sock_ptr = sock_ptr;
                {
                    std::unique_lock<std::mutex> lockGuard(connect_socks_mtx);
                    connect_socks.emplace_back(msg);
                }
                {
                    clean_dead();
                }


            }
        }
        std::cout << "ThreadLoop Exit" << std::endl;
        {
            std::lock_guard<std::mutex> lockGuard(connect_socks_mtx);
            connect_socks.clear();
        }

    });
    return true;
}


ssize_t SocketServer::Message::send(void *buffer, size_t buffer_len) {
    auto& sock = *sock_ptr;

    if (is_fault) {
        std::cerr << "Error need reset" << std::endl;
        return -1;
    }
    ssize_t ret = sock.write_n(buffer, buffer_len);
    if (ret != (ssize_t) buffer_len) {
        std::cerr << "Error writing to the UNIX stream" << std::endl;
        is_fault = true;
        valid = false;
    }
    return ret;
}

bool SocketServer::clean_dead() {

    std::lock_guard<std::mutex> lockGuard(connect_socks_mtx);

    bool need_remove = false;

    for(auto& s: connect_socks){
        s.valid = s.valid && common::ToMillSeconds(common::FromUnixNow() - s.last_recv_time) < connect_dead_ms;
    }
    for (auto &s: connect_socks) {
        need_remove = need_remove || (!s.valid);
    }
    if (need_remove) {

        auto it = std::remove_if(connect_socks.begin(), connect_socks.end(), [](auto &e) {
            return !e.valid;
        });
        connect_socks.erase(it, connect_socks.end());
    }
    return true;
}
ssize_t SocketServer::Message::recv(){
    auto& sock = *sock_ptr;

    if (is_fault) {
        std::cerr << "Error need reset" << std::endl;
        return -1;
    }
    auto ret = sock.read(recv_buffer, MAX_BUFFER_SIZE);
    recv_buffer_size = ret > 0 ? ret : 0;
    if (ret == 0) {
        std::cerr << "Error reading from UNIX stream" << std::endl;
    }else{
        last_recv_time = common::FromUnixNow();
    }

    valid = valid && common::ToMillSeconds(common::FromUnixNow() - last_recv_time) < 5000;
    return ret;
}



