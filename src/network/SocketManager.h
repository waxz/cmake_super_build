//
// Created by waxz on 10/24/23.
//

#ifndef CMAKE_SUPER_BUILD_SOCKETMANAGER_H
#define CMAKE_SUPER_BUILD_SOCKETMANAGER_H

#include "common/task.h"

#include <mutex>
#include <atomic>
#include "sockpp/tcp_connector.h"
#include "sockpp/tcp_acceptor.h"
#include "sockpp/version.h"


#include <unordered_map>
class SocketClient: public  sockpp::tcp_connector{
public:
    /** The base class */
    using base = sockpp::tcp_connector;

public:
    static const size_t MAX_BUFFER_SIZE = 1024;
    char recv_buffer[MAX_BUFFER_SIZE] = {0};
    size_t recv_buffer_size = 0;

    addr_t remote_addr;
    bool is_fault = false;


    SocketClient(){}

    SocketClient(const addr_t& addr):remote_addr(addr),base(addr){
    }

    SocketClient(SocketClient&& rhs):remote_addr(rhs.remote_addr),base(std::move(rhs)){

    }
    bool connect(const addr_t& addr);

    ssize_t send(void* buffer, size_t buffer_len);

    ssize_t recv();

    void reconnect();
};

class SocketServer:public sockpp::tcp_acceptor{

public:
    struct Message{
        bool valid = true;
        bool is_fault = false;
        static const size_t MAX_BUFFER_SIZE = 1024;
        char recv_buffer[MAX_BUFFER_SIZE] = {0};
        size_t recv_buffer_size = 0;
        std::shared_ptr< sockpp::tcp_socket> sock_ptr;

        common::Time last_recv_time;

        Message():recv_buffer_size(0),valid(true),last_recv_time(common::FromUnixNow()){}
        Message(const Message& rhs):recv_buffer_size(0),valid(rhs.valid), sock_ptr(rhs.sock_ptr),last_recv_time(common::FromUnixNow()){}

        ssize_t send(void* buffer, size_t buffer_len);

        ssize_t recv();


//        Message(Message&& rhs):recv_buffer_size(0),valid(rhs.valid), sock_ptr(rhs.sock_ptr){}
    };

public:
    /** The base class */
    using base = sockpp::tcp_acceptor;
public:
    common::TaskManager task_manager;

    addr_t bind_addr;

    std::atomic_bool listen_thread_run;
    std::shared_ptr<std::thread> listen_thread;

    std::mutex connect_socks_mtx;
    std::vector<Message> connect_socks;

    uint16_t connect_dead_ms = 3000;

    void set(uint16_t timeout_ms){connect_dead_ms = timeout_ms;}

    SocketServer(){};
    ~SocketServer(){base::close();if (listen_thread->joinable()) listen_thread->join();}

    SocketServer(const addr_t& addr, int queSize=DFLT_QUE_SIZE):bind_addr(addr), base(addr,queSize){};

    SocketServer(in_port_t port, int queSize=DFLT_QUE_SIZE):bind_addr(port), base(port,queSize){

    }
    SocketServer(SocketServer&& rhs):bind_addr(rhs.bind_addr),base(std::move(rhs)){}
    bool bind(const addr_t& addr);

    bool start_loop();
    bool stop_loop();

    bool clean_dead();


};


#endif //CMAKE_SUPER_BUILD_SOCKETMANAGER_H
