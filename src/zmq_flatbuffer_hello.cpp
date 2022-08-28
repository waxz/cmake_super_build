//
// Created by waxz on 8/28/22.
//
//
// Created by waxz on 8/28/22.
//
//
// Created by waxz on 8/28/22.
//
#include <zmq.hpp>
#include "zmq_addon.hpp"

#include <thread>
#include <iostream>

//#include "helpers/zmq_helper.hpp"
//#include "helpers/flatbuffers_helper.hpp"

#include "schema_generated.h"
int main(int argc, char** argv)
{
    if (argc < 2) {

        std::cout << "please set mode" << std::endl;
        return 0;

    }

    const int mode  = atoi(argv[1]);;

    std::cout << "argv[1] = " << argv[1] << std::endl;
    if (mode == 0) {
        std::cout << "start subscriber" << std::endl;

        zmq::context_t context(1);
        zmq::socket_t subscriber(context, ZMQ_SUB);

        std::string TOPIC = "";
        subscriber.setsockopt(ZMQ_SUBSCRIBE, TOPIC.c_str(), TOPIC.length()); // allow all messages

        int linger = 0; // Proper shutdown ZeroMQ
        subscriber.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

        subscriber.connect("tcp://localhost:6666");

        while (false){
            zmq::message_t message;
            auto rc = subscriber.recv(message);
            if (rc)
            {
                std::string rpl = std::string(static_cast<char*>(message.data()), message.size());

                std::cout << "recv rpl:\n" <<  rpl << std::endl;

            }
        }

        // ok
        while (false){
            // Receive all parts of the message
            std::vector<zmq::message_t> recv_msgs;
            zmq::recv_result_t result =
                    zmq::recv_multipart(subscriber, std::back_inserter(recv_msgs));
//            assert(result && "recv failed");
//            assert(*result == 2);

            std::cout << "recv_msgs : [" << recv_msgs[0].to_string() << "] " << std::endl;
        }

        // ok , rpl is real content
        while (true)
        {
            zmq::message_t message;
            int rc = subscriber.recv(&message);
            if (rc)
            {
                std::string rpl = std::string(static_cast<char*>(message.data()), message.size());

                std::cout << "recv rpl:\n" <<  rpl << std::endl;

            }


        }

        // Clean up your socket and context here
        subscriber.close();
        context.close();
    }
    if (mode == 1) {
        std::cout << "start publisher" << std::endl;
        flatbuffers::FlatBufferBuilder builder(1024);

        //  Prepare our context and publisher
        zmq::context_t context(1);
        zmq::socket_t publisher(context, ZMQ_PUB);
        publisher.bind("tcp://*:6666");



        int count = 0;
        while (true)
        {
            std::stringstream msg_str_stream;
            msg_str_stream << "Hello_no._" << count;
            std::string msg_str = msg_str_stream.str();

            zmq::message_t message(msg_str.length());
            memcpy(message.data(), msg_str.c_str(), msg_str.length());
            std::cout << "publish message:\n" << message << std::endl;
            std::cout << "publish msg_str:\n" << msg_str << std::endl;

            publisher.send(message);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            count++;
        }
        // Clean up your socket and context here
        publisher.close();
        context.close();
    }
}