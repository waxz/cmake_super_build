//
// Created by waxz on 8/30/23.
//

#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>

/* Include the C++ DDS API. */
#include "dds/dds.hpp"

/* Include data type and specific traits to be used with the C++ DDS API. */
#include "RoundTrip.hpp"
#include "ShmThroughput.hpp"


#include "common/signals.h"
#include "common/clock_time.h"
#include "common/functions.h"

using namespace org::eclipse::cyclonedds;

int main(int argc, char** argv){

    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
     set_signal_handler(my_handler);



    static const std::string shm_config {
            "<CycloneDDS><Domain><SharedMemory><Enable>false</Enable><LogLevel>verbose</LogLevel></SharedMemory></Domain></CycloneDDS>"};



    /* First, a domain participant is needed.
 * Create one on the default domain. */
    dds::domain::DomainParticipant participant= dds::domain::DomainParticipant(
            0xFFFFFFFF,
            dds::domain::DomainParticipant::default_participant_qos(),
            nullptr,
            dds::core::status::StatusMask::none()
            //, shm_config
            );

    using MSG_TYPE = ThroughputModule::DataType_1048576;

    /* To subscribe to something, a topic is needed. */
    dds::topic::Topic<MSG_TYPE> topic(participant, "RoundTripModule_Msg");

    /* A reader also needs a subscriber. */
    dds::sub::Subscriber subscriber(participant);

    /* Now, the reader can be created to subscribe to a HelloWorld message. */
    dds::sub::DataReader<MSG_TYPE> reader(subscriber, topic);


    /* A writer also needs a publisher. */
    dds::pub::Publisher publisher(participant);

    /* Now, the writer can be created to publish a HelloWorld message. */
    dds::pub::DataWriter<MSG_TYPE> writer(publisher, topic);

    auto p = writer.listener();
    std::cout << "=== [Pong] p: " << p << std::endl;

    std::cout << "=== [Pong] Waiting for subscriber." << std::endl;
    std::cout << "=== [Pong] writer.publication_matched_status().current_count() = " << writer.publication_matched_status().current_count() <<  std::endl;

//    while (program_run && writer.publication_matched_status().current_count() < 1) {
//        std::this_thread::sleep_for(std::chrono::milliseconds(20));
//    }
    std::cout << "=== [Pong] start loop" <<  std::endl;

    /* Create a message to write. */

//    auto loanResult = publisher.loan();


    MSG_TYPE pub_data ;

    pub_data.payloadsize(1048576 - sizeof (ThroughputModule::DataType_Base));
    const size_t payload_size = pub_data.payloadsize();

    // received buffer
    dds::sub::LoanedSamples<MSG_TYPE> recv_samples;


    common::Time start_time = common::FromUnixNow();
    size_t recv_cnt = 0;

    while (program_run){




        /* Write the message. */
//        std::cout << "=== [Ping] Write sample." << std::endl;

        {
            size_t instances_cnt = 1;
            // if writer supports loaning
            if (writer.delegate()->is_loan_supported()) {
                for (int32_t i = 0; i < instances_cnt; i++) {
                    // loan memory from the middleware
                    auto & loaned_sample = writer.delegate()->loan_sample();
                    loaned_sample.payload()[0]++;
                    // make sample
                    // store sample for comparison later
                    // write sample (which will also release the loan to the middleware)
                    writer.write(loaned_sample);
                }
            }else{
                pub_data.payload()[0]++;
                writer.write(pub_data);
            }
        }

//        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//        pub_data.payload()[0] = 100;
//        writer.write(pub_data);
        continue;
        //wait respond
        /* Try taking samples from the reader. */
        recv_samples = reader.take();
        /* Are samples read? */
        if(recv_cnt == 0){
            start_time = common::FromUnixNow();
        }
        if (recv_samples.length() > 0) {
            /* Use an iterator to run over the set of samples. */
            dds::sub::LoanedSamples<MSG_TYPE>::const_iterator sample_iter;
            for (sample_iter = recv_samples.begin();
                 sample_iter < recv_samples.end();
                 ++sample_iter) {
                /* Get the message and sample information. */
                const MSG_TYPE & msg = sample_iter->data();
                const dds::sub::SampleInfo& info = sample_iter->info();

                /* Sometimes a sample is read, only to indicate a data
                 * state change (which can be found in the info). If
                 * that's the case, only the key value of the sample
                 * is set. The other data parts are not.
                 * Check if this sample has valid data. */
                if (info.valid()) {
//                    std::cout << "=== [Subscriber] Message received:" << std::endl;
//                    std::cout << "    userID  : " << msg.userID() << std::endl;
                    recv_cnt++;
                    float rps = recv_cnt/( 0.001f*common::ToMillSeconds(common::FromUnixNow() - start_time));
                    float band_width = 1e-6f*rps*payload_size;
                    std::cout << "msg.payload()[0] = [" << int(msg.payload()[0]) << "]\n";

                    std::cout << "band_width = " << band_width << "MB/s\n";
                    std::cout << "rps = " << rps << "request/s\n";

                }
            }
        } else {
//            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

}