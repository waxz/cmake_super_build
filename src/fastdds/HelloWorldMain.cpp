//
// Created by waxz on 4/16/24.
//


#include <thread>
#include <vector>
#include <memory>
#include <unordered_map>

//DDS
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>


#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>


// message
#include "message/HelloWorldPubSubTypes.h"


// common
#include "fastdds_builder.h"


#include "lyra/lyra.hpp"


#include "common/signals.h"
#include "common/functions.h"
#include "common/suspend.h"
#include "common/task.h"





using namespace eprosima::fastdds::dds;



struct Message{

    uint32_t m_index{0};
    std::array<char, 1024*1024> m_data{0};
};

struct reader_option{

};


int main(int argc, char** argv){



    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    set_signal_handler(my_handler);

//    signal(SIGINT, [](int signum)
//    {
//        std::cout << "SIGINT received, stopping Server execution." << std::endl;
//        static_cast<void>(signum);
//
//    });
//    signal(SIGTERM, [](int signum)
//    {
//        std::cout << "SIGTERM received, stopping Server execution." << std::endl;
//        static_cast<void>(signum);
//
//    });

    bool get_help = false;
    std::string exe_name;
    std::string xml_file;
    std::string toml_file;
    std::string profile;
    int task_id = 0;
    float fps = 20.0;

    bool no_sleep = false;
    bool simple_sleep = false;
    std::string message = "hello from dds!";


    // get topic xml
    auto cli
            = lyra::exe_name(exe_name)
              | lyra::help(get_help)
              | lyra::opt(xml_file, "xml_file")["-x"]["--xml_file"]("xml_file")
              | lyra::opt(toml_file, "toml_file")["-t"]["--toml_file"]("toml_file")
                | lyra::opt(profile, "profile")["-p"]["--profile"]("profile")
                  | lyra::opt(task_id, "task_id")["-i"]["--id"]("task_id")
                    | lyra::opt(fps, "fps")["-f"]["--fps"]("fps")
                      | lyra::opt(message, "message")["-m"]["--message"]("message")
                        | lyra::opt(no_sleep, "no_sleep")["-n"]["--no_sleep"]("no_sleep")
                          | lyra::opt(simple_sleep, "simple_sleep")["-s"]["--simple_sleep"]("simple_sleep")

    ;

    auto result = cli.parse({argc, argv});
    if (get_help) {
        std::cout << cli << std::endl;
        return 0;
    }


    if (!result) {
        std::cerr << "Error in command line: " << result.message() << std::endl;
        return 0;
    }
    if (xml_file.empty() || profile.empty()) {
        std::cout << cli << std::endl;
        return 0;
    }


    common::TaskManager task(10);
    task.set_loop(100.0,100000);

    auto ret = eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->load_XML_profiles_file(xml_file);
    if (ReturnCode_t::RETCODE_OK != ret){


        return 0;
    }

    std::cout << "run task_id: " << task_id << std::endl;

    common::Time start_time = common::FromUnixNow();
    size_t recv_count = 0;


    using MessageType =Test2:: Message8x1024;
    using MessageSupportType = Test2 :: Message8x1024PubSubType;

    if (task_id == 0 ){
        std::cout << "run Participant: " << "Participant_sub" << std::endl;

        common::JThread t1( std::thread( [&program_run,&profile,fps,message,&recv_count,&start_time,no_sleep,simple_sleep]{

            common::LoopHelper loopHelper;
            loopHelper.set_fps(fps);


            // subscriber
            eprosima::fastdds::dds::DomainParticipant* participant_;
            eprosima::fastdds::dds::Subscriber* subscriber_;
            eprosima::fastdds::dds::Topic* topic_;
            eprosima::fastdds::dds::DataReader* reader_;
            eprosima::fastdds::dds::TypeSupport type_(new MessageSupportType());



            {
                //CREATE THE PARTICIPANT
                eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_participant_qos_from_profile(
                        profile,
                        pqos);
                pqos.name("Participant_sub");
                participant_ = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
                if (participant_ == nullptr)
                {
                    return false;
                }

                //REGISTER THE TYPE
                type_.register_type(participant_);

                //CREATE THE SUBSCRIBER
                subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
                if (subscriber_ == nullptr)
                {
                    return false;
                }

                //CREATE THE TOPIC
                topic_ = participant_->create_topic(
                        "HelloWorldTopic",
                        type_.get_type_name(),
                        TOPIC_QOS_DEFAULT);
                if (topic_ == nullptr)
                {
                    return false;
                }

                //CREATE THE READER
                DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
            rqos.history().depth = 10;
                rqos.history().kind = KEEP_LAST_HISTORY_QOS ;

            rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
            rqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
//                rqos.data_sharing().automatic("/tmp/");
//            rqos.data_sharing().on("/tmp/b");
                rqos.data_sharing().off();

                reader_ = subscriber_->create_datareader(topic_, rqos);
                if (reader_ == nullptr)
                {
                    return false;
                }
                std::cout << "MessageType DataReader created." << std::endl;

            }

            FASTDDS_CONST_SEQUENCE(DataSeq, MessageType );
            DataSeq dataseq;

            SampleInfo info;
            SampleInfoSeq infoseq;
            common::Suspend suspend;
            float suspend_ms = 1000.0/fps;

            while (program_run){
                if(!no_sleep)
                loopHelper.start();

//            std::cout << "run in " << std::this_thread::get_id() << std::endl;


                {
                    {
                        // take

// Create a data and SampleInfo instance
//                        MessageType  data;
//                    std::cout << "Recv type_.is_plain()" << type_.is_plain() << "\n";

//                    if (type_.is_plain())
                        {



//Define a timeout of 5 seconds
                            eprosima::fastrtps::Duration_t timeout (0, 1000);

                            if (reader_->wait_for_unread_message(timeout))
                            {
                                if (ReturnCode_t::RETCODE_OK == reader_->take(dataseq, infoseq,2,NOT_READ_SAMPLE_STATE))

//                                if (ReturnCode_t::RETCODE_OK == reader_->read(dataseq, infoseq))

                                {
                                    for (LoanableCollection::size_type i = 0; i < infoseq.length(); ++i)
                                    {
                                        if (infoseq[i].valid_data)
                                        {
                                            // Print your structure data here.
                                            const MessageType& sample = dataseq[i];

//                                            if((recv_count%10) == 0 )
//                                            std::cout << "**** Participant_sub_udp  recv:message: " << size_t(sample.index()) << ", data: " << (char* )(&sample.data()[0]) << "\n";

                                            recv_count++;


                                            if(recv_count > 10 ){

                                                auto dur = common::FromUnixNow() - start_time;
                                                auto dur_us = double (common::ToMicroSeconds(dur));
                                                auto recv_count_valid = recv_count - 10;
                                                auto band_width = double (recv_count_valid*sample.data().size())/ dur_us;

                                                auto rqs = recv_count_valid/(dur_us * 1e-6);
                                                if((recv_count%1000) == 0 )
                                                std::cout << "**** Participant_sub_udp  band_width = " << band_width << "MB/s, rqs = " << rqs << "/s\n";
                                            }else{
                                                start_time = common::FromUnixNow();
                                            }

                                        }
                                    }
                                    reader_->return_loan(dataseq, infoseq);
                                }
                            }
                            else
                            {
//                                std::cout << "No data this time" << std::endl;
                            }


                        }






                    }

                }


                if(!no_sleep) {
                    if(simple_sleep){
                        suspend.sleep(suspend_ms);
                    }else
                    loopHelper.sleep();
                };
            }
            {
                if (reader_ != nullptr)
                {
                    subscriber_->delete_datareader(reader_);
                }
                if (topic_ != nullptr)
                {
                    participant_->delete_topic(topic_);
                }
                if (subscriber_ != nullptr)
                {
                    participant_->delete_subscriber(subscriber_);
                }
                DomainParticipantFactory::get_instance()->delete_participant(participant_);
                std::cout << "\nSubscriber close." << std::endl;

            }


            return false;


        }));

        return 0;

    }


    if (task_id == 1){
        std::cout << "run Participant: " << "Participant_pub" << std::endl;

        common::JThread t2( std::thread( [&program_run,&profile,fps,message,no_sleep,simple_sleep]{
            common::LoopHelper loopHelper;
            loopHelper.set_fps(fps);

            eprosima::fastdds::dds::DomainParticipant* participant_;
            eprosima::fastdds::dds::Publisher* publisher_;
            eprosima::fastdds::dds::Topic* topic_;
            eprosima::fastdds::dds::DataWriter* writer_;
            eprosima::fastdds::dds::TypeSupport  type_(new MessageSupportType());

            {
                // publisher



                {
                    /* Initialize data_ here */

                    //CREATE THE PARTICIPANT
                    eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
                    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_participant_qos_from_profile(
                            profile,
                            pqos);
                    pqos.name("Participant_pub");
                    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
                    if (participant_ == nullptr)
                    {
                        return false;
                    }

                    //REGISTER THE TYPE
                    type_.register_type(participant_);

                    //CREATE THE PUBLISHER
                    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
                    if (publisher_ == nullptr)
                    {
                        return false;
                    }

                    //CREATE THE TOPIC
                    topic_ = participant_->create_topic(
                            "HelloWorldTopic",
                            type_.get_type_name(),
                            TOPIC_QOS_DEFAULT);
                    if (topic_ == nullptr)
                    {
                        return false;
                    }

                    DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;
                    wqos.history().depth = 10;
                    wqos.history().kind = KEEP_LAST_HISTORY_QOS ;

                    wqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
                    wqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
//                    wqos.data_sharing().automatic("/tmp/");
//                     wqos.data_sharing().on("/tmp/b");
                    wqos.data_sharing().off();


                    // CREATE THE WRITER
                    writer_ = publisher_->create_datawriter(topic_, wqos);
                    if (writer_ == nullptr)
                    {
                        return false;
                    }

                    std::cout << "MessageType DataWriter created." << std::endl;
                }

            }

            uint32_t msgsent = 0;
            common::Suspend suspend;
            float suspend_ms = 1000.0/fps;
            while (program_run){

                loopHelper.start();

//            std::cout << "run in " << std::this_thread::get_id() << std::endl;

                {
//                std::cout << "Sending type_.is_plain()" << type_.is_plain() << "\n";

                    if (type_.is_plain())
                    {

                        void* sample = nullptr;

                        if (ReturnCode_t::RETCODE_OK
                            ==  writer_->loan_sample(
                                sample,
                                DataWriter::LoanInitializationKind::NO_LOAN_INITIALIZATION))
                        {
                            // initialize and send the sample
                            MessageType* data = static_cast<MessageType*>(sample);
                            data->index() = msgsent + 1;
                            memset(data->data().data(),0 , message.size()+1);
                            memcpy(data->data().data(), message.data(), message.size());

                            ++msgsent;
                            if (!writer_->write(sample))
                            {
                                writer_->discard_loan(sample);
                            }
                        }
                    }else{
                        MessageType data;
                        data.index() = msgsent + 1;
                        memset(data.data().data(),0 , message.size()+1);
                        memcpy(data.data().data(), message.data(), message.size());
                        writer_->write(&data);
                        ++msgsent;
//                    std::cout << "Non loan Sending sample, count=" << msgsent << "\n";
                    }


                }
                if(!no_sleep) {
                    if(simple_sleep){
                        suspend.sleep(suspend_ms);
                    }else
                        loopHelper.sleep();
                };
            }

            {
                {
                    if (writer_ != nullptr)
                    {
                        publisher_->delete_datawriter(writer_);
                    }
                    if (publisher_ != nullptr)
                    {
                        participant_->delete_publisher(publisher_);
                    }
                    if (topic_ != nullptr)
                    {
                        participant_->delete_topic(topic_);
                    }
                    DomainParticipantFactory::get_instance()->delete_participant(participant_);
                    std::cout << "\nPublisher close." << std::endl;
                }
            }

            return false;

        }));


        return 0;
    }

    if (task_id == 2){
        std::cout << "run Participant: " << "Participant_sub_shm" << std::endl;

        common::JThread t3( std::thread( [&program_run,&profile,fps,&recv_count,&start_time,no_sleep,simple_sleep]{

            common::LoopHelper loopHelper;
            loopHelper.set_fps(fps);


            // subscriber
            eprosima::fastdds::dds::DomainParticipant* participant_;
            eprosima::fastdds::dds::Subscriber* subscriber_;
            eprosima::fastdds::dds::Topic* topic_;
            eprosima::fastdds::dds::DataReader* reader_;
            eprosima::fastdds::dds::TypeSupport type_(new MessageSupportType());



            {
                //CREATE THE PARTICIPANT
                eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
                eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_participant_qos_from_profile(
                        profile,
                        pqos);
                pqos.name("Participant_sub_shm");
                participant_ = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
                if (participant_ == nullptr)
                {
                    return false;
                }

                //REGISTER THE TYPE
                type_.register_type(participant_);

                //CREATE THE SUBSCRIBER
                subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
                if (subscriber_ == nullptr)
                {
                    return false;
                }

                //CREATE THE TOPIC
                topic_ = participant_->create_topic(
                        "HelloWorldTopic",
                        type_.get_type_name(),
                        TOPIC_QOS_DEFAULT);
                if (topic_ == nullptr)
                {
                    return false;
                }

                //CREATE THE READER
                DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
                rqos.history().depth = 10;
                rqos.history().kind = KEEP_LAST_HISTORY_QOS ;

            rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
            rqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
//            rqos.data_sharing().automatic("/tmp/dds");
            rqos.data_sharing().on("/tmp/dds");
//                rqos.data_sharing().off();

                reader_ = subscriber_->create_datareader(topic_, rqos);
                if (reader_ == nullptr)
                {
                    return false;
                }
                std::cout << "MessageType DataReader created." << std::endl;

            }

            FASTDDS_CONST_SEQUENCE(DataSeq, MessageType );
            DataSeq dataseq;

            SampleInfo info;
            SampleInfoSeq infoseq;

            eprosima::fastrtps::Duration_t timeout (0, 100);
            common::Suspend suspend;
            float suspend_ms = 1000.0/fps;
            while (program_run){
                if(!no_sleep)
                loopHelper.start();

//            std::cout << "run in " << std::this_thread::get_id() << std::endl;


                {
                    {
                        // take

// Create a data and SampleInfo instance
//                        MessageType  data;
//                    std::cout << "Recv type_.is_plain()" << type_.is_plain() << "\n";

//                    if (type_.is_plain())
                        {



//Define a timeout of 5 seconds

                            if (reader_->wait_for_unread_message(timeout))
                            {
                                if (ReturnCode_t::RETCODE_OK == reader_->take(dataseq, infoseq,2,NOT_READ_SAMPLE_STATE))
//                            if (ReturnCode_t::RETCODE_OK == reader_->read(dataseq, infoseq))
                                {
                                    for (LoanableCollection::size_type i = 0; i < infoseq.length(); ++i)
                                    {
                                        if (infoseq[i].valid_data)
                                        {
                                            // Print your structure data here.
                                            const MessageType& sample = dataseq[i];

//                                            if((recv_count%10) == 0 )  std::cout << "**** Participant_sub_shm  recv:message: " << size_t(sample.index()) << ", data: " << (char* )(&sample.data()[0]) << "\n";

                                            recv_count++;

                                            if(recv_count > 10 ){

                                                auto dur = common::FromUnixNow() - start_time;
                                                auto dur_us = double (common::ToMicroSeconds(dur));
                                                auto recv_count_valid = recv_count - 10;
                                                auto band_width = double (recv_count_valid*sample.data().size())/ dur_us;

                                                auto rqs = recv_count_valid/(dur_us * 1e-6);
                                                if((recv_count%1000) == 0 )
                                                    std::cout << "**** Participant_sub_shm  band_width = " << band_width << "MB/s, rqs = " << rqs << "/s\n";
                                            }else{
                                                start_time = common::FromUnixNow();

                                            }
                                        }
                                    }
                                    reader_->return_loan(dataseq, infoseq);
                                }
                            }
                            else
                            {
//                                std::cout << "No data this time" << std::endl;
                            }


                        }






                    }

                }


                if(!no_sleep) {
                    if(simple_sleep){
                        suspend.sleep(suspend_ms);
                    }else
                        loopHelper.sleep();
                };
            }
            {
                if (reader_ != nullptr)
                {
                    subscriber_->delete_datareader(reader_);
                }
                if (topic_ != nullptr)
                {
                    participant_->delete_topic(topic_);
                }
                if (subscriber_ != nullptr)
                {
                    participant_->delete_subscriber(subscriber_);
                }
                DomainParticipantFactory::get_instance()->delete_participant(participant_);
                std::cout << "\nSubscriber close." << std::endl;

            }


            return false;


        }));

        return 0;
    }


    if (task_id == 3){
        std::cout << "run Participant: " << "Participant_pub_shm" << std::endl;

        common::JThread t4( std::thread( [&program_run,&profile,fps,message,no_sleep,simple_sleep]{
            common::LoopHelper loopHelper;
            loopHelper.set_fps(fps);

            eprosima::fastdds::dds::DomainParticipant* participant_;
            eprosima::fastdds::dds::Publisher* publisher_;
            eprosima::fastdds::dds::Topic* topic_;
            eprosima::fastdds::dds::DataWriter* writer_;
            eprosima::fastdds::dds::TypeSupport  type_(new MessageSupportType());

            {
                // publisher



                {
                    /* Initialize data_ here */

                    //CREATE THE PARTICIPANT
                    eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
                    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->get_participant_qos_from_profile(
                            profile,
                            pqos);
                    pqos.name("Participant_pub_shm");
                    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
                    if (participant_ == nullptr)
                    {
                        return false;
                    }

                    //REGISTER THE TYPE
                    type_.register_type(participant_);

                    //CREATE THE PUBLISHER
                    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
                    if (publisher_ == nullptr)
                    {
                        return false;
                    }

                    //CREATE THE TOPIC
                    topic_ = participant_->create_topic(
                            "HelloWorldTopic",
                            type_.get_type_name(),
                            TOPIC_QOS_DEFAULT);
                    if (topic_ == nullptr)
                    {
                        return false;
                    }

                    DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;
                    wqos.history().depth = 10;
                    wqos.history().kind = KEEP_LAST_HISTORY_QOS ;

                    wqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
                    wqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
//                wqos.data_sharing().automatic("/tmp/dds");
                wqos.data_sharing().on("/tmp/dds");
//                    wqos.data_sharing().off();

                    // CREATE THE WRITER
                    writer_ = publisher_->create_datawriter(topic_, wqos);
                    if (writer_ == nullptr)
                    {
                        return false;
                    }

                    std::cout << "MessageType DataWriter created." << std::endl;
                }

            }

            uint32_t msgsent = 0;
            common::Suspend suspend;
            float suspend_ms = 1000.0/fps;
            while (program_run){
                if(!no_sleep)
                loopHelper.start();

//            std::cout << "run in " << std::this_thread::get_id() << std::endl;

                {
//                std::cout << "Sending type_.is_plain()" << type_.is_plain() << "\n";

//                    if (type_.is_plain())
                    {

                        void* sample = nullptr;

                        if (ReturnCode_t::RETCODE_OK
                            ==  writer_->loan_sample(
                                sample,
                                DataWriter::LoanInitializationKind::NO_LOAN_INITIALIZATION))
                        {
                            // initialize and send the sample
                            MessageType* data = static_cast<MessageType*>(sample);
                            data->index() = msgsent + 1;
                            memset(data->data().data(),0 , message.size()+1);
                            memcpy(data->data().data(), message.data(), message.size());

                            ++msgsent;
                            if (!writer_->write(sample))
                            {
                                writer_->discard_loan(sample);
                            }
                        }
                    }

//                    {
//                        MessageType data;
//                        data.index() = msgsent + 1;
//                        memset(data.data().data(),0 , message.size()+1);
//                        memcpy(data.data().data(), message.data(), message.size());
//                        writer_->write(&data);
//                        ++msgsent;
////                    std::cout << "Non loan Sending sample, count=" << msgsent << "\n";
//                    }


                }



                if(!no_sleep) {
                    if(simple_sleep){
                        suspend.sleep(suspend_ms);
                    }else
                        loopHelper.sleep();
                };


            }

            {
                {
                    if (writer_ != nullptr)
                    {
                        publisher_->delete_datawriter(writer_);
                    }
                    if (publisher_ != nullptr)
                    {
                        participant_->delete_publisher(publisher_);
                    }
                    if (topic_ != nullptr)
                    {
                        participant_->delete_topic(topic_);
                    }
                    DomainParticipantFactory::get_instance()->delete_participant(participant_);
                    std::cout << "\nPublisher close." << std::endl;
                }
            }

            return false;

        }));
        return 0;

    }




    while (program_run){
        task.run();
    }





}