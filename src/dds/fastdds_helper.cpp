//
// Created by waxz on 9/11/23.
//


// types
#include "absl/types/any.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "absl/types/span.h"

// string
#include "absl/strings/str_format.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"
#include "absl/strings/match.h"

// container
#include "absl/container/fixed_array.h"
#include "absl/container/inlined_vector.h"
#include "absl/container/flat_hash_map.h"

// dds
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastdds/dds/subscriber/Subscriber.hpp>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>

#include <fastdds/dds/subscriber/Subscriber.hpp>

#include <fastdds/dds/subscriber/DataReader.hpp>

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>


// toml
#include <toml.hpp>

// user code
#include "fastdds_helper.h"
#include "../c_test/tcc_helper.h"
#include "common/string_logger.h"

// IDL generate
#include "ShmThroughput.h"
#include "ShmThroughputPubSubTypes.h"

#include "tinyalloc/tinyalloc.h"

#include "common/data_holder.h"

// callback

/*

 */


typedef void(*process_func_t)(void*);
typedef void (*read_func_t)(eprosima::fastdds::dds::DataReader* ,DataHolderPtr );
typedef eprosima::fastdds::dds::TopicDataType* (*new_type_t)();

template<typename DATATYPE>
eprosima::fastdds::dds::TopicDataType* new_DataType(){
    return  new DATATYPE();
}

template<typename DATATYPE>
void read_DataType(eprosima::fastdds::dds::DataReader* reader,DataHolderPtr data_holderPtr){


    FASTDDS_CONST_SEQUENCE(DataSeq,DATATYPE );
    DataSeq data;
    eprosima::fastdds::dds::SampleInfoSeq infos;
    while (ReturnCode_t::RETCODE_OK == reader->take(data, infos))
    {
        for (size_t i = 0; i < infos.length(); ++i)
        {
            if (infos[i].valid_data)
            {
//                const DATATYPE& sample = data[i];

            }
        }
        reader->return_loan(data, infos);
    }
}



// const data

const std::map<std::string,read_func_t > DDS_READ_FUNC_MAP ={
        {"ThroughputModule::DataType_1048576",read_DataType<ThroughputModule::DataType_1048576>},
        {"ThroughputModule::DataType_1024",read_DataType<ThroughputModule::DataType_1024>}
};

const std::map<std::string,new_type_t > DDS_NEW_TYPE_FUNC_MAP ={
        {"ThroughputModule::DataType_1048576",new_DataType<ThroughputModule::DataType_1048576PubSubType>},
        {"ThroughputModule::DataType_1024",new_DataType<ThroughputModule::DataType_1024PubSubType>}
};
const char* const TOML_DDS_FILED_KEY         = "dds";
const char* const TOML_CHANNEL_KEY         = "channel";
const char* const TOML_DDS_CHANNEL_MODE_KEY    = "channel_type";
const char* const TOML_DDS_CHANNEL_MODE_WRITE  = "write";
const char* const TOML_DDS_CHANNEL_MODE_READ   = "read";
const char* const TOML_DDS_TOPIC_NAME_KEY      = "topic_name";
const char* const TOML_DDS_TOPIC_TYPE_KEY      = "topic_type";
// use shm
const char* const TOML_DDS_TOPIC_SHARE_KEY      = "share";
// qos

// code



const char* const TOML_CHECK_KEY_ARRAY[] = {TOML_DDS_CHANNEL_MODE_KEY,TOML_DDS_CHANNEL_MODE_KEY,TOML_DDS_TOPIC_SHARE_KEY};
absl::Span<const char* const> TOML_CHECK_KEY(TOML_CHECK_KEY_ARRAY);

struct DDSWriter{

    DataHolder data_holder;
    eprosima::fastdds::dds::TypeSupport type_;
    eprosima::fastdds::dds::DomainParticipant* participant_ = nullptr;
    eprosima::fastdds::dds::Publisher* publisher_ = nullptr;
    eprosima::fastdds::dds::Topic* topic_ = nullptr;
    eprosima::fastdds::dds::DataWriter* writer_ = nullptr;
    process_func_t process_func = nullptr;


    int create_from_toml(const toml::basic_value<toml::discard_comments>& config){

        std::cout << "writer get config:\n" << config<< std::endl;


        //dds
        std::string topic_type = toml::find(config, TOML_DDS_TOPIC_TYPE_KEY).as_string().str;
        std::string topic_name = toml::find(config, TOML_DDS_TOPIC_NAME_KEY).as_string().str;
        bool use_share = toml::find(config, TOML_DDS_TOPIC_SHARE_KEY).as_boolean();


        auto new_type_func_it = DDS_NEW_TYPE_FUNC_MAP.find(topic_type);
        if(new_type_func_it == DDS_NEW_TYPE_FUNC_MAP.end()){
            return -1;
        }


        type_.reset(new_type_func_it->second());

        //CREATE THE PARTICIPANT
        eprosima::fastdds::dds::DomainParticipantQos pqos;
        pqos.name("Participant_pub");
        participant_ =  eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(0, pqos);
        if (participant_ == nullptr)
        {
            return -1;
        }

        //REGISTER THE TYPE
        type_.register_type(participant_);

        //CREATE THE PUBLISHER
        publisher_ = participant_->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT, nullptr);
        if (publisher_ == nullptr)
        {
            return -1;
        }

        //CREATE THE TOPIC
        topic_ = participant_->create_topic(
                topic_name,
                type_.get_type_name(),
                eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
        if (topic_ == nullptr)
        {
            return -1;
        }


        // CREATE THE WRITER
        eprosima::fastdds::dds::DataWriterQos wqos = publisher_->get_default_datawriter_qos();
        wqos.history().depth = 10;
        wqos.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
        wqos.reliability().kind = eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
//        wqos.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;

        if(use_share){
            wqos.data_sharing().automatic();
        }


        writer_ = publisher_->create_datawriter(topic_, wqos, nullptr);
        if (writer_ == nullptr)
        {
            return -1;
        }
        absl::PrintF("success to create channel, topic_name : %s, topic_type : %s, use_share : %v\n", topic_name, topic_type, use_share);
        return 0;
    }
    int write(){


        void* sample = nullptr;
        if (ReturnCode_t::RETCODE_OK == writer_->loan_sample(sample))
        {

            if(process_func){
                process_func(sample);
            }
            writer_->write(sample);

            return 0;
        }
        return -1;
    }

    void clean(){

        if(writer_ != nullptr && publisher_ != nullptr){
            publisher_->delete_datawriter(writer_);
        }

        if(publisher_ != nullptr && participant_ != nullptr){
            participant_->delete_publisher(publisher_);
        }

        if(topic_ != nullptr && participant_ != nullptr){
            participant_->delete_topic(topic_);
        }

        if(participant_ != nullptr){
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
        }


    }
};

struct DDSReader{
    DataHolder data_holder;
    eprosima::fastdds::dds::TypeSupport type_;
    eprosima::fastdds::dds::DomainParticipant* participant_ = nullptr;
    eprosima::fastdds::dds::Subscriber* subscriber_ = nullptr;
    eprosima::fastdds::dds::Topic* topic_ = nullptr;
    eprosima::fastdds::dds::DataReader * reader_ = nullptr;
    process_func_t process_func = nullptr;
    read_func_t read_func = nullptr;



    int create_from_toml(const toml::basic_value<toml::discard_comments>& config){

        std::cout << "reader get config:\n" << config<< std::endl;

        //dds
        std::string topic_type = toml::find(config, TOML_DDS_TOPIC_TYPE_KEY).as_string().str;
        std::string topic_name = toml::find(config, TOML_DDS_TOPIC_NAME_KEY).as_string().str;
        bool use_share = toml::find(config, TOML_DDS_TOPIC_SHARE_KEY).as_boolean();

        auto new_type_func_it = DDS_NEW_TYPE_FUNC_MAP.find(topic_type);
        if(new_type_func_it == DDS_NEW_TYPE_FUNC_MAP.end()){
            return -1;
        }

        auto read_sample_func_it = DDS_READ_FUNC_MAP.find(topic_type);
        if(read_sample_func_it == DDS_READ_FUNC_MAP.end()){
            return -1;
        }
        read_func = read_sample_func_it->second;



        type_.reset(new_type_func_it->second());

        //CREATE THE PARTICIPANT
        eprosima::fastdds::dds::DomainParticipantQos pqos;
        pqos.name("Participant_pub");
        participant_ =  eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(0, pqos);
        if (participant_ == nullptr)
        {
            return -1;
        }

        //REGISTER THE TYPE
        type_.register_type(participant_);

        //CREATE THE SUBSCRIBER
        subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT, nullptr);
        if (subscriber_ == nullptr)
        {
            return -1;
        }

        //CREATE THE TOPIC
        topic_ = participant_->create_topic(
                topic_name,
                type_.get_type_name(),
                eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
        if (topic_ == nullptr)
        {
            return -1;
        }
        //CREATE THE READER
        eprosima::fastdds::dds::DataReaderQos rqos = subscriber_->get_default_datareader_qos();
        rqos.history().depth = 10;
        rqos.reliability().kind = eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
        rqos.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;

        if(use_share){
            rqos.data_sharing().automatic();
        }

        reader_ = subscriber_->create_datareader(topic_, rqos, nullptr);
        if (reader_ == nullptr)
        {
            return -1;
        }

        // create holder

        size_t data_holder_size = 10;

        data_holder = DataHolder_create(topic_type.c_str());


        absl::PrintF("success to create channel, topic_name : %s, topic_type : %s, use_share : %v\n", topic_name, topic_type, use_share);
        return 0;
    }
    int read(){


        if(read_func){
            read_func(reader_, &data_holder);
        }else{
            return -1;
        }
        return 0;
    }
    void clean(){

        if(reader_ != nullptr && subscriber_!= nullptr){
            subscriber_->delete_datareader(reader_);
        }

        if(subscriber_ != nullptr && participant_ != nullptr){
            participant_->delete_subscriber(subscriber_);
        }

        if(topic_ != nullptr && participant_ != nullptr){
            participant_->delete_topic(topic_);
        }
        if(participant_ != nullptr){
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
        }

    }
};
struct DdsHandler{
    using TopicHandlerType = absl::variant<DDSWriter, DDSReader>;
    absl::flat_hash_map<std::string,TopicHandlerType> channel_holder_map;

    int create_from_toml(const char* filename){

        // parse toml file
        // check channel name
        // check channel type: read or write
        // check topic name
        // check topic type
        // check tcc function

        auto root_data = toml::parse(filename);
        if (!root_data.contains(TOML_DDS_FILED_KEY)){
            MLOGW("%s does not contain %s", filename, TOML_DDS_FILED_KEY);
            return -1;
        }

        auto dds_filed_data = toml::find(root_data,TOML_DDS_FILED_KEY);

        if(!dds_filed_data.contains(TOML_CHANNEL_KEY)){
            MLOGW("%s does not contain %s", filename, TOML_CHANNEL_KEY);
            return -1;
        }
        auto dds_channel_data = toml::find(dds_filed_data,TOML_CHANNEL_KEY);
        if(!dds_channel_data.is_table()){
            MLOGI("%s is not table\n",TOML_CHANNEL_KEY );
            return -1;
        }

        for(const auto& x : dds_channel_data.as_table()) {
            bool error = false;


            for (int i = 0; i < TOML_CHECK_KEY.size(); i++) {
                if (!x.second.contains(TOML_CHECK_KEY[i])) {
                    MLOGW("%s, key %s has problem in finding key %s\n", TOML_CHANNEL_KEY, x.first.c_str(), TOML_CHECK_KEY[i]);
                    error = true;
                }
            }
            if (error) {
                return -1;
            }

            std::string topic_type = toml::find(x.second, TOML_DDS_TOPIC_TYPE_KEY).as_string().str;

            std::string channel_type = toml::find(x.second, TOML_DDS_CHANNEL_MODE_KEY).as_string().str;

            if(DDS_NEW_TYPE_FUNC_MAP.find(topic_type) == DDS_NEW_TYPE_FUNC_MAP.end()
               ||DDS_READ_FUNC_MAP.find(topic_type) == DDS_READ_FUNC_MAP.end() ){
                MLOGI("%s, key %s has problem in finding topic_type %s\n", TOML_CHANNEL_KEY, x.first.c_str(), topic_type.c_str());
                error = true;
            }
            if (error) {
                return -1;
            }

            if(std::strcmp(channel_type.c_str(),TOML_DDS_CHANNEL_MODE_WRITE ) == 0){
                auto it = channel_holder_map.try_emplace(x.first, DDSWriter());
                if(!it.second){
                    MLOGI("%s, key %s has problem in create_from_toml %s, already existed\n", TOML_CHANNEL_KEY, x.first.c_str(), channel_type.c_str());
                    error = true;
                }else{
                    if(absl::get<DDSWriter>(it.first->second).create_from_toml(x.second) < 0){
                        MLOGI("%s, key %s has problem in create_from_toml %s, error\n", TOML_CHANNEL_KEY, x.first.c_str(), channel_type.c_str());

                        error = true;
                    }
                }

            }else if(std::strcmp(channel_type.c_str(),TOML_DDS_CHANNEL_MODE_READ ) == 0){
                auto it = channel_holder_map.try_emplace(x.first, DDSReader());
                if(!it.second){
                    MLOGI("%s, key %s has problem in create_from_toml %s, already existed\n", TOML_CHANNEL_KEY, x.first.c_str(), channel_type.c_str());
                    error = true;
                }else{

                    if(absl::get<DDSReader>(it.first->second).create_from_toml(x.second)){
                        MLOGI("%s, key %s has problem in create_from_toml %s, error\n", TOML_CHANNEL_KEY, x.first.c_str(), channel_type.c_str());

                        error = true;
                    }
                }

            }else{
                MLOGI("%s, key %s has problem in finding channel_type %s\n", TOML_CHANNEL_KEY, x.first.c_str(), channel_type.c_str());
                error = true;
            }

            if (error) {
                return -1;
            }
        }
        return 0;

    }


    int write(const char* channel_name){

        auto it  = channel_holder_map.find(channel_name);
        if(it == channel_holder_map.end()){
            MLOGW("%s not exist\n", channel_name);
            return -1;
        }
        if(!absl::holds_alternative<DDSWriter>(it->second) ){
            MLOGW("%s is not writer\n", channel_name);
            return -1;
        }
        return absl::get<DDSWriter>(it->second).write();
    }


    int read(const char* channel_name){

        auto it  = channel_holder_map.find(channel_name);
        if(it == channel_holder_map.end()){
            MLOGW("%s not exist\n", channel_name);
            return -1;
        }
        if(!absl::holds_alternative<DDSReader>(it->second) ){
            MLOGW("%s is not reader\n", channel_name);
            return -1;
        }
        return absl::get<DDSReader>(it->second).read();
    }

    void clean(){

        for(auto it = channel_holder_map.begin(); it != channel_holder_map.end();it++){
            auto& holder = it->second;
            if( absl::holds_alternative<DDSWriter>(holder)){
                absl::get<DDSWriter>(holder).clean();
            }
            if( absl::holds_alternative<DDSReader>(holder)){
                absl::get<DDSReader>(holder).clean();
            }

        }
        channel_holder_map.clear();
    }
};



void* dds_create_from_toml(const char* filename){
    DdsHandler* handler = new DdsHandler();
    int rt = handler->create_from_toml(filename);

    if(rt < 0 ){
        handler->clean();
        delete handler;
        handler = nullptr;
    }

    return handler;
}
void dds_clean_handler(void* vp_handler){
    if(vp_handler){
        DdsHandler* handler = (DdsHandler*)vp_handler;
        handler->clean();
        delete handler;
    }
}



int dds_write_channel(void* vp_handler, const char* channel_name){
    if(!vp_handler){
        return -1;
    }
    DdsHandler* handler = (DdsHandler*)vp_handler;
    return handler->write(channel_name);
}
int dds_read_channel(void* vp_handler, const char* channel_name){
    if(!vp_handler){
        return -1;
    }
    DdsHandler* handler = (DdsHandler*)vp_handler;
    return handler->read(channel_name);

}


