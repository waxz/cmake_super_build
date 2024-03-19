//
// Created by waxz on 9/3/23.
//

#include "tcc_dds_helper.h"
#include "common/string_logger.h"

#include "ShmThroughput.h"
//#include "ShmThroughput.he"

#include "dds/ddsc/dds_internal_api.h"

extern const unsigned char ShmThroughput_c[];
extern const unsigned int ShmThroughput_c_len;

//===================
// MACRO
//===================

#include <stdlib.h>

#define new(type, length) malloc(sizeof(type)*(length))
#define delete(x) free(x)


//===================
// DDS
//===================
dds_entity_t ddsh_create_participant(uint32_t domain_id){
    /* A domain participant is created for the default domain. */
    dds_entity_t participant, domain;
//    domain1 = dds_create_domain(4, "<CycloneDDS><Domain><Id>any</Id></Domain></CycloneDDS>");

    participant = dds_create_participant (domain_id, NULL, NULL);
    if (participant < 0)
        DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return participant;
}



void* ddsh_create_writer(uint32_t domain_id, const char* topic_type, const char* topic_name,const char* partition_name, const char* callback, const char* callback_impl){
    dds_entity_t participant, domain;

    const char* config = "${CYCLONEDDS_URI}${CYCLONEDDS_URI:+,}<Discovery><ExternalDomainId>0</ExternalDomainId></Discovery>";

    char *conf_pub = ddsrt_expand_envvars (config, domain_id);


    domain = dds_create_domain(domain_id , conf_pub);
    participant = dds_create_participant (domain_id, NULL, NULL);
    if (participant < 0){
        DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
        return NULL;
    }


    ddsh_writer_t* handler = new(ddsh_writer_t,1);
    handler->create_buffer_func = NULL;
    handler->free_buffer_func = NULL;

    // create tcc function
    handler->script = tcch_create_script();
    if( !handler->script.tcc){
        delete(handler);
        return NULL;
    }


    dds_entity_t topic = DDS_RETCODE_BAD_PARAMETER;
    dds_entity_t publisher;
    dds_entity_t writer;

    const char *pubParts[1];
    dds_qos_t *pubQos;
    dds_qos_t *dwQos;



    // tcc
    const char *code_template =
            "#include <ShmThroughput.h>"
            "void dummy_hello(){"
            "void* sample = 0;"
            "sample = ThroughputModule_DataType_1048576__alloc();"
            "ThroughputModule_DataType_Base* ptr = (ThroughputModule_DataType_Base*)sample;"
            "ptr->payloadsize = 1024 - (uint32_t) sizeof(ThroughputModule_DataType_Base);"
            "ptr->count = 0;"
            "ThroughputModule_DataType_1048576_free(sample, DDS_FREE_ALL);"
            "}"
            "void* create_buffer(){"
            "printf(\"create %s\\n\");"
            "void* sample = 0;"
            "sample = %s__alloc();"
            "return sample;"
            "}"
            "void free_buffer(void *sample){"
            "printf(\"free %s\\n\");"
            "%s_free(sample, DDS_FREE_ALL);"
            "}"
            "";
    char string_code[1000];
    sprintf(string_code,code_template,topic_type,topic_type,topic_type,topic_type );
    printf("string_code len = %lu\n", strlen(string_code));

    // compile 1
    handler->script.string_code = string_code;
    int rt = tcch_compile_program(&handler->script,NULL);
    if(rt < 0){
        delete(handler);
        return NULL;
    }

    // compile 2
    if(callback){
        handler->script.string_code = callback;
        rt = tcch_compile_program(&handler->script,callback_impl);
        if(rt < 0){
            delete(handler);
            return NULL;
        }
    }


    rt = tcch_relocate_program(&handler->script);
    if(rt < 0){
        delete(handler);
        return NULL;
    }
    // topic
    char topic_date_desc[200];
    sprintf(topic_date_desc, "%s_desc", topic_type);
    dds_topic_descriptor_t * topic_descriptor;

    topic_descriptor = tcch_get_symbol(&handler->script, topic_date_desc);

    if(!topic_descriptor){
        printf("[TCC:ERR] Failed to create topic_descriptor for [%s]!\n",topic_date_desc);

        delete(handler);
        return NULL;
    }

    topic  = dds_create_topic(participant,topic_descriptor , topic_name, NULL, NULL);

    /* A publisher is created on the domain participant. */
    pubQos = dds_create_qos ();
    pubParts[0] = partition_name;
    dds_qset_partition (pubQos, 1, pubParts);
    publisher  = dds_create_publisher (participant, pubQos, NULL);
    if (publisher < 0){
        DDS_FATAL("dds_create_publisher: %s\n", dds_strretcode(-publisher));
        delete(handler);
        return NULL;
    }
    dds_delete_qos (pubQos);

    /* A DataWriter is created on the publisher. */
    dwQos = dds_create_qos ();
#if 1
    dds_qset_reliability (dwQos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(10));
    dds_qset_history (dwQos, DDS_HISTORY_KEEP_LAST, 1);
    dds_qset_deadline(dwQos, DDS_INFINITY);
    dds_qset_durability(dwQos, DDS_DURABILITY_VOLATILE);
    dds_qset_liveliness(dwQos, DDS_LIVELINESS_AUTOMATIC, DDS_MSECS(10));
    dds_qset_resource_limits (dwQos, 2, DDS_LENGTH_UNLIMITED, DDS_LENGTH_UNLIMITED);
//    dds_qset_writer_batching(dwQos, true);
#endif
#if 0
    dds_qset_reliability (dwQos, DDS_RELIABILITY_RELIABLE, DDS_SECS (10));
    dds_qset_history (dwQos, DDS_HISTORY_KEEP_ALL, 0);
    dds_qset_resource_limits (dwQos, 100, DDS_LENGTH_UNLIMITED, DDS_LENGTH_UNLIMITED);
    dds_qset_writer_batching (dwQos, true);
#endif

    writer = dds_create_writer (publisher, topic, dwQos, NULL);
    dds_delete_qos (dwQos);


    // bind function
    handler->prepare_sample_func = NULL;
    handler->prepare_sample_func = tcch_get_symbol(&handler->script,"prepare_sample");

    handler->create_buffer_func = tcch_get_symbol(&handler->script,"create_buffer");
    if(!handler->create_buffer_func ){
        printf("[TCC:ERR] Failed to create create_buffer_func for [%s]!\n",topic_date_desc);

        delete(handler);
        return NULL;
    }
    handler->free_buffer_func = tcch_get_symbol(&handler->script,"free_buffer");
    if(!handler->free_buffer_func ){
        printf("[TCC:ERR] Failed to create free_buffer_func for [%s]!\n",topic_date_desc);

        delete(handler);
        return NULL;
    }

    for(int i = 0 ; i < DDS_MAX_BUFFER_SIZE;i++){
        handler->buffer[i] = handler->create_buffer_func();
    }
    if(!handler->buffer ){
        printf("[TCC:ERR] Failed to create buffer for [%s]!\n",topic_date_desc);

        delete(handler);
        return NULL;
    }


    handler->domain =domain;
    handler->participant = participant;
    handler->topic = topic;
    handler->publisher = publisher;
    handler->writer = writer;

    return handler;
}
void* ddsh_writer_get_symbol(void *handler, const char* name){
    if(!handler){
        DDS_FATAL("handler: %p is stopped\n",handler  );
        return NULL;
    }
    ddsh_writer_t* data_writer = (ddsh_writer_t*) handler;
    script_t* script = &data_writer->script;

    return tcch_get_symbol(script,name);
}
void ddsh_write(void* handler,void* sample,size_t msg_size){
    if(!handler){
        DDS_FATAL("handler: %p is stopped\n",handler  );
        return;
    }


    ddsh_writer_t* data_writer = (ddsh_writer_t*) handler;
    dds_entity_t writer = data_writer->writer;
    bool timedOut = false;

    dds_return_t status;
    void *loaned_sample;

    if(dds_is_loan_available(writer)){
//        printf("shm send data\n");

        if ((status = dds_loan_sample(writer, &loaned_sample)) < 0) {
            DDS_FATAL("dds_loan_sample: %s\n", dds_strretcode(-status));
        }else{
            memcpy(loaned_sample, sample, msg_size);
            status = dds_write (writer, loaned_sample);
            if (status == DDS_RETCODE_TIMEOUT)
            {
                timedOut = true;
            }
            else if (status < 0)
            {
                DDS_FATAL("dds_write: %s\n", dds_strretcode(-status));
            }
            else
            {

            }
        }
    }else{
//        printf("simple send data\n");
        status = dds_write (writer, sample);
        if (status != DDS_RETCODE_OK)
            DDS_FATAL("dds_write: %s\n", dds_strretcode(-status));
    }

    dds_write_flush (writer);


}
uint32_t get_ddsc_state_mask(unsigned long s_state,unsigned long v_state, unsigned long i_state )
{
/* Translate DataState to sample, view and instance ulong states. */
//unsigned long s_state = state.sample_state().to_ulong();
//unsigned long v_state = state.view_state().to_ulong();
//unsigned long i_state = state.instance_state().to_ulong();

/* Truncate 'any' status to specific bits. */
s_state &= 0x3;
v_state &= 0x3;
i_state &= 0x7;

/*
 * The IsoCpp state bits should match the ddsc state bits.
 * The only difference is the location within the uint32.
 * So, perform a shift to let the IsoCpp bits match ddsc bits.
 */
/* s_state <<= 0; */
v_state <<= 2;
i_state <<= 4;

/* The mask is all states or-ed. */
return (uint32_t)(s_state | v_state | i_state);
}
int ddsh_read_signal(void *handler){
    if(!handler){
        MLOGI("handler: %p is stopped\n",handler  );
        return -1;
    }


    ddsh_reader_t * data_reader = (ddsh_reader_t*) handler;



    if(!data_reader->process_sample_func){
        MLOGW("%s","process_sample_func is not found");

        return -1;
    }
    dds_entity_t  reader = data_reader->reader;
    dds_return_t status;

    dds_instance_handle_t ph = 0;

    dds_sample_info_t infos[DDS_MAX_BUFFER_SIZE];
    void **samples = data_reader->buffer;
    int samples_received = 0;
//    MLOGI("%s : %i, %i","dds_is_shared_memory_available", dds_is_shared_memory_available(reader)  , dds_is_loan_available(reader));
#if 0
    if(dds_is_loan_available(reader)){

//        MLOGI("%s : %i","dds_is_loan_available", dds_is_loan_available(reader));
        void* loaned_samples[DDS_MAX_BUFFER_SIZE]={NULL};
        for(int i = 0 ; i < DDS_MAX_BUFFER_SIZE;i++){
            loaned_samples[i] = NULL;
        }


        samples_received = dds_take(reader, loaned_samples, infos, DDS_MAX_BUFFER_SIZE,DDS_MAX_BUFFER_SIZE);
        for(int i = 0 ; i < samples_received;i++){
            if(infos[i].valid_data){
                ph = infos[i].publication_handle;

                data_reader->process_sample_func(loaned_samples[i]);
            }
        }

        if(samples_received > 0)
        dds_return_loan(reader,loaned_samples,samples_received);


    }
#endif


#if 1
//cmake-build-relwithdebinfo/force_CycloneDDS-CXX/build/lib/CycloneDDS-CXX/src/CycloneDDS-CXX/src/ddscxx/src/org/eclipse/cyclonedds/sub/AnyDataReaderDelegate.cpp:344

    void ** c_sample_pointers = NULL;
    dds_sample_info_t * c_sample_infos = NULL;
    size_t c_sample_pointers_size = 0;
    uint32_t samples_to_read_cnt = 0;
    bool expect_samples;

    uint32_t ddsc_mask ;//= get_ddsc_state_mask(mask);
    ddsc_mask = 127;
    // init
    c_sample_pointers_size = dds_reader_lock_samples(reader);
    samples_to_read_cnt = DDS_READ_WITHOUT_LOCK;
    if (c_sample_pointers_size)
    {
        c_sample_pointers = calloc(c_sample_pointers_size, sizeof(void*)  );
        c_sample_infos = calloc(c_sample_pointers_size, sizeof(dds_sample_info_t) );
    }
    expect_samples = (c_sample_pointers_size > 0);




    // take
    if (expect_samples)
    {
        dds_return_t ret;
        /* The reader can also be a condition. */
        ret = dds_takecdr(reader,
                          c_sample_pointers,
                          samples_to_read_cnt,
                          c_sample_infos,
                          ddsc_mask);

//        MLOGI("dds_takecdr get %i\n",ret);
        if (ret > 0) {
            /* When > 0, ret represents the number of samples read. */
            samples_received = ret;
            for(int i = 0 ; i < samples_received;i++){
                if(c_sample_infos[i].valid_data){

                    data_reader->process_sample_func(c_sample_pointers[i]);
                }
            }
        }

        free(c_sample_pointers);
        free(c_sample_infos);

    }

//    dds_return_loan(reader,loaned_samples,samples_received);

#endif
#if 0

// dds_take is ok , but slow
    {
        samples_received = dds_take(reader, samples, infos, DDS_MAX_BUFFER_SIZE, DDS_MAX_BUFFER_SIZE);


        for(int i = 0 ; i < samples_received;i++){
            if(infos[i].valid_data){
                ph = infos[i].publication_handle;

                data_reader->process_sample_func(samples[i]);
            }
        }
    }
#endif




    return samples_received;

}

int ddsh_write_signal(void *handler){

    if(!handler){
        MLOGI("handler: %p is stopped\n",handler  );
        return -1;
    }


    ddsh_writer_t* data_writer = (ddsh_writer_t*) handler;

    if(!data_writer->prepare_sample_func){
        MLOGW("%s","prepare_sample_func is not found");

        return -1;
    }



    dds_entity_t writer = data_writer->writer;
    bool timedOut = false;

    dds_return_t status;
    void **loaned_sample;
    void *sample = data_writer->buffer[0];
    if(dds_is_loan_available(writer)){
//        printf("shm send data\n");

        if ((status = dds_loan_sample(writer, &loaned_sample))  == DDS_RETCODE_ERROR ) {

            MLOGI("dds_loan_sample: %s\n", dds_strretcode(-status));
        }else{
            int loaned_samples_size = (int)status;
            data_writer->prepare_sample_func(loaned_sample);
            status = dds_write (writer, loaned_sample);
            if (status == DDS_RETCODE_TIMEOUT)
            {
                timedOut = true;
            }
            else if (status < 0)
            {
                MLOGI("dds_write: %s\n", dds_strretcode(-status));
            }
            else
            {

            }

            status  = dds_return_loan(writer, loaned_sample, 1);
            if(status < 0)
            MLOGI("dds_return_loan: %s\n", dds_strretcode(-status));

        }
    }else{
//        printf("simple send data\n");
        data_writer->prepare_sample_func(sample);
        status = dds_write (writer, sample);

        ThroughputModule_DataType_1048576* data = sample;
        size_t signal = data->count;


        if (status != DDS_RETCODE_OK)
        MLOGI("dds_write: %s\n", dds_strretcode(-status));
    }
    return 0;
}

void ddsh_stop_writer(void* handler){
    if(handler){
        ddsh_writer_t* data_writer = (ddsh_writer_t* )handler;
        dds_entity_t writer = data_writer->writer;
        dds_return_t status = dds_dispose(writer, data_writer->buffer);
        if (status != DDS_RETCODE_TIMEOUT && status < 0)
            DDS_FATAL("dds_dispose: %s\n", dds_strretcode(-status));
        for(int i = 0 ; i < DDS_MAX_BUFFER_SIZE;i++){
            data_writer->free_buffer_func(data_writer->buffer[i]);
        }
        /* delete the state */
        tcc_delete(data_writer->script.tcc);

        //dds_free (sample.payload._buffer);
        status = dds_delete (data_writer->participant);
        if (status < 0)
            DDS_FATAL("dds_delete: %s\n", dds_strretcode(-status));

        free(handler);
    }
}

//static void data_available_handler (dds_entity_t reader, void *arg)
//{
//    (void)arg;
//    (void) do_take (reader);
//}

void* ddsh_reader_get_symbol(void *handler, const char* name){
    if(!handler){
        DDS_FATAL("handler: %p is stopped\n",handler  );
        return NULL;
    }
    ddsh_reader_t * data_reader = (ddsh_reader_t*) handler;
    script_t* script = &data_reader->script;

    return tcch_get_symbol(script,name);
}

void* ddsh_create_reader(uint32_t domain_id, const char* topic_type, const char* topic_name,const char* partition_name, const char* callback, const char* callback_impl){

    dds_entity_t participant, domain;

    const char* config = "${CYCLONEDDS_URI}${CYCLONEDDS_URI:+,}<Discovery><ExternalDomainId>0</ExternalDomainId></Discovery>";

    char *conf_pub = ddsrt_expand_envvars (config, domain_id);


    domain = dds_create_domain(domain_id , conf_pub);
    participant = dds_create_participant (domain_id, NULL, NULL);


    ddsh_reader_t * handler = new(ddsh_reader_t,1);
    handler->create_buffer_func = NULL;
    handler->free_buffer_func = NULL;

    // create tcc function
    handler->script = tcch_create_script();
    if( !handler->script.tcc){
        delete(handler);
        return NULL;
    }

    dds_entity_t topic = DDS_RETCODE_BAD_PARAMETER;
    dds_entity_t subscriber;
    dds_entity_t reader;
    dds_listener_t *rd_listener;

    const char *subParts[1];
    dds_qos_t *subQos = dds_create_qos ();
    dds_qos_t *drQos = dds_create_qos ();


    // tcc
    const char* code_template =
            "#include <ShmThroughput.h>"
            "void dummy_hello(){"
            "void* sample = 0;"
            "sample = ThroughputModule_DataType_1048576__alloc();"
            "ThroughputModule_DataType_Base* ptr = (ThroughputModule_DataType_Base*)sample;"
            "ptr->payloadsize = 1024 - (uint32_t) sizeof(ThroughputModule_DataType_Base);"
            "ptr->count = 0;"
            "ThroughputModule_DataType_1048576_free(sample, DDS_FREE_ALL);"
            "}"
            "void* create_buffer(){"
            "printf(\"create %s\\n\");"
            "void* sample = 0;"
            "sample = %s__alloc();"
            "return sample;"
            "}"
            "void free_buffer(void *sample){"
            "printf(\"free %s\\n\");"
            "%s_free(sample, DDS_FREE_ALL);"
            "}"
            "";
    char string_code[1000];
    sprintf(string_code,code_template,topic_type,topic_type,topic_type,topic_type );
    printf("string_code len = %lu\n", strlen(string_code));

    // compile 1
    handler->script.string_code = string_code;
    int rt = tcch_compile_program(&handler->script,"NULL");
    if(rt < 0){
        delete(handler);
        return NULL;
    }

    // compile 2
    if(callback){
        handler->script.string_code = callback;
        rt = tcch_compile_program(&handler->script,callback_impl);
        if(rt < 0){
            delete(handler);
            return NULL;
        }
    }


    rt = tcch_relocate_program(&handler->script);
    if(rt < 0){
        delete(handler);
        return NULL;
    }
    // topic
    char topic_date_desc[200];
    sprintf(topic_date_desc, "%s_desc", topic_type);
    dds_topic_descriptor_t * topic_descriptor;

    topic_descriptor = tcch_get_symbol(&handler->script, topic_date_desc);
    if(!topic_descriptor){
        printf("[TCC:ERR] Failed to create topic_descriptor for [%s]!\n",topic_date_desc);

        delete(handler);
        return NULL;
    }

    topic  = dds_create_topic(participant,topic_descriptor , topic_name, NULL, NULL);

    /* A Subscriber is created on the domain participant. */

    subParts[0] = partition_name;
    dds_qset_partition (subQos, 1, subParts);
    subscriber = dds_create_subscriber (participant, subQos, NULL);
    if (subscriber < 0)
        DDS_FATAL("dds_create_subscriber: %s\n", dds_strretcode(-subscriber));
    dds_delete_qos (subQos);

    /* A Reader is created on the Subscriber & Topic with a modified Qos. */

    dds_qset_reliability(drQos, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
    dds_qset_history(drQos, DDS_HISTORY_KEEP_LAST, 16);
    dds_qset_deadline(drQos, DDS_INFINITY);
    dds_qset_durability(drQos, DDS_DURABILITY_VOLATILE);
    dds_qset_liveliness(drQos, DDS_LIVELINESS_AUTOMATIC, DDS_SECS(1));
    dds_qset_resource_limits (drQos, 10, DDS_LENGTH_UNLIMITED, DDS_LENGTH_UNLIMITED);

    reader = dds_create_reader (subscriber, topic, drQos, NULL);
    if (reader < 0)
        DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));

    dds_delete_qos (drQos);

    // bind function
    handler->process_sample_func = NULL;
    handler->process_sample_func = tcch_get_symbol(&handler->script,"process_sample");


    handler->create_buffer_func = tcch_get_symbol(&handler->script,"create_buffer");

    if(!handler->create_buffer_func ){
        printf("[TCC:ERR] Failed to create create_buffer_func for [%s]!\n",topic_date_desc);

        delete(handler);
        return NULL;
    }
    handler->free_buffer_func = tcch_get_symbol(&handler->script,"free_buffer");
    if(!handler->free_buffer_func ){
        printf("[TCC:ERR] Failed to create free_buffer_func for [%s]!\n",topic_date_desc);

        delete(handler);
        return NULL;
    }
    for(int i = 0 ; i < DDS_MAX_BUFFER_SIZE;i++){
        handler->buffer[i] = handler->create_buffer_func();
    }
    if(!handler->buffer ){
        printf("[TCC:ERR] Failed to create buffer for [%s]!\n",topic_date_desc);

        delete(handler);
        return NULL;
    }

    handler->domain =domain;
    handler->participant = participant;
    handler->topic = topic;
    handler->subscriber = subscriber;
    handler->reader = reader;

    return handler;

}

void ddsh_stop_reader(void* handler){
    if(handler){
        ddsh_reader_t * data_reader = (ddsh_reader_t* )handler;
        dds_entity_t reader = data_reader->reader;
        dds_return_t status;
        for(int i = 0 ; i < DDS_MAX_BUFFER_SIZE;i++){
            data_reader->free_buffer_func(data_reader->buffer[i]);
        }
        /* delete the state */
        tcc_delete(data_reader->script.tcc);

        //dds_free (sample.payload._buffer);
        status = dds_delete (data_reader->participant);
        if (status < 0)
            DDS_FATAL("dds_delete: %s\n", dds_strretcode(-status));
        free(handler);
    }
}

void ddsh_read_batch(void* handler,void* sample,size_t msg_num){
    if(!handler){
        DDS_FATAL("handler: %p is stopped\n",handler  );
        return;
    }
    ddsh_reader_t * data_reader = (ddsh_reader_t *) handler;
    dds_entity_t reader = data_reader->reader;

    int samples_received;
    dds_sample_info_t *info = NULL;
    dds_instance_handle_t ph = 0;
//    HandleEntry * current = NULL;

    /* Take samples and iterate through them */

    dds_free (info);
}

void ddsh_read_batch_loan(void* handler,void* sample,size_t msg_num){

}
