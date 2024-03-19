//
// Created by waxz on 9/3/23.
//

#ifndef CMAKE_SUPER_BUILD_TCC_DDS_HELPER_H
#define CMAKE_SUPER_BUILD_TCC_DDS_HELPER_H

#include "dds/dds.h"
#include "dds/ddsc/dds_loan_api.h"
#include "dds/ddsrt/environ.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdint.h>


#include "tcc_helper.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef void *(*create_buffer)(void);

typedef void *(*remove_buffer)(void *);

typedef void(* prepare_sample)(void*);
typedef void(* process_sample)(void*);

#define DDS_MAX_BUFFER_SIZE 20

typedef struct {
    dds_entity_t domain;
    dds_entity_t participant;

    dds_entity_t writer;
    dds_entity_t topic;
    dds_entity_t publisher;
    void *buffer[DDS_MAX_BUFFER_SIZE];

    script_t script;

    create_buffer create_buffer_func;
    remove_buffer free_buffer_func;
    prepare_sample prepare_sample_func;

} ddsh_writer_t;


typedef struct {
    dds_entity_t domain;
    dds_entity_t participant;

    dds_entity_t reader;
    dds_entity_t topic;
    dds_entity_t subscriber;
    void *buffer[DDS_MAX_BUFFER_SIZE];


    script_t script;

    process_sample process_sample_func;
    create_buffer create_buffer_func;
    remove_buffer free_buffer_func;

} ddsh_reader_t;

// create domain
dds_entity_t ddsh_create_participant(uint32_t domain_id);


// create writer

void * ddsh_create_writer(uint32_t domain_id, const char *topic_type, const char *topic_name, const char *partition_name, const char* callback, const char* callback_impl);

void* ddsh_writer_get_symbol(void *handler, const char* name);

void* ddsh_set_symbol(void *handler, const char* name);

int ddsh_write_signal(void *handler);
int ddsh_read_signal(void *handler);

/// msg_size is total byte number, for example ThroughputModule_DataType_1048576, msg_size = count(8bytes) + payloadsize(4bytes) + payload_buffer(dynamic_size)
void ddsh_write(void *handler, void *sample, size_t msg_size);


void ddsh_write_batch(void *handler, void *sample, size_t msg_num);

void ddsh_stop_writer(void *handler);

// create reader
void *
ddsh_create_reader(uint32_t domain_id, const char *topic_type, const char *topic_name, const char *partition_name, const char* callback, const char* callback_impl);

void* ddsh_reader_get_symbol(void *handler, const char* name);

void ddsh_stop_reader(void *handler);

// normal read from
void ddsh_read_batch(void *handler, void *sample, size_t msg_num);

// use dds_take dds_take_wl
void ddsh_read_batch_loan(void *handler, void *sample, size_t msg_num);

#ifdef __cplusplus
}
#endif
#endif //CMAKE_SUPER_BUILD_TCC_DDS_HELPER_H
