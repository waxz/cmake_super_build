//
// Created by waxz on 4/25/24.
//

#include "message_center_types.h"

#include "tinyalloc/tinyalloc.h"

#include <stdio.h>


// 10000000 Byte = 10 MB
// 100000000 Byte = 100 MB

#define STATIC_MEMORY_SIZE 100000000
static unsigned char memory_pool[STATIC_MEMORY_SIZE];
static const ta_cfg_t memory_pool_cfg = {
        .base = memory_pool,
        .limit = &memory_pool[sizeof(memory_pool)],
        .max_blocks = 512,
        .split_thresh = 16,
        .alignment = 8,
};


void ta_print(){
    printf("ta_num_used : %lu\n",ta_num_used(&memory_pool_cfg));
    printf("ta_num_free : %lu\n",ta_num_free(&memory_pool_cfg));
    printf("ta_num_fresh : %lu\n",ta_num_fresh(&memory_pool_cfg));
    printf("ta_check : %i\n",ta_check(&memory_pool_cfg));
}

int main(int argc, char** argv){
    ta_init(&memory_pool_cfg);

    MessageBase  msg = MessageBase_create();
    printf("msg: %s, %u, %u\n", msg.type_name, msg.base_size, msg.buffer_size);

    LaserScan_ptr  scan_ptr = LaserScan_alloc(3000, &memory_pool_cfg);
    printf("scan_ptr: %p, %s, %u, %u\n",scan_ptr, scan_ptr->type_name, scan_ptr->base_size, scan_ptr->buffer_size);
    scan_ptr = LaserScan_realloc(2999, scan_ptr, &memory_pool_cfg);
    printf("scan_ptr: %p, %s, %u, %u\n",scan_ptr, scan_ptr->type_name, scan_ptr->base_size, scan_ptr->buffer_size);


    // [600,800,3] 5.7MB
    // [1200,800,3] 11.4MB
    Pointcloud_ptr cloud_xyz_ptr = Pointcloud_alloc(1200,800,3,&memory_pool_cfg);
    printf("cloud_xyz_ptr: %p, %s, %u, %u\n",cloud_xyz_ptr, cloud_xyz_ptr->type_name, cloud_xyz_ptr->base_size, cloud_xyz_ptr->buffer_size);
    ta_print();


}