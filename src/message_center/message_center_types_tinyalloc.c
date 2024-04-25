//
// Created by waxz on 4/25/24.
//

#include "message_center_types.h"
#include "tinyalloc/tinyalloc.h"


//==== MessageBasePtrT
MessageBase MessageBase_create(){
    MessageBase t = {0};
    t.type_name = "MessageBase";
    t.base_size = sizeof(MessageBase);
    return t;
}


//==== MessageBasePtrT


STRUCT_NEW(LaserScan);

void LaserScan_set_buffer(LaserScan_ptr t, u32_t size){
    t->ranges_size = size;
    t->buffer_size = (size + size)*sizeof(f32_t) ;
    t->full_size = t->base_size + t->buffer_size;
}
LaserScan_ptr LaserScan_alloc(u32_t size, const ta_cfg_t* cfg){
    LaserScan target = LaserScan_create();
    LaserScan_set_buffer(&target,size);
    LaserScan_ptr ptr = ta_alloc(cfg, target.full_size);
    *ptr = target;
    return ptr;
}
LaserScan_ptr LaserScan_realloc(u32_t size, void* ptr, const ta_cfg_t* cfg){
    LaserScan target = LaserScan_create();
    LaserScan_set_buffer(&target,size);
    LaserScan_ptr new_ptr = ta_realloc(cfg, ptr, target.full_size);
    *new_ptr = target;
    return new_ptr;
}

//
STRUCT_NEW(Pointcloud);
void Pointcloud_set_buffer(Pointcloud_ptr t, u32_t height, u32_t width, u32_t channel){
    t->height = height;
    t->width = width;
    t->channel = channel;

    t->buffer_size = channel*height * width * sizeof (f32_t);
    t->full_size = t->base_size + t->buffer_size;
}

Pointcloud_ptr Pointcloud_alloc(u32_t height, u32_t width, u32_t channel, const ta_cfg_t* cfg){
    Pointcloud target = Pointcloud_create();
    Pointcloud_set_buffer(&target,height, width,channel);
    Pointcloud_ptr ptr = ta_alloc(cfg, target.full_size);
    *ptr = target;
    return ptr;
}

Pointcloud_ptr Pointcloud_realloc(u32_t height, u32_t width, u32_t channel, void* ptr, const ta_cfg_t* cfg){
    Pointcloud target = Pointcloud_create();
    Pointcloud_set_buffer(&target,height, width,channel);
    Pointcloud_ptr new_ptr = ta_realloc(cfg, ptr, target.full_size);
    *new_ptr = target;
    return new_ptr;
}

