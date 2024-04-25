//
// Created by waxz on 4/24/24.
//

#ifndef CMAKE_SUPER_BUILD_MESSAGE_CENTER_TYPES_H
#define CMAKE_SUPER_BUILD_MESSAGE_CENTER_TYPES_H


#include "common/c_style.h"
#include "tinyalloc/tinyalloc.h"


#ifdef __cplusplus
extern "C" {
#endif


typedef struct MessageBase{

    const char* type_name ;
    u64_t stamp;
    u32_t base_size ;
    u32_t buffer_size ;
    u32_t full_size ;


    // buffer
    u32_t buffer[0];

}MessageBase,*MessageBasePtr ;

MessageBase MessageBase_create();
MessageBasePtr MessageBaseT_allocate(u32_t size, ta_cfg_t * cfg);
MessageBasePtr MessageBaseT_reallocate(u32_t size, MessageBasePtr ptr, ta_cfg_t* cfg);
u32_t* MessageBaseT_getBuffer(MessageBasePtr ptr);


STRUCT_BEGIN(LaserScan)


    char frame_id[50];
    u64_t stamp;
    f32_t range_min;
    f32_t range_max;
    f32_t angle_min;
    f32_t angle_max;
    f32_t angle_increment;
    u32_t ranges_size;
    f32_t buffer[0];

STRUCT_END(LaserScan)
void LaserScan_set_buffer(LaserScan* t, u32_t size);
LaserScan_ptr LaserScan_alloc(u32_t size, const ta_cfg_t* cfg);
LaserScan_ptr LaserScan_realloc(u32_t size, void* ptr, const ta_cfg_t* cfg);
f32_t* LaserScan_get_ranges(LaserScan* t);
f32_t* LaserScan_get_intensities(LaserScan* t);


STRUCT_BEGIN(Pointcloud)
    char frame_id[50];
    u64_t stamp;
    u32_t height;
    u32_t channel;

    u32_t width;
    f32_t buffer[0];
STRUCT_END(Pointcloud)
void Pointcloud_set_buffer(Pointcloud_ptr t, u32_t height, u32_t width, u32_t channel);
Pointcloud_ptr Pointcloud_alloc(u32_t height, u32_t width, u32_t channel, const ta_cfg_t* cfg);
Pointcloud_ptr Pointcloud_realloc(u32_t height, u32_t width, u32_t channel, void* ptr, const ta_cfg_t* cfg);



#ifdef __cplusplus
}
#endif



#endif //CMAKE_SUPER_BUILD_MESSAGE_CENTER_TYPES_H
