//
// Created by waxz on 9/21/23.
//

#ifndef CMAKE_SUPER_BUILD_DATA_HOLDER_H
#define CMAKE_SUPER_BUILD_DATA_HOLDER_H


//#include "common/string_logger.h"
#include "common/c_style.h"



#ifdef __cplusplus
extern "C" {
#endif

#include "tinyalloc/tinyalloc.h"
#include <string.h>
#include <stdio.h>

//#define DEBUG

#ifdef DEBUG
#define printf_i(...) printf(__VA_ARGS__)
#else
#define printf_i(...)
#endif


// combine array and lookup table and liked list
// each node should set id string
// each node with same id should link togather
// all node is arranged like an array

// search process: start search as normal array, if the matched node is found, search as linked list
// if reach end but no valid node is found, new node should be created





#ifdef __cplusplus
};
#endif


#endif //CMAKE_SUPER_BUILD_DATA_HOLDER_H
