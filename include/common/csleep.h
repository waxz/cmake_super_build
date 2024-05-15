//
// Created by waxz on 9/8/23.
//

#ifndef CMAKE_SUPER_BUILD_CSLEEP_H
#define CMAKE_SUPER_BUILD_CSLEEP_H


//https://stackoverflow.com/questions/13156031/measuring-time-in-c
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

    static inline void cnanosleep(int ms){
        struct timespec t = {};
        t.tv_sec = 1;
        t.tv_nsec = 0;
        nanosleep(&t, &t);
    }




#ifdef __cplusplus
}
#endif

#endif //CMAKE_SUPER_BUILD_CSLEEP_H
