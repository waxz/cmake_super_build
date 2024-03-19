//
// Created by waxz on 9/6/23.
//

#ifndef CMAKE_SUPER_BUILD_TCC_HELPER_H
#define CMAKE_SUPER_BUILD_TCC_HELPER_H

#include "libtcc.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <signal.h>
#include <stdbool.h>

#include "cjson_helper.h"

#ifdef __cplusplus
extern "C" {
#endif


#define TCC_BASE_DEF "#include <stdbool.h>\n"\
                 "#include <stdio.h>\n"\
                 "#include <stddef.h>\n"\
                 "#include <stdlib.h>\n"\
                 "#include <pthread.h>\n"      \
                 "#include <time.h>\n"\
                 //typedef signed char int8_t;\n"\
                 "//typedef unsigned char uint8_t;\n"\
                 "//typedef short int16_t;\n"\
                 "//typedef unsigned short uint16_t;\n"\
                 "//typedef int int32_t;\n"\
                 "//typedef unsigned uint32_t;\n"\
                 "//typedef long long int64_t;\n"\
                 "typedef unsigned long long uint64_t;"\
                 "typedef _Atomic(_Bool)              atomic_bool;\n"\
                 "typedef _Atomic(char)               atomic_char;\n"\
                 "typedef _Atomic(signed char)        atomic_schar;\n"\
                 "typedef _Atomic(unsigned char)      atomic_uchar;\n"\
                 "typedef _Atomic(short)              atomic_short;\n"\
                 "typedef _Atomic(unsigned short)     atomic_ushort;\n"\
                 "typedef _Atomic(int)                atomic_int;\n"\
                 "typedef _Atomic(unsigned int)       atomic_uint;\n"\
                 "typedef _Atomic(long)               atomic_long;\n"\
                 "typedef _Atomic(unsigned long)      atomic_ulong;\n"\
                 "typedef _Atomic(long long)          atomic_llong;\n"\
                 "typedef _Atomic(unsigned long long) atomic_ullong;\n"



//====
//tcc
//====



typedef struct {
    TCCState *tcc;
    const char *string_code;
    void *program;
    time_t time_modified;
    bool is_relocated;

} tcc_script_t;

TCCState *tcch_create_tcc();

tcc_script_t tcch_create_script();

int tcch_compile_program(tcc_script_t *script, const char *imp);

int tcch_relocate_program(tcc_script_t *script);

void *tcch_get_symbol(tcc_script_t *script, const char *name);

int tcch_add_symbol(tcc_script_t *script, const char *name, void *ptr);

void tcch_delete_script(tcc_script_t *script);


#ifdef __cplusplus
}
#endif

#endif //CMAKE_SUPER_BUILD_TCC_HELPER_H
