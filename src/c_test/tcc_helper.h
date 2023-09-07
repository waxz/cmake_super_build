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

#include "cjson_helper.h"

#ifdef __cplusplus
extern "C" {
#endif




//====
//tcc
//====



typedef struct
script_t {
    TCCState *tcc;
    char *string_code;
    void *program;
    time_t time_modified;

} script_t;

TCCState *tcch_create_tcc();

script_t tcch_create_script();

int tcch_compile_program(script_t *script, const char* imp);

int tcch_relocate_program(script_t *script);

void *tcch_get_symbol(script_t *script, const char *name);

int tcch_add_symbol(script_t *script, const char *name, void *ptr);

void tcch_delete_script(script_t *script);

#ifdef __cplusplus
}
#endif

#endif //CMAKE_SUPER_BUILD_TCC_HELPER_H
