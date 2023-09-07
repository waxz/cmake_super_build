//
// Created by waxz on 9/6/23.
//


#include <stdbool.h>
#include "common/string_func.h"

#include "tcc_helper.h"

#include "cjson_helper.h"


//===================
// TCC
//===================

void tcc_error(void *opaque, const char *msg) {
    printf("[TCC:ERR] %s\n", msg);
}


TCCState *tcch_create_tcc() {
    const char *TCC_HOME = getenv("TCC_HOME");
    if (TCC_HOME == NULL) {
        printf("[TCC:ERR] TCC_HOME is NOT DEFINED\n");
        return NULL;
    }

    printf("GCC version %i.%i.%i\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    char gcc_include_path[100];
    sprintf(gcc_include_path, "/usr/lib/gcc/x86_64-linux-gnu/%i/include", __GNUC__);
    printf("gcc_include_path: %s\n", gcc_include_path);


    char lib_path[100];
    sprintf(lib_path, "%s/lib/tcc", TCC_HOME);
    char include_path[100];
    sprintf(include_path, "%s/lib/tcc/include", TCC_HOME);

    TCCState *tcc = tcc_new();
    if (tcc) {

        tcc_set_lib_path(tcc, lib_path);

        ///usr/lib/gcc/x86_64-linux-gnu/9/include/stdint-gcc.h:60: error: ';' expected (got "int_least8_t")
        //tcc_add_include_path(tcc, gcc_include_path);

        tcc_add_include_path(tcc, include_path);
        tcc_set_error_func(tcc, 0x0, tcc_error);
        tcc_set_options(tcc, "-g -std=c11 -pedantic-errors -Wall -m64 -bench -I/usr/include");
        tcc_set_output_type(tcc, TCC_OUTPUT_MEMORY);

        {
            const char *TCC_HEADER = getenv("TCC_HEADER");
            if (TCC_HEADER) {
                printf("[TCC:INFO] TCC_HEADER:%s\n", TCC_HEADER);
                char *x;
                int xi = 0;
                SPLIT_STRING(TCC_HEADER, ":", xi, x, tcc_add_include_path(tcc, x));
            }
        }
        {
            const char *TCC_LINK_PATH = getenv("TCC_LINK_PATH");
            if (TCC_LINK_PATH) {
                printf("[TCC:INFO] TCC_LINK_PATH:%s\n", TCC_LINK_PATH);
                char *x;
                int xi = 0;
                SPLIT_STRING(TCC_LINK_PATH, ":", xi, x, tcc_add_library_path(tcc, x));
            }
        }
        {
            const char *TCC_LINK_LIB = getenv("TCC_LINK_LIB");
            if (TCC_LINK_LIB) {
                printf("[TCC:INFO] TCC_LINK_LIB:%s\n", TCC_LINK_LIB);
                char *x;
                int xi = 0;

                SPLIT_STRING(TCC_LINK_LIB, ":", xi, x, { tcc_add_library(tcc, x); });
            }
        }
        {
            const char *TCC_SOURCE = getenv("TCC_SOURCE");
            if (TCC_SOURCE) {
                printf("[TCC:INFO] TCC_ADD_SOURCE:%s\n", TCC_SOURCE);
                char *x;
                int xi = 0;

                SPLIT_STRING(TCC_SOURCE, ":", xi, x, tcc_add_file(tcc, x));
            }

        }
        {
            const char *TCC_OPTION = getenv("TCC_OPTION");
            if (TCC_OPTION) {
                printf("[TCC:INFO] TCC_OPTION:%s\n", TCC_OPTION);
                char *x;
                int xi = 0;

                SPLIT_STRING(TCC_OPTION, ":", xi, x, tcc_set_options(tcc, x));
            }

        }

        {
            // add cjson binding

            tcc_add_symbol(tcc,"json_parse", json_parse);
            tcc_add_symbol(tcc,"json_delete", json_delete);

            tcc_add_symbol(tcc,"json_print", json_print);

            tcc_add_symbol(tcc,"json_set_int", json_set_int);
            tcc_add_symbol(tcc,"json_set_double", json_set_double);
            tcc_add_symbol(tcc,"json_set_str", json_set_str);
            tcc_add_symbol(tcc,"json_get_int", json_get_int);
            tcc_add_symbol(tcc,"json_get_double", json_get_double);
            tcc_add_symbol(tcc,"json_get_str", json_get_str);


            tcc_add_symbol(tcc,"test_pointer", test_pointer);

        }




        printf("[TCC:INFO] tcc config done!\n");

    } else {
        printf("[TCC:ERR] Failed to create tcc context!\n");
    }
    return tcc;
}

script_t tcch_create_script() {
    script_t script;
    script.tcc = tcch_create_tcc();
    return script;
}

int tcch_compile_program(script_t *script, const char* imp) {
    if (!script->tcc) {
        printf("[TCC:ERR] Failed to create tcc context!\n");
        return -1;
    }

    size_t len = strlen(script->string_code);


    char* full_code = script->string_code;
    bool add_forward_declaration = false;
    if(imp != NULL){

        if(strcmp(imp,"cjson") == 0){

            // there may be bug in tcc_add_symbol, void* json_parse(const char *str) will return 4 byte pointer, if funtion is not forward declared
            // but tcc has actually pointer with 8 byte
            //https://stackoverflow.com/questions/34914820/function-without-any-return-type
            // The return type would then default to int if not stated explicitly. Similarly if no function prototype was visible,
            // the compiler would make up a nonsense function declaration based on the function call,
            // where the return type would default to int, if no prototype was visible.
            char const* cjson_header = "void* json_parse(const char *str);\n"
                                       "void json_delete(void *j);\n"
                                       "void *json_print(void *j);\n"
                                       "int json_get_int(void *j, const char *key, int * dst);\n"
                                       "int json_get_double(void *j, const char *key, double * dst);\n"
                                       "int json_get_str(void *j, const char *key, char** dst);\n"
                                       "int json_set_int(void *j, const char *key, int dst);\n"
                                       "int json_set_double(void *j, const char *key, double dst);\n"
                                       "int json_set_str(void *j, const char *key, char* dst);";

            char* dst_str = (char *) malloc(1 + strlen(cjson_header)+ strlen(full_code) );
            strcpy(dst_str, cjson_header);
            strcat(dst_str, full_code);
            full_code = dst_str;
            add_forward_declaration = true;
        }
    }
    int ret = tcc_compile_string(script->tcc, full_code);

    if (ret < 0) {
        printf("[TCC:ERR] Failed to add tcc file!\n");
        tcc_delete(script->tcc);
        script->tcc = NULL;
        return -1;
    }
    printf("[TCC:INFO] Success to add tcc file!\n");

    if(add_forward_declaration){
        free(full_code);
    }
    return 0;
}

int tcch_relocate_program(script_t *script) {
    if (!script->tcc) {
        printf("[TCC:ERR] Failed to create tcc context!\n");
        return -1;
    }
    // tcc_relocate called with NULL returns the size that's necessary for the added files.
    script->program = calloc(1, tcc_relocate(script->tcc, NULL));
    if (!script->program) {
        printf("[TCC:ERR] Failed to allocate memory for the program!\n");
        tcc_delete(script->tcc);
        script->tcc = NULL;

        return -1;
    }


    // Copy code to memory passed by the caller. This is where the compilation happens (I think...).
    int ret = tcc_relocate(script->tcc, script->program);
    if (ret < 0) {
        printf("[TCC:ERR] Failed to allocate memory for the program!\n");
        tcc_delete(script->tcc);
        script->tcc = NULL;

        return -1;
    }

    return 0;

}

/// run after tcch_compile_program(&script); tcch_relocate_program(&script);
void *tcch_get_symbol(script_t *script, const char *name) {
    void *ptr = tcc_get_symbol(script->tcc, name);
    if (!ptr) {
        printf("[TCC:ERR] Failed to get symbol [%s]!\n", name);
        return NULL;
    }
    return ptr;
}

int tcch_add_symbol(script_t *script, const char *name, void *ptr) {
    int rt = tcc_add_symbol(script->tcc, name, ptr);
    if (!ptr) {
        printf("[TCC:ERR] Failed to add symbol [%s], ptr [%p]!\n", name, ptr);
    }
    return rt;
}


void tcch_delete_script(script_t *script) {

    if (script->tcc) {
        printf("[TCC:INFO] tcc delete done!");
        tcc_delete(script->tcc);
        script->tcc = NULL;

    }

}
