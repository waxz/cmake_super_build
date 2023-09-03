//
// Created by waxz on 9/3/23.
//

//https://github.com/TinyCC/tinycc
//https://ciesie.com/post/tinycc_dynamic_compilation/
//https://github.com/MrOneTwo/tinycc-dynamic-compilation/tree/b1e34c2acf778f13015afc9bbe2b99614c008777

//https://stackoverflow.com/questions/9017573/define-preprocessor-macro-through-cmake
//https://stackoverflow.com/questions/9639449/cmake-how-to-pass-preprocessor-macros


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <signal.h>

#include "libtcc.h"


#include "tcc_libc.h"

void signal_handler(int sig)
{
    exit(EXIT_SUCCESS);
}


typedef int (*script_hello)(int);
typedef float (*script_bye)(float);


typedef struct
script_t
{
    char* string_code;
    void* program;
    time_t time_modified;

} script_t;

static void
tcc_error(void* opaque, const char* msg)
{
    printf("[TCC:ERR] %s\n", msg);
}

/* this strinc is referenced by the generated code */
const char hello[] = "Hello World!";
/* this function is called by the generated code */
int add(int a, int b)
{
    return a + b;
}



TCCState* create_tcc(){
    const char* TCC_HOME = getenv("TCC_HOME");
    if(TCC_HOME == NULL){
        printf("[TCC:ERR] TCC_HOME : %s NOT DEFINE\n", TCC_HOME);
        return NULL;
    }

    char lib_path[100];
    sprintf(lib_path,"%s/lib/tcc", TCC_HOME);
    char include_path[100];
    sprintf(include_path,"%s/lib/tcc/include", TCC_HOME);

    TCCState* tcc = tcc_new();
    if (tcc){

        tcc_set_lib_path(tcc, lib_path);
        tcc_add_include_path(tcc, include_path);

    }else
    {
        printf("[TCC:ERR] Failed to create tcc context!\n");
    }
    return tcc;
}


static int
compile_program(TCCState* tcc, script_t* script)
{

    if (!tcc)
    {
        printf("[TCC:ERR] Failed to create tcc context!\n");
        return -1;
    }


    tcc_set_error_func(tcc, 0x0, tcc_error);
    tcc_set_options(tcc, "-g");
    tcc_set_output_type(tcc, TCC_OUTPUT_MEMORY);

    int ret = tcc_compile_string(tcc, script->string_code);

    if (ret < 0)
    {
        printf("[TCC:ERR] Failed to add tcc file!\n");
        tcc_delete(tcc);
        return -1;
    }




    return 0;
}

static int relocate_program(TCCState* tcc, script_t* script){
    // tcc_relocate called with NULL returns the size that's necessary for the added files.
    script->program = calloc(1, tcc_relocate(tcc, NULL));
    if (!script->program)
    {
        printf("[TCC:ERR] Failed to allocate memory for the program!\n");
        tcc_delete(tcc);
        return -1;
    }


    // Copy code to memory passed by the caller. This is where the compilation happens (I think...).
    int ret = tcc_relocate(tcc, script->program);
    if (ret < 0)
    {
        printf("[TCC:ERR] Failed to allocate memory for the program!\n");
        tcc_delete(tcc);
        return -1;
    }

    return 0;

}

typedef struct{
    float x;
    float y;
    float z;
}  Point;

struct Position{
    float x;
    float y;
    float z;
};
void change_point(Point *p,float a){p->x = a;}




int test_1(){

    Point p;
    change_point(&p,0.1f);


    printf("PATH : %s\n", getenv("PATH"));
    printf("HOME : %s\n", getenv("HOME"));
    printf("ROOT : %s\n", getenv("ROOT"));
    printf("TCC_HOME : %s\n", getenv("TCC_HOME"));
    const char* TCC_HOME = getenv("TCC_HOME");
    printf("TCC_HOME : %s\n", TCC_HOME);
    if(TCC_HOME == NULL){
        printf("TCC_HOME : %s NOT DEFINE\n", TCC_HOME);
        return 0;
    }


    signal(SIGINT, signal_handler);

    script_t script;



    int num = 0;


    script.string_code = "#include <stdio.h>"
                         "extern const char hello[];\n"
                         "extern int num;\n"
                         "extern int add(int a, int b);\n"
                         "extern void test_c(int a );\n"

                         "int target_hello(int a){ "
                         "num = a;"
                         "test_c(a);"
                         " printf(\"tcc hello %s\",hello); "
                         "printf(\"tcc: a=%i\\n\",a); return add(a,10);};"
                         " float target_bye(float a){printf(\"tcc: a=%f\\n\",a);return a + 0.5;};";
    {
        struct timespec t = {};
        t.tv_sec = 0;
        t.tv_nsec = 200000000;
        nanosleep(&t, &t);


        TCCState* tcc = create_tcc();

        if( !tcc){
            return -1;
        }


//        tcc_add_library_path(tcc, "/tmp/cmake_super_build/cmake-build-relwithdebinfo/lib/libtcc_liba.so");
//        tcc_set_lib_path(tcc, "/tmp/cmake_super_build/cmake-build-relwithdebinfo/lib/");
//        tcc_add_library(tcc, "tcc_liba");


        /* relocate the code */
//        if (tcc_relocate(tcc, TCC_RELOCATE_AUTO) < 0)
//            return 1;
//        tcc_add_symbol(tcc, "hello", hello);

        int rt = compile_program(tcc, &script);
        {
            tcc_add_symbol(tcc, "hello", hello);
            tcc_add_symbol(tcc, "add", add);
            tcc_add_symbol(tcc, "num", &num);
            tcc_add_symbol(tcc, "test_c", test_c);

        }
        relocate_program(tcc,&script);


        script_hello hello_func;
        hello_func = (script_hello)tcc_get_symbol(tcc, "target_hello");
        script_bye bye_func;
        bye_func= (script_bye)tcc_get_symbol(tcc, "target_bye");

        if(rt == 0){
            printf("tcc hello: %i\n", hello_func(345));
            printf("tcc bye: %f\n", bye_func(0.1f));
            printf("tcc result: num=%i\n",num);
        }



        /* delete the state */
        tcc_delete(tcc);

    }
}

int main(){

    test_1();

    return 0;
}