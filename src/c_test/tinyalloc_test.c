//
// Created by waxz on 9/6/23.
//

#include "tcc_helper.h"
#include "cjson/cJSON.h"

#include <threads.h>
#include <stdlib.h>
#include <string.h>
#include "tinyalloc/tinyalloc.h"

//https://github.com/SundayRX/tinyalloc

#include "common/data_holder.h"


typedef struct{
    float x;
    float y;
    float z;
} Point;
typedef struct{
    float x;
    float y;
    float z;
} Point2;


//https://www.geeksforgeeks.org/difference-between-malloc-and-calloc-with-examples/
//https://stackoverflow.com/questions/1538420/difference-between-malloc-and-calloc
/*


Number of blocks:
malloc() assigns single block of requested memory,
calloc() assigns multiple blocks of the requested memory

Initialization:
malloc() - doesn't clear and initialize the allocated memory.
calloc() - initializes the allocated memory by zero.

Speed:
malloc() is fast.
calloc() is slower than malloc().

Arguments & Syntax:
malloc() takes 1 argument:

    bytes
        The number of bytes to be allocated

calloc() takes 2 arguments:

    length
        the number of blocks of memory to be allocated

    bytes
        the number of bytes to be allocated at each block of memory

void *malloc(size_t bytes);
void *calloc(size_t length, size_t bytes);

Manner of memory Allocation:
The malloc function assigns memory of the desired 'size' from the available heap.
The calloc function assigns memory that is the size of what’s equal to ‘num *size’.

Meaning on name:
The name malloc means "memory allocation".
The name calloc means "contiguous allocation".



 *
 * */
typedef void (*hello)(int);

typedef struct{
    // byte number
    size_t size;
    // buffer pointer
    void* buffer;
} CVector, *CVectorPtr;


void copy(CVectorPtr vec_source,CVectorPtr vec_dest){
    vec_dest->size = vec_source->size;
    if(!vec_dest->buffer){
        vec_dest->buffer = ta_alloc(vec_dest->size);
    }else{
        ta_realloc(vec_dest->buffer,vec_dest->size);
    }
    memcpy(vec_dest->buffer, vec_source->buffer,vec_source->size );
}


#define MEMORY_SIZE  1000000
static thread_local unsigned char memory_pool[MEMORY_SIZE];


int main(int argc, char** argv){

    ta_init(memory_pool,memory_pool + MEMORY_SIZE,5000,16,8);


    {

        DataHolderContainer  c = Container_create();

        DataHolder D = DataHolder_create(c.typename);
        D.set_struct_buffer(&D, 5);
        D.set_pool_buffer(&D, 1024);
        D.reset_allocate(&D);
        printf("get pool_buffer  = %p , pool_buffer+400  = %p\n",D.pool_buffer,  D.pool_buffer + 400);


        float cnt = 0.007f;
        for(int i = 0 ; i < 10;i++){
            DataHolderContainer *p = ta_alloc(sizeof (DataHolderContainer));
            *p = Container_create();

            D.add_struct(&D,p,10);

            p->float_array_size = 400;
            p->float_array_index = D.allocate(&D, p->float_array_size , sizeof (float),100);
            float* fp = D.get_buffer(&D, p->float_array_index);
            printf("index = %lu, p->float_array = %p, size = %lu\n",D.pool_index,   fp, p->float_array_size * sizeof (float));
            for(int j = 0; j < p->float_array_size;j++){
                fp[j] = cnt;
                cnt+=1;
            }
        }

        printf("get pool_buffer  = %p , pool_buffer+400  = %p\n",D.pool_buffer,  D.pool_buffer + 400);

        for(int i = 0 ; i < 11;i++){
            DataHolderContainer *p = D.get_struct(&D,i);
            if( !p){
                break;
            }
            float*fp = D.get_buffer(&D, p->float_array_index);
            printf("get get_struct[%i] = %p, float_array pointer = %p , float_array = [ ",i, p, fp);
            for(int j = 0; j < p->float_array_size;j++){
                printf("%f, ", fp[j] );
            }
            printf("]\n");

        }

        printf("edge condition, %lu, %lu,  %p\n", D.pool_size_allocated, 0,  D.get_buffer(&D, 0));
        printf("edge condition, %lu, %lu,  %p\n", D.pool_size_allocated, D.pool_size_allocated-1,  D.get_buffer(&D, D.pool_size_allocated-1));
        printf("edge condition, %lu, %lu,  %p\n", D.pool_size_allocated, D.pool_size_allocated,  D.get_buffer(&D, D.pool_size_allocated));
        printf("edge condition, %lu, %lu,  %p\n", D.pool_size_allocated, D.pool_size_allocated+1,  D.get_buffer(&D, D.pool_size_allocated+1));

        printf("ta_num_used : %lu\n",ta_num_used());
        printf("ta_num_free : %lu\n",ta_num_free());
        printf("ta_num_fresh : %lu\n",ta_num_fresh());
        printf("ta_check : %i\n",ta_check());

        D.clean(&D);
        printf("ta_num_used : %lu\n",ta_num_used());
        printf("ta_num_free : %lu\n",ta_num_free());
        printf("ta_num_fresh : %lu\n",ta_num_fresh());
        printf("ta_check : %i\n",ta_check());

        return 0;
    }

    Point2 p2;

    Point* p = ta_alloc(sizeof(Point));
    if(p){

        p->x = 1.0f;

        ta_free(p);

    }


    CVector v1;
    float float_array[] = {1.,2.,3.,4.};

    v1.size = sizeof(float) * 4;
    v1.buffer = float_array;
    CVector v2;
    v2.buffer = NULL;
    copy(&v1,&v2);

    for(int i =0 ; i < 4;i++){
        printf("v2 = %f",*((float*)(v2.buffer) + i) );

    }



    tcc_script_t script;

    script = tcch_create_script();
    if(!script.tcc){
        return 0;
    }


    int rt;
    int num = 555;
    tcch_add_symbol(&script,"num",&num);

    script.string_code =
            "#include <tinyalloc/tinyalloc.h>"
            "extern int num;"
                         "unsigned char memory_pool_embed[10240];"
//

                         "void hello(int a){"
//                         "ta_init(memory_pool_embed,memory_pool_embed + 10240,256,16,8);"
                         "printf(\"ta_init in tcc\\n   \");"
                         "printf(\"num=%i , a = %i\\n\",num, a);"
                         "num = a;"
                         "};";
    rt = tcch_compile_program(&script,NULL);

    if(rt < 0){
        tcch_delete_script(&script);

        return 0;
    }



    script.string_code =

                         "#include <tinyalloc/tinyalloc.h>"
//                         "#include <threads.h>"
                         "typedef struct{\n"
                         "    float x;\n"
                         "    float y;\n"
                         "    float z;\n"
                         "} Point2;"
                         "typedef struct\n"
                         "{\n"
                         " int a;\n"
                         " int b;\n"
                         "} S1, *S1PTR;"
                         "extern int num;"
                         "int nx = 34;"
                         ""
                         "void test_alloc(int a){"
                         "int* p = ta_alloc(40);;"
                         "int*  s;"
                         "S1PTR s2;"
                         " s = NULL;;"
                         ""
                         ";"
                         "if(p){"
                         "printf(\"ta_alloc p = %p\\n\",p);"
                         "for(int i = 0 ; i < 10;i++){"
                         "p[i] = i;"
                         "}"
                         "}"

                         "\n"

                         "};";


    rt = tcch_compile_program(&script,NULL);




    if(rt < 0){
        tcch_delete_script(&script);

        return 0;
    }

    tcch_relocate_program(&script);

    int* nx = tcch_get_symbol(&script,"nx");
    if(nx){
        printf("nx: %p, %i\n",nx,*nx);

    }
    printf("nx: %p\n",nx);

    hello hello_func = tcch_get_symbol(&script,"hello");
    hello test_alloc = tcch_get_symbol(&script,"test_alloc");
    hello_func(12);
    test_alloc(12);
    if(nx){
        printf("nx: %p, %i\n",nx,*nx);

    }
    printf("nx: %p\n",nx);

    tcch_delete_script(&script);


}
