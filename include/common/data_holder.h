//
// Created by waxz on 9/21/23.
//

#ifndef CMAKE_SUPER_BUILD_DATA_HOLDER_H
#define CMAKE_SUPER_BUILD_DATA_HOLDER_H

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

typedef struct DataHolder {


    // type as string, how to avoid mistakes
    char *type_name;

    size_t struct_size_allocated;
    size_t struct_size_used;

    size_t pool_size_allocated;
    size_t pool_index;


    // each DataHolder hold the same struct
    // each struct can save dynamic data to pool_buffer
    // each struct can save its pointer to struct_buffer
    // final cleaning: free each pointer in struct_buffer, free struct_buffer, free pool_buffer
    void **struct_buffer;
    char *pool_buffer;


    // function
    void (*clean)(struct DataHolder *);

    void (*set_struct_buffer)(struct DataHolder *, size_t);

    void (*set_pool_buffer)(struct DataHolder *, size_t);


    void (*reset_allocate)(struct DataHolder *);

    /// allocate memory in pool_buffer, get index in pool_buffer
    /// \param t: DataHolder
    /// \param n_mem: request member number
    /// \param mem_size: byte number for each member
    /// \param extra_n: extra byte if pool_buffer need expand
    /// \return : when allocate error return 0, when allocate success return index
    size_t (*allocate)(struct DataHolder *t, size_t n_mem, size_t mem_size, size_t extra_n);

    /// get memory address by index
    /// \param t: DataHolder
    /// \param index: index in pool_buffer
    /// \return :  when compute error return NULL, when allocate success return pointer
    void *(*get_buffer)(struct DataHolder *t, size_t index);


    // get n_th struct
    void *(*get_struct)(struct DataHolder *, size_t n);

    void (*add_struct)(struct DataHolder *, void *ptr, size_t);


} DataHolder, *DataHolderPtr;


/// create DataHolder
/// \param type_name: hold data type name
/// \return
DataHolder DataHolder_create(const char *type_name);

/// clean all resource in holder
/// \param t : holder
void DataHolder_clean(DataHolderPtr t);

///
/// \param t
/// \param n
void DataHolder_set_struct_buffer(DataHolderPtr t, size_t n);

///
/// \param t
/// \param n
void DataHolder_set_pool_buffer(DataHolderPtr t, size_t n);

/// reset pool_buffer index
/// \param t
void DataHolder_reset_allocate(DataHolderPtr t);

size_t DataHolder_allocate(DataHolderPtr t, size_t n_mem, size_t mem_size, size_t extra_n);

void *DataHolder_get_buffer(DataHolderPtr t, size_t index);

void *DataHolder_get_struct(DataHolderPtr t, size_t n);

void DataHolder_add_struct(DataHolderPtr t, void *ptr, size_t extra_n);


// data holder
DataHolder DataHolder_create(const char *type_name) {
    DataHolder t;

    t.type_name = ta_alloc(strlen(type_name) + 1);
    printf("t.typename = %p\n", t.type_name);
    strcpy(t.type_name, type_name);

    t.struct_size_allocated = 0;
    t.pool_size_allocated = 0;
    t.struct_size_used = 0;
    t.pool_index = 10;

    t.clean = DataHolder_clean;
    t.set_struct_buffer = DataHolder_set_struct_buffer;
    t.set_pool_buffer = DataHolder_set_pool_buffer;

    t.reset_allocate = DataHolder_reset_allocate;
    t.allocate = DataHolder_allocate;
    t.get_buffer = DataHolder_get_buffer;

    t.get_struct = DataHolder_get_struct;
    t.add_struct = DataHolder_add_struct;

    return t;
}

void DataHolder_clean(DataHolderPtr t) {
    printf_i("clean type_name %s \n", t->type_name);
    printf_i("clean struct_size_allocated %lu \n", t->struct_size_allocated);
    printf_i("clean pool_size_allocated %lu \n", t->pool_size_allocated);
    printf_i("clean struct_size_used %lu \n", t->struct_size_used);

    for (size_t i = 0; i < t->struct_size_used; i++) {
        printf_i("clean struct_buffer [%lu] : %p \n", i, t->struct_buffer[i]);
        ta_free(t->struct_buffer[i]);
    }
    printf_i("clean struct_buffer %p \n", t->struct_buffer);
    printf_i("clean pool_buffer %p \n", t->pool_buffer);

    ta_free(t->type_name);
    ta_free(t->struct_buffer);
    t->struct_buffer = NULL;
    ta_free(t->pool_buffer);
    t->pool_buffer = NULL;

    t->struct_size_allocated = 0;
    t->pool_size_allocated = 0;
    t->struct_size_used = 0;

}

void DataHolder_set_struct_buffer(DataHolderPtr t, size_t n) {
    if (t->struct_size_allocated > 0) {
        printf_i("DataHolder_set_struct_buffer ta_realloc n = %lu \n", n);
        t->struct_buffer = ta_realloc(t->struct_buffer, n * sizeof(void *));
    } else {
        printf_i("DataHolder_set_struct_buffer ta_calloc n = %lu\n", n);

        t->struct_buffer = ta_calloc(n, sizeof(void *));
    }
    t->struct_size_allocated = n;
    printf_i("DataHolder_set_struct_buffer struct_size_allocated n = %lu\n", t->struct_size_allocated);

}

void DataHolder_set_pool_buffer(DataHolderPtr t, size_t n) {
    if (t->pool_size_allocated > 0) {
        printf_i("DataHolder_set_pool_buffer ta_realloc n = %lu \n", n);

        t->pool_buffer = ta_realloc(t->pool_buffer, n * sizeof(char));
    } else {
        printf_i("DataHolder_set_pool_buffer ta_calloc n = %lu\n", n);
        t->pool_buffer = ta_calloc(n, sizeof(char));
    }
    t->pool_size_allocated = n;
    printf_i("DataHolder_set_pool_buffer pool_buffer = %p\n", t->pool_buffer);

    printf_i("DataHolder_set_pool_buffer pool_size_allocated n = %lu\n", t->pool_size_allocated);

}

void DataHolder_reset_allocate(DataHolderPtr t) {
    t->pool_index = 10;
}

size_t DataHolder_allocate(DataHolderPtr t, size_t n_mem, size_t mem_size, size_t extra_n) {

    size_t pool_index = t->pool_index;
    size_t full_size = pool_index + n_mem*mem_size;
    if (full_size <= t->pool_size_allocated) {
    } else {
        t->set_pool_buffer(t, full_size + extra_n);
    }
    t->pool_index = full_size;
    printf_i("DataHolder_allocate pool_buffer = %p, pool_index = %lu\n", t->pool_buffer, pool_index);

    return (t->pool_buffer) ? pool_index : 0;

}

void *DataHolder_get_buffer(DataHolderPtr t, size_t index) {
    return (t->pool_buffer && index >=10 && index < t->pool_size_allocated) ? t->pool_buffer + index : NULL;
}


void *DataHolder_get_struct(DataHolderPtr t, size_t n) {

    if (n >= t->struct_size_used) {
        return NULL;
    } else {
        return t->struct_buffer[n];
    }
}

void DataHolder_add_struct(DataHolderPtr t, void *ptr, size_t extra_n) {
    size_t struct_size_used = t->struct_size_used;
    size_t full_size = struct_size_used + 1;
    if (full_size <= t->struct_size_allocated) {
    } else {
        t->set_struct_buffer(t, full_size + extra_n);
    }
    t->struct_buffer[struct_size_used] = ptr;
    t->struct_size_used++;
}



//
typedef struct DataHolderContainer{
    // must set typename
    const char* typename;
    size_t float_array_size;
    // relative pointer index in memory pool
    // this is fixed value
    // this is still valid after memory pool reallocate
    size_t float_array_index;
}DataHolderContainer,*DataHolderContainerPtr;

DataHolderContainer Container_create(){
    DataHolderContainer t;
    t.typename = "Container";
    t.float_array_size = 0;
    return t;
}

#ifdef __cplusplus
};
#endif


#endif //CMAKE_SUPER_BUILD_DATA_HOLDER_H
