//
// Created by waxz on 9/7/23.
//

#ifndef CMAKE_SUPER_BUILD_CJSON_HELPER_H
#define CMAKE_SUPER_BUILD_CJSON_HELPER_H


#include "cjson/cJSON.h"




#ifdef __cplusplus
extern "C" {
#endif


//====
// json
//====
void* test_pointer(void*);
cJSON *json_get(void *j, const char *key);
cJSON *json_get_array(void *j, int key);
cJSON * json_get_recursive(cJSON * json , const char *key);

void* json_parse(const char *str);
void json_delete(void *j);
void json_print(void *j);
int json_get_int(void *j, const char *key, int * dst);
int json_get_double(void *j, const char *key, double * dst);
int json_get_str(void *j, const char *key, char** dst);
int json_set_int(void *j, const char *key, int dst);
int json_set_double(void *j, const char *key, double dst);
int json_set_str(void *j, const char *key,const char* dst);


#ifdef __cplusplus
}
#endif
#endif //CMAKE_SUPER_BUILD_CJSON_HELPER_H
