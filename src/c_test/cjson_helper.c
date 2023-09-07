//
// Created by waxz on 9/7/23.
//

#include "cjson_helper.h"
#include <stdio.h>
#include <stdbool.h>
#include "common/string_func.h"

//===================
// json
//===================
void *test_pointer(void *p) {
    return p;
}

void *json_parse(const char *str) {

    cJSON *json = cJSON_Parse(str);

    printf("[cJSON:INFO] parse: %s\n", str);
    if (json == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            fprintf(stderr, "Error before: %s\n", error_ptr);
        }
    }
    printf("[cJSON:INFO] parse result: %p\n", json);

    return json;
}

void json_delete(void *j) {
    cJSON *json = (cJSON *) j;
    printf("[cJSON:INFO] delete: %p\n", json);

    if (json) {
        cJSON_Delete(json);
    }
}

void json_print(void *j) {
    const cJSON *json = (cJSON *) j;
    if (json) {
        char *encode_string = cJSON_Print(json);
        if (encode_string == NULL) {
            fprintf(stderr, "Failed to print monitor.\n");
        }

        printf("encode_string: %s\n", encode_string);
        free(encode_string);

    }
}

cJSON *json_get(void *j, const char *str) {
    const cJSON *json = (cJSON *) j;

    return cJSON_GetObjectItemCaseSensitive(json, str);
}

cJSON *json_get_array(void *j, int key) {
    const cJSON *json = (cJSON *) j;

    return cJSON_GetArrayItem(json, key);
}


cJSON *json_get_recursive(cJSON *json, const char *key) {

    if (json == NULL) {
        return NULL;
    }

    cJSON *temp_object = json;
    char *x;
    int xi;
    int array_index = 0;
    bool fault = false;
    SPLIT_STRING(key, ":", xi, x, {
        if (!fault && temp_object != NULL) {
            if (str2int(&array_index, x, 10) == STR2INT_SUCCESS) {
                temp_object = json_get_array(temp_object, array_index);
            }
            else { temp_object = json_get(temp_object, x); }
        }
        else { fault = true; }
    });

    if (fault) {
        return NULL;
    }
    return temp_object;

}


int json_get_int(void *j, const char *key, int *dst) {
    cJSON *json = (cJSON *) j;

    if (json == NULL) {
        return -1;
    }
    json = json_get_recursive(json, key);
    if (!json || !cJSON_IsNumber(json)) {

        return -1;
    }
    *dst = json->valueint;
    return 0;
}

int json_get_double(void *j, const char *key, double *dst) {
    cJSON *json = (cJSON *) j;

    if (json == NULL) {
        return -1;
    }
    json = json_get_recursive(json, key);
    if (!json || !cJSON_IsNumber(json)) {

        return -1;
    }
    *dst = json->valuedouble;
    return 0;

}


int json_get_str(void *j, const char *key, char **dst) {
    cJSON *json = (cJSON *) j;

    if (json == NULL) {
        return -1;
    }
    json = json_get_recursive(json, key);
    if (!json || !cJSON_IsString(json)) {

        return -1;
    }
    *dst = json->valuestring;
    return 0;

}

int json_set_int(void *j, const char *key, int dst) {
    cJSON *json = (cJSON *) j;

    if (json == NULL) {
        return -1;
    }
    json = json_get_recursive(json, key);
    if (!json || !cJSON_IsNumber(json)) {

        return -1;
    }
    cJSON_SetIntValue(json, dst);
    return 0;
}

int json_set_double(void *j, const char *key, double dst) {
    cJSON *json = (cJSON *) j;

    if (json == NULL) {
        return -1;
    }
    json = json_get_recursive(json, key);
    if (!json || !cJSON_IsNumber(json)) {

        return -1;
    }
    cJSON_SetNumberHelper(json, dst);
    return 0;
}

int json_set_str(void *j, const char *key, const char *dst) {
    cJSON *json = (cJSON *) j;

    if (json == NULL) {
        return -1;
    }
    json = json_get_recursive(json, key);
    if (!json || !cJSON_IsString(json)) {

        return -1;
    }
    cJSON_SetValuestring(json, dst);
    return 0;
}