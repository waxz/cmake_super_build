//
// Created by waxz on 9/6/23.
//

#include <string.h>
#include "tcc_helper.h"

typedef void (*hello)(int);

typedef void (*str_func)(char* );

#define STR1(z) #z
#define STR(z) STR1(z)
#define JOIN(a,b,c) a STR(b) c


int main(int argc, char** argv){

    char* str1;
    char* str2;
    str1 = "sssss";
    str2 = "kkkk";
    char * str3 = (char *) malloc(1 + strlen(str1)+ strlen(str2) );
    strcpy(str3, str1);
    strcat(str3, str2);
    printf("str3 = %s\n", str3);
    free(str3);



    char* json_string ="{\n"
                       "    \"name\": \"Awesome 4K\",\n"
                       "    \"price\": 2121.56,\n"
                       "    \"date\": {"
                       "\"year\" : 2023,"
                       "\"month\" : 09,"
                       "\"day\" : 07,"
                       "\"time\" :[16,32,35]"
                       "},\n"
                       "    \"resolutions\": [\n"
                       "        {\n"
                       "            \"width\": 1280,\n"
                       "            \"height\": 720\n"
                       "        },\n"
                       "        {\n"
                       "            \"width\": 1920,\n"
                       "            \"height\": 1080\n"
                       "        },\n"
                       "        {\n"
                       "            \"width\": 3840,\n"
                       "            \"height\": 2160\n"
                       "        }\n"
                       "    ]\n"
                       "}";


    printf("json_string: %s\n",json_string);


    {
        cJSON *json = json_parse(json_string);

        cJSON * date_year_json = json_get_recursive(json,"date:year");

        if(date_year_json){
            printf("date_year_json: %i\n", date_year_json->valueint);

        }

        json_set_int(json,"date:time:0",1);
        json_set_int(json,"date:time:1",11);
        json_set_int(json,"date:time:2",111);

        int year,month,day;
        json_get_int(json,"date:time:0",&year);
        json_get_int(json,"date:time:1",&month);
        json_get_int(json,"date:time:2",&day);
        printf("json_get_ year:%i, month:%i, day:%i\n",year,month,day);
        char * name;
        double price;
        json_set_str(json,"name","Apple");
        json_set_double(json,"price",9.99);
        json_get_str(json,"name",&name);
        json_get_double(json,"price",&price);
        printf("json_get_ name:%s, price:%f\n",name,price);

        char *encode_string = NULL;
        encode_string = cJSON_Print(json);
        if (encode_string == NULL)
        {
            fprintf(stderr, "Failed to print monitor.\n");
        }

        printf("encode_string: %s\n",encode_string);
        free(encode_string);
        json_delete(json);
    }








    script_t script;

    script = tcch_create_script();
    if(!script.tcc){
        return 0;
    }


    int rt;
    int num = 555;
    tcch_add_symbol(&script,"num",&num);

    script.string_code = "extern int num;"
                         "void hello(int a){"
                         "printf(\"num=%i , a = %i\\n\",num, a);"
                         "num = a;"
                         "};";
    rt = tcch_compile_program(&script,NULL);

    if(rt < 0){
        tcch_delete_script(&script);

        return 0;
    }

    script.string_code = "extern int num;"
                         "void hello2(int a){"
                         "printf(\"num=%i , a = %i\\n\",num, a);"
                         "num = a;"
                         "};";


    rt = tcch_compile_program(&script,NULL);

    if(rt < 0){
        tcch_delete_script(&script);

        return 0;
    }

    script.string_code = ""
//                         "#include <x86intrin.h>"
                         "void test_asm(int src){"
                         "int vi[4] = {1,2,3,4};"
                         "int dst;\n"
                         "\n"
                         "asm (\"mov %1, %0\\n\\t\"\n"
                         "    \"add $1, %0\"\n"
                         "    : \"=r\" (dst)\n"
                         "    : \"r\" (src));\n"
                         "\n"
                         "printf(\"asm, dst = %d\\n\", dst);"
                         "typedef int v4si __attribute__ ((vector_size (16)));\n"
                         "typedef float v4sf __attribute__ ((vector_size (16)));\n"
                         "typedef double v4df __attribute__ ((vector_size (32)));\n"
                         "typedef unsigned long long v4di __attribute__ ((vector_size (32)));\n"
//                         "v4si a = {1,2,3,4};"
//                         "v4sf b = {1.5f,-2.5f,3.f,7.f};\n"
//                         "v4di c = {1ULL,5ULL,0ULL,10ULL};\n"
//                         "v4sf d = __builtin_convertvector (a, v4sf);"
//                         "v4sf d2 = { (float)a[0], (float)a[1], (float)a[2], (float)a[3] };;"
//                         "v4df e = __builtin_convertvector (a, v4df);\n "
//                         "v4df f = __builtin_convertvector (b, v4df);\n"
//                         "v4si g = __builtin_convertvector (f, v4si);\n"
//                         "v4si h = __builtin_convertvector (c, v4si);\n"
//                         "c = a >  b;"
//                         "printf(\"simd c  = a >  b= [%i,%i,%i,%i]\", c[0], c[1],c[2],c[3]);"
//                         "c = a == b;"
//                         "printf(\"simd c  = a == b= [%i,%i,%i,%i]\", c[0], c[1],c[2],c[3]);"
                         "}"
                         ;


    rt = tcch_compile_program(&script,NULL);

    if(rt < 0){
        tcch_delete_script(&script);

        return 0;
    }



    int a =123;
    printf("host pointer of a %p\n",&a);



    script.string_code =
                         "void test_json(char* json_string){"
                         "printf(\"\\n===================test_json\\n\");"

                         "unsigned long j = json_parse(json_string);"
                         "printf(\"tcc pointer of json = %p ,sizeof(p) = %i \\n\", j, sizeof(j));"
                         "json_print(j);"
                         "        json_set_int(j,\"date:time:0\",1);\n"
                         "        json_set_int(j,\"date:time:1\",11);\n"
                         "        json_set_int(j,\"date:time:2\",111);"
                         "json_print(j);"
                         "json_delete(j);"
                         ""
                         "}"
                         ;
    rt = tcch_compile_program(&script,"cjson");

    if(rt < 0){

        tcch_delete_script(&script);

        return 0;
    }

    rt = tcch_relocate_program(&script);

    if(rt < 0){

        tcch_delete_script(&script);

        return 0;
    }

    hello hello_func = tcch_get_symbol(&script,"hello");
    hello hello2_func = tcch_get_symbol(&script,"hello2");
    hello asm_func = tcch_get_symbol(&script,"test_asm");
    str_func test_json = tcch_get_symbol(&script,"test_json");


    hello_func(1);
    printf("num after hello_func: %i\n",num);
    hello2_func(10);
    printf("num after hello2_func: %i\n",num);
    asm_func(55);

    char * simple_json_str = "{ \"a\" : 1  }";
    test_json(json_string);


    tcch_delete_script(&script);

    printf("exit\n");

}