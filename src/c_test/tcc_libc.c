//
// Created by waxz on 9/3/23.
//
#include "tcc_liba.h"
#include "tcc_libb.h"
#include "tcc_libc.h"
#include "stdio.h"

void test_c(int a){
    test_b(a);
    printf("liba c = %i\n",a);
}