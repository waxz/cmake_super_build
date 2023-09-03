//
// Created by waxz on 9/3/23.
//
#include "tcc_liba.h"
#include "tcc_libb.h"
#include "stdio.h"

void test_b(int a){
    test_a(a);
    printf("libb a = %i\n",a);
}