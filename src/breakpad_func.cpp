//
// Created by waxz on 4/8/23.
//

#include "breakpad_func.h"

void crash()
{
    volatile int *a = (int *) (nullptr);
    *a = 1;
}