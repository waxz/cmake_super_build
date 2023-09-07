# tiny compiler
https://developers.redhat.com/blog/2021/04/27/the-mir-c-interpreter-and-just-in-time-jit-compiler


# tcc
//https://github.com/TinyCC/tinycc

# tcc test
```shell
export TCC_HOME=/tmp/cmake_super_build/cmake-build-relwithdebinfo/force_tinycc/install
./bin/tinycc_test
```

# embed file
https://mort.coffee/home/fast-cpp-embeds/
https://github.com/mortie/strliteral
https://github.com/weigert/c-embed
https://github.com/vector-of-bool/cmrc
https://github.com/MiSo1289/cmake-embed
https://github.com/batterycenter/embed
https://github.com/MKlimenko/embed


# run tcc + dds
```shell
export TCC_HOME=/tmp/cmake_super_build/cmake-build-relwithdebinfo/force_tinycc/install

export TCC_HEADER=/tmp/cmake_super_build/cmake-build-relwithdebinfo/force_CycloneDDS/install/include:/tmp/cmake_super_build/cmake-build-relwithdebinfo/src/c_test
export TCC_SOURCE=/tmp/cmake_super_build/cmake-build-relwithdebinfo/src/c_test/ShmThroughput.c 
```




# try catch
https://stackoverflow.com/questions/10586003/try-catch-statements-in-c


# thread_local
```c++
#include <threads.h>

static thread_local CANPipe canpipes[MAX_NB_CAN_PIPES] = {{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},{0,},};

```

# tinyalloc
https://github.com/cionicwear/tinyalloc/tree/cionic_stable
https://github.com/thi-ng/tinyalloc/tree/40aea2bb77a0cec2f2676a547188a3c4eab2784c


# json
https://github.com/DaveGamble/cJSON