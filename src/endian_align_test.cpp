//
// Created by waxz on 4/29/23.
//

#include <stdio.h>
#include <iostream>
#include <bitset>         // std::bitset


//https://www.cnblogs.com/luxiaoxun/archive/2012/09/05/2671697.html

/*
一、大端和小端的问题

对于整型、长整型等数据类型，Big endian 认为第一个字节是最高位字节（按照从低地址到高地址的顺序存放数据的高位字节到低位字节）；
而 Little endian 则相反，它认为第一个字节是最低位字节（按照从低地址到高地址的顺序存放据的低位字节到高位字节）。

例如，假设从内存地址 0x0000 开始有以下数据：
memory address : |0x0000 | 0x0001 | 0x0002 | 0x0003|
byte:            | 0x12  | 0x34   | 0xab   | 0xcd  |
如果我们去读取一个地址为 0x0000 的四个字节变量，若字节序为big-endian，则读出结果为0x1234abcd；若字节序为little-endian，则读出结果为0xcdab3412。

如果我们将0x1234abcd 写入到以 0x0000 开始的内存中，则Little endian 和 Big endian 模式的存放结果如下：
地址           0x0000         0x0001        0x0002          0x0003
big-endian   0x12           0x34            0xab            0xcd
little-endian  0xcd           0xab            0x34            0x12

一般来说，x86 系列 CPU 都是 little-endian 的字节序，PowerPC 通常是 big-endian，网络字节顺序也是 big-endian还有的CPU 能通过跳线来设置 CPU 工作于 Little endian 还是 Big endian 模式。

对于0x12345678的存储：

小端模式：（从低字节到高字节）
地位地址 0x78 0x56 0x34 0x12 高位地址

大端模式：（从高字节到低字节）
地位地址 0x12 0x34 0x56 0x78 高位地址


 可以使用`lscpu`指令直接获取cpu架构信息
 Byte Order:                      Little Endian

 */



//explain
/*
 大小端存储问题，如果小端方式中（i占至少两个字节的长度）则i所分配的内存最小地址那个字节中就存着1，其他字节是0.大端的话则1在i的最高地址字节处存放，char是一个字节.
 强制将char型量p指向i则p指向的一定是i的最低地址，那么就可以判断p中的值是不是1来确定是不是小端。
 */
int checkEndian(){
    int i = 1;
    char *p = (char *)&i;
    if(*p == 1) {
        printf("Little Endian");
        return 1;
    }
    else {
        printf("Big Endian");
        return 0;
    }
    return -1;
}


/*return 1: little-endian, return 0: big-endian*/
// explain:
// 联合体union的存放顺序是所有成员都从低地址开始存放，利用该特性就可以轻松地获得了CPU对内存采用Little-endian还是Big-endian模式读写。
int checkEndianUnion()
{
    union
    {
        unsigned int a;
        unsigned char b;
    }c;
    c.a = 1;
    return (c.b == 1);
}

//Linux 的内核作者们仅仅用一个union 变量和一个简单的宏定义就实现了一大段代码同样的功能！（如果ENDIANNESS=’l’表示系统为little endian，为’b’表示big endian）
char checkEndianUnionLinux(){
    union { char c[4]; unsigned long mylong; } endian_test = {{ 'l', '?', '?', 'b' } };
    return ((char)endian_test.mylong);
}

void printBytes(char* p, int len){
    printf("\n****printBytes" );
    printf("\nindex: " );

    for(int i = 0 ;i < len;i++){
        printf("%d       ,",i);
    }
    printf("\naddre: " );

    for(int i = 0 ;i < len;i++){
        printf("%x,",p+i);
    }
    printf("\nbytes: " );

    for(int i = 0 ;i < len;i++){
        printf("%x      ,",*(p+i));
    }
    printf("\n****\n" );

}

// explain
// 1. 字节数组在内存中的布局与下标是一致的，不受大端小端的影响，低位元素对应这低字节
// 2. 获取指针的操作总是会获取最低位的字节地址
// 3. int *p = (int*)sz, p 中存储着sz字符数组中的最低位地址 , p = &(sz[0])
// 4. 但是 p 的类型是int*, 当p++递增时， p指向的地址将是，sz数组中第5个元素, p = &(sz[4])
// 5. *p 对应的内存范围将是，从地址p 到 p+3， 对应这数组sz[4] 到 sz[7]
// 6. 当打印int数据时，字节数组将会被解释成int数据，这时候，大端小端将会起作用。
// 7. 在小端的系统中，第一个字节是最低位字节（按照从低地址到高地址的顺序存放据的低位字节到高位字节）
// 8. 因此int数据的，最低位对应sz[4], 最高位对应sz[7]，


void test_char_array_1(){
    printf("check test_char_array_1\n");
    char sz[] ={0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
    printBytes(sz,10);
    int *p = (int*)sz;
    ++p;
    printf("check address: %x\n",p);
    printf("check int: %x\n",*p);
    printBytes((char*)p,4);
}
void test_char_array_2(){
    printf("check test_char_array_2\n");
    char *sz = "0123456789";
    printBytes(sz,10);
    int *p = (int*)sz;
    ++p;
    printf("check address: %x\n",p);
    printf("check int: %x\n",*p);
    printBytes((char*)p,4);
}


// explain
// 1. 系统存储int数据时需要将int数据进行解释，这时候大端小端将发挥影响
// 2. 在小端的系统中，字节最低位保存数据的最低位
// 3. 所以将int转换成内存中实际的字节数组时候， sz[0] 保存 int最低位即 0x78, sz[1] 保存数据0x56， 依次类推

void test_int(){
    printf("check test_int\n");

    int a = 0x12345678;
    char* sz = (char*)&a;
    printBytes((char*)(&a),4);
    printBytes((char*)sz,4);

    char *p = (char*)(&a);
    printf("%x\n",*(p+1));
}

void test_bit_set(){
    std::bitset<16> foo;
    std::bitset<16> bar (0x3130);
    std::bitset<16> baz (std::string("0101111001"));

    char bar0 = bar[0];
    char bar1 = bar[1];
    foo = 0x3031;

    std::cout << "foo: " << foo << '\n';
    std::cout << "bar: " << bar << '\n';
    std::cout << "bar0: " << (int)bar0 << '\n';
    std::cout << "bar1: " << (int)bar1 << '\n';
    std::cout << "baz: " << baz << '\n';


    std::bitset<64> data;
    std::bitset<16> cob_id = 0x181;

    char data_byte_array[8];
    data_byte_array[0] = 0x57;
    data_byte_array[7] = 0x57;


    data = *data_byte_array;
//    *data_byte_array = data.to_ullong();

    std::cout << "1 data: " << data << '\n';

    for(size_t i = 0; i < 11;i++){
        data[i] = cob_id[i];
    }
    std::cout << "data: " << data << '\n';
    std::cout << "data.to_ulong: " << data.to_ulong() << '\n';
    std::cout << "data.to_string: " << data.to_string<char>() << '\n';

    std::cout << "cob_id: " << cob_id << '\n';

}
int main(int argc, char** argv){
    checkEndian();
    std::cout << "checkEndianUnion = " << checkEndianUnion() << std::endl;
    std::cout << "checkEndianUnionLinux = " <<  checkEndianUnionLinux() << std::endl;

    test_char_array_1();
    test_char_array_2();
    test_int();

    test_bit_set();



}