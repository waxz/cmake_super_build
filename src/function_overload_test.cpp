//
// Created by waxz on 23-2-4.
//



#include <iostream>




//https://stackoverflow.com/questions/3585846/color-text-in-terminal-applications-in-unix
#include <stdio.h>
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define KRESET "\x1B[0m"
#define MLOGW(__msg) {printf(KRED "%s:%i @%s:%s\n" KRESET, __FILE__, __LINE__, __FUNCTION__ , __msg);}
#define MLOGI(__msg) {printf(KGRN "%s:%i @%s:%s\n" KRESET, __FILE__, __LINE__, __FUNCTION__ , __msg);}


struct Point{
    float x = 0.0;
    float y = 0.0;
    float getLen() {

        MLOGI("hello");
        return x*x + y*y;
    }
    float getLen()const{
        MLOGI("hello");

        return x*x + y*y;
    }
};

void testPrint(){

    printf("%sred\n", KRED);
    printf("%sgreen\n", KGRN);
    printf("%syellow\n", KYEL);
    printf("%sblue\n", KBLU);
    printf("%smagenta\n", KMAG);
    printf("%scyan\n", KCYN);
    printf("%swhite\n", KWHT);
    printf("%snormal\n", KNRM);

    printf(KRED "red\n"     KRESET);
    printf(KGRN "green\n"   KRESET);
    printf(KYEL "yellow\n"  KRESET);
    printf(KBLU "blue\n"    KRESET);
    printf(KMAG "magenta\n" KRESET);
    printf(KCYN "cyan\n"    KRESET);
    printf(KWHT "white\n"   KRESET);

}

int main(){
    MLOGI("test function overload bug");
    std::cout << "cout color" << std::endl;

    Point p1;
    p1.getLen();

    const Point & p2 = p1;
    p2.getLen();

    MLOGW("exit");

}