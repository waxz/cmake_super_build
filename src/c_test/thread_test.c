//
// Created by waxz on 4/25/23.
//
#include <stdio.h>
#include "threads.h"
#include <string.h>


#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>


typedef enum {
    STR2INT_SUCCESS,
    STR2INT_OVERFLOW,
    STR2INT_UNDERFLOW,
    STR2INT_INCONVERTIBLE
} str2int_errno;

/* Convert string s to int out.
 *
 * @param[out] out The converted int. Cannot be NULL.
 *
 * @param[in] s Input string to be converted.
 *
 *     The format is the same as strtol,
 *     except that the following are inconvertible:
 *
 *     - empty string
 *     - leading whitespace
 *     - any trailing characters that are not part of the number
 *
 *     Cannot be NULL.
 *
 * @param[in] base Base to interpret string in. Same range as strtol (2 to 36).
 *
 * @return Indicates if the operation succeeded, or why it failed.
 */
str2int_errno str2int(int *out, char *s, int base) {
    char *end;
    if (s[0] == '\0' || isspace(s[0]))
        return STR2INT_INCONVERTIBLE;
    errno = 0;
    long l = strtol(s, &end, base);
    /* Both checks are needed because INT_MAX == LONG_MAX is possible. */
    if (l > INT_MAX || (errno == ERANGE && l == LONG_MAX))
        return STR2INT_OVERFLOW;
    if (l < INT_MIN || (errno == ERANGE && l == LONG_MIN))
        return STR2INT_UNDERFLOW;
    if (*end != '\0')
        return STR2INT_INCONVERTIBLE;
    *out = l;
    return STR2INT_SUCCESS;
}

#define SIZE 5


int func(void *id)
{
    //_Thread_local variable
    static thread_local int var = 5;
    var += 5;

    //Print id of current thread and addr of var
    printf("Thread ID:[%d], Value of var: %d\n", *(int*)id, var);

    return 0;
}

typedef struct {
    int x;
    int y;
} Point;

int main(void)
{

    {
       Point arr[5];

       Point * p = &(arr[0]);
       p->x;

    }

    {
        int i = 0;
        assert(str2int(&i, "11", 10) == STR2INT_SUCCESS);
        assert(i == 11);

        /* Negative number . */
        assert(str2int(&i, "-11", 10) == STR2INT_SUCCESS);
        assert(i == -11);

    }

    {
        char str[] ="Where there is will, there is a way.";
        char* st = str ;
        char *ch;
        ch = strtok(st, " ");
        while (ch != NULL) {
            printf("%s\n", ch);
            ch = strtok(NULL, " ,");
        }

    }

    {
        char str[] ="USBCAN-I:0:";
        char* st = str ;
        char *ch;
        ch = strtok(st, ":");
        int count = 0;
        while (ch != NULL) {
            printf("get %s\n", ch);

            switch (count) {
                case 0:
                    printf("device %s\n", ch);
                    break;
                case 1:
                    printf("id %s\n", ch);
                    int dev_id;
                    if(str2int(&dev_id, ch, 10) == STR2INT_SUCCESS){
                        printf("dev_id %d\n", dev_id);
                    }

                    break;
                default:
                    break;
            }
            count ++;
            ch = strtok(NULL, ":");
        }

    }
    {
        {
            char str[] ="0:11";
            char* st = str ;
            char *ch;
            ch = strtok(st, ":");
            int count = 0;
            while (ch != NULL) {
                printf("get %s\n", ch);

                switch (count) {
                    case 0:
                        printf("get %s\n", ch);
                        int dev_id1;
                        if(str2int(&dev_id1, ch, 10) == STR2INT_SUCCESS){
                            printf("dev_id1 %d\n", dev_id1);
                        }
                        break;
                    case 1:
                        printf("get %s\n", ch);
                        int dev_id2;
                        if(str2int(&dev_id2, ch, 10) == STR2INT_SUCCESS){
                            printf("dev_id2 %d\n", dev_id2);
                        }

                        break;
                    default:
                        break;
                }
                count ++;
                ch = strtok(NULL, ":");
            }

        }
    }
    thrd_t id[SIZE];

    //thread ID arr
    int arr[SIZE] = {1, 2, 3, 4, 5};

    //Creating 5 threads
    for(int i = 0; i < SIZE; i++) {
        thrd_create(&id[i], func, &arr[i]);
    }

    //Wait for threads to complete
    for(int i = 0; i < SIZE; i++) {
        thrd_join(id[i], NULL);
    }
}