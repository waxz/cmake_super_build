//
// Created by waxz on 4/21/23.
//

#if defined(WIN32) && !defined(__CYGWIN__)
#include <windows.h>
#include "getopt.h"
void pause(void)
{
	system("PAUSE");
}
#else
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#endif

#include "canfestival.h"


void mySyncHandler( CO_Data* d )
{
    printf( "  got a SYNC message...\n" );
}

#include <type_traits>
#include <iostream>

template <typename CT, typename ... A> struct function
        : public function<decltype(&CT::operator())(A...)> {};

template <typename C> struct function<C> {
private:
    C mObject;

public:
    function(const C & obj)
            : mObject(obj) {}

    template<typename... Args> typename
    std::result_of<C(Args...)>::type operator()(Args... a) {
        return this->mObject.operator()(a...);
    }

    template<typename... Args> typename
    std::result_of<const C(Args...)>::type operator()(Args... a) const {
        return this->mObject.operator()(a...);
    }
};

namespace make {
    template<typename C> auto function(const C & obj) {
        return ::function<C>(obj);
    }
}


// test2
#include<type_traits>
#include<utility>

template<typename Callable>
union storage
{
    storage() {}
    std::decay_t<Callable> callable;
};

template<int, typename Callable, typename Ret, typename... Args>
auto fnptr_(Callable&& c, Ret (*)(Args...))
{
    static bool used = false;
    static storage<Callable> s;
    using type = decltype(s.callable);

    if(used)
        s.callable.~type();
    new (&s.callable) type(std::forward<Callable>(c));
    used = true;

    return [](Args... args) -> Ret {
        return Ret(s.callable(std::forward<Args>(args)...));
    };
}

template<typename Fn, int N = 0, typename Callable>
Fn* fnptr(Callable&& c)
{
    return fnptr_<N>(std::forward<Callable>(c), (Fn*)nullptr);
}

int main(int argc, char** argv){



    s_BOARD  bd;
    CO_Data  myData;
    CAN_PORT port;
    Message myMessage;
    const char busname[]  = "/dev/comedi0";
    const char baudrate[] = "1M";

    memset( &myData, 0x00, sizeof(CO_Data) );
    myData.CurrentCommunicationState.csSYNC = 1;
    myData.post_sync = mySyncHandler;

    myData.post_sync = []( CO_Data* d )
    {
        printf( "  got a SYNC message...\n" );
    };
    int sync_num = 0;



    myData.post_sync = fnptr<void(CO_Data*)>([&sync_num]( CO_Data* d )
                            {
                                sync_num ++;
                                printf( "  got a SYNC message...\n" );
                            });



    bd.busname  = (char*)busname;
    bd.baudrate = (char*)baudrate;



    port = canOpen( &bd, &myData );

    if( port == NULL )
    {
        /* something strange happenend */
        return 0;
    }


    memset( &myMessage, 0x00, sizeof(Message) );
    myMessage.cob_id = 0x80; /* SYNC message */
    myMessage.len = 1;
    myMessage.data[0] = 0xA5;

    /* SEND HERE */
    canSend( port, &myMessage );

    myMessage.data[0] = 0x5A;
    canSend( port, &myMessage );

    myMessage.data[0] = 0xA5;
    canSend( port, &myMessage );


    /* mySyncHandler() is called by the receive thread and shows a received SYNC message in the kernel log */

    canClose( &myData );

}