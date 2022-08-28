//
// Created by waxz on 8/27/22.
//
#include <fruit/fruit.h>
//#include "fruit.h"
#include <iostream>

class Writer {
public:
    virtual void write(std::string s) = 0;
};

class StdoutWriter : public Writer {
public:
    // Like "StdoutWriter() = default;" but also marks this constructor as the one to use for injection.
    INJECT(StdoutWriter()) = default;

    virtual void write(std::string s) override {
        std::cout << s;
    }
};

class Greeter {
public:
    virtual void greet() = 0;
};

class GreeterImpl : public Greeter {
private:
    Writer* writer;

public:
    // Like "GreeterImpl(Writer* writer) : ... {...}" but also marks this constructor as the one to use for injection.
    INJECT(GreeterImpl(Writer* writer))
    : writer(writer) {
    }

    virtual void greet() override {
        writer->write("Hello world!\n");
    }
};

fruit::Component<Greeter> getGreeterComponent() {
    return fruit::createComponent().bind<Greeter, GreeterImpl>()
            .bind<Writer, StdoutWriter>()
            ;
}
fruit::Component<Greeter, Writer, GreeterImpl, StdoutWriter> getGreeterWriterComponent(){
    return fruit::createComponent().bind<Greeter, GreeterImpl>()
            .bind<Writer, StdoutWriter>()
            ;
}

fruit::Component<Writer> getWriterComponent(){
    return fruit::createComponent().bind<Greeter, GreeterImpl>()
            .bind<Writer, StdoutWriter>()
            ;

}
int main() {
    fruit::Injector<Greeter> injector(getGreeterComponent);
    Greeter* greeter_1 = injector.get<Greeter*>();
    greeter_1->greet();
    fruit::Injector<Writer> injector2(getWriterComponent);
    Writer* writer_1 = injector2.get<Writer*>();
    writer_1->write("hello writer_1\n");

    fruit::Injector<Greeter, Writer, GreeterImpl, StdoutWriter> injector3(getGreeterWriterComponent);
    Greeter* greeter_2 = injector3.get<Greeter*>();
    Writer* writer_2 = injector3.get<Writer*>();


    greeter_2->greet();
    writer_2->write("hello writer_2\n");

    return 0;
}