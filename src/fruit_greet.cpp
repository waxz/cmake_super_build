//
// Created by waxz on 8/27/22.
//
#include <fruit/fruit.h>
//#include "fruit.h"
#include <iostream>

class Writer {
public:
    std::string name;
public:
    virtual void write(std::string s) = 0;
};

class StdoutWriter : public Writer {

public:
    // Like "StdoutWriter() = default;" but also marks this constructor as the one to use for injection.
    INJECT(StdoutWriter()) = default;

    virtual void write(std::string s) override {
        std::cout << name << ", say: " << s;
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
fruit::Component<Greeter, Writer> getGreeterWriterComponentSimple(){
    return fruit::createComponent().bind<Greeter, GreeterImpl>()
            .bind<Writer, StdoutWriter>()
            ;
}

fruit::Component<> getGreeterWriterComponentSimpleMore(){
    return fruit::createComponent().bind<Greeter, GreeterImpl>()
            .addMultibinding<Writer, StdoutWriter>()
            ;
}
fruit::Component<Writer> getWriterComponent(){
    return fruit::createComponent().bind<Greeter, GreeterImpl>()
            .bind<Writer, StdoutWriter>()
            ;

}
int main() {
    fruit::Injector<Greeter> injector(getGreeterComponent);


    fruit::Injector<Writer> injector2(getWriterComponent);
    Writer* writer_1 = injector2.get<Writer*>();

    writer_1->name.assign("ww1");
    writer_1->write("hello writer_1\n");
    Greeter* greeter_1 = injector.get<Greeter*>();
    greeter_1->greet();
    fruit::Injector<Greeter, Writer, GreeterImpl, StdoutWriter> injector3(getGreeterWriterComponent);
    Greeter* greeter_2 = injector3.get<Greeter*>();
    Writer* writer_2 = injector3.get<Writer*>();
    StdoutWriter* stdowriter_2 = injector3.get<StdoutWriter*>();

    writer_2->name.assign("ww2");
    stdowriter_2->name.assign("ww22");

    greeter_2->greet();
    writer_2->write("hello writer_2\n");
    fruit::Injector<Greeter, Writer> injector4(getGreeterWriterComponentSimple);
    Greeter* greeter_4 = injector4.get<Greeter*>();
    Writer* writer_4 = injector4.get<Writer*>();

    writer_4->name.assign("ww4");

    greeter_4->greet();
    writer_4->write("hello writer_4\n");

    fruit::Injector<> injector5(getGreeterWriterComponentSimpleMore);

    std::vector<Greeter*> greeter_array_5 = injector5.getMultibindings<Greeter>();
    std::vector<Writer*> writer_array_5 = injector5.getMultibindings<Writer>();

    std::cout << "greeter_array_5: " << greeter_array_5.size() << std::endl;
    std::cout << "writer_array_5: " << writer_array_5.size() << std::endl;

    writer_array_5[0]->name.assign("ww5");
    writer_array_5[0]->write("hello");

    return 0;
}