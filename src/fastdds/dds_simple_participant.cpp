//
// Created by waxz on 4/17/24.
//


#include "dds_handler.h"



#include "lyra/lyra.hpp"


#include "common/signals.h"
#include "common/functions.h"
#include "common/suspend.h"
#include "common/task.h"


#include "dds_common.h"

using namespace eprosima::fastdds::dds;



int main(int argc, char** argv){
    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    set_signal_handler(my_handler);

    bool get_help = false;
    std::string exe_name;
//    std::string name;

    std::string xml_file;
    std::string profile;



    // get topic xml
    auto cli
            = lyra::exe_name(exe_name)
              | lyra::help(get_help)
              | lyra::opt(xml_file, "xml_file")["-x"]["--xml_file"]("xml_file")
              | lyra::opt(profile, "profile")["-p"]["--profile"]("profile")
//                | lyra::opt(name, "name")["-n"]["--name"]("name")


    ;

    auto result = cli.parse({argc, argv});
    if (get_help) {
        std::cout << cli << std::endl;
        return 0;

    }


    if (!result) {
        std::cerr << "Error in command line: " << result.message() << std::endl;
        return 0;
    }



    if (xml_file.empty()
//    || name.empty()
    || profile.empty()){
        std::cout << cli << std::endl;
        return 0;
    }

    // create participant with profile

    DdsSimpleHandler handler;
    bool ok = handler.init( xml_file.c_str(), profile.c_str());

    if(!ok){
        std::cout << "create dds handler failed, exit"<< std::endl;
        return 0;
    }
    DdsSimpleHandler handler2 = handler ;

    common::LoopHelper loopHelper;
    loopHelper.set_fps(10.0);
    while (program_run){
        loopHelper.start();

        loopHelper.sleep();

    }

}