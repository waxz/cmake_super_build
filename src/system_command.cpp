//
// Created by waxz on 9/15/22.
//
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <array>
#include <utility>
#include <thread>
#include <chrono>

#include "common/subprocess.hpp"
#include "absl/strings/match.h"
#include "absl/strings/str_split.h"
#include "absl/strings/substitute.h"

#include "md5/md5.h"

using namespace std;
pair<string, int> exec(const char* cmd) {
    array<char, 128> buffer;
    string result;
    int return_code = -1;
    auto pclose_wrapper = [&return_code](FILE* cmd){ return_code = pclose(cmd); };
    { // scope is important, have to make sure the ptr goes out of scope first
        const unique_ptr<FILE, decltype(pclose_wrapper)> pipe(popen(cmd, "r"), pclose_wrapper);
        if (pipe) {
            while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
                result += buffer.data();
            }
        }
    }
    return make_pair(result, return_code);
}

int main(int argc, char* argv[]) {





    {
        namespace sp = subprocess;

        {
            std::string cmd =  "echo 123 >> /tmp/a.txt";
            if (0){
                for(int i = 0 ; i < 4; i++){
                    std::string cmd_string = absl::Substitute("\''$0'\'", cmd);

                    auto p =  sp::Popen({"bash", "-c",  cmd_string}, sp::output{sp::PIPE},sp::defer_spawn{true});
                    p.start_process();
                }
                for(int i = 0 ; i < 4; i++){
                    std::string cmd_string = absl::Substitute("\''$0'\'", cmd);

                    auto p =  sp::Popen({"bash", "-c",  cmd_string}, sp::shell{true},sp::defer_spawn{true});
                    p.start_process();
                }
                for(int i = 0 ; i < 4; i++){
                    std::string cmd_string = absl::Substitute("\''$0'\'", cmd);
                    auto p = sp::Popen(
                            {"bash", "-c", cmd_string}, sp::output{sp::PIPE},
                            sp::error{sp::PIPE}, sp::defer_spawn{true});
//                p.start_process();
                }
            }


            for(int i = 0 ; i < 4; i++){
                cmd =  "echo 123667 >> /tmp/a.txt";
                std::string cmd_string = absl::Substitute("\''$0'\'", cmd);
                char msg_array[100];
                sprintf(msg_array, R"(%s )", cmd.c_str());
                std::string cc = std::string (msg_array);
                std::cout << "cc: [" << cc <<"]"<<std::endl;
                auto p = sp::Popen(
                        {"bash", "-c",cmd }, sp::output{sp::PIPE},
                        sp::error{sp::PIPE}, sp::defer_spawn{true});
                p.start_process();
            }
            return 0;

        }
        {
            std::string cmd =  argv[1] ;
            std::string cmd_string = absl::Substitute("\''$0'\'", cmd);


            auto p = sp::Popen({"bash", "-c",  cmd_string}, sp::output{sp::PIPE});
            auto obuf = p.communicate().first;
            std::cout << "Data : " << obuf.buf.data() << std::endl;
            std::cout << "Data len: " << obuf.length << std::endl;
            {
                sp::Popen({"bash", "-c",  cmd_string}, sp::output{sp::PIPE});
                sp::Popen({"bash", "-c",  cmd_string}, sp::output{sp::PIPE});
                sp::Popen({"bash", "-c",  cmd_string}, sp::output{sp::PIPE});
            }
            return 1;

        }


        std::string node_name = std::string(argv[1]);
        std::cout << "node_name: " << node_name << std::endl;

        std::vector<std::string> cmd_args{"rosnode","ping", node_name ,"-c 4"};
        std::string cmd_string = absl::Substitute("rosnode ping $0 -c 5", node_name);

        cmd_string = "roslaunch rviz_visual_tools demo_rviz.launch";

        cmd_args = absl::StrSplit(cmd_string, ' ');
        std::cout << "cmd_string : " << cmd_string << std::endl;
        for(auto& c : cmd_args){
            std::cout << "    cmd_args: " << c << std::endl;

        }



        auto p0 = sp::Popen(cmd_args, sp::output{sp::PIPE});
//        auto p0 = sp::Popen(cmd_args, sp::shell{true});
        std::cout << "***** 1 execute cmd: " << cmd_string << std::endl;

        std::vector<char> buf(20);
        int rbytes = sp::util::read_all(p0.output(), buf);

        std::string out(buf.begin(), buf.end());
        std::cout << "***** 1 execute out: " << out << std::endl;


        {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(5s);

        }



        cmd_string = "rosnode ping /base_to_world -c 5";

        cmd_args = absl::StrSplit(cmd_string, ' ');
        auto p = sp::Popen(cmd_args, sp::output{sp::PIPE});
        std::cout << "***** 2 execute cmd: " << cmd_string << std::endl;


        auto obuf = p.communicate().first;
        std::cout << "Data : " << obuf.buf.data() << std::endl;
        std::cout << "Data len: " << obuf.length << std::endl;
        // Assume "msg" is a line from a logs entry
        if (absl::StrContains(obuf.buf.data(), "reply from")) {
            std::cout << "node is running"   << std::endl;

        }else{
            std::cout << "node is not running"   << std::endl;

        }
        if (absl::StrContains(obuf.buf.data(), "WARNING")) {
        }

        p0.kill();
        p.kill();

    }
    return 0;
}