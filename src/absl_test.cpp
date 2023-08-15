//
// Created by waxz on 8/6/23.
//


#define  USE_CATCH 1

// fakeit
#if USE_FAKEIT
#include "catch_amalgamated.hpp"
#include "fakeit.hpp"
#endif
#if USE_CATCH
#include "catch2/catch_amalgamated.hpp"
#endif

#include <iostream>

//#undef CHECK
#include "absl/strings/str_format.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"
#include "absl/log/log.h"
#include "absl/log/check.h"
#include "absl/status/statusor.h"




//https://abseil.io/
//https://google.github.io/styleguide/cppguide.html
//https://abseil.io/resources/swe-book/html/toc.html

TEST_CASE("str format"){

    {
        std::string s = absl::StrFormat("Welcome to %s, Number %d!", "The Village", 6);
        CHECK ("Welcome to The Village, Number 6!" ==  s);
    }
    {
        constexpr absl::string_view kFormatString = "Welcome to %s, Number %d!";
        std::string s = absl::StrFormat(kFormatString, "The Village", 6);
        CHECK ("Welcome to The Village, Number 6!" ==  s);

    }

    //deduce type
    {
        // %v
        std::string title = "hello";
        std::string user = "tom";

        unsigned int x = 16;
//        CHECK(absl::StrFormat("%v", s) == "hello");
        absl::PrintF("Welcome to %s, Number %d!", "The Village", 6);
    }

}

void raise_error(int x, int y){
    CHECK_EQ(2 * x, y) << "oops!";

}

TEST_CASE("log") {
    LOG(INFO) << "aaa";

    std::string filenames_sorted;
    {
        int x = 3, y = 6;
        raise_error(x,y);
    }

}


struct Point{
    float x = 0.0;

};
absl::StatusOr<Point> getPoint(float x){
    Point p{x};

    if(x> 0.0){
        return p;

    }else{
        return absl::InvalidArgumentError("bad mode");
    }
    return absl::InvalidArgumentError("bad mode");

}
TEST_CASE("status"){
    absl::StatusOr<Point> result = getPoint(-1.0);
    if (result.ok()) {
        std::cout << result->x ;
    } else {
        LOG(ERROR) << result.status();
    }

}