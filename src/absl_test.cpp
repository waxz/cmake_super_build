//
// Created by waxz on 8/6/23.
//


// test framework
// https://nanobench.ankerl.com/comparison.html
// https://github.com/martinus/nanobench
// https://github.com/doctest/doctest

#define USE_CATCH 0
#define USE_FAKEIT 0
#define USE_DOCTEST_NANOBENCHMARK 1

// fakeit
#if USE_FAKEIT
#include "catch_amalgamated.hpp"
#include "fakeit.hpp"
#endif
#if USE_CATCH
#include "catch2/catch_amalgamated.hpp"
#endif

#ifdef USE_DOCTEST_NANOBENCHMARK
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "nanobench.h"
#endif

#include <iostream>

//#undef CHECK
#include "absl/strings/str_format.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"

#include "absl/strings/match.h"

#include "absl/log/log.h"
#include "absl/log/check.h"
#include "absl/status/statusor.h"

#include "absl/time/time.h"                // holds abstractions for absolute time (and durations)
#include "absl/time/civil_time.h"          // holds abstractions for civil time
#include "absl/time/clock.h"               // holds utility functions for creating time objects using the system clock


// container
#include "absl/container/fixed_array.h"
#include "absl/container/inlined_vector.h"


//https://abseil.io/
//https://google.github.io/styleguide/cppguide.html
//https://abseil.io/resources/swe-book/html/toc.html





// -----------------------------------------------------------------------------
// FixedArray
// -----------------------------------------------------------------------------
//
// A `FixedArray` provides a run-time fixed-size array, allocating a small array
// inline for efficiency.
//
// Most users should not specify the `N` template parameter and let `FixedArray`
// automatically determine the number of elements to store inline based on
// `sizeof(T)`. If `N` is specified, the `FixedArray` implementation will use
// inline storage for arrays with a length <= `N`.
TEST_CASE("FixedArray"){

    uint64_t x = 1;

    ankerl::nanobench::Bench().run("x += x", [&]() {

        ankerl::nanobench::doNotOptimizeAway(x += x);

    });


    ankerl::nanobench::Bench().run("FixedArray write", [&]() {
        absl::FixedArray<size_t> array(50);
        for(int j = 0 ; j < 100;j++){
            for (size_t i = 0; i < 50; i++) {
                ankerl::nanobench::doNotOptimizeAway(array[i] = i);
            }
        }

    });

    ankerl::nanobench::Bench().run("std::array write", [&]() {
        std::array<size_t,50> array;
        for(int j = 0 ; j < 100;j++){
            for (size_t i = 0; i < 50; i++) {
                ankerl::nanobench::doNotOptimizeAway(array[i] = i);
            }
        }

    });




}

std::uint64_t Fibonacci(std::uint64_t number) {
    return number < 2 ? 1 : Fibonacci(number - 1) + Fibonacci(number - 2);
}
// -----------------------------------------------------------------------------
// InlinedVector
// -----------------------------------------------------------------------------
//
// An `absl::InlinedVector` is designed to be a drop-in replacement for
// `std::vector` for use cases where the vector's size is sufficiently small
// that it can be inlined. If the inlined vector does grow beyond its estimated
// capacity, it will trigger an initial allocation on the heap, and will behave
// as a `std::vector`. The API of the `absl::InlinedVector` within this file is
// designed to cover the same API footprint as covered by `std::vector`.
TEST_CASE("InlinedVector"){

    SUBCASE("std::vector"){
        // now let's benchmark:
        ankerl::nanobench::Bench().run("std::vector push_back", [&]() {
            std::vector<size_t> array;

            for (size_t i = 0; i < 100; i++) {
                array.push_back(i);
            }
        });
    }


    SUBCASE("InlinedVector"){
        ankerl::nanobench::Bench().run("InlinedVector push_back", [&]() {
            absl::InlinedVector<size_t, 100> array;

            for (size_t i = 0; i < 100; i++) {
                array.push_back(i);
            }
        });
    }




}



//https://abseil.io/docs/cpp/guides/time#proper-time-hygiene
/*

 Proper Time Hygiene

    Use the Abseil time library types everywhere—including in your interfaces!
    Convert to/from other types at the boundary of your system—use Abseil Time types exclusively within your system.
    Never use time-zone offsets in calculations. Let the Abseil time library do that!
    Prefer explicit time-zone names. Don’t assume “localtime”.
    Be explicit about your code’s time-zone requirements.
    If possible, make your code time-zone agnostic. Failing that, prefer UTC.
    Include offsets (“%z”) in time strings. Prefer RFC3339/ISO8601 format (e.g., absl::RFC3339_full)
    Use absl::FormatTime to format time strings, and absl::ParseTime to parse time strings.

 */

TEST_CASE("time"){

    // from system clock
    absl::Time t1 = absl::Now();

    //from unix epoch
    absl::Time t2 = absl::Time();

    // direction from epoch
    absl::Time t3 = absl::UnixEpoch();

    absl::Duration d1 = t2-t1;
    absl::Duration d2 = t3-t2;


    absl::PrintF("duration d1 : %v", ToChronoHours(d1).count());
    absl::PrintF("duration d2 : %v", absl::ToChronoNanoseconds(d2).count());



}


TEST_CASE("str"){

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
        std::string text = absl::StrFormat("Welcome to %s, Number %d!, say %v", "The Village", 6, title);


        bool has_hello = absl::StrContains(text, "hello");
        absl::PrintF("has_hello: %v",has_hello);

        std::string decode;
        bool decode_ok =  absl::Base64Unescape(title,&decode);

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