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


#include <fruit/fruit.h>

// string
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

// debug
#include "absl/debugging/symbolize.h"
#include "absl/debugging/leak_check.h"

// random
#include "absl/random/random.h"

// types
#include "absl/types/any.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "absl/types/span.h"

// thread
#include "absl/base/thread_annotations.h"
#include "absl/base/call_once.h"

#include "absl/synchronization/barrier.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/blocking_counter.h"
#include "absl/synchronization/notification.h"



// common
#include "common/functions.h"

//https://abseil.io/
//https://google.github.io/styleguide/cppguide.html
//https://abseil.io/resources/swe-book/html/toc.html



using fruit::Component;
using fruit::Injector;

class DDSWriter {
public:
    virtual void write(std::string s) = 0;
};

class StdoutWriter : public DDSWriter {
public:
    // Like "StdoutWriter() = default;" but also marks this constructor as the
    // one to use for injection.
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
    DDSWriter *writer;

public:
    // Like "GreeterImpl(Writer* writer) {...}" but also marks this constructor
    // as the one to use for injection.
    INJECT(GreeterImpl(Writer * writer)) : writer(writer) {}

    virtual void greet() override {
        writer->write("Hello world!\n");
    }
};

Component<Greeter> getGreeterComponent() {
    return fruit::createComponent().bind<DDSWriter, StdoutWriter>().bind<Greeter, GreeterImpl>();
}


TEST_CASE("fruit 1") {

    auto getGreeterComponent_lambda = common::fnptr<Component<DDSWriter>()>(
            []() -> Component<DDSWriter> { return fruit::createComponent().bind<DDSWriter, StdoutWriter>().bind<Greeter, GreeterImpl>(); });

    Injector<Greeter> injector(getGreeterComponent);
    Greeter *greeter = injector.get<Greeter *>();

    greeter->greet();

    Injector<DDSWriter> injector1(getGreeterComponent_lambda);
    DDSWriter *writer = injector1.get<DDSWriter *>();
    writer->write("hello motto");;

}


TEST_CASE("random") {
    {
        // Create an absl::BitGen. There is no need to seed this bit generator.
        absl::BitGen gen;

        // Generate an integer value in the closed interval [1,6]
        int die_roll = absl::uniform_int_distribution<int>(1, 6)(gen);

        absl::PrintF("die_roll:%v", die_roll);
    }

    {
        // Create an absl::BitGen using an std::seed_seq seed sequence
        std::seed_seq seq{1, 2, 3};
        absl::BitGen gen_with_seed(seq);


        auto uniform_int_distribution = absl::uniform_int_distribution<int>(1, 6);
        auto uniform_real_distribution = absl::uniform_real_distribution<float>(-0.1, 0.1);

        auto gaussian_distribution = absl::gaussian_distribution<float>(0.0, 0.1);

        absl::PrintF("uniform_int_distribution:uniform_real_distribution,gaussian_distribution\n");

        for (int i = 0; i < 10; i++) {
            absl::PrintF("[%v, %v, %v], ",
                         uniform_int_distribution(gen_with_seed),
                         uniform_real_distribution(gen_with_seed),
                         gaussian_distribution(gen_with_seed)
            );
        }

    }

}

/*
Concurrent operations may conflict if they are not used (or designed) properly within a multi-threaded environment, resulting in the following problems:

    Operations may require write access to shared resources. We call these issues memory access issues.
    Operations may need to occur in a specific order. We sometimes call these issues synchronization issues (although memory access issues are also synchronization issues).

In either case, lack of control on the shared resources or lack of control on the operation order can lead to race conditions. The purpose of the concurrency abstractions within this library is to address these issues and avoid such race conditions.



 Memory Access Issues

Memory access issues are often addressed through a variety of means, including:

    Making the shared resource private or read-only (for data where this is appropriate)
    Converting the data access into a “message passing” scheme, to provide copies of the shared information for temporary use rather than direct access to the memory.
    Locking access to the shared resource, typically for write operations, to prevent more than one user from reading or writing concurrently.
    Using atomic operations to access to the shared resource, such as those provided by std::atomic. Note that the rules for properly applying atomic operations are quite complicated, which is one of many reasons you should avoid atomics.

Locking access to shared resources is usually addressed through mutually-exclusive locks known as mutexes. Abseil provides its own Mutex class for this purpose; similarly, the C++ standard library provides a std::mutex class for the same purpose. (Reasons why we implement our own Mutex class are discussed in Mutex Design Notes.)

Types that behave correctly regardless of the order, scheduling, or interleaving of their operations are known as thread-safe. In most cases, such types use mutexes and atomic operations underneath the hood to guard access to the object’s internal state.


 */


// +[](int *count){} explain
//https://stackoverflow.com/questions/17822131/resolving-ambiguous-overload-on-function-pointer-and-stdfunction-for-a-lambda


// todo: compare thread pool and framework
// https://github.com/lzpong/threadpool
// https://github.com/tghosgor/threadpool11
// https://github.com/ConorWilliams/libfork


// https://github.com/martinmoene/jthread-lite
//https://github.com/facebook/folly

// TASKFLOW
// TBB
// MARL
// OpenMP


// SIMD

TEST_CASE("thread") {


    {
        // Example using LockWhen/Unlock:

        absl::Mutex mu_;

        // assume count_ is not internal reference count
        int count_ ABSL_GUARDED_BY(mu_);


        absl::Condition count_is_zero(+[](int *count) { return *count == 0; }, &count_);

        mu_.LockWhen(count_is_zero);
        // ...
        mu_.Unlock();

// Example using a scope guard:

        {
            absl::MutexLock lock(&mu_, count_is_zero);
            // ...
        }


    }


    absl::once_flag once;

    absl::Mutex counters_mu(absl::kConstInit);

    int running_thread_count ABSL_GUARDED_BY(counters_mu) = 0;
    int call_once_invoke_count ABSL_GUARDED_BY(counters_mu) = 0;
    int call_once_finished_count ABSL_GUARDED_BY(counters_mu) = 0;
    int call_once_return_count ABSL_GUARDED_BY(counters_mu) = 0;
    bool done_blocking ABSL_GUARDED_BY(counters_mu) = false;


    int cnt = 0;
    for (size_t i = 0; i < 10; i++) {
        std::thread t([&] {
            absl::SleepFor(absl::Milliseconds(10));

            running_thread_count++;
            cnt++;
        });

        t.detach();
//        counters_mu.Lock();
//        counters_mu.Unlock();
    }
    absl::SleepFor(absl::Seconds(1));
    absl::PrintF("running_thread_count = %v", running_thread_count);
    absl::PrintF("cnt = %v", cnt);

}

// https://stackoverflow.com/questions/56303939/c-stdvariant-vs-stdany
//https://stackoverflow.com/questions/64479978/in-c-when-should-we-use-stdany-stdvariant-or-stdoptional
//https://stackoverflow.com/questions/46053099/stdvariant-vs-stdany-when-type-is-move-constructible

// https://en.cppreference.com/w/cpp/utility/variant
TEST_CASE("types") {

    {
        // Construct a Span explicitly from a container:
        std::vector<int> v = {1, 2, 3, 4, 5};
        auto span_1 = absl::Span<const int>(v);

        absl::PrintF("span_1 size: %v", span_1.size());
        // Construct a Span explicitly from a C-style array:
        int a[5] = {1, 2, 3, 4, 5};
        auto span_2 = absl::Span<int>(a);
        span_2[0] = 100;
        absl::PrintF("span_2 size: %v", span_2.size());
        absl::PrintF("span_2[0]: %v", span_2[0]);


    }
    {

        auto a = absl::any(65);
        if (a.has_value()) {
            absl::PrintF("optional a = %v", absl::any_cast<int>(a));
        }
    }
    {
        absl::optional<int> a = 1;
        if (a.has_value()) {
            absl::PrintF("optional a = %v", a.value());
        }
        a = absl::nullopt;
        if (!a.has_value()) {
            absl::PrintF("optional a is null");
        }
    }

    {


        {
            // Construct a variant that holds either an integer or a std::string and
            // assign it to a std::string.
            absl::variant<int, std::string> v = std::string("abc");

            // A default-constructed variant will hold a value-initialized value of
            // the first alternative type.
            auto a = absl::variant<int, std::string>();   // Holds an int of value '0'.

            // assignment through type conversion
            a = 128;         // variant contains int
            a = "128";       // variant contains std::string
        }
        {
            // variants are assignable.

            // copy assignment
            auto v1 = absl::variant<int, std::string>("abc");
            auto v2 = absl::variant<int, std::string>(10);
            v2 = v1;  // copy assign
        }

        {
            // move assignment
            auto v1 = absl::variant<int, std::string>("abc");
            v1 = absl::variant<int, std::string>(10);
            int v2 = absl::get<int>(v1);

        }
        {
            absl::variant<int, std::string> v;
            v = 1;
            try {
                const std::string& s= absl::get<std::string>(v);
            } catch (const absl::bad_variant_access &e) {
                std::cout << "Bad variant access: " << e.what() << '\n';
            }
            v = 1;

            if (absl::holds_alternative<int>(v)) {
                std::cout << "holds_alternative int: " << absl::get<int>(v) << std::endl;
            }
            v = "hello";

            if (absl::holds_alternative<std::string>(v)) {
                std::cout << "holds_alternative string: " << absl::get<std::string>(v) << std::endl;

            }


        }


    }


//   absl::any_cast<char>(a);        // throws absl::bad_any_cast
//   absl::any_cast<std::string>(a); // throws absl::bad_any_cast
}

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
TEST_CASE("FixedArray") {

    uint64_t x = 1;

    ankerl::nanobench::Bench().run("x += x", [&]() {

        ankerl::nanobench::doNotOptimizeAway(x += x);

    });


    ankerl::nanobench::Bench().run("FixedArray write", [&]() {
        absl::FixedArray<size_t> array(50);
        for (int j = 0; j < 100; j++) {
            for (size_t i = 0; i < 50; i++) {
                ankerl::nanobench::doNotOptimizeAway(array[i] = i);
            }
        }

    });

    ankerl::nanobench::Bench().run("std::array write", [&]() {
        std::array<size_t, 50> array;
        for (int j = 0; j < 100; j++) {
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
TEST_CASE("InlinedVector") {

    SUBCASE("std::vector") {
        // now let's benchmark:
        ankerl::nanobench::Bench().run("std::vector push_back", [&]() {
            std::vector<size_t> array;

            for (size_t i = 0; i < 100; i++) {
                array.push_back(i);
            }
        });
    }


    SUBCASE("InlinedVector") {
        ankerl::nanobench::Bench().run("InlinedVector push_back", [&]() {
            absl::InlinedVector<size_t, 100> array;

            for (size_t i = 0; i < 100; i++) {
                array.push_back(i);
            }
        });
    }


}


static void DumpPCAndSymbol(void *pc) {
    char tmp[1024];
    const char *symbol = "(unknown)";
    if (absl::Symbolize(pc, tmp, sizeof(tmp))) {
        symbol = tmp;
    }
    absl::PrintF("%p  %s\n", pc, symbol);
}

TEST_CASE("debug") {

    if (absl::LeakCheckerIsActive()) {

        auto foo = absl::IgnoreLeak(new std::string("some ignored leaked string"));
        ABSL_RAW_LOG(INFO, "Ignoring leaked string %s", foo->c_str());

        char *foo2 = strdup("lsan should complain about this leaked string");
        ABSL_RAW_LOG(INFO, "Should detect leaked string %s", foo2);
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

TEST_CASE("time") {

    // from system clock
    absl::Time t1 = absl::Now();

    //from unix epoch
    absl::Time t2 = absl::Time();

    // direction from epoch
    absl::Time t3 = absl::UnixEpoch();

    absl::Duration d1 = t2 - t1;
    absl::Duration d2 = t3 - t2;


    absl::PrintF("duration d1 : %v", ToChronoHours(d1).count());
    absl::PrintF("duration d2 : %v", absl::ToChronoNanoseconds(d2).count());


}


TEST_CASE("str") {

    {
        std::string s = absl::StrFormat("Welcome to %s, Number %d!", "The Village", 6);
        CHECK ("Welcome to The Village, Number 6!" == s);
    }
    {
        constexpr absl::string_view kFormatString = "Welcome to %s, Number %d!";
        std::string s = absl::StrFormat(kFormatString, "The Village", 6);
        CHECK ("Welcome to The Village, Number 6!" == s);

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
        absl::PrintF("has_hello: %v", has_hello);

        std::string decode;
        bool decode_ok = absl::Base64Unescape(title, &decode);

    }

}

void raise_error(int x, int y) {
    CHECK_EQ(2 * x, y) << "oops!";

}

TEST_CASE("log") {
    LOG(INFO) << "aaa";

    std::string filenames_sorted;
    {
        int x = 3, y = 6;
        raise_error(x, y);
    }

}


struct Point {
    float x = 0.0;

};

absl::StatusOr<Point> getPoint(float x) {
    Point p{x};

    if (x > 0.0) {
        return p;

    } else {
        return absl::InvalidArgumentError("bad mode");
    }
    return absl::InvalidArgumentError("bad mode");

}

TEST_CASE("status") {
    absl::StatusOr<Point> result = getPoint(-1.0);
    if (result.ok()) {
        std::cout << result->x;
    } else {
        LOG(ERROR) << result.status();
    }

}