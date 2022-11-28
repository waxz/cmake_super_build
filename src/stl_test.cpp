//
// Created by waxz on 22-10-22.
//

#include <cstring>
#include <string>

#include <thread>
#include <mutex>
#include <atomic>

#include <memory>
#include <condition_variable>
#include <mutex>

#include <functional>

#include <chrono>
#include <algorithm>
#include <vector>
#include <deque>
#include <map>
#include <set>
#include <array>

#include <queue>
#include <iostream>

#include "common/clock_time.h"
#include "common/suspend.h"
#include "common/mio.hpp"

#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs = std::filesystem;
#endif
#endif
#ifndef GHC_USE_STD_FS
//#include "filesystem.hpp"
#include "ghc/filesystem.hpp"

namespace fs = ghc::filesystem;
#endif


#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<optional>)
#define GHC_USE_STD_OPTIONAL
#include <optional>

template<typename T>
using optional = std::optional<T>;
const auto& nullopt = std::nullopt;

#endif
#endif
#ifndef GHC_USE_STD_OPTIONAL
#include "tl/optional.hpp"

template<typename T>
using optional = tl::optional<T>;
const auto& nullopt = tl::nullopt;

#endif



struct SimpleThread {

    std::thread run_thread;


    template<typename ...ARGS>
    explicit SimpleThread(ARGS ...args):run_thread(args...) {

//        std::cout << "construct \n" <<   __FILE__ << ":" << __LINE__ << std::endl;

    }

    //auto st2 = std::move(st1);
    SimpleThread(SimpleThread &&rhv) noexcept {
//        std::cout << "construct \n" <<   __FILE__ << ":" << __LINE__ << std::endl;

        this->run_thread = std::move(rhv.run_thread);
    }

    //auto st2 = st1;
    SimpleThread(SimpleThread &rhv) noexcept {
//        std::cout << "construct \n" <<   __FILE__ << ":" << __LINE__ << std::endl;
        this->run_thread = std::move(rhv.run_thread);
    }

    ~SimpleThread() {
        if (run_thread.joinable()) {
            run_thread.join();
        }
//        std::cout << "destroy \n" <<   __FILE__ << ":" << __LINE__ << std::endl;

    }
};


/** @file */

/**
 * @brief Thread-safe queue. Particularly useful when multiple threads need to write to and/or read
 * from a queue.
 */
template <typename T>
class Queue {
public:
    /**
     * @brief Get the size of the queue
     * @return Queue size
     */
    auto size() const noexcept {
          const std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /**
     * @brief Check if the queue is empty
     * @return True if the queue is empty, otherwise false
     */
    auto empty() const noexcept {
        const std::lock_guard<std::mutex> lock(mutex_);

        return queue_.empty();
    }

    /**
     * @brief Push data into the queue
     * @param value Data to push into the queue
     */
    void push(T value) noexcept {
        const std::lock_guard<std::mutex> lock(mutex_);

        queue_.push(std::move(value));
        cv_.notify_one();
    }

    /**
     * @brief Clear the queue
     */
    void clear() noexcept {
        const std::lock_guard<std::mutex> lock(mutex_);


        // Swap queue with an empty queue of the same type to ensure queue_ is left in a
        // default-constructed state
        decltype(queue_)().swap(queue_);
    }

    /**
     * @brief Wait for given duration then pop from the queue and return the element
     * @param wait_time Maximum time to wait for queue to be non-empty
     * @return Data popped from the queue or error
     */
    auto pop(std::chrono::nanoseconds wait_time = {}) -> optional<T> {
        std::unique_lock<std::mutex> lock(mutex_);

        // If queue is empty after wait_time, return nothing
        if (!cv_.wait_for(lock, wait_time, [this] { return !queue_.empty(); })) return nullopt;

        auto value = queue_.front();
        queue_.pop();
        return value;
    }

private:
    std::queue<T> queue_;
    std::condition_variable cv_;
    mutable std::mutex mutex_;
};

struct ThreadPool {

    std::vector<SimpleThread> run_thread_pool;

    std::deque<std::function<int()> > task_queue;
    std::mutex task_queue_mtx;


    std::atomic_bool stop;
    std::atomic_int task_num;

    ThreadPool(int thread_num) : stop(false), task_num(0) {
        thread_num =
                thread_num > std::thread::hardware_concurrency() ? std::thread::hardware_concurrency() : thread_num;

        for (int i = 0; i < thread_num; i++) {

            run_thread_pool.emplace_back(SimpleThread([&] {

                common::Suspend suspend;
                std::function<int()> task;

                do {
//                    std::cout <<" check task_num " << task_num << std::endl;
                    if (task_num > 0) {
                        std::lock_guard<std::mutex> locker(task_queue_mtx);
                        if (task_queue.empty()) continue;
                        task = task_queue.back();
                        task_queue.pop_back();
                        task_num--;
                        task();
                    } else {
//                        std::cout << "suspend " << std::this_thread::get_id() << std::endl;
                        suspend.sleep(1000);
                    }
                } while (!stop);


            }));
        }
    }


    void addTask(const std::function<int()> &task) {

        std::lock_guard<std::mutex> locker(task_queue_mtx);


        task_queue.emplace_front(task);
        task_num++;

    }

    ~ThreadPool() {
        stop = true;

    }
};


struct ThreadPool_V2 {


    std::vector<SimpleThread> run_thread_pool;

    Queue<std::function<int()> > task_queue;


    std::atomic_bool stop;

    ThreadPool_V2(int thread_num) : stop(false) {
        thread_num =
                thread_num > std::thread::hardware_concurrency() ? std::thread::hardware_concurrency() : thread_num;

        for (int i = 0; i < thread_num; i++) {

            run_thread_pool.emplace_back(SimpleThread([&] {

                common::Suspend suspend;
                std::function<int()> task;

                do {
                    if (!task_queue.empty()) {
                        using namespace std::chrono_literals;
                        auto task_opt = task_queue.pop(10ms);
                        if(task_opt){
                            (*task_opt)();
                        }

                    } else {
//                        std::cout << "suspend " << std::this_thread::get_id() << std::endl;
                        suspend.sleep(1000);
                    }
                } while (!stop);


            }));
        }
    }


    void addTask(  std::function<int()> &&task) {
        task_queue.push(std::forward<std::function<int()> &&>(task));

    }

    ~ThreadPool_V2() {
        stop = true;

    }
};
inline void simple_cast(std::string &str, const char *c_str) {

    str.assign(c_str, std::strlen(c_str));
}


#include <random>
#include <type_traits>
#include <array>
#include <cassert>
#include <iostream>
#include <iomanip>
//#include <Eigen/Geometry>

/** @file */

/**
 * @brief Get a random number generator
 *
 * The first time this function is called it creates a thread_local random number generator. If a
 * seed sequence is provided on that first call it is used to create the generator, otherwise the
 * random device is used to seed the generator. After the first call to this function, if a seed
 * sequence is provided this function throws. The returned value is a reference to a thread_local
 * static generator.
 *
 * @param seed_sequence Seed sequence for random number generator
 *
 * @return Seeded random number generator
 */
auto rng(std::seed_seq seed_sequence = {}) -> std::mt19937&;

/**
 * @brief Get a uniform real number in a given range
 *
 * @param lower Lower bound, inclusive
 * @param upper Upper bound, exclusive
 *
 * @tparam RealType Floating point type
 *
 * @return Uniform real in range [lower, upper)
 */
template <typename RealType>
auto uniform_real(RealType lower, RealType upper) {
    static_assert(std::is_floating_point<RealType>::value , "RealType is not floating_point");
    assert(lower < upper);
    return std::uniform_real_distribution<>(lower, upper)(rng());
}


template <typename RealType>
auto normal_real(RealType mean, RealType stddev) {
    static_assert(std::is_floating_point<RealType>::value , "RealType is not floating_point");
    return std::normal_distribution<>(mean, stddev)(rng());
}

/**
 * @brief Get a uniform integer number in a given range
 *
 * @param lower Lower bound, inclusive
 * @param upper Upper bound, inclusive
 *
 * @tparam IntType Integral type
 *
 * @return Uniform integer in range [lower, upper]
 */
template <typename IntType>
auto uniform_int(IntType lower, IntType upper) {
    static_assert(std::is_integral<IntType>::value , "RealType is not integral");
    assert(lower <= upper);
    return std::uniform_int_distribution<>(lower, upper)(rng());
}

/**
 * @brief Generate a random unit quaternion of doubles
 * @return Random unit quaternion
 */
#if 0
auto random_unit_quaternion() -> Eigen::Quaterniond;

#endif

auto rng(std::seed_seq seed_sequence) -> std::mt19937& {
    thread_local auto first = true;
    thread_local auto generator = [&seed_sequence]() {
        first = false;
        if (seed_sequence.size() > 0) return std::mt19937(seed_sequence);
        auto seed_data = std::array<int, std::mt19937::state_size>();
          std::random_device random_device;
        std::generate_n(seed_data.data(), seed_data.size(), std::ref(random_device));
        std::seed_seq sequence (std::begin(seed_data), std::end(seed_data));
        return std::mt19937(sequence);
    }();

    if (!first && seed_sequence.size() > 0)
        throw std::runtime_error("rng cannot be re-seeded on this thread");
    return generator;
}

#if 0
auto random_unit_quaternion() -> Eigen::Quaterniond {
    // From "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, pg. 124-132
    auto const x0 = uniform_real(0., 1.);
    auto const r1 = std::sqrt(1 - x0);
    auto const r2 = std::sqrt(x0);
    auto const t1 = uniform_real(0., 2 * M_PI);
    auto const t2 = uniform_real(0., 2 * M_PI);
    auto const x = r1 * std::sin(t1);
    auto const y = r1 * std::cos(t1);
    auto const z = r2 * std::sin(t2);
    auto const w = r2 * std::cos(t2);
    return Eigen::Quaterniond(w, x, y, z).normalized();
}
#endif

void test_random(){

    std::random_device rd{};
    std::mt19937 gen{rd()};

    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d{5,2};


    std::map<int, int> hist{};
    for(int n=0; n<10000; ++n) {
//        ++hist[std::round(d(gen))];
        ++hist[std::round( 50.0 * normal_real(2.0 , 0.1))];

    }


    for(auto p : hist) {
        std::cout << std::setw(2)
                  << p.first/50.0 << ' ' << std::string(p.second/10, '*') << '\n';
    }


}
void test_thread() {

    int i = 9;
    std::cout << "i = " << i << ", i-- = " << (i--) << ", i = " << i << std::endl;


    auto st1 = SimpleThread([] {
        std::cout << " hello from thread " << std::this_thread::get_id() << std::endl;

    });
    auto st2 = std::move(st1);

    auto st3 = st2;

    auto func = [](int a) {
        std::cout << "int " << a << std::endl;
        return a + 23;
    };
    auto st4 = SimpleThread(func, 4);


    ThreadPool_V2 tp(5);

    std::shared_ptr<int> rt;

    int share_num = 1;
    tp.addTask([&] {
        std::cout << "run share_num " << share_num << std::endl;

        std::cout << "time " << common::getCurrentDateTime();
        share_num = 2;

        std::cout << "run in " << std::this_thread::get_id() << std::endl;
        return share_num;
    });
    tp.addTask([&] {
        std::cout << "run share_num " << share_num << std::endl;
        std::cout << "time " << common::getCurrentDateTime();

        share_num = 3;
        std::cout << "run in " << std::this_thread::get_id() << std::endl;
        return share_num;

    });
    tp.addTask([&] {
        std::cout << "run share_num " << share_num << std::endl;
        std::cout << "time " << common::getCurrentDateTime();
        common::Suspend suspend;
        suspend.sleep(5000);
        std::cout << "sleep in " << std::this_thread::get_id() << std::endl;

        std::cout << "run in " << std::this_thread::get_id() << std::endl;
        return share_num;

    });


    common::Suspend suspend;
    suspend.sleep(1000);
}

void test_string() {

    char cstr[100];
    float a = 1.9, b = 2.4;
    sprintf(cstr, "%.6f %.6f", a, b);
    std::string str;
    str.assign(cstr, std::strlen(cstr));
    std::cout << "cstr: " << cstr << ", str: " << str;

    int N = 10000000;
    common::Time t1, t2;

    t1 = common::FromUnixNow();
    for (int i = 0; i < N; i++) {
        str.assign(cstr, std::strlen(cstr));

    }
    std::cout << "tese 1:\n" << __FILE__ << ":" << __LINE__ << std::endl;

    std::cout << common::ToMillSeconds(common::FromUnixNow() - t1) << std::endl;

    t1 = common::FromUnixNow();

    for (int i = 0; i < N; i++) {
        str = std::string(cstr);
    }
    std::cout << "tese 2:\n" << __FILE__ << ":" << __LINE__ << std::endl;
    std::cout << common::ToMillSeconds(common::FromUnixNow() - t1) << std::endl;


    t1 = common::FromUnixNow();

    for (int i = 0; i < N; i++) {
        simple_cast(str, cstr);
    }
    std::cout << "tese 3:\n" << __FILE__ << ":" << __LINE__ << std::endl;


    std::cout << common::ToMillSeconds(common::FromUnixNow() - t1) << std::endl;

}


void preciseSleep(double seconds) {
    using namespace std;
    using namespace std::chrono;

    static double estimate = 5e-3;
    static double mean = 5e-3;
    static double m2 = 0;
    static int64_t count = 1;

    while (seconds > estimate) {
        auto start = high_resolution_clock::now();
        this_thread::sleep_for(milliseconds(1));
        auto end = high_resolution_clock::now();

        double observed = (end - start).count() / 1e9;
        seconds -= observed;

        ++count;
        double delta = observed - mean;
        mean += delta / count;
        m2   += delta * (observed - mean);
        double stddev = sqrt(m2 / (count - 1));
        estimate = mean + stddev;
    }

    // spin lock
    auto start = high_resolution_clock::now();
    while ((high_resolution_clock::now() - start).count() / 1e9 < seconds);
}



struct TaskManager{
    struct Task{
        bool valid = true;
        common::Time time;
        double delay_ms = 100;
        std::function<bool()> func;
        Task(const std::function<bool()>& func_,float delay_ms_ = 100):time(common::FromUnixNow()),delay_ms(delay_ms_),func(func_){
        }
        Task(const Task& rhv){
            valid = rhv.valid;
            time = rhv.time;
            delay_ms = rhv.delay_ms;
            func = std::move(rhv.func);
        }

    };
    std::vector<Task> task_queue;

    TaskManager(){}

    /*
     const std::function<bool()>& func
     return true, keep running at given rate
     return false, run once
     */
    void addTask(const std::function<bool()>& func,float delay_ms = 100){
        task_queue.emplace_back(func,delay_ms);

    }
    bool call(){
        common::Time now = common::FromUnixNow();

        bool run = false;

        if (!task_queue.empty()) {

            for(auto& task_opt: task_queue)
            {
                if(common::ToMillSeconds(now - task_opt.time) >= task_opt.delay_ms){
                    run = true;
                    std::cout << "run TaskManager time " << common::getCurrentDateTime();
                    task_opt.valid = task_opt.func();
                    if(task_opt.valid){
                        task_opt.time = now;
                    }
                }

            }
            if(run){

                auto it  = std::remove_if(task_queue.begin(),task_queue.end(),[](auto& e){
                    return !e.valid;
                });

                task_queue.erase(it, task_queue.end());
            }


        }

        return !task_queue.empty();

    }

};

void test_clock(){

    common::Time  time = common::FromUnixNow();

    common::Suspend sleep2;

    while (true){

        preciseSleep(1);
//        sleep2.sleep(16.7);

        std::cout << common::ToMicroSeconds(common::FromUnixNow() - time) << "\n";
        time = common::FromUnixNow();
    }
}


int handle_error(const std::error_code& error)
{
    const auto& errmsg = error.message();
    std::printf("error mapping file: %s, exiting...\n", errmsg.c_str());
    return error.value();
}

void allocate_file(const std::string& path, const int size)
{
    std::ofstream file(path);
    std::string s(size, '0');
    file << s;
}


void test_delay_task(){

    TaskManager td;

    td.addTask([&]{
        std::cout << "hello 1" << std::endl;
        return false;
    },120);

    td.addTask([&]{
        std::cout << "hello 2" << std::endl;
        return true;
    },1200);

    common::Time  time = common::FromUnixNow();
    std::cout << "run main time " << common::getCurrentDateTime();

    while (true){
        if(!td.call()){
            break;
        }
    }



}

struct Test{
    Test(){
    std::cout << "construct test"<<std::endl;
    }
    Test(Test&& rhv){
        std::cout << "move construct test"<<std::endl;

    }
    Test(const Test&  rhv){
        std::cout << "copy move construct test"<<std::endl;

    }
    ~Test(){
        std::cout << "deconstruct test"<<std::endl;

    }

};

int mm_test()
{
    const auto path = "file.txt";

    // NOTE: mio does *not* create the file for you if it doesn't exist! You
    // must ensure that the file exists before establishing a mapping. It
    // must also be non-empty. So for illustrative purposes the file is
    // created now.
    allocate_file(path, 155);


    // Read-write memory map the whole file by using `map_entire_file` where the
    // length of the mapping is otherwise expected, with the factory method.
    std::error_code error;
    mio::mmap_sink rw_mmap = mio::make_mmap_sink(
            path, 0, mio::map_entire_file, error);
    if (error) { return handle_error(error); }

    // You can use any iterator based function.
    std::fill(rw_mmap.begin(), rw_mmap.end(), 'a');




    // Or manually iterate through the mapped region just as if it were any other
    // container, and change each byte's value (since this is a read-write mapping).
    for (auto& b : rw_mmap) {
        b += 10;
    }

    // Or just change one value with the subscript operator.
    const int answer_index = rw_mmap.size() / 2;
    rw_mmap[answer_index] = 42;

    float* num = reinterpret_cast<float*>(rw_mmap.data());
    *num = 12.3;

    // Don't forget to flush changes to disk before unmapping. However, if
    // `rw_mmap` were to go out of scope at this point, the destructor would also
    // automatically invoke `sync` before `unmap`.
    rw_mmap.sync(error);
    if (error) { return handle_error(error); }

    // We can then remove the mapping, after which rw_mmap will be in a default
    // constructed state, i.e. this and the above call to `sync` have the same
    // effect as if the destructor had been invoked.
    rw_mmap.unmap();

    // Now create the same mapping, but in read-only mode. Note that calling the
    // overload without the offset and file length parameters maps the entire
    // file.
    mio::mmap_source ro_mmap;
    ro_mmap.map(path, error);
    if (error) { return handle_error(error); }

    const float* num2 = reinterpret_cast<const float*>(ro_mmap.data());
    std::cout << "get float from mm file: " << *num2 << std::endl;

    const int the_answer_to_everything = ro_mmap[answer_index];
    assert(the_answer_to_everything == 42);

    return 0;
}

int main(int argc, char **argv) {

    std::cout << "std::thread::hardware_concurrency() = " << std::thread::hardware_concurrency() << std::endl;

//    test_thread();
//    test_string();
//    test_random();

//test_clock();

    mm_test();
}