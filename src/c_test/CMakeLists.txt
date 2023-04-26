

add_executable(c_thread_test thread_test.c)
target_link_libraries( c_thread_test PUBLIC -pthread)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

set_property(TARGET c_thread_test PROPERTY C_STANDARD 11)