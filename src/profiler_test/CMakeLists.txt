add_executable(easy_profiler_test easy_profiler_test.cpp )

target_link_libraries(easy_profiler_test PUBLIC  easy_profiler)  # STEP 2 ##########
target_include_directories(easy_profiler_test PUBLIC ${CMAKE_SOURCE_DIR}/include)