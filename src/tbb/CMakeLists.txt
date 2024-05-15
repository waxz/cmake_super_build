

add_executable(simple_loop simple_loop.cpp)
target_link_libraries(simple_loop PUBLIC TBB::tbb)
set_omp(simple_loop)
target_include_directories(simple_loop PUBLIC ${CMAKE_SOURCE_DIR}/include  )
