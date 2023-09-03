

add_executable(c_thread_test thread_test.c)
target_link_libraries( c_thread_test PUBLIC -pthread)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

set_property(TARGET c_thread_test PROPERTY C_STANDARD 11)


add_library(tcc_liba SHARED tcc_liba.c)
add_library(tcc_libb SHARED tcc_libb.c)
add_library(tcc_libc SHARED tcc_libc.c)
target_link_libraries(tcc_libb PUBLIC tcc_liba)
target_link_libraries(tcc_libc PUBLIC tcc_libb)


add_executable(tinycc_test tinycc_test.c)
target_include_directories(tinycc_test PUBLIC ${tinycc_LIBTCC_INCLUDE_DIR}  )
target_link_libraries(tinycc_test PUBLIC ${tinycc_LIB} dl pthread m rt tcc_libc)

target_compile_definitions(tinycc_test PRIVATE tinycc_COMPILER_DIR=${tinycc_COMPILER_DIR} tinycc_LIBTCC_INCLUDE_DIR=${tinycc_LIBTCC_INCLUDE_DIR} tinycc_TCCLIB_INCLUDE_DIR=${tinycc_TCCLIB_INCLUDE_DIR})


set_property(TARGET tinycc_test PROPERTY C_STANDARD 11)
#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/tcc_local
#        DESTINATION ${CMAKE_BINARY_DIR}/bin)
#
#
#add_custom_command(
#        TARGET tinycc_test POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#        ${CMAKE_CURRENT_SOURCE_DIR}/tcc_local
#        ${CMAKE_BINARY_DIR}/bin)
