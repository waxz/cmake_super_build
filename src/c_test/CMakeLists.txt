

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

#target_compile_definitions(tinycc_test PRIVATE tinycc_COMPILER_DIR=${tinycc_COMPILER_DIR} tinycc_LIBTCC_INCLUDE_DIR=${tinycc_LIBTCC_INCLUDE_DIR} tinycc_TCCLIB_INCLUDE_DIR=${tinycc_TCCLIB_INCLUDE_DIR})


set_property(TARGET tinycc_test PROPERTY C_STANDARD 11)


message("print_target_properties(tinycc_test)\n" )

print_target_properties(tinycc_test)

#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/tcc_local
#        DESTINATION $<TARGET_PROPERTY:tinycc_test,RUNTIME_OUTPUT_DIRECTORY>)
#get_target_property(tinycc_test_dir tinycc_test  RUNTIME_OUTPUT_DIRECTORY)
#message("tinycc_test_dir: " ${tinycc_test_dir}  )


#
#
#add_custom_command(
#        TARGET tinycc_test POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#        ${CMAKE_CURRENT_SOURCE_DIR}/tcc_local
#        ${CMAKE_BINARY_DIR}/bin)


add_library(tcc_helper STATIC tcc_helper.c  cjson/cJSON.c cjson_helper.c )
target_include_directories(tcc_helper PUBLIC ${tinycc_LIBTCC_INCLUDE_DIR}  ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(tcc_helper PUBLIC ${tinycc_LIB} dl pthread m rt tcc_libc)



idlc_generate(TARGET ShmThroughput_lib_c_tcc FILES "ShmThroughput.idl")

strliteral_build("ShmThroughput.c" "ShmThroughput.he")

add_library(tcc_dds_helper STATIC tcc_dds_helper.c)
target_link_libraries(tcc_dds_helper PUBLIC  CycloneDDS::ddsc tcc_helper ShmThroughput_lib_c_tcc)
target_include_directories(tcc_dds_helper PUBLIC  ${tinycc_LIBTCC_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include)


message("print_target_properties(ShmThroughput_lib_c_tcc)\n" )
print_target_properties(ShmThroughput_lib_c_tcc)
get_target_property(ShmThroughput_lib_c_tcc_INTERFACE_SOURCES ShmThroughput_lib_c_tcc INTERFACE_SOURCES)
message ("ShmThroughput_lib_c_tcc INTERFACE_SOURCES = ${ShmThroughput_lib_c_tcc_INTERFACE_SOURCES}")
get_target_property(ShmThroughput_lib_c_tcc_INTERFACE_INCLUDE_DIRECTORIES ShmThroughput_lib_c_tcc INTERFACE_INCLUDE_DIRECTORIES)
message ("ShmThroughput_lib_c_tcc INTERFACE_INCLUDE_DIRECTORIES = ${ShmThroughput_lib_c_tcc_INTERFACE_INCLUDE_DIRECTORIES}")


add_executable(tcc_dds_pub tcc_dds_pub.c)
add_executable(tcc_dds_sub tcc_dds_sub.c)

target_link_libraries(tcc_dds_pub PUBLIC tcc_dds_helper)
target_link_libraries(tcc_dds_sub PUBLIC tcc_dds_helper)

#target_link_libraries(tcc_dds_pub PUBLIC ${tinycc_LIB} dl pthread m rt tcc_libc)
#target_link_libraries(tcc_dds_sub PUBLIC ${tinycc_LIB} dl pthread m rt tcc_libc)

set_property(TARGET tcc_dds_pub PROPERTY C_STANDARD 11)
set_property(TARGET tcc_dds_sub PROPERTY C_STANDARD 11)




add_executable(tcc_helper_test tcc_helper_test.c )

target_include_directories(tcc_helper_test PUBLIC ${tinycc_LIBTCC_INCLUDE_DIR}  ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(tcc_helper_test PUBLIC tcc_helper)
set_asan(tcc_helper)
set_asan(tcc_helper_test)

add_library(tinyalloc SHARED tinyalloc/tinyalloc.c)

add_executable(tinyalloc_test tinyalloc_test.c)
target_include_directories(tinyalloc_test PUBLIC ${tinycc_LIBTCC_INCLUDE_DIR}  ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(tinyalloc_test PUBLIC tcc_helper tinyalloc)


add_executable(dds_tcc_alloc_test  dds_tcc_alloc_test.c)
target_include_directories(dds_tcc_alloc_test PUBLIC ${tinycc_LIBTCC_INCLUDE_DIR}  ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(dds_tcc_alloc_test PUBLIC tcc_helper tinyalloc tcc_dds_helper)

