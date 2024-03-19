set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED TRUE)


if(NOT TARGET CycloneDDS-CXX::ddscxx)
    find_package(CycloneDDS-CXX REQUIRED)
endif()

idlcxx_generate(TARGET helloworlddata FILES HelloWorldData.idl WARNINGS no-implicit-extensibility)
idlcxx_generate(TARGET roundtripdata FILES RoundTrip.idl WARNINGS no-implicit-extensibility)
idlcxx_generate(TARGET ShmThroughput_lib FILES ShmThroughput.idl WARNINGS no-implicit-extensibility)

idlc_generate(TARGET ShmThroughput_lib_c FILES "ShmThroughput.idl")


add_executable(ddscxxHelloworldPublisher publisher.cpp)
add_executable(ddscxxHelloworldSubscriber subscriber.cpp)

add_executable(ddscxxPing ping.cpp)
add_executable(ddscxxPong pong.cpp)


add_executable(ShmThroughputPublisher shmpublisher.c)
add_executable(ShmThroughputSubscriber shmsubscriber.c)

# Link both executables to idl data type library and ddscxx.
target_link_libraries(ddscxxHelloworldPublisher CycloneDDS-CXX::ddscxx helloworlddata)
target_link_libraries(ddscxxHelloworldSubscriber CycloneDDS-CXX::ddscxx helloworlddata)
target_link_libraries(ddscxxPing CycloneDDS-CXX::ddscxx roundtripdata ShmThroughput_lib)
target_link_libraries(ddscxxPong CycloneDDS-CXX::ddscxx roundtripdata ShmThroughput_lib)


target_link_libraries(ShmThroughputPublisher ShmThroughput_lib_c CycloneDDS::ddsc)
target_link_libraries(ShmThroughputSubscriber ShmThroughput_lib_c CycloneDDS::ddsc)


target_include_directories(ddscxxPing PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(ddscxxPong PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_compile_options(ddscxxPing PRIVATE -g -O0)
target_compile_options(ddscxxPong PRIVATE -g -O0)


# Disable the static analyzer in GCC to avoid crashing the GNU C++ compiler
# on Azure Pipelines
if(DEFINED ENV{SYSTEM_TEAMFOUNDATIONSERVERURI})
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND ANALYZER STREQUAL "on")
        target_compile_options(ddscxxHelloworldPublisher PRIVATE -fno-analyzer)
        target_compile_options(ddscxxHelloworldSubscriber PRIVATE -fno-analyzer)
    endif()
endif()

set_property(TARGET ddscxxHelloworldPublisher PROPERTY CXX_STANDARD 11)
set_property(TARGET ddscxxHelloworldSubscriber PROPERTY CXX_STANDARD 11)


add_executable(listtopics listtopics.c)
target_link_libraries(listtopics CycloneDDS::ddsc)



add_executable(fastdds_pub fastdds_pub.cpp ShmThroughput.cxx ShmThroughputPubSubTypes.cxx)
target_compile_definitions(fastdds_pub PRIVATE
        $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
        $<$<BOOL:${INTERNAL_DEBUG}>:__INTERNALDEBUG> # Internal debug activated.
)
target_link_libraries(fastdds_pub fastrtps fastcdr foonathan_memory)


add_executable(fastdds_sub fastdds_sub.cpp ShmThroughput.cxx ShmThroughputPubSubTypes.cxx)
target_compile_definitions(fastdds_sub PRIVATE
        $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
        $<$<BOOL:${INTERNAL_DEBUG}>:__INTERNALDEBUG> # Internal debug activated.
)
target_link_libraries(fastdds_sub fastrtps fastcdr foonathan_memory)

target_include_directories(fastdds_pub PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_include_directories(fastdds_sub PUBLIC ${CMAKE_SOURCE_DIR}/include)


add_subdirectory(ZeroCopyExample)
add_subdirectory(HelloWorldExampleDataSharing)
add_subdirectory(HelloWorldExampleSharedMem)
add_subdirectory(HelloWorldExampleTCP)


add_library(fastdds_helper STATIC fastdds_helper.cpp ShmThroughput.cxx ShmThroughputPubSubTypes.cxx)
target_include_directories( fastdds_helper PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${tinycc_LIBTCC_INCLUDE_DIR}
)

target_link_libraries(fastdds_helper PUBLIC
        absl::strings  absl::str_format
        absl::log
        #        absl::log_internal_check_impl
        absl::log_internal_check_op
        absl::statusor
        absl::debugging
        absl::random_random
        absl::any
        absl::inlined_vector
        absl::flat_hash_map
)
target_link_libraries(fastdds_helper PUBLIC toml11::toml11 tcc_helper  fastrtps fastcdr foonathan_memory)

#set_asan(fastdds_helper)
add_executable(fastdds_helper_test fastdds_helper_test.c)

target_include_directories( fastdds_helper_test PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${tinycc_LIBTCC_INCLUDE_DIR}
)

target_link_libraries(fastdds_helper_test PUBLIC fastdds_helper
)

set_property(TARGET fastdds_helper_test PROPERTY C_STANDARD 11)

message("ROS_DISTRO = " ${ROS_DISTRO})
if (ROS_DISTRO)
    message("ROS_DISTRO = " ${ROS_DISTRO})

    add_library(ros_helper_impl ros_helper_impl.cpp ros_helper_topic.cpp )
    add_ros(ros_helper_impl roscpp)

    target_include_directories( ros_helper_impl PUBLIC
            ${CMAKE_SOURCE_DIR}/include
            ${tinycc_LIBTCC_INCLUDE_DIR}
    )
    target_link_libraries(ros_helper_impl PUBLIC
            absl::strings  absl::str_format
            absl::log
            #        absl::log_internal_check_impl
            absl::log_internal_check_op
            absl::statusor
            absl::debugging
            absl::random_random
            absl::any
            absl::inlined_vector
            absl::flat_hash_map
    )
    target_link_libraries(ros_helper_impl PUBLIC toml11::toml11 tcc_helper)

    target_link_libraries(ros_helper_impl PUBLIC tf )
    set_asan(ros_helper_impl)

    add_library(ros_helper ros_helper.cpp)
    target_link_libraries(ros_helper PUBLIC ros_helper_impl)
    add_executable(ros_helper_test ros_helper_test.c)
    target_link_libraries( ros_helper_test PUBLIC ros_helper)
endif ()


add_executable(data_holder_test_c data_holder_test.c)
add_executable(data_holder_test_cpp data_holder_test.cpp)

target_include_directories( data_holder_test_c PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${tinycc_LIBTCC_INCLUDE_DIR}
)
target_include_directories( data_holder_test_cpp PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${tinycc_LIBTCC_INCLUDE_DIR}
)

target_link_libraries( data_holder_test_c PUBLIC tinyalloc)
target_link_libraries( data_holder_test_cpp PUBLIC tinyalloc)
