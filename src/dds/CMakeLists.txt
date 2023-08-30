if(NOT TARGET CycloneDDS-CXX::ddscxx)
    find_package(CycloneDDS-CXX REQUIRED)
endif()

idlcxx_generate(TARGET helloworlddata FILES HelloWorldData.idl WARNINGS no-implicit-extensibility)
idlcxx_generate(TARGET roundtripdata FILES RoundTrip.idl WARNINGS no-implicit-extensibility)
idlcxx_generate(TARGET ShmThroughput_lib FILES ShmThroughput.idl WARNINGS no-implicit-extensibility)


add_executable(ddscxxHelloworldPublisher publisher.cpp)
add_executable(ddscxxHelloworldSubscriber subscriber.cpp)

add_executable(ddscxxPing ping.cpp)
add_executable(ddscxxPong pong.cpp)

# Link both executables to idl data type library and ddscxx.
target_link_libraries(ddscxxHelloworldPublisher CycloneDDS-CXX::ddscxx helloworlddata)
target_link_libraries(ddscxxHelloworldSubscriber CycloneDDS-CXX::ddscxx helloworlddata)
target_link_libraries(ddscxxPing CycloneDDS-CXX::ddscxx roundtripdata ShmThroughput_lib)
target_link_libraries(ddscxxPong CycloneDDS-CXX::ddscxx roundtripdata ShmThroughput_lib)


target_include_directories(ddscxxPing PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(ddscxxPong PUBLIC ${CMAKE_SOURCE_DIR}/include)

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