add_library(socket_manager STATIC SocketManager.cpp)
target_link_libraries(socket_manager PUBLIC Sockpp::sockpp-static)
target_include_directories( socket_manager PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)


add_executable(socket_client_test socket_client_test.cpp)
target_link_libraries(socket_client_test PUBLIC socket_manager )
target_include_directories( socket_client_test PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

add_executable(socket_server_test socket_server_test.cpp)
target_link_libraries(socket_server_test PUBLIC socket_manager )
target_include_directories( socket_server_test PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

#set_asan(socket_manager)
#set_asan(socket_client_test)
#set_asan(socket_server_test)
