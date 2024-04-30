# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

#add_library(fastdds_builder STATIC fastdds_builder.cpp)

#https://zhuanlan.zhihu.com/p/82804123
#target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)
#set_property(TARGET fastdds_builder PROPERTY POSITION_INDEPENDENT_CODE ON)


#target_link_libraries(fastdds_builder PUBLIC fastrtps fastcdr )
#target_include_directories( fastdds_builder PUBLIC  ${CMAKE_SOURCE_DIR}/include  )
#
#target_link_libraries(fastdds_builder PUBLIC
#        toml11::toml11
#        absl::strings
#        absl::str_format
#        absl::log
#        #        absl::log_internal_check_impl
#        absl::log_internal_check_op
#        absl::statusor
#        absl::debugging
#        absl::random_random
#        absl::any
#)


#add_executable(fastdds_builder_test_pub fastdds_builder_test_pub.cpp)
#target_link_libraries(fastdds_builder_test_pub fastdds_builder)
#
#target_include_directories( fastdds_builder_test_pub PUBLIC  ${CMAKE_SOURCE_DIR}/include  )
#
#add_executable(fastdds_builder_test_sub fastdds_builder_test_sub.cpp)
#target_link_libraries(fastdds_builder_test_sub fastdds_builder)
#target_include_directories( fastdds_builder_test_sub PUBLIC ${CMAKE_SOURCE_DIR}/include  )

# Usage example:


copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}" "*toml" "${EXECUTABLE_OUTPUT_PATH}" TOML_COPIED_FILES)
copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}" "*xml" "${EXECUTABLE_OUTPUT_PATH}" XML_COPIED_FILES)
list(APPEND COMBINED_LIST ${TOML_COPIED_FILES} ${XML_COPIED_FILES})

message("TOML_COPIED_FILES ${TOML_COPIED_FILES}")
message("XML_COPIED_FILES ${XML_COPIED_FILES}")
message("COMBINED_LIST ${COMBINED_LIST}")

add_custom_target(fastdds_builder_test_pub_file_copy DEPENDS ${COMBINED_LIST})

# Make your target depend on the custom target for copied files
#add_dependencies(fastdds_builder_test_pub fastdds_builder_test_pub_file_copy)


# Glob all .cxx files in the source directory
file(GLOB DDS_MESSAGE_SOURCES "./message/*.cxx")
MESSAGE(STATUS "DDS_MESSAGE_SOURCES=${DDS_MESSAGE_SOURCES}")

# Add the globbed sources to your library
add_library(DDS_MESSAGE_LIB STATIC ${DDS_MESSAGE_SOURCES})
set_pic(DDS_MESSAGE_LIB)
target_link_libraries(DDS_MESSAGE_LIB PUBLIC fastcdr fastrtps)
add_executable(HelloWorldMain HelloWorldMain.cpp)
target_link_libraries(HelloWorldMain PUBLIC DDS_MESSAGE_LIB fastrtps fastcdr)
add_dependencies(HelloWorldMain fastdds_builder_test_pub_file_copy)
target_include_directories( HelloWorldMain PUBLIC  ${CMAKE_SOURCE_DIR}/include  )

#set_asan(HelloWorldMain)


add_executable(dds_simple_participant dds_simple_participant.cpp)
target_link_libraries(dds_simple_participant PUBLIC fastcdr fastrtps)
target_include_directories( dds_simple_participant PUBLIC ${CMAKE_SOURCE_DIR}/include  )
add_dependencies(dds_simple_participant fastdds_builder_test_pub_file_copy)





