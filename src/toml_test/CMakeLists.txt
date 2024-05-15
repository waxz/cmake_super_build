# Copy .toml files to the build directory before building toml_test





add_executable(toml_test toml_test.cpp)
target_link_libraries(toml_test PUBLIC toml11::toml11 )
target_include_directories(toml_test PUBLIC ${CMAKE_SOURCE_DIR}/include)

#file(COPY ./ DESTINATION ${PROJECT_BINARY_DIR} PATTERN *.toml)
# Add dependency between copy_toml_files and toml_test

if (FALSE)
    file(GLOB TOML_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.toml")
    foreach(TOML_FILE ${TOML_FILES})
        get_filename_component(FILENAME ${TOML_FILE} NAME)
        set(dst_path ${EXECUTABLE_OUTPUT_PATH}/${FILENAME})
        add_custom_command(
                OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FILENAME}"
                COMMAND ${CMAKE_COMMAND} -E copy "${TOML_FILE}" "${dst_path}"
                DEPENDS "${TOML_FILE}"
                COMMENT "Copying ${FILENAME} to build directory ${dst_path}"
        )
        list(APPEND COPIED_FILES "${CMAKE_CURRENT_BINARY_DIR}/${FILENAME}")
    endforeach()

    # Make sure file copying is executed before building toml_test
    add_custom_target(copy_toml_files DEPENDS ${COPIED_FILES})
    add_dependencies(toml_test copy_toml_files)
endif ()


copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}" "*toml" "${EXECUTABLE_OUTPUT_PATH}" TOML_COPIED_FILES)
copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}" "*json" "${EXECUTABLE_OUTPUT_PATH}" JSON_COPIED_FILES)
list(APPEND COMBINED_LIST ${TOML_COPIED_FILES} ${JSON_COPIED_FILES})

message("TOML_COPIED_FILES ${TOML_COPIED_FILES}")
message("XML_COPIED_FILES ${XML_COPIED_FILES}")
message("COMBINED_LIST ${COMBINED_LIST}")

add_custom_target(toml_test_file_copy DEPENDS ${COMBINED_LIST})

# Make your target depend on the custom target for copied files
add_dependencies(toml_test toml_test_file_copy)
