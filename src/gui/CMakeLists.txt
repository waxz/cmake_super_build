# Find the QtWidgets library
if (BUILD_QT_DEMO)
find_package(Qt5 5.3.1 COMPONENTS Svg Gui PrintSupport)
add_executable(qt_example qt_example.cpp qt_PlotWindow.cpp)
target_link_libraries(qt_example  PLPLOT::plplot PLPLOT::plplotqt  PLPLOT::plplotcxx  Qt5::Svg Qt5::Gui Qt5::PrintSupport)
set_target_properties(qt_example PROPERTIES
        AUTOMOC ON
        AUTOMOC_MOC_OPTIONS "${PLPLOT_AUTOMOC_MOC_OPTIONS}"
        )
target_include_directories( qt_example PUBLIC ${plplot_ROOT}/include/plplot)

endif ()


add_executable(plplot_test plplot_test.cpp ../control/MobileRobotController.cpp  ../control/PathGenerator.cpp )
target_link_libraries(plplot_test PUBLIC PLPLOT::plplot PLPLOT::plplotqt  PLPLOT::plplotcxx PLPLOT::plplot )
target_include_directories( plplot_test PUBLIC ${plplot_ROOT}/include/plplot
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${EIGEN3_INCLUDE_DIR}
        )

set_target_rpath(plplot_test  "$ORIGIN/../force_plplot/install/lib")
