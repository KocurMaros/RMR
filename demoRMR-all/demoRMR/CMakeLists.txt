set(SOURCES
	main.cpp
	mainwindow.cpp
    ramp.cpp
    controller.cpp
)

set(HEADER
    mainwindow.h
    ramp.h
    point.h
    controller.h

)
set(UI
	mainwindow.ui
)
add_library(demoRMR ${SOURCES} ${HEADER} ${UI})
# Add headers
target_include_directories(demoRMR PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
	demoRMR
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# Specify output directory
set_target_properties(demoRMR PROPERTIES
	LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../bin
)
