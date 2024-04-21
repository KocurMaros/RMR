cmake_minimum_required(VERSION 3.22)
project(DemoRMR LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

set(SDL_INCLUDE_DIRS /usr/include/SDL2)
set(SDL_LIBRARIES /usr/lib/x86_64-linux-gnu/libSDL2.so)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)

add_subdirectory(robot)
add_subdirectory(QJoysticks-master)
# add_subdirectory(demoRMR)

message("SDL_LIBRARIES: ${SDL_LIBRARIES}")
message("SDL_INCLUDE_DIRS: ${SDL_INCLUDE_DIRS}")
message("OpenCV_LIBS: ${OpenCV_LIBS}")
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

set(SOURCES
	demoRMR/main.cpp
	demoRMR/mainwindow.cpp
    demoRMR/ramp.cpp
    demoRMR/controller.cpp
    demoRMR/hash_map.cpp
    demoRMR/mapping.cpp
)

set(HEADER
    demoRMR/mainwindow.h
    demoRMR/ramp.h
    demoRMR/point.h
    demoRMR/controller.h
    demoRMR/hash_map.h
    demoRMR/mapping.h
)
set(UI
	demoRMR/mainwindow.ui
)

add_executable(rmr
	${HEADERS}
	${SOURCES}
	${UI}
)
target_link_libraries(
	rmr
PUBLIC
	Qt::Core
	Qt::Widgets
	${OpenCV_LIBS}
	robot
	qjoysticks
    # demoRMR
)

