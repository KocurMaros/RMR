# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/laptop/school/RMR/demoRMR-all

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/laptop/school/RMR/demoRMR-all/build_sim

# Include any dependencies generated for this target.
include CMakeFiles/rmr.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/rmr.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/rmr.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rmr.dir/flags.make

CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.o: CMakeFiles/rmr.dir/flags.make
CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.o: rmr_autogen/mocs_compilation.cpp
CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.o: CMakeFiles/rmr.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build_sim/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.o -MF CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.o.d -o CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.o -c /home/laptop/school/RMR/demoRMR-all/build_sim/rmr_autogen/mocs_compilation.cpp

CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/build_sim/rmr_autogen/mocs_compilation.cpp > CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.i

CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/build_sim/rmr_autogen/mocs_compilation.cpp -o CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.s

CMakeFiles/rmr.dir/demoRMR/main.cpp.o: CMakeFiles/rmr.dir/flags.make
CMakeFiles/rmr.dir/demoRMR/main.cpp.o: /home/laptop/school/RMR/demoRMR-all/demoRMR/main.cpp
CMakeFiles/rmr.dir/demoRMR/main.cpp.o: CMakeFiles/rmr.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build_sim/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/rmr.dir/demoRMR/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rmr.dir/demoRMR/main.cpp.o -MF CMakeFiles/rmr.dir/demoRMR/main.cpp.o.d -o CMakeFiles/rmr.dir/demoRMR/main.cpp.o -c /home/laptop/school/RMR/demoRMR-all/demoRMR/main.cpp

CMakeFiles/rmr.dir/demoRMR/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rmr.dir/demoRMR/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/demoRMR/main.cpp > CMakeFiles/rmr.dir/demoRMR/main.cpp.i

CMakeFiles/rmr.dir/demoRMR/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rmr.dir/demoRMR/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/demoRMR/main.cpp -o CMakeFiles/rmr.dir/demoRMR/main.cpp.s

CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.o: CMakeFiles/rmr.dir/flags.make
CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.o: /home/laptop/school/RMR/demoRMR-all/demoRMR/mainwindow.cpp
CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.o: CMakeFiles/rmr.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build_sim/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.o -MF CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.o.d -o CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.o -c /home/laptop/school/RMR/demoRMR-all/demoRMR/mainwindow.cpp

CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/demoRMR/mainwindow.cpp > CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.i

CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/demoRMR/mainwindow.cpp -o CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.s

CMakeFiles/rmr.dir/demoRMR/ramp.cpp.o: CMakeFiles/rmr.dir/flags.make
CMakeFiles/rmr.dir/demoRMR/ramp.cpp.o: /home/laptop/school/RMR/demoRMR-all/demoRMR/ramp.cpp
CMakeFiles/rmr.dir/demoRMR/ramp.cpp.o: CMakeFiles/rmr.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build_sim/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/rmr.dir/demoRMR/ramp.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rmr.dir/demoRMR/ramp.cpp.o -MF CMakeFiles/rmr.dir/demoRMR/ramp.cpp.o.d -o CMakeFiles/rmr.dir/demoRMR/ramp.cpp.o -c /home/laptop/school/RMR/demoRMR-all/demoRMR/ramp.cpp

CMakeFiles/rmr.dir/demoRMR/ramp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rmr.dir/demoRMR/ramp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/demoRMR/ramp.cpp > CMakeFiles/rmr.dir/demoRMR/ramp.cpp.i

CMakeFiles/rmr.dir/demoRMR/ramp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rmr.dir/demoRMR/ramp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/demoRMR/ramp.cpp -o CMakeFiles/rmr.dir/demoRMR/ramp.cpp.s

CMakeFiles/rmr.dir/demoRMR/controller.cpp.o: CMakeFiles/rmr.dir/flags.make
CMakeFiles/rmr.dir/demoRMR/controller.cpp.o: /home/laptop/school/RMR/demoRMR-all/demoRMR/controller.cpp
CMakeFiles/rmr.dir/demoRMR/controller.cpp.o: CMakeFiles/rmr.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build_sim/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/rmr.dir/demoRMR/controller.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rmr.dir/demoRMR/controller.cpp.o -MF CMakeFiles/rmr.dir/demoRMR/controller.cpp.o.d -o CMakeFiles/rmr.dir/demoRMR/controller.cpp.o -c /home/laptop/school/RMR/demoRMR-all/demoRMR/controller.cpp

CMakeFiles/rmr.dir/demoRMR/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rmr.dir/demoRMR/controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/demoRMR/controller.cpp > CMakeFiles/rmr.dir/demoRMR/controller.cpp.i

CMakeFiles/rmr.dir/demoRMR/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rmr.dir/demoRMR/controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/demoRMR/controller.cpp -o CMakeFiles/rmr.dir/demoRMR/controller.cpp.s

CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.o: CMakeFiles/rmr.dir/flags.make
CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.o: /home/laptop/school/RMR/demoRMR-all/demoRMR/hash_map.cpp
CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.o: CMakeFiles/rmr.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build_sim/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.o -MF CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.o.d -o CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.o -c /home/laptop/school/RMR/demoRMR-all/demoRMR/hash_map.cpp

CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/demoRMR/hash_map.cpp > CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.i

CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/demoRMR/hash_map.cpp -o CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.s

CMakeFiles/rmr.dir/demoRMR/mapping.cpp.o: CMakeFiles/rmr.dir/flags.make
CMakeFiles/rmr.dir/demoRMR/mapping.cpp.o: /home/laptop/school/RMR/demoRMR-all/demoRMR/mapping.cpp
CMakeFiles/rmr.dir/demoRMR/mapping.cpp.o: CMakeFiles/rmr.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build_sim/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/rmr.dir/demoRMR/mapping.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rmr.dir/demoRMR/mapping.cpp.o -MF CMakeFiles/rmr.dir/demoRMR/mapping.cpp.o.d -o CMakeFiles/rmr.dir/demoRMR/mapping.cpp.o -c /home/laptop/school/RMR/demoRMR-all/demoRMR/mapping.cpp

CMakeFiles/rmr.dir/demoRMR/mapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rmr.dir/demoRMR/mapping.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/demoRMR/mapping.cpp > CMakeFiles/rmr.dir/demoRMR/mapping.cpp.i

CMakeFiles/rmr.dir/demoRMR/mapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rmr.dir/demoRMR/mapping.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/demoRMR/mapping.cpp -o CMakeFiles/rmr.dir/demoRMR/mapping.cpp.s

# Object files for target rmr
rmr_OBJECTS = \
"CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/rmr.dir/demoRMR/main.cpp.o" \
"CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.o" \
"CMakeFiles/rmr.dir/demoRMR/ramp.cpp.o" \
"CMakeFiles/rmr.dir/demoRMR/controller.cpp.o" \
"CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.o" \
"CMakeFiles/rmr.dir/demoRMR/mapping.cpp.o"

# External object files for target rmr
rmr_EXTERNAL_OBJECTS =

rmr: CMakeFiles/rmr.dir/rmr_autogen/mocs_compilation.cpp.o
rmr: CMakeFiles/rmr.dir/demoRMR/main.cpp.o
rmr: CMakeFiles/rmr.dir/demoRMR/mainwindow.cpp.o
rmr: CMakeFiles/rmr.dir/demoRMR/ramp.cpp.o
rmr: CMakeFiles/rmr.dir/demoRMR/controller.cpp.o
rmr: CMakeFiles/rmr.dir/demoRMR/hash_map.cpp.o
rmr: CMakeFiles/rmr.dir/demoRMR/mapping.cpp.o
rmr: CMakeFiles/rmr.dir/build.make
rmr: robot/librobot.a
rmr: QJoysticks-master/libqjoysticks.a
rmr: /usr/local/lib/libopencv_gapi.so.4.9.0
rmr: /usr/local/lib/libopencv_highgui.so.4.9.0
rmr: /usr/local/lib/libopencv_ml.so.4.9.0
rmr: /usr/local/lib/libopencv_objdetect.so.4.9.0
rmr: /usr/local/lib/libopencv_photo.so.4.9.0
rmr: /usr/local/lib/libopencv_stitching.so.4.9.0
rmr: /usr/local/lib/libopencv_video.so.4.9.0
rmr: /usr/local/lib/libopencv_calib3d.so.4.9.0
rmr: /usr/local/lib/libopencv_dnn.so.4.9.0
rmr: /usr/local/lib/libopencv_features2d.so.4.9.0
rmr: /usr/local/lib/libopencv_flann.so.4.9.0
rmr: /usr/local/lib/libopencv_videoio.so.4.9.0
rmr: /usr/local/lib/libopencv_imgcodecs.so.4.9.0
rmr: /usr/local/lib/libopencv_imgproc.so.4.9.0
rmr: /usr/local/lib/libopencv_core.so.4.9.0
rmr: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
rmr: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
rmr: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
rmr: /usr/lib/x86_64-linux-gnu/libSDL2.so
rmr: CMakeFiles/rmr.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/laptop/school/RMR/demoRMR-all/build_sim/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable rmr"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rmr.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rmr.dir/build: rmr
.PHONY : CMakeFiles/rmr.dir/build

CMakeFiles/rmr.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rmr.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rmr.dir/clean

CMakeFiles/rmr.dir/depend:
	cd /home/laptop/school/RMR/demoRMR-all/build_sim && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/laptop/school/RMR/demoRMR-all /home/laptop/school/RMR/demoRMR-all /home/laptop/school/RMR/demoRMR-all/build_sim /home/laptop/school/RMR/demoRMR-all/build_sim /home/laptop/school/RMR/demoRMR-all/build_sim/CMakeFiles/rmr.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rmr.dir/depend

