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
CMAKE_BINARY_DIR = /home/laptop/school/RMR/demoRMR-all/build

# Include any dependencies generated for this target.
include robot/CMakeFiles/robot.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include robot/CMakeFiles/robot.dir/compiler_depend.make

# Include the progress variables for this target.
include robot/CMakeFiles/robot.dir/progress.make

# Include the compile flags for this target's objects.
include robot/CMakeFiles/robot.dir/flags.make

robot/CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.o: robot/CMakeFiles/robot.dir/flags.make
robot/CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.o: robot/robot_autogen/mocs_compilation.cpp
robot/CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.o: robot/CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object robot/CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.o"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot/CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.o -MF CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.o.d -o CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.o -c /home/laptop/school/RMR/demoRMR-all/build/robot/robot_autogen/mocs_compilation.cpp

robot/CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.i"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/build/robot/robot_autogen/mocs_compilation.cpp > CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.i

robot/CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.s"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/build/robot/robot_autogen/mocs_compilation.cpp -o CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.s

robot/CMakeFiles/robot.dir/CKobuki.cpp.o: robot/CMakeFiles/robot.dir/flags.make
robot/CMakeFiles/robot.dir/CKobuki.cpp.o: /home/laptop/school/RMR/demoRMR-all/robot/CKobuki.cpp
robot/CMakeFiles/robot.dir/CKobuki.cpp.o: robot/CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object robot/CMakeFiles/robot.dir/CKobuki.cpp.o"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot/CMakeFiles/robot.dir/CKobuki.cpp.o -MF CMakeFiles/robot.dir/CKobuki.cpp.o.d -o CMakeFiles/robot.dir/CKobuki.cpp.o -c /home/laptop/school/RMR/demoRMR-all/robot/CKobuki.cpp

robot/CMakeFiles/robot.dir/CKobuki.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/CKobuki.cpp.i"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/robot/CKobuki.cpp > CMakeFiles/robot.dir/CKobuki.cpp.i

robot/CMakeFiles/robot.dir/CKobuki.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/CKobuki.cpp.s"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/robot/CKobuki.cpp -o CMakeFiles/robot.dir/CKobuki.cpp.s

robot/CMakeFiles/robot.dir/robot.cpp.o: robot/CMakeFiles/robot.dir/flags.make
robot/CMakeFiles/robot.dir/robot.cpp.o: /home/laptop/school/RMR/demoRMR-all/robot/robot.cpp
robot/CMakeFiles/robot.dir/robot.cpp.o: robot/CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object robot/CMakeFiles/robot.dir/robot.cpp.o"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot/CMakeFiles/robot.dir/robot.cpp.o -MF CMakeFiles/robot.dir/robot.cpp.o.d -o CMakeFiles/robot.dir/robot.cpp.o -c /home/laptop/school/RMR/demoRMR-all/robot/robot.cpp

robot/CMakeFiles/robot.dir/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/robot.cpp.i"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/robot/robot.cpp > CMakeFiles/robot.dir/robot.cpp.i

robot/CMakeFiles/robot.dir/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/robot.cpp.s"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/robot/robot.cpp -o CMakeFiles/robot.dir/robot.cpp.s

robot/CMakeFiles/robot.dir/rplidar.cpp.o: robot/CMakeFiles/robot.dir/flags.make
robot/CMakeFiles/robot.dir/rplidar.cpp.o: /home/laptop/school/RMR/demoRMR-all/robot/rplidar.cpp
robot/CMakeFiles/robot.dir/rplidar.cpp.o: robot/CMakeFiles/robot.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/laptop/school/RMR/demoRMR-all/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object robot/CMakeFiles/robot.dir/rplidar.cpp.o"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT robot/CMakeFiles/robot.dir/rplidar.cpp.o -MF CMakeFiles/robot.dir/rplidar.cpp.o.d -o CMakeFiles/robot.dir/rplidar.cpp.o -c /home/laptop/school/RMR/demoRMR-all/robot/rplidar.cpp

robot/CMakeFiles/robot.dir/rplidar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robot.dir/rplidar.cpp.i"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/laptop/school/RMR/demoRMR-all/robot/rplidar.cpp > CMakeFiles/robot.dir/rplidar.cpp.i

robot/CMakeFiles/robot.dir/rplidar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robot.dir/rplidar.cpp.s"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/laptop/school/RMR/demoRMR-all/robot/rplidar.cpp -o CMakeFiles/robot.dir/rplidar.cpp.s

# Object files for target robot
robot_OBJECTS = \
"CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/robot.dir/CKobuki.cpp.o" \
"CMakeFiles/robot.dir/robot.cpp.o" \
"CMakeFiles/robot.dir/rplidar.cpp.o"

# External object files for target robot
robot_EXTERNAL_OBJECTS =

robot/librobot.a: robot/CMakeFiles/robot.dir/robot_autogen/mocs_compilation.cpp.o
robot/librobot.a: robot/CMakeFiles/robot.dir/CKobuki.cpp.o
robot/librobot.a: robot/CMakeFiles/robot.dir/robot.cpp.o
robot/librobot.a: robot/CMakeFiles/robot.dir/rplidar.cpp.o
robot/librobot.a: robot/CMakeFiles/robot.dir/build.make
robot/librobot.a: robot/CMakeFiles/robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/laptop/school/RMR/demoRMR-all/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library librobot.a"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && $(CMAKE_COMMAND) -P CMakeFiles/robot.dir/cmake_clean_target.cmake
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
robot/CMakeFiles/robot.dir/build: robot/librobot.a
.PHONY : robot/CMakeFiles/robot.dir/build

robot/CMakeFiles/robot.dir/clean:
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && $(CMAKE_COMMAND) -P CMakeFiles/robot.dir/cmake_clean.cmake
.PHONY : robot/CMakeFiles/robot.dir/clean

robot/CMakeFiles/robot.dir/depend:
	cd /home/laptop/school/RMR/demoRMR-all/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/laptop/school/RMR/demoRMR-all /home/laptop/school/RMR/demoRMR-all/robot /home/laptop/school/RMR/demoRMR-all/build /home/laptop/school/RMR/demoRMR-all/build/robot /home/laptop/school/RMR/demoRMR-all/build/robot/CMakeFiles/robot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : robot/CMakeFiles/robot.dir/depend
