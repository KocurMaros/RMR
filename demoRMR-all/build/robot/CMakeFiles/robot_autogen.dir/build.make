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

# Utility rule file for robot_autogen.

# Include any custom commands dependencies for this target.
include robot/CMakeFiles/robot_autogen.dir/compiler_depend.make

# Include the progress variables for this target.
include robot/CMakeFiles/robot_autogen.dir/progress.make

robot/CMakeFiles/robot_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/laptop/school/RMR/demoRMR-all/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target robot"
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && /usr/bin/cmake -E cmake_autogen /home/laptop/school/RMR/demoRMR-all/build/robot/CMakeFiles/robot_autogen.dir/AutogenInfo.json Debug

robot_autogen: robot/CMakeFiles/robot_autogen
robot_autogen: robot/CMakeFiles/robot_autogen.dir/build.make
.PHONY : robot_autogen

# Rule to build all files generated by this target.
robot/CMakeFiles/robot_autogen.dir/build: robot_autogen
.PHONY : robot/CMakeFiles/robot_autogen.dir/build

robot/CMakeFiles/robot_autogen.dir/clean:
	cd /home/laptop/school/RMR/demoRMR-all/build/robot && $(CMAKE_COMMAND) -P CMakeFiles/robot_autogen.dir/cmake_clean.cmake
.PHONY : robot/CMakeFiles/robot_autogen.dir/clean

robot/CMakeFiles/robot_autogen.dir/depend:
	cd /home/laptop/school/RMR/demoRMR-all/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/laptop/school/RMR/demoRMR-all /home/laptop/school/RMR/demoRMR-all/robot /home/laptop/school/RMR/demoRMR-all/build /home/laptop/school/RMR/demoRMR-all/build/robot /home/laptop/school/RMR/demoRMR-all/build/robot/CMakeFiles/robot_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : robot/CMakeFiles/robot_autogen.dir/depend

