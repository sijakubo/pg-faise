# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pg/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pg/catkin_ws/build

# Include any dependencies generated for this target.
include epos2_control/CMakeFiles/TankSteering.dir/depend.make

# Include the progress variables for this target.
include epos2_control/CMakeFiles/TankSteering.dir/progress.make

# Include the compile flags for this target's objects.
include epos2_control/CMakeFiles/TankSteering.dir/flags.make

epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o: epos2_control/CMakeFiles/TankSteering.dir/flags.make
epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o: /home/pg/catkin_ws/src/epos2_control/src/TankSteering.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pg/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o"
	cd /home/pg/catkin_ws/build/epos2_control && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o -c /home/pg/catkin_ws/src/epos2_control/src/TankSteering.cpp

epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TankSteering.dir/src/TankSteering.cpp.i"
	cd /home/pg/catkin_ws/build/epos2_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pg/catkin_ws/src/epos2_control/src/TankSteering.cpp > CMakeFiles/TankSteering.dir/src/TankSteering.cpp.i

epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TankSteering.dir/src/TankSteering.cpp.s"
	cd /home/pg/catkin_ws/build/epos2_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pg/catkin_ws/src/epos2_control/src/TankSteering.cpp -o CMakeFiles/TankSteering.dir/src/TankSteering.cpp.s

epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o.requires:
.PHONY : epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o.requires

epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o.provides: epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o.requires
	$(MAKE) -f epos2_control/CMakeFiles/TankSteering.dir/build.make epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o.provides.build
.PHONY : epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o.provides

epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o.provides.build: epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o

# Object files for target TankSteering
TankSteering_OBJECTS = \
"CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o"

# External object files for target TankSteering
TankSteering_EXTERNAL_OBJECTS =

/home/pg/catkin_ws/devel/lib/libTankSteering.so: epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o
/home/pg/catkin_ws/devel/lib/libTankSteering.so: epos2_control/CMakeFiles/TankSteering.dir/build.make
/home/pg/catkin_ws/devel/lib/libTankSteering.so: epos2_control/CMakeFiles/TankSteering.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/pg/catkin_ws/devel/lib/libTankSteering.so"
	cd /home/pg/catkin_ws/build/epos2_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TankSteering.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
epos2_control/CMakeFiles/TankSteering.dir/build: /home/pg/catkin_ws/devel/lib/libTankSteering.so
.PHONY : epos2_control/CMakeFiles/TankSteering.dir/build

epos2_control/CMakeFiles/TankSteering.dir/requires: epos2_control/CMakeFiles/TankSteering.dir/src/TankSteering.cpp.o.requires
.PHONY : epos2_control/CMakeFiles/TankSteering.dir/requires

epos2_control/CMakeFiles/TankSteering.dir/clean:
	cd /home/pg/catkin_ws/build/epos2_control && $(CMAKE_COMMAND) -P CMakeFiles/TankSteering.dir/cmake_clean.cmake
.PHONY : epos2_control/CMakeFiles/TankSteering.dir/clean

epos2_control/CMakeFiles/TankSteering.dir/depend:
	cd /home/pg/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pg/catkin_ws/src /home/pg/catkin_ws/src/epos2_control /home/pg/catkin_ws/build /home/pg/catkin_ws/build/epos2_control /home/pg/catkin_ws/build/epos2_control/CMakeFiles/TankSteering.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : epos2_control/CMakeFiles/TankSteering.dir/depend

