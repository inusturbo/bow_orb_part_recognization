# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Desktop/new

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Desktop/new/build

# Include any dependencies generated for this target.
include CMakeFiles/tst.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tst.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tst.dir/flags.make

CMakeFiles/tst.dir/all.cpp.o: CMakeFiles/tst.dir/flags.make
CMakeFiles/tst.dir/all.cpp.o: ../all.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tst.dir/all.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tst.dir/all.cpp.o -c /home/pi/Desktop/new/all.cpp

CMakeFiles/tst.dir/all.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tst.dir/all.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/new/all.cpp > CMakeFiles/tst.dir/all.cpp.i

CMakeFiles/tst.dir/all.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tst.dir/all.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/new/all.cpp -o CMakeFiles/tst.dir/all.cpp.s

# Object files for target tst
tst_OBJECTS = \
"CMakeFiles/tst.dir/all.cpp.o"

# External object files for target tst
tst_EXTERNAL_OBJECTS =

tst: CMakeFiles/tst.dir/all.cpp.o
tst: CMakeFiles/tst.dir/build.make
tst: /usr/lib/libwiringPi.so
tst: CMakeFiles/tst.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Desktop/new/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable tst"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tst.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tst.dir/build: tst

.PHONY : CMakeFiles/tst.dir/build

CMakeFiles/tst.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tst.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tst.dir/clean

CMakeFiles/tst.dir/depend:
	cd /home/pi/Desktop/new/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Desktop/new /home/pi/Desktop/new /home/pi/Desktop/new/build /home/pi/Desktop/new/build /home/pi/Desktop/new/build/CMakeFiles/tst.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tst.dir/depend

