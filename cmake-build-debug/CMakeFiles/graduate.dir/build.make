# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = "/Users/mashanpeng/Library/Application Support/JetBrains/Toolbox/apps/CLion/ch-0/203.7717.62/CLion.app/Contents/bin/cmake/mac/bin/cmake"

# The command to remove a file.
RM = "/Users/mashanpeng/Library/Application Support/JetBrains/Toolbox/apps/CLion/ch-0/203.7717.62/CLion.app/Contents/bin/cmake/mac/bin/cmake" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/mashanpeng/CLionProjects/graduate

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/mashanpeng/CLionProjects/graduate/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/graduate.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/graduate.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/graduate.dir/flags.make

CMakeFiles/graduate.dir/main.cpp.o: CMakeFiles/graduate.dir/flags.make
CMakeFiles/graduate.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/mashanpeng/CLionProjects/graduate/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/graduate.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/graduate.dir/main.cpp.o -c /Users/mashanpeng/CLionProjects/graduate/main.cpp

CMakeFiles/graduate.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/graduate.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/mashanpeng/CLionProjects/graduate/main.cpp > CMakeFiles/graduate.dir/main.cpp.i

CMakeFiles/graduate.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/graduate.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/mashanpeng/CLionProjects/graduate/main.cpp -o CMakeFiles/graduate.dir/main.cpp.s

# Object files for target graduate
graduate_OBJECTS = \
"CMakeFiles/graduate.dir/main.cpp.o"

# External object files for target graduate
graduate_EXTERNAL_OBJECTS =

graduate: CMakeFiles/graduate.dir/main.cpp.o
graduate: CMakeFiles/graduate.dir/build.make
graduate: CMakeFiles/graduate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/mashanpeng/CLionProjects/graduate/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable graduate"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/graduate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/graduate.dir/build: graduate

.PHONY : CMakeFiles/graduate.dir/build

CMakeFiles/graduate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/graduate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/graduate.dir/clean

CMakeFiles/graduate.dir/depend:
	cd /Users/mashanpeng/CLionProjects/graduate/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/mashanpeng/CLionProjects/graduate /Users/mashanpeng/CLionProjects/graduate /Users/mashanpeng/CLionProjects/graduate/cmake-build-debug /Users/mashanpeng/CLionProjects/graduate/cmake-build-debug /Users/mashanpeng/CLionProjects/graduate/cmake-build-debug/CMakeFiles/graduate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/graduate.dir/depend

