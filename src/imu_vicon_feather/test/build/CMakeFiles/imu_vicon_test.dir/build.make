# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.22.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.22.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/build

# Include any dependencies generated for this target.
include CMakeFiles/imu_vicon_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/imu_vicon_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/imu_vicon_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imu_vicon_test.dir/flags.make

CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.o: CMakeFiles/imu_vicon_test.dir/flags.make
CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.o: ../imu_vicon_test.cpp
CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.o: CMakeFiles/imu_vicon_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.o"
	/usr/local/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.o -MF CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.o.d -o CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.o -c /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/imu_vicon_test.cpp

CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.i"
	/usr/local/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/imu_vicon_test.cpp > CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.i

CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.s"
	/usr/local/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/imu_vicon_test.cpp -o CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.s

# Object files for target imu_vicon_test
imu_vicon_test_OBJECTS = \
"CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.o"

# External object files for target imu_vicon_test
imu_vicon_test_EXTERNAL_OBJECTS =

imu_vicon_test: CMakeFiles/imu_vicon_test.dir/imu_vicon_test.cpp.o
imu_vicon_test: CMakeFiles/imu_vicon_test.dir/build.make
imu_vicon_test: CMakeFiles/imu_vicon_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable imu_vicon_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imu_vicon_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imu_vicon_test.dir/build: imu_vicon_test
.PHONY : CMakeFiles/imu_vicon_test.dir/build

CMakeFiles/imu_vicon_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imu_vicon_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imu_vicon_test.dir/clean

CMakeFiles/imu_vicon_test.dir/depend:
	cd /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/build /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/build /Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/test/build/CMakeFiles/imu_vicon_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imu_vicon_test.dir/depend

