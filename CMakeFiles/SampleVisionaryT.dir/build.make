# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/rodrigo/Volumetry/volumetry-visionaryt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rodrigo/Volumetry/volumetry-visionaryt

# Include any dependencies generated for this target.
include CMakeFiles/SampleVisionaryT.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SampleVisionaryT.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SampleVisionaryT.dir/flags.make

CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o: CMakeFiles/SampleVisionaryT.dir/flags.make
CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o: SampleVisionaryT/SampleVisionaryT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rodrigo/Volumetry/volumetry-visionaryt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o -c /home/rodrigo/Volumetry/volumetry-visionaryt/SampleVisionaryT/SampleVisionaryT.cpp

CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rodrigo/Volumetry/volumetry-visionaryt/SampleVisionaryT/SampleVisionaryT.cpp > CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.i

CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rodrigo/Volumetry/volumetry-visionaryt/SampleVisionaryT/SampleVisionaryT.cpp -o CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.s

CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o.requires:

.PHONY : CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o.requires

CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o.provides: CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o.requires
	$(MAKE) -f CMakeFiles/SampleVisionaryT.dir/build.make CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o.provides.build
.PHONY : CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o.provides

CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o.provides.build: CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o


# Object files for target SampleVisionaryT
SampleVisionaryT_OBJECTS = \
"CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o"

# External object files for target SampleVisionaryT
SampleVisionaryT_EXTERNAL_OBJECTS =

SampleVisionaryT: CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o
SampleVisionaryT: CMakeFiles/SampleVisionaryT.dir/build.make
SampleVisionaryT: VisionaryCommon/libsick_visionary_common.a
SampleVisionaryT: CMakeFiles/SampleVisionaryT.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rodrigo/Volumetry/volumetry-visionaryt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable SampleVisionaryT"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SampleVisionaryT.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SampleVisionaryT.dir/build: SampleVisionaryT

.PHONY : CMakeFiles/SampleVisionaryT.dir/build

CMakeFiles/SampleVisionaryT.dir/requires: CMakeFiles/SampleVisionaryT.dir/SampleVisionaryT/SampleVisionaryT.cpp.o.requires

.PHONY : CMakeFiles/SampleVisionaryT.dir/requires

CMakeFiles/SampleVisionaryT.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SampleVisionaryT.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SampleVisionaryT.dir/clean

CMakeFiles/SampleVisionaryT.dir/depend:
	cd /home/rodrigo/Volumetry/volumetry-visionaryt && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rodrigo/Volumetry/volumetry-visionaryt /home/rodrigo/Volumetry/volumetry-visionaryt /home/rodrigo/Volumetry/volumetry-visionaryt /home/rodrigo/Volumetry/volumetry-visionaryt /home/rodrigo/Volumetry/volumetry-visionaryt/CMakeFiles/SampleVisionaryT.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SampleVisionaryT.dir/depend

