# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/bear/Desktop/Github/VIN-100_reader

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bear/Desktop/Github/VIN-100_reader/build

# Include any dependencies generated for this target.
include CMakeFiles/vin_100.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/vin_100.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/vin_100.dir/flags.make

CMakeFiles/vin_100.dir/main.cpp.o: CMakeFiles/vin_100.dir/flags.make
CMakeFiles/vin_100.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bear/Desktop/Github/VIN-100_reader/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/vin_100.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vin_100.dir/main.cpp.o -c /home/bear/Desktop/Github/VIN-100_reader/main.cpp

CMakeFiles/vin_100.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vin_100.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bear/Desktop/Github/VIN-100_reader/main.cpp > CMakeFiles/vin_100.dir/main.cpp.i

CMakeFiles/vin_100.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vin_100.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bear/Desktop/Github/VIN-100_reader/main.cpp -o CMakeFiles/vin_100.dir/main.cpp.s

CMakeFiles/vin_100.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/vin_100.dir/main.cpp.o.requires

CMakeFiles/vin_100.dir/main.cpp.o.provides: CMakeFiles/vin_100.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/vin_100.dir/build.make CMakeFiles/vin_100.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/vin_100.dir/main.cpp.o.provides

CMakeFiles/vin_100.dir/main.cpp.o.provides.build: CMakeFiles/vin_100.dir/main.cpp.o


# Object files for target vin_100
vin_100_OBJECTS = \
"CMakeFiles/vin_100.dir/main.cpp.o"

# External object files for target vin_100
vin_100_EXTERNAL_OBJECTS =

vin_100: CMakeFiles/vin_100.dir/main.cpp.o
vin_100: CMakeFiles/vin_100.dir/build.make
vin_100: /usr/lib/libPocoXML.so.50
vin_100: vnproglib-1.2.0.0/cpp/liblibvncxx.a
vin_100: /usr/lib/libPocoFoundation.so.50
vin_100: /usr/lib/x86_64-linux-gnu/libpcre.so
vin_100: /usr/lib/x86_64-linux-gnu/libz.so
vin_100: /usr/lib/x86_64-linux-gnu/libexpat.so
vin_100: CMakeFiles/vin_100.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bear/Desktop/Github/VIN-100_reader/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable vin_100"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vin_100.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/vin_100.dir/build: vin_100

.PHONY : CMakeFiles/vin_100.dir/build

CMakeFiles/vin_100.dir/requires: CMakeFiles/vin_100.dir/main.cpp.o.requires

.PHONY : CMakeFiles/vin_100.dir/requires

CMakeFiles/vin_100.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/vin_100.dir/cmake_clean.cmake
.PHONY : CMakeFiles/vin_100.dir/clean

CMakeFiles/vin_100.dir/depend:
	cd /home/bear/Desktop/Github/VIN-100_reader/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bear/Desktop/Github/VIN-100_reader /home/bear/Desktop/Github/VIN-100_reader /home/bear/Desktop/Github/VIN-100_reader/build /home/bear/Desktop/Github/VIN-100_reader/build /home/bear/Desktop/Github/VIN-100_reader/build/CMakeFiles/vin_100.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/vin_100.dir/depend

