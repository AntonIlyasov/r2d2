# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/anton202/r2d2_general_ws/tcp_client_rx_tx

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build

# Include any dependencies generated for this target.
include CMakeFiles/simple_tcp_client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/simple_tcp_client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simple_tcp_client.dir/flags.make

CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.o: CMakeFiles/simple_tcp_client.dir/flags.make
CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.o: ../simple_tcp_client.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anton202/r2d2_general_ws/tcp_client_rx_tx/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.o -c /home/anton202/r2d2_general_ws/tcp_client_rx_tx/simple_tcp_client.cpp

CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/anton202/r2d2_general_ws/tcp_client_rx_tx/simple_tcp_client.cpp > CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.i

CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/anton202/r2d2_general_ws/tcp_client_rx_tx/simple_tcp_client.cpp -o CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.s

CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.o: CMakeFiles/simple_tcp_client.dir/flags.make
CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.o: ../umba_crc_table.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/anton202/r2d2_general_ws/tcp_client_rx_tx/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.o   -c /home/anton202/r2d2_general_ws/tcp_client_rx_tx/umba_crc_table.c

CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/anton202/r2d2_general_ws/tcp_client_rx_tx/umba_crc_table.c > CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.i

CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/anton202/r2d2_general_ws/tcp_client_rx_tx/umba_crc_table.c -o CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.s

# Object files for target simple_tcp_client
simple_tcp_client_OBJECTS = \
"CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.o" \
"CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.o"

# External object files for target simple_tcp_client
simple_tcp_client_EXTERNAL_OBJECTS =

simple_tcp_client: CMakeFiles/simple_tcp_client.dir/simple_tcp_client.cpp.o
simple_tcp_client: CMakeFiles/simple_tcp_client.dir/umba_crc_table.c.o
simple_tcp_client: CMakeFiles/simple_tcp_client.dir/build.make
simple_tcp_client: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
simple_tcp_client: CMakeFiles/simple_tcp_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/anton202/r2d2_general_ws/tcp_client_rx_tx/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable simple_tcp_client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple_tcp_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simple_tcp_client.dir/build: simple_tcp_client

.PHONY : CMakeFiles/simple_tcp_client.dir/build

CMakeFiles/simple_tcp_client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simple_tcp_client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simple_tcp_client.dir/clean

CMakeFiles/simple_tcp_client.dir/depend:
	cd /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anton202/r2d2_general_ws/tcp_client_rx_tx /home/anton202/r2d2_general_ws/tcp_client_rx_tx /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build /home/anton202/r2d2_general_ws/tcp_client_rx_tx/build/CMakeFiles/simple_tcp_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simple_tcp_client.dir/depend

