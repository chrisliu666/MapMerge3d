# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lenovo/CLionProjects/CalculateRotAngle

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lenovo/CLionProjects/CalculateRotAngle/build

# Include any dependencies generated for this target.
include CMakeFiles/cal.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cal.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cal.dir/flags.make

CMakeFiles/cal.dir/cal.cpp.o: CMakeFiles/cal.dir/flags.make
CMakeFiles/cal.dir/cal.cpp.o: ../cal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lenovo/CLionProjects/CalculateRotAngle/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cal.dir/cal.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cal.dir/cal.cpp.o -c /home/lenovo/CLionProjects/CalculateRotAngle/cal.cpp

CMakeFiles/cal.dir/cal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cal.dir/cal.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lenovo/CLionProjects/CalculateRotAngle/cal.cpp > CMakeFiles/cal.dir/cal.cpp.i

CMakeFiles/cal.dir/cal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cal.dir/cal.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lenovo/CLionProjects/CalculateRotAngle/cal.cpp -o CMakeFiles/cal.dir/cal.cpp.s

CMakeFiles/cal.dir/cal.cpp.o.requires:

.PHONY : CMakeFiles/cal.dir/cal.cpp.o.requires

CMakeFiles/cal.dir/cal.cpp.o.provides: CMakeFiles/cal.dir/cal.cpp.o.requires
	$(MAKE) -f CMakeFiles/cal.dir/build.make CMakeFiles/cal.dir/cal.cpp.o.provides.build
.PHONY : CMakeFiles/cal.dir/cal.cpp.o.provides

CMakeFiles/cal.dir/cal.cpp.o.provides.build: CMakeFiles/cal.dir/cal.cpp.o


# Object files for target cal
cal_OBJECTS = \
"CMakeFiles/cal.dir/cal.cpp.o"

# External object files for target cal
cal_EXTERNAL_OBJECTS =

cal: CMakeFiles/cal.dir/cal.cpp.o
cal: CMakeFiles/cal.dir/build.make
cal: /usr/lib/x86_64-linux-gnu/libboost_system.so
cal: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
cal: /usr/lib/x86_64-linux-gnu/libboost_thread.so
cal: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cal: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
cal: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
cal: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
cal: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
cal: /usr/lib/x86_64-linux-gnu/libboost_regex.so
cal: /usr/local/lib/libpcl_common.so
cal: /usr/local/lib/libpcl_octree.so
cal: /usr/lib/libOpenNI.so
cal: /usr/lib/libOpenNI2.so
cal: /usr/local/lib/libpcl_io.so
cal: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
cal: /usr/local/lib/libpcl_kdtree.so
cal: /usr/local/lib/libpcl_search.so
cal: /usr/lib/x86_64-linux-gnu/libqhull.so
cal: /usr/local/lib/libpcl_surface.so
cal: /usr/local/lib/libpcl_sample_consensus.so
cal: /usr/local/lib/libpcl_filters.so
cal: /usr/local/lib/libpcl_features.so
cal: /usr/local/lib/libpcl_visualization.so
cal: /usr/local/lib/libpcl_tracking.so
cal: /usr/local/lib/libpcl_ml.so
cal: /usr/local/lib/libpcl_segmentation.so
cal: /usr/local/lib/libpcl_people.so
cal: /usr/local/lib/libpcl_registration.so
cal: /usr/local/lib/libpcl_stereo.so
cal: /usr/local/lib/libpcl_keypoints.so
cal: /usr/local/lib/libpcl_recognition.so
cal: /usr/local/lib/libpcl_outofcore.so
cal: /usr/lib/x86_64-linux-gnu/libboost_system.so
cal: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
cal: /usr/lib/x86_64-linux-gnu/libboost_thread.so
cal: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cal: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
cal: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
cal: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
cal: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
cal: /usr/lib/x86_64-linux-gnu/libboost_regex.so
cal: /usr/lib/x86_64-linux-gnu/libqhull.so
cal: /usr/lib/libOpenNI.so
cal: /usr/lib/libOpenNI2.so
cal: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
cal: /usr/lib/libvtkGenericFiltering.so.5.8.0
cal: /usr/lib/libvtkGeovis.so.5.8.0
cal: /usr/lib/libvtkCharts.so.5.8.0
cal: /usr/local/lib/libpcl_common.so
cal: /usr/local/lib/libpcl_octree.so
cal: /usr/local/lib/libpcl_io.so
cal: /usr/local/lib/libpcl_kdtree.so
cal: /usr/local/lib/libpcl_search.so
cal: /usr/local/lib/libpcl_surface.so
cal: /usr/local/lib/libpcl_sample_consensus.so
cal: /usr/local/lib/libpcl_filters.so
cal: /usr/local/lib/libpcl_features.so
cal: /usr/local/lib/libpcl_visualization.so
cal: /usr/local/lib/libpcl_tracking.so
cal: /usr/local/lib/libpcl_ml.so
cal: /usr/local/lib/libpcl_segmentation.so
cal: /usr/local/lib/libpcl_people.so
cal: /usr/local/lib/libpcl_registration.so
cal: /usr/local/lib/libpcl_stereo.so
cal: /usr/local/lib/libpcl_keypoints.so
cal: /usr/local/lib/libpcl_recognition.so
cal: /usr/local/lib/libpcl_outofcore.so
cal: /usr/lib/libvtkViews.so.5.8.0
cal: /usr/lib/libvtkInfovis.so.5.8.0
cal: /usr/lib/libvtkWidgets.so.5.8.0
cal: /usr/lib/libvtkVolumeRendering.so.5.8.0
cal: /usr/lib/libvtkHybrid.so.5.8.0
cal: /usr/lib/libvtkParallel.so.5.8.0
cal: /usr/lib/libvtkRendering.so.5.8.0
cal: /usr/lib/libvtkImaging.so.5.8.0
cal: /usr/lib/libvtkGraphics.so.5.8.0
cal: /usr/lib/libvtkIO.so.5.8.0
cal: /usr/lib/libvtkFiltering.so.5.8.0
cal: /usr/lib/libvtkCommon.so.5.8.0
cal: /usr/lib/libvtksys.so.5.8.0
cal: CMakeFiles/cal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lenovo/CLionProjects/CalculateRotAngle/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cal"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cal.dir/build: cal

.PHONY : CMakeFiles/cal.dir/build

CMakeFiles/cal.dir/requires: CMakeFiles/cal.dir/cal.cpp.o.requires

.PHONY : CMakeFiles/cal.dir/requires

CMakeFiles/cal.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cal.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cal.dir/clean

CMakeFiles/cal.dir/depend:
	cd /home/lenovo/CLionProjects/CalculateRotAngle/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lenovo/CLionProjects/CalculateRotAngle /home/lenovo/CLionProjects/CalculateRotAngle /home/lenovo/CLionProjects/CalculateRotAngle/build /home/lenovo/CLionProjects/CalculateRotAngle/build /home/lenovo/CLionProjects/CalculateRotAngle/build/CMakeFiles/cal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cal.dir/depend

