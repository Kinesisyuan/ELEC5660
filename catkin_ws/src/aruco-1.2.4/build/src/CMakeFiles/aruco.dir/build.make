# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/kinesis/catkin_ws/src/aruco-1.2.4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kinesis/catkin_ws/src/aruco-1.2.4/build

# Include any dependencies generated for this target.
include src/CMakeFiles/aruco.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/aruco.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/aruco.dir/flags.make

src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o: ../src/arucofidmarkers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kinesis/catkin_ws/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/arucofidmarkers.cpp.o -c /home/kinesis/catkin_ws/src/aruco-1.2.4/src/arucofidmarkers.cpp

src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/arucofidmarkers.cpp.i"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kinesis/catkin_ws/src/aruco-1.2.4/src/arucofidmarkers.cpp > CMakeFiles/aruco.dir/arucofidmarkers.cpp.i

src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/arucofidmarkers.cpp.s"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kinesis/catkin_ws/src/aruco-1.2.4/src/arucofidmarkers.cpp -o CMakeFiles/aruco.dir/arucofidmarkers.cpp.s

src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o.requires:

.PHONY : src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o.requires

src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o.provides: src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o.provides

src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o.provides.build: src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o


src/CMakeFiles/aruco.dir/board.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/board.cpp.o: ../src/board.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kinesis/catkin_ws/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/aruco.dir/board.cpp.o"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/board.cpp.o -c /home/kinesis/catkin_ws/src/aruco-1.2.4/src/board.cpp

src/CMakeFiles/aruco.dir/board.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/board.cpp.i"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kinesis/catkin_ws/src/aruco-1.2.4/src/board.cpp > CMakeFiles/aruco.dir/board.cpp.i

src/CMakeFiles/aruco.dir/board.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/board.cpp.s"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kinesis/catkin_ws/src/aruco-1.2.4/src/board.cpp -o CMakeFiles/aruco.dir/board.cpp.s

src/CMakeFiles/aruco.dir/board.cpp.o.requires:

.PHONY : src/CMakeFiles/aruco.dir/board.cpp.o.requires

src/CMakeFiles/aruco.dir/board.cpp.o.provides: src/CMakeFiles/aruco.dir/board.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/board.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/board.cpp.o.provides

src/CMakeFiles/aruco.dir/board.cpp.o.provides.build: src/CMakeFiles/aruco.dir/board.cpp.o


src/CMakeFiles/aruco.dir/boarddetector.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/boarddetector.cpp.o: ../src/boarddetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kinesis/catkin_ws/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/aruco.dir/boarddetector.cpp.o"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/boarddetector.cpp.o -c /home/kinesis/catkin_ws/src/aruco-1.2.4/src/boarddetector.cpp

src/CMakeFiles/aruco.dir/boarddetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/boarddetector.cpp.i"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kinesis/catkin_ws/src/aruco-1.2.4/src/boarddetector.cpp > CMakeFiles/aruco.dir/boarddetector.cpp.i

src/CMakeFiles/aruco.dir/boarddetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/boarddetector.cpp.s"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kinesis/catkin_ws/src/aruco-1.2.4/src/boarddetector.cpp -o CMakeFiles/aruco.dir/boarddetector.cpp.s

src/CMakeFiles/aruco.dir/boarddetector.cpp.o.requires:

.PHONY : src/CMakeFiles/aruco.dir/boarddetector.cpp.o.requires

src/CMakeFiles/aruco.dir/boarddetector.cpp.o.provides: src/CMakeFiles/aruco.dir/boarddetector.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/boarddetector.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/boarddetector.cpp.o.provides

src/CMakeFiles/aruco.dir/boarddetector.cpp.o.provides.build: src/CMakeFiles/aruco.dir/boarddetector.cpp.o


src/CMakeFiles/aruco.dir/cameraparameters.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/cameraparameters.cpp.o: ../src/cameraparameters.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kinesis/catkin_ws/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/aruco.dir/cameraparameters.cpp.o"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/cameraparameters.cpp.o -c /home/kinesis/catkin_ws/src/aruco-1.2.4/src/cameraparameters.cpp

src/CMakeFiles/aruco.dir/cameraparameters.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/cameraparameters.cpp.i"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kinesis/catkin_ws/src/aruco-1.2.4/src/cameraparameters.cpp > CMakeFiles/aruco.dir/cameraparameters.cpp.i

src/CMakeFiles/aruco.dir/cameraparameters.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/cameraparameters.cpp.s"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kinesis/catkin_ws/src/aruco-1.2.4/src/cameraparameters.cpp -o CMakeFiles/aruco.dir/cameraparameters.cpp.s

src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.requires:

.PHONY : src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.requires

src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.provides: src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.provides

src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.provides.build: src/CMakeFiles/aruco.dir/cameraparameters.cpp.o


src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o: ../src/cvdrawingutils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kinesis/catkin_ws/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/cvdrawingutils.cpp.o -c /home/kinesis/catkin_ws/src/aruco-1.2.4/src/cvdrawingutils.cpp

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/cvdrawingutils.cpp.i"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kinesis/catkin_ws/src/aruco-1.2.4/src/cvdrawingutils.cpp > CMakeFiles/aruco.dir/cvdrawingutils.cpp.i

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/cvdrawingutils.cpp.s"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kinesis/catkin_ws/src/aruco-1.2.4/src/cvdrawingutils.cpp -o CMakeFiles/aruco.dir/cvdrawingutils.cpp.s

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.requires:

.PHONY : src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.requires

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.provides: src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.provides

src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.provides.build: src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o


src/CMakeFiles/aruco.dir/marker.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/marker.cpp.o: ../src/marker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kinesis/catkin_ws/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/aruco.dir/marker.cpp.o"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/marker.cpp.o -c /home/kinesis/catkin_ws/src/aruco-1.2.4/src/marker.cpp

src/CMakeFiles/aruco.dir/marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/marker.cpp.i"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kinesis/catkin_ws/src/aruco-1.2.4/src/marker.cpp > CMakeFiles/aruco.dir/marker.cpp.i

src/CMakeFiles/aruco.dir/marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/marker.cpp.s"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kinesis/catkin_ws/src/aruco-1.2.4/src/marker.cpp -o CMakeFiles/aruco.dir/marker.cpp.s

src/CMakeFiles/aruco.dir/marker.cpp.o.requires:

.PHONY : src/CMakeFiles/aruco.dir/marker.cpp.o.requires

src/CMakeFiles/aruco.dir/marker.cpp.o.provides: src/CMakeFiles/aruco.dir/marker.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/marker.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/marker.cpp.o.provides

src/CMakeFiles/aruco.dir/marker.cpp.o.provides.build: src/CMakeFiles/aruco.dir/marker.cpp.o


src/CMakeFiles/aruco.dir/markerdetector.cpp.o: src/CMakeFiles/aruco.dir/flags.make
src/CMakeFiles/aruco.dir/markerdetector.cpp.o: ../src/markerdetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kinesis/catkin_ws/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/aruco.dir/markerdetector.cpp.o"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco.dir/markerdetector.cpp.o -c /home/kinesis/catkin_ws/src/aruco-1.2.4/src/markerdetector.cpp

src/CMakeFiles/aruco.dir/markerdetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco.dir/markerdetector.cpp.i"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kinesis/catkin_ws/src/aruco-1.2.4/src/markerdetector.cpp > CMakeFiles/aruco.dir/markerdetector.cpp.i

src/CMakeFiles/aruco.dir/markerdetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco.dir/markerdetector.cpp.s"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kinesis/catkin_ws/src/aruco-1.2.4/src/markerdetector.cpp -o CMakeFiles/aruco.dir/markerdetector.cpp.s

src/CMakeFiles/aruco.dir/markerdetector.cpp.o.requires:

.PHONY : src/CMakeFiles/aruco.dir/markerdetector.cpp.o.requires

src/CMakeFiles/aruco.dir/markerdetector.cpp.o.provides: src/CMakeFiles/aruco.dir/markerdetector.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/aruco.dir/build.make src/CMakeFiles/aruco.dir/markerdetector.cpp.o.provides.build
.PHONY : src/CMakeFiles/aruco.dir/markerdetector.cpp.o.provides

src/CMakeFiles/aruco.dir/markerdetector.cpp.o.provides.build: src/CMakeFiles/aruco.dir/markerdetector.cpp.o


# Object files for target aruco
aruco_OBJECTS = \
"CMakeFiles/aruco.dir/arucofidmarkers.cpp.o" \
"CMakeFiles/aruco.dir/board.cpp.o" \
"CMakeFiles/aruco.dir/boarddetector.cpp.o" \
"CMakeFiles/aruco.dir/cameraparameters.cpp.o" \
"CMakeFiles/aruco.dir/cvdrawingutils.cpp.o" \
"CMakeFiles/aruco.dir/marker.cpp.o" \
"CMakeFiles/aruco.dir/markerdetector.cpp.o"

# External object files for target aruco
aruco_EXTERNAL_OBJECTS =

src/libaruco.so.1.2.4: src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o
src/libaruco.so.1.2.4: src/CMakeFiles/aruco.dir/board.cpp.o
src/libaruco.so.1.2.4: src/CMakeFiles/aruco.dir/boarddetector.cpp.o
src/libaruco.so.1.2.4: src/CMakeFiles/aruco.dir/cameraparameters.cpp.o
src/libaruco.so.1.2.4: src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o
src/libaruco.so.1.2.4: src/CMakeFiles/aruco.dir/marker.cpp.o
src/libaruco.so.1.2.4: src/CMakeFiles/aruco.dir/markerdetector.cpp.o
src/libaruco.so.1.2.4: src/CMakeFiles/aruco.dir/build.make
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_superres3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_face3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_plot3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_reg3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_text3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_shape3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_video3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_viz3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_flann3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_ml3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_photo3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.2.0
src/libaruco.so.1.2.4: /opt/ros/kinetic/lib/libopencv_core3.so.3.2.0
src/libaruco.so.1.2.4: src/CMakeFiles/aruco.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kinesis/catkin_ws/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX shared library libaruco.so"
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco.dir/link.txt --verbose=$(VERBOSE)
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && $(CMAKE_COMMAND) -E cmake_symlink_library libaruco.so.1.2.4 libaruco.so.1.2 libaruco.so

src/libaruco.so.1.2: src/libaruco.so.1.2.4
	@$(CMAKE_COMMAND) -E touch_nocreate src/libaruco.so.1.2

src/libaruco.so: src/libaruco.so.1.2.4
	@$(CMAKE_COMMAND) -E touch_nocreate src/libaruco.so

# Rule to build all files generated by this target.
src/CMakeFiles/aruco.dir/build: src/libaruco.so

.PHONY : src/CMakeFiles/aruco.dir/build

src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/arucofidmarkers.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/board.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/boarddetector.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/cameraparameters.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/cvdrawingutils.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/marker.cpp.o.requires
src/CMakeFiles/aruco.dir/requires: src/CMakeFiles/aruco.dir/markerdetector.cpp.o.requires

.PHONY : src/CMakeFiles/aruco.dir/requires

src/CMakeFiles/aruco.dir/clean:
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src && $(CMAKE_COMMAND) -P CMakeFiles/aruco.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/aruco.dir/clean

src/CMakeFiles/aruco.dir/depend:
	cd /home/kinesis/catkin_ws/src/aruco-1.2.4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kinesis/catkin_ws/src/aruco-1.2.4 /home/kinesis/catkin_ws/src/aruco-1.2.4/src /home/kinesis/catkin_ws/src/aruco-1.2.4/build /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src /home/kinesis/catkin_ws/src/aruco-1.2.4/build/src/CMakeFiles/aruco.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/aruco.dir/depend

