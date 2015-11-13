# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dingej/work/project/opencv/linux/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /work/project/opencv/linux/test

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/cmake-gui -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /work/project/opencv/linux/test/CMakeFiles /work/project/opencv/linux/test/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /work/project/opencv/linux/test/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named 24ColorChecker

# Build rule for target.
24ColorChecker: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 24ColorChecker
.PHONY : 24ColorChecker

# fast build rule for target.
24ColorChecker/fast:
	$(MAKE) -f CMakeFiles/24ColorChecker.dir/build.make CMakeFiles/24ColorChecker.dir/build
.PHONY : 24ColorChecker/fast

#=============================================================================
# Target rules for targets named CameraCanny

# Build rule for target.
CameraCanny: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 CameraCanny
.PHONY : CameraCanny

# fast build rule for target.
CameraCanny/fast:
	$(MAKE) -f CMakeFiles/CameraCanny.dir/build.make CMakeFiles/CameraCanny.dir/build
.PHONY : CameraCanny/fast

#=============================================================================
# Target rules for targets named DualCamera

# Build rule for target.
DualCamera: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 DualCamera
.PHONY : DualCamera

# fast build rule for target.
DualCamera/fast:
	$(MAKE) -f CMakeFiles/DualCamera.dir/build.make CMakeFiles/DualCamera.dir/build
.PHONY : DualCamera/fast

#=============================================================================
# Target rules for targets named FeaturePoint

# Build rule for target.
FeaturePoint: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 FeaturePoint
.PHONY : FeaturePoint

# fast build rule for target.
FeaturePoint/fast:
	$(MAKE) -f CMakeFiles/FeaturePoint.dir/build.make CMakeFiles/FeaturePoint.dir/build
.PHONY : FeaturePoint/fast

#=============================================================================
# Target rules for targets named StereoMatch

# Build rule for target.
StereoMatch: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 StereoMatch
.PHONY : StereoMatch

# fast build rule for target.
StereoMatch/fast:
	$(MAKE) -f CMakeFiles/StereoMatch.dir/build.make CMakeFiles/StereoMatch.dir/build
.PHONY : StereoMatch/fast

24ColorChecker.o: 24ColorChecker.cpp.o
.PHONY : 24ColorChecker.o

# target to build an object file
24ColorChecker.cpp.o:
	$(MAKE) -f CMakeFiles/24ColorChecker.dir/build.make CMakeFiles/24ColorChecker.dir/24ColorChecker.cpp.o
.PHONY : 24ColorChecker.cpp.o

24ColorChecker.i: 24ColorChecker.cpp.i
.PHONY : 24ColorChecker.i

# target to preprocess a source file
24ColorChecker.cpp.i:
	$(MAKE) -f CMakeFiles/24ColorChecker.dir/build.make CMakeFiles/24ColorChecker.dir/24ColorChecker.cpp.i
.PHONY : 24ColorChecker.cpp.i

24ColorChecker.s: 24ColorChecker.cpp.s
.PHONY : 24ColorChecker.s

# target to generate assembly for a file
24ColorChecker.cpp.s:
	$(MAKE) -f CMakeFiles/24ColorChecker.dir/build.make CMakeFiles/24ColorChecker.dir/24ColorChecker.cpp.s
.PHONY : 24ColorChecker.cpp.s

CameraCanny.o: CameraCanny.cpp.o
.PHONY : CameraCanny.o

# target to build an object file
CameraCanny.cpp.o:
	$(MAKE) -f CMakeFiles/CameraCanny.dir/build.make CMakeFiles/CameraCanny.dir/CameraCanny.cpp.o
.PHONY : CameraCanny.cpp.o

CameraCanny.i: CameraCanny.cpp.i
.PHONY : CameraCanny.i

# target to preprocess a source file
CameraCanny.cpp.i:
	$(MAKE) -f CMakeFiles/CameraCanny.dir/build.make CMakeFiles/CameraCanny.dir/CameraCanny.cpp.i
.PHONY : CameraCanny.cpp.i

CameraCanny.s: CameraCanny.cpp.s
.PHONY : CameraCanny.s

# target to generate assembly for a file
CameraCanny.cpp.s:
	$(MAKE) -f CMakeFiles/CameraCanny.dir/build.make CMakeFiles/CameraCanny.dir/CameraCanny.cpp.s
.PHONY : CameraCanny.cpp.s

DualCamera.o: DualCamera.cpp.o
.PHONY : DualCamera.o

# target to build an object file
DualCamera.cpp.o:
	$(MAKE) -f CMakeFiles/DualCamera.dir/build.make CMakeFiles/DualCamera.dir/DualCamera.cpp.o
.PHONY : DualCamera.cpp.o

DualCamera.i: DualCamera.cpp.i
.PHONY : DualCamera.i

# target to preprocess a source file
DualCamera.cpp.i:
	$(MAKE) -f CMakeFiles/DualCamera.dir/build.make CMakeFiles/DualCamera.dir/DualCamera.cpp.i
.PHONY : DualCamera.cpp.i

DualCamera.s: DualCamera.cpp.s
.PHONY : DualCamera.s

# target to generate assembly for a file
DualCamera.cpp.s:
	$(MAKE) -f CMakeFiles/DualCamera.dir/build.make CMakeFiles/DualCamera.dir/DualCamera.cpp.s
.PHONY : DualCamera.cpp.s

LocalFeature.o: LocalFeature.cpp.o
.PHONY : LocalFeature.o

# target to build an object file
LocalFeature.cpp.o:
	$(MAKE) -f CMakeFiles/FeaturePoint.dir/build.make CMakeFiles/FeaturePoint.dir/LocalFeature.cpp.o
.PHONY : LocalFeature.cpp.o

LocalFeature.i: LocalFeature.cpp.i
.PHONY : LocalFeature.i

# target to preprocess a source file
LocalFeature.cpp.i:
	$(MAKE) -f CMakeFiles/FeaturePoint.dir/build.make CMakeFiles/FeaturePoint.dir/LocalFeature.cpp.i
.PHONY : LocalFeature.cpp.i

LocalFeature.s: LocalFeature.cpp.s
.PHONY : LocalFeature.s

# target to generate assembly for a file
LocalFeature.cpp.s:
	$(MAKE) -f CMakeFiles/FeaturePoint.dir/build.make CMakeFiles/FeaturePoint.dir/LocalFeature.cpp.s
.PHONY : LocalFeature.cpp.s

main.o: main.cpp.o
.PHONY : main.o

# target to build an object file
main.cpp.o:
	$(MAKE) -f CMakeFiles/FeaturePoint.dir/build.make CMakeFiles/FeaturePoint.dir/main.cpp.o
.PHONY : main.cpp.o

main.i: main.cpp.i
.PHONY : main.i

# target to preprocess a source file
main.cpp.i:
	$(MAKE) -f CMakeFiles/FeaturePoint.dir/build.make CMakeFiles/FeaturePoint.dir/main.cpp.i
.PHONY : main.cpp.i

main.s: main.cpp.s
.PHONY : main.s

# target to generate assembly for a file
main.cpp.s:
	$(MAKE) -f CMakeFiles/FeaturePoint.dir/build.make CMakeFiles/FeaturePoint.dir/main.cpp.s
.PHONY : main.cpp.s

stereo_match.o: stereo_match.cpp.o
.PHONY : stereo_match.o

# target to build an object file
stereo_match.cpp.o:
	$(MAKE) -f CMakeFiles/StereoMatch.dir/build.make CMakeFiles/StereoMatch.dir/stereo_match.cpp.o
.PHONY : stereo_match.cpp.o

stereo_match.i: stereo_match.cpp.i
.PHONY : stereo_match.i

# target to preprocess a source file
stereo_match.cpp.i:
	$(MAKE) -f CMakeFiles/StereoMatch.dir/build.make CMakeFiles/StereoMatch.dir/stereo_match.cpp.i
.PHONY : stereo_match.cpp.i

stereo_match.s: stereo_match.cpp.s
.PHONY : stereo_match.s

# target to generate assembly for a file
stereo_match.cpp.s:
	$(MAKE) -f CMakeFiles/StereoMatch.dir/build.make CMakeFiles/StereoMatch.dir/stereo_match.cpp.s
.PHONY : stereo_match.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... 24ColorChecker"
	@echo "... CameraCanny"
	@echo "... DualCamera"
	@echo "... FeaturePoint"
	@echo "... StereoMatch"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... 24ColorChecker.o"
	@echo "... 24ColorChecker.i"
	@echo "... 24ColorChecker.s"
	@echo "... CameraCanny.o"
	@echo "... CameraCanny.i"
	@echo "... CameraCanny.s"
	@echo "... DualCamera.o"
	@echo "... DualCamera.i"
	@echo "... DualCamera.s"
	@echo "... LocalFeature.o"
	@echo "... LocalFeature.i"
	@echo "... LocalFeature.s"
	@echo "... main.o"
	@echo "... main.i"
	@echo "... main.s"
	@echo "... stereo_match.o"
	@echo "... stereo_match.i"
	@echo "... stereo_match.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

