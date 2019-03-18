# set the binaries that have to be built
TARGETS := DenseTrack

# set the build configuration set 
BUILD := release
#BUILD := debug

# set bin and build dirs
BUILDDIR := .build_$(BUILD)
BINDIR := $(BUILD)

# libraries 
LDLIBS = $(addprefix -l, $(LIBS) $(LIBS_$(notdir $*)))
LIBS := \
	opencv_core opencv_highgui opencv_video opencv_imgproc \
	avformat avdevice avutil avcodec swscale roscpp rosconsole rostime roscpp_serialization cv_bridge vl :libsvm.so.2

# set some flags and compiler/linker specific commands
CXXFLAGS = -pipe -D __STDC_CONSTANT_MACROS -D STD=std -Wall $(CXXFLAGS_$(BUILD)) -I. -I/opt/include -I/home/nick/catkin_ws/src/action_dense_track/src/vlfeat-0.9.20/vl -I/home/nick/catkin_ws/src/action_dense_track/src/libsvm-3.17/
CXXFLAGS_debug := -ggdb
CXXFLAGS_release := -O3 -DNDEBUG -ggdb
LDFLAGS = -L/opt/lib -L/opt/ros/groovy/lib/ -L/home/nick/ExternalPackages/opencv/lib/ -L/home/nick/ExternalPackages/ffmpeg/ffmpeg_build/lib/ -L/home/nick/catkin_ws/src/action_dense_track/src/vlfeat-0.9.20/bin/glnxa64 -L/home/nick/catkin_ws/src/action_dense_track/src/vlfeat-0.9.20/bin/glnxa64/objs/ -L/home/nick/catkin_ws/src/action_dense_track/src/libsvm-3.17 -pipe -Wall $(LDFLAGS_$(BUILD))
LDFLAGS_debug := -ggdb
LDFLAGS_release := -O3 -ggdb

include make/generic.mk
