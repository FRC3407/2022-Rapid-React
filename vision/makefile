rwildcard = $(foreach d,$(wildcard $(addsuffix *,$(1))),$(call rwildcard,$(d)/,$(2)) $(filter $(subst *,%,$(2)),$(d)))

OPT ?= release
ifeq ($(filter $(OPT),release debug),)
$(error invalid 'OPT' option - choose from {release/debug})
endif

CROSS_PREFIX := arm-raspbian10-linux-gnueabihf-
CXX := g++
STD := c++17

SRC_DIR := src
OBJ_DIR := obj
OUT_DIR := out
VS_DIR := VisionServer/lib-vs

NAME := vision-RR2022
PROGRAM := $(OUT_DIR)/$(NAME)
#PROGRAM_VS1 := $(PROGRAM)-vs1
VISIONSERVER := $(VS_DIR)/out/libvs3407.so
PACKAGE_DEPS := $(VS_DIR)/lib/libtensorflowlite.so $(VS_DIR)/lib/libedgetpu.so $(VISIONSERVER)

SRCS := $(call rwildcard,$(SRC_DIR)/,*.cpp *.c *.S *.s)
OBJS := $(SRCS:$(SRC_DIR)/%=$(OBJ_DIR)/%.o)
#SRCS_VS1 := main.cpp $(call rwildcard,$(SRC_DIR)/additions-compile/,*.cpp *.c) $(call rwildcard,$(VS_DIR)/src/vs1-legacy/,*.cpp *.c *.S *.s)
#SRCS_VS1 := $(SRCS:%=$(OBJ_DIR)/%.o)

ifeq ($(OPT),release)
#release options for Compiling and Linking
COPTS := -O3 -D__RELEASE__
LOPTS :=
else
#debug options for Compiling and Linking
COPTS := -g -Og -D__DEBUG__
LOPTS := -g
endif

#preprocessor flags
CPPFLAGS := -pthread -I$(VS_DIR)/include -I$(VS_DIR)/src \
	-I$(VS_DIR)/additions -I$(VS_DIR)/include/opencv4 -MMD -MP
#compile flags
CFLAGS := -Wall -fpermissive
#flags for compiling assembly source
ASMFLAGS := -mcpu=cortex-a72 -mfpu=neon-fp-armv8
#linker flags
LDFLAGS := -pthread -Wall -L$(VS_DIR)/out -L$(VS_DIR)/lib \
	-Wl,--unresolved-symbols=ignore-in-shared-libs -Wl,-rpath=$$ORIGIN
#libs to link against - see complete list in VisionServer/lib-vs/makefile
LDLIBS := -ltensorflowlite -ledgetpu -lopencv_gapi \
	-lopencv_highgui -lopencv_ml -lopencv_objdetect \
	-lopencv_photo -lopencv_stitching -lopencv_video \
	-lopencv_calib3d -lopencv_features2d -lopencv_dnn \
	-lopencv_flann -lopencv_videoio -lopencv_imgcodecs \
	-lopencv_imgproc -lopencv_core -lvs3407 -lcameraserver \
	-lntcore -lcscore -lwpiutil #-lpigpio


ifeq ($(OS),Windows_NT)
CXX := $(CROSS_PREFIX)$(CXX)
RM-R := del /s /Q
CP := copy
/ := \\

else
RM-R := rm -r
CP := cp
/ := /

endif


$(OBJ_DIR)/%.cpp.o : $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(COPTS) -c -o $(OBJ_DIR)/$(@F) -std=$(STD) $(CPPFLAGS) $(CFLAGS) $<

$(OBJ_DIR)/%.c.o : $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CXX) $(COPTS) -c -o $(OBJ_DIR)/$(@F) -std=$(STD) $(CPPFLAGS) $(CFLAGS) $<

$(OBJ_DIR)/%.S.o : $(SRC_DIR)/%.S | $(OBJ_DIR)
	$(CXX) $(COPTS) -c -o $(OBJ_DIR)/$(@F) -std=$(STD) $(ASMFLAGS) $(CPPFLAGS) $(CFLAGS) $<

$(OBJ_DIR)/%.s.o : $(SRC_DIR)/%.s | $(OBJ_DIR)
	$(CXX) $(COPTS) -c -o $(OBJ_DIR)/$(@F) -std=$(STD) $(ASMFLAGS) $(CPPFLAGS) $(CFLAGS) $<


$(PROGRAM): $(OBJS) | $(OUT_DIR)
	$(CXX) $(LOPTS) -o $@ $(LDFLAGS) $(foreach file,$(^F),$(OBJ_DIR)/$(file)) $(LDLIBS)

$(VISIONSERVER):
	$(MAKE) -C VisionServer/lib-vs shared

$(OUT_DIR) $(OBJ_DIR):
	mkdir $@


.PHONY: build package clean rebuild with-lib

build: $(PROGRAM)

with-lib: $(VISIONSERVER) build

package: build	#fix this
	$(CP) $(subst /,\,$(VS_DIR)/lib/libtensorflowlite.so) $(OUT_DIR)
	$(CP) $(subst /,\,$(VS_DIR)/lib/libedgetpu.so $(OUT_DIR)/libedgetpu.so.1)
	$(CP) $(subst /,\,$(VISIONSERVER)) $(OUT_DIR)

clean:
	$(RM-R) $(OBJ_DIR)
	$(RM_R) $(OUT_DIR)

rebuild: build | clean


-include $(OBJS:.o=.d)