
CC = g++
# raspi = armv7l
SYSTEM := $(shell uname -m)   # aarch64,  x86_64, ...
ARM = aarch64
ARM32 = armv7l	# raspi
X86 = x86_64


OPENCVD = ../../OpenCVD/include/opencvd.hpp
OPENCVD += ../../OpenCVD/include/opencvd_basic.hpp
OPENCVD += ../../OpenCVD/include/opencvd_func.hpp
OPENCVD += ../../OpenCVD/include/opencvd_mat.hpp
OPENCVD += ../../OpenCVD/include/opencvd_cvdstd.hpp
OPENCVD += ../../OpenCVD/include/opencvd_types.hpp
OPENCVD += ../../OpenCVD/include/specdef.hpp

# ---- get pkg-config files -----
ifeq ($(shell pkg-config --list-all | grep -ow opencv4), opencv4)
OPENCV_LIBS := opencv4
else
OPENCV_LIBS := opencv
endif

# --- gnu++11 uses GNU extensions. ---
CFLAGS = --std=c++1y	# --std=c++1y   # --std=c++14   # Es ist c++17 wegen #include <filesystem> erforderlich
CFLAGS += -Wall -Wextra -c -O0 -DNDEBUG   # no ABI Warnings // undocumented option // used for Raspy
CFLAGS += $(shell pkg-config --cflags $(OPENCV_LIBS))

LDFLAGS += $(shell pkg-config --libs $(OPENCV_LIBS))
LDFLAGS += -lpthread
LDFLAGS += -lstdc++fs

# ---------- Raspberry PI -----------
CFLAGS_RPI = --std=c++1y	# --std=c++14
CFLAGS_RPI += -Wall -c -O0 -DNDEBUG   # no ABI Warnings // undocumented option // used for Raspy
CFLAGS_RPI += -I/home/pi/c_source/lookat/

LDFLAGS_RPI += -lpthread
LPATH = /home/pi/c_source/lookat/libs/

LDFLAGS_RPI += $(LPATH)libopencv_stitching.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_superres.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_videostab.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_aruco.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_bgsegm.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_bioinspired.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_ccalib.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_dnn_objdetect.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_dpm.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_highgui.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_videoio.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_face.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_freetype.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_fuzzy.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_hfs.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_img_hash.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_line_descriptor.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_optflow.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_reg.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_rgbd.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_saliency.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_stereo.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_structured_light.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_phase_unwrapping.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_surface_matching.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_tracking.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_datasets.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_text.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_dnn.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_plot.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_xfeatures2d.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_shape.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_video.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_ml.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_ximgproc.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_xobjdetect.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_objdetect.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_calib3d.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_imgcodecs.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_features2d.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_flann.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_xphoto.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_photo.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_imgproc.so.3.4
LDFLAGS_RPI += $(LPATH)libopencv_core.so.3.4

LDFLAGS_RPI += -lwiringPi
LDFLAGS_RPI += -lstdc++fs
# ---------- End of Raspberry PI -----------

FILENAME = main
BUILDFILE = lookat

SOURCE = $(FILENAME).cpp
HEADER = Save_Vid.hpp histogram.h

OBJ = $(FILENAME).o 
BIN = $(BUILDFILE)


.PHONEY: all
all: $(BIN)

$(BIN): $(OBJ)
ifeq ($(SYSTEM),armv7l)
	$(CC) -o $@ $< $(LDFLAGS_RPI)
else
	$(CC) -o $@ $< $(LDFLAGS)
endif
	
# $(OBJ): $(SOURCE) $(HEADER) $(OPENCVD)
$(OBJ): $(SOURCE) $(HEADER)
ifeq ($(SYSTEM),armv7l)
	$(CC) $(CFLAGS_RPI) $(INC) $<
else
	$(CC) $(CFLAGS) $(INC) $<
endif


.PHONEY: clean
clean:	
	$(RM) -r -f $(OBJ)	
	$(RM) -r -f $(BUILDFILE)



# Es wird die CPU-Architektur angezeigt
.PHONEY: system
system:
ifeq ($(shell uname -m),$(ARM32))
	@echo "ARM32=armv7l    32-Bit-Version der ARMv7-Architektur"
endif
ifeq ($(shell uname -m),$(X86))
	@echo "X86=x86_64    AMD64 oder Intel 64"
endif
ifeq ($(shell uname -m),$(ARM))
	@echo "ARM=aarch64    ARMv8-Architektur"
endif


.PHONEY: help
help:
	@echo "------- Target's -----------"
	@echo "help     this messaage"
	@echo "all      build"
	@echo "clean    clear build"
	@echo "system   show CPU"