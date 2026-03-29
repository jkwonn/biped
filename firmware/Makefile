# Biped Robot — Daisy Seed Firmware
# Receives servo commands over USB serial, drives PCA9685 PWM.

TARGET = biped_firmware
CPP_SOURCES = src/main.cpp src/pca9685.cpp

# Path to libDaisy — clone into firmware/ if not present:
#   git clone https://github.com/electro-smith/libDaisy.git
LIBDAISY_DIR ?= libDaisy

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
