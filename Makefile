# Project Name
TARGET = FreqDetect

# Sources
CPP_SOURCES = FreqDetect.cpp

# Library Locations
LIBDAISY_DIR = ../libdaisy
DAISYSP_DIR = ../DaisySP

# Core location, and generic makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

