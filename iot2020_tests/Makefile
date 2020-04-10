#======================================================================
# Makefile for mixed C/C++ sources
#======================================================================

#----------------------------------------------------------------------
# Project Structure
#----------------------------------------------------------------------
# TARGET    is the name of the output
# BUILD     is the directory where object files & intermediate files will be placed
# OUT       is the directory where target files will be placed
# SOURCES   is a list of directories containing source code
# INCLUDES  is a list of directories containing header files
#----------------------------------------------------------------------
TARGET    := main
BUILD     := .
OUT       := .
SOURCES   := .
INCLUDES  := .

#----------------------------------------------------------------------
# Defined Symbols
#----------------------------------------------------------------------
DEFINES += -DDEBUG
DEFINES += -DENABLE_LOGGING

#----------------------------------------------------------------------
# Sources & Files
#----------------------------------------------------------------------
OUTPUT     := $(CURDIR)/$(OUT)/$(TARGET)
SYMBOLS    := $(CURDIR)/$(BUILD)/$(TARGET).out

CSOURCES   := $(foreach dir,$(SOURCES),$(wildcard $(dir)/*.c))
CSOURCES   += $(foreach dir,$(SOURCES),$(wildcard $(dir)/avr/*.c))
CXXSOURCES := $(foreach dir,$(SOURCES),$(wildcard $(dir)/*.cpp))
ASMSOURCES := $(foreach dir,$(SOURCES),$(wildcard $(dir)/*.s))

OBJS       := $(patsubst %,$(BUILD)/%.o,$(basename $(CSOURCES)) $(basename $(CXXSOURCES)) $(basename $(ASMSOURCES)))
DEPS       := $(patsubst %,$(BUILD)/%.d,$(basename $(CSOURCES)) $(basename $(CXXSOURCES)) $(basename $(ASMSOURCES)))

INCLUDE    += $(addprefix -I,$(foreach dir,$(INCLUDES), $(wildcard $(dir))))
INCLUDE    += $(addprefix -I,$(foreach dir,$(LIBRARIES),$(wildcard $(dir)/avr)))

#----------------------------------------------------------------------
# Compiler & Linker
#----------------------------------------------------------------------
CC  := gcc
CXX := g++

#----------------------------------------------------------------------
# Compiler & Linker Flags
#----------------------------------------------------------------------
# CPPFLAGS  C and C++ Compiler Flags
# CFLAGS    C Compiler Flags
# CXXFLAGS  C++ Compiler Flags
# ASFLAGS   ASM Compiler Flags
# LDFLAGS   Linker Flags
#----------------------------------------------------------------------
CPPFLAGS  += $(DEFINES) $(INCLUDE)

CFLAGS    += $(CPPFLAGS)
CFLAGS    += -I.
CFLAGS    += -lm
CFLAGS    += -lrt
CFLAGS    += -lmraa
CFLAGS    += -lpthread
CFLAGS    += -DDEBUG
CFLAGS    += -ggdb
#CFLAGS    += -Wall

CXXFLAGS  += $(CFLAGS)

ASFLAGS   += $(CPPFLAGS)
ASFLAGS   += -x assembler-with-cpp

#----------------------------------------------------------------------
# Compiler & Linker Commands
#----------------------------------------------------------------------
# LINK.o      link object files to binary
# COMPILE.c   compile C source files
# COMPILE.cpp compile C++ source files
#----------------------------------------------------------------------
ifeq ($(strip $(CXXSOURCES)),)
    LD := $(CC)
else
    LD := $(CXX)
endif

DEPFLAGS   += -MT $@
DEPFLAGS   += -MMD
DEPFLAGS   += -MP
DEPFLAGS   += -MF $(BUILD)/$*.d

LINK.o      = $(LD) $(LDFLAGS) $(CFLAGS) $(LDLIBS) $^ -o $@
COMPILE.c   = $(CC) $(C_STANDARD) $(CFLAGS) $(DEPFLAGS) -c $< -o $@
COMPILE.cpp = $(CXX) $(CXX_STANDARD) $(CXXFLAGS) $(DEPFLAGS) -c $< -o $@
COMPILE.s   = $(AS) $(ASFLAGS) $(DEPFLAGS) -c $< -o $@
SYMBOLS.out = $(NM) $@ > $(SYMBOLS)

#----------------------------------------------------------------------
# Special Built-in Target
#----------------------------------------------------------------------
# .SUFFIXES     disable built-in wildcard rules
# .INTERMEDIATE make will treat targets as intermediate files, and delete them
# .PRECIOUS     make will not be deleted after it is no longer needed. Keep objects to speed up recompilation
# .PHONY        make will run this targets unconditionally, regardless of whether a file with that name exists or what its last-modification time is
#----------------------------------------------------------------------
.SUFFIXES:
.INTERMEDIATE:
.PRECIOUS: $(OBJS) $(DEPS)
.PHONY: all clean help

#----------------------------------------------------------------------
# Targets
#----------------------------------------------------------------------
all: $(OUTPUT)

$(OUTPUT): $(OBJS)
	@mkdir -p $(@D)
	$(LINK.o)
	$(SYMBOLS.out)

$(BUILD)/%.o: %.c
	@mkdir -p $(@D)
	$(COMPILE.c)

$(BUILD)/%.o: %.cpp
	@mkdir -p $(@D)
	$(COMPILE.cpp)

$(BUILD)/%.o: %.s
	@mkdir -p $(@D)
	$(COMPILE.s)

#run: $(OUTPUT)
#	@$<

clean:
	@$(RM) *o *d

help:
	@echo available targets: all run clean

-include $(DEPS)
