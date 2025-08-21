OUT_DIR ?= .

# Cross compiler
CROSS_COMPILE := $(TARGET_KERNEL_CROSS)
CC            ?= $(CROSS_COMPILE)gcc

# Compiler flags
CFLAGS        += -Wall -g -O2 -Wno-unused-function -Wunused-result -Wno-unused-variable

LDFLAGS := -pthread
LDFLAGS += -L $(OUT_DIR)/

# Sources and objects
SRCS := aml_multibt.c hciattach.c hciattach_aml.c hciattach_aml_usb.c aml_comm.c \
        aml_conf.c

OBJS := $(SRCS:.c=.o)

# Output binary
TARGET := aml_hciattach
TARGET_FULLPATH := $(patsubst %, $(OUT_DIR)/%, $(TARGET))

# Default target
all: $(TARGET)

# Link rule
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(patsubst %, $(OUT_DIR)/%, $^) -o $(OUT_DIR)/$@

# Compile rule
%.o: %.c
	@mkdir -p $(OUT_DIR)
	$(CC) -c $(CFLAGS) $< -o $(OUT_DIR)/$@

# Clean
clean:
	rm -f $(OUT_DIR)/*.o $(OUT_DIR)/*.lo $(TARGET_FULLPATH)
	rm -f cscope.files cscope.out tags

# Tags
tags: FORCE
	@echo "Generating ctags..."
	ctags -R .
	@echo "Generating cscope.files..."
	find ./ -name "*.c" -o -name "*.cc" -o -name "*.cpp" -o -name "*.h" > cscope.files
	@echo "Generating cscope.out..."
	cscope -bkq -i cscope.files
	@echo "Done."

FORCE:

.PHONY: all clean tags FORCE
