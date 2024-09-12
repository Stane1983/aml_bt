CFLAGS := -Wall -g
CROSS_COMPILE=$(TARGET_KERNEL_CROSS)
CC ?= $(CROSS_COMPILE)gcc
all: aml_hciattach
#OBJS := hciattach.o hciattach_aml.o
OBJS := hciattach.o

aml_hciattach: $(OBJS)
	$(CC) -o aml_hciattach $(OBJS)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

clean:
	rm -f $(OBJS)  aml_hciattach

tags: FORCE
	ctags -R
	find ./ -name "*.h" -o -name "*.c" -o -name "*.cc" -o -name "*.cpp" > cscope.files
	cscope -bkq -i cscope.files
PHONY += FORCE
FORCE:
