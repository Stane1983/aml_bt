KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(PWD)
#EXTRA_SYMBOLS_USB_PATH ?= $(M)/../../../../../aml-5.15/wifi/amlogic/drivers/w2/aml_drv/Module.symvers
EXTRA_SYMBOLS_USB_PATH ?= $(M)/../../../../wifi/amlogic/drivers/w2/aml_drv/Module.symvers
###############################################################################
# AMLBT
###############################################################################

AMLBT_MOD_NAME = w2_bt

AMLBT_CFILES := \
    w2_bt_entry.c \
    w2_sdio_bt.c \
    w2_usb_bt.c \
    rc_list.c \
    debug_dev.c \
    common.c

$(AMLBT_MOD_NAME)-objs := $(AMLBT_CFILES:.c=.o)

$(warning KBUILD_EXTRA_SYMBOLS=$(KBUILD_EXTRA_SYMBOLS))
$(warning M=$(M))

###############################################################################
# Common
###############################################################################
obj-m += $(AMLBT_MOD_NAME).o

EXTRA_CFLAGS += -Wno-pointer-to-int-cast
EXTRA_CFLAGS += -Wno-format
EXTRA_CFLAGS += -Wno-unused-function
EXTRA_CFLAGS += -Wno-unused-variable
EXTRA_CFLAGS += -Wno-return-type
EXTRA_CFLAGS += -Wno-implicit-fallthrough
EXTRA_CFLAGS += -Wno-sizeof-pointer-memaccess

all:
	make -C $(KERNEL_SRC) M=$(M) KBUILD_EXTRA_SYMBOLS=$(EXTRA_SYMBOLS_USB_PATH) modules
modules_install:
	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install
	mkdir -p ${OUT_DIR}/../vendor_lib/modules
	cd ${OUT_DIR}/$(M)/; find -name "*.ko" -exec cp {} ${OUT_DIR}/../vendor_lib/modules/ \;

usb:
	make -C $(KERNEL_SRC) M=$(PWD) $(AMLBT_MOD_NAME).ko

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean



