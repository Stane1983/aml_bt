ARCH ?= arm64

M ?= $(PWD)

AMLBT_MOD_NAME = w2l_bt

AMLBT_CFILES := hci_aml.c hci_aml_thread.c hci_aml_zigbee.c

$(AMLBT_MOD_NAME)-objs := $(AMLBT_CFILES:.c=.o)

obj-m := $(AMLBT_MOD_NAME).o

EXTRA_SYMBOLS_PATH_USB_BT=$(KERNEL_SRC)/Module.symvers

ccflags-y += -I$(KERNEL_SRC)/drivers/bluetooth

all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(M) modules

modules_install:
	@$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(PWD) clean

