AML_HCIATTACH_PROJDIR := $(ROKU_WLAN_DIR)/amlogic/drivers/aml_hciattach
AML_HCIATTACH_OSS_LIC := COPYING README
AML_HCIATTACH_NAMEVER := aml_hciattach-v1
AML_HCIATTACH_TMP_SRC := hardlink
AML_HCIATTACH_BINARIES := \
	lib/firmware/aml/w1u_bt_fw_uart.bin \
	lib/firmware/aml/w1ub_bt_fw_uart.bin \
	lib/firmware/aml/w1u_bt_fw_uart_test.bin \
	lib/firmware/aml/w1u_bt_fw_usb.bin \
	lib/firmware/aml/w2l_bt_15p4_fw_usb.bin \
	lib/firmware/aml/aml_bt_rf.txt  \
	lib/firmware/aml/a2dp_mode_cfg.txt \
	lib/firmware/aml/aml_bt.conf \
	usr/sbin/aml_hciattach\
	usr/sbin/aml_hciattach_usb \
	usr/sbin/hcidump\

$(eval $(call prep_proj,AML_HCIATTACH,aml_hciattach,$(AML_HCIATTACH_PROJDIR),))
AML_HCIATTACH_MAKE_DEFS += \
	$(AML_HCIATTACH_CONFIG)

# Hack we shouldn't be doing this
# Hack specifying rtl as default for now but will change... porting kit somehow...?
AML_HCIATTACH_MAKE_ENVR = \
	CC="$(CC)" \

#CFLAGS+=-flto=auto -fuse-linker-plugin
#LFLAGS+=-flto=auto -fuse-linker-plugin

AML_HCIATTACH_MAKE_ENVR += \

aml_hciattach-build:
	$(AML_HCIATTACH_MAKE_ENVR) $(MAKE) -C $(AML_HCIATTACH_SRCDIR) $(AML_HCIATTACH_MAKE_DEFS)

aml_hciattach-install:
	mkdir -p $(AML_HCIATTACH_INSTDIR)/usr/sbin $(AML_HCIATTACH_INSTDIR)/lib/firmware/aml
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/w1u_bt_fw_uart.bin $(AML_HCIATTACH_INSTDIR)/lib/firmware/aml/w1u_bt_fw_uart.bin
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/w1ub_bt_fw_uart.bin $(AML_HCIATTACH_INSTDIR)/lib/firmware/aml/w1ub_bt_fw_uart.bin
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/w1u_bt_fw_uart_test.bin $(AML_HCIATTACH_INSTDIR)/lib/firmware/aml/w1u_bt_fw_uart_test.bin
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/w1u_bt_fw_usb.bin $(AML_HCIATTACH_INSTDIR)/lib/firmware/aml/w1u_bt_fw_usb.bin
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/w2l_bt_15p4_fw_usb.bin $(AML_HCIATTACH_INSTDIR)/lib/firmware/aml/w2l_bt_15p4_fw_usb.bin
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/aml_bt_rf.txt $(AML_HCIATTACH_INSTDIR)/lib/firmware/aml/aml_bt_rf.txt
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/a2dp_mode_cfg.txt $(AML_HCIATTACH_INSTDIR)/lib/firmware/aml/a2dp_mode_cfg.txt
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/aml_bt.conf $(AML_HCIATTACH_INSTDIR)/lib/firmware/aml/aml_bt.conf
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/aml_hciattach $(AML_HCIATTACH_INSTDIR)/usr/sbin/aml_hciattach
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/aml_hciattach_usb $(AML_HCIATTACH_INSTDIR)/usr/sbin/aml_hciattach_usb
	install -m 755 -D $(AML_HCIATTACH_SRCDIR)/hcidump $(AML_HCIATTACH_INSTDIR)/usr/sbin/hcidump

aml_hciattach-clean:
	$(MAKE) -C $(AML_HCIATTACH_SRCDIR) clean || true
	rm -rf $(AML_HCIATTACH_INSTDIR)/usr/sbin/
