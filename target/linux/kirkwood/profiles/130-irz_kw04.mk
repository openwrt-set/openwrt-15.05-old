#
# Copyright (C) 2013 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/IRZ_KW04
  NAME:=iRZ kw04 router board
  DEFAULT_PACKAGES:= \
	kmod-usb2 kmod-usb-storage uboot-envtools \
	kmod-input-evdev kmod-input-gpio-keys kmod-input-gpio-keys-polled \
	kmod-gpio-button-hotplug kmod-input-polldev \
	kmod-fs-msdos kmod-fs-vfat kmod-scsi-core \
	kmod-switch-mv88e61xx kmod-gpio-pca953x \
	kmod-rtc-pcf85063 kmod-thermal kmod-thermal-kirkwood \
	kmod-wdt-orion kmod-usb-serial-option kmod-usb-serial-pl2303 \
	kmod-usb-serial-pl2303 kmod-usb-serial-pl2303 \
	kmod-usb-serial-qualcomm kmod-usb-serial-sierrawireless \
	kmod-usb-storage kmod-usb-serial kmod-maxim59xx \
	kmod-i2c-core kmod-i2c-algo-bit kmod-i2c-gpio
endef

define Profile/IRZ_KW04/Description
 Package set compatible with iRZ kw04 router board
endef

IRZ_KW04_UBIFS_OPTS:="-m 2048 -e 126KiB -c 4096"
IRZ_KW04_UBI_OPTS:="-m 2048 -p 128KiB -s 512"

$(eval $(call Profile,IRZ_KW04))
