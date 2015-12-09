#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/BBB
	NAME:=BeagleBone Black
	FEATURES:= usb ext4 targz
	DEFAULT_PACKAGES += kmod-usb2 kmod-usb2-omap
endef

define Profile/BBB/Description
	Package set for the BeagleBone Black and similar devices.
endef

$(eval $(call Profile,BBB))
