#
# Copyright (C) 2013 Radiofid
# based on OpenWRT.org source
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/R1
	NAME:=iRZ r1 router board
	PACKAGES:=kmod-usb-core kmod-usb-ohci kmod-usb2 swconfig
endef

define Profile/R1/Description
	Package set for r1 series router board.
endef

$(eval $(call Profile,R1))

define Profile/R2
	NAME:=iRZ r2 router board
	PACKAGES:=kmod-usb-core kmod-usb-ohci kmod-usb2 swconfig
endef

define Profile/R2/Description
	Package set for r2 series router board.
endef

$(eval $(call Profile,R2))

define Profile/irz
	NAME:=iRZ ramips 5350 routers
	PACKAGES:=kmod-usb-core kmod-usb-ohci kmod-usb2 swconfig
endef

define Profile/irz/Description
	Profile for irz ramips based routers
endef

$(eval $(call Profile,irz))