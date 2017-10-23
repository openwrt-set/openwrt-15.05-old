#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=ublox-gps-i2c
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/ublox-gps-i2c
  SUBMENU:=Other modules
  TITLE:=UBLOX GPS over i2c driver
  FILES:=$(PKG_BUILD_DIR)/ublox-gps-i2c.ko
  AUTOLOAD:=$(call AutoLoad,90,ublox-gps-i2c,1)
  DEPENDS:= +kmod-i2c-core
  KCONFIG:=
endef

define KernelPackage/ublox-gps-i2c/description
endef

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)"

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,ublox-gps-i2c))
