# 
# Copyright (C) 2013 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

PKG_NAME:=lemon
PKG_VERSION:=1.69.3


include $(INCLUDE_DIR)/host-build.mk

define Host/Configure
endef

define Host/Compile
	$(PATCH) -o $(HOST_BUILD_DIR)/lemon.c ./src/lemon.c ./patches/lemon_share.diff
	$(CC) $(HOST_BUILD_DIR)/lemon.c -o $(HOST_BUILD_DIR)/lemon
endef

define Host/Install
	mkdir -p $(STAGING_DIR_HOST)/bin
	$(CP) $(HOST_BUILD_DIR)/lemon $(STAGING_DIR_HOST)/bin
	mkdir -p $(STAGING_DIR_HOST)/share/lemon
	$(CP) ./src/lempar.c $(STAGING_DIR_HOST)/share/lemon/lempar.c
endef

$(eval $(call HostBuild))
