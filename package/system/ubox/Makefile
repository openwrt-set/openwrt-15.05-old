include $(TOPDIR)/rules.mk

PKG_NAME:=ubox
PKG_VERSION:=2016-07-19
PKG_RELEASE=$(PKG_SOURCE_VERSION)

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL=$(OPENWRT_GIT)/ubox.git
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_VERSION:=aead2c0cbffdda9b46d74a998a4c6aeef423b21a
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION)-$(PKG_SOURCE_VERSION).tar.gz
CMAKE_INSTALL:=1

PKG_LICENSE:=GPL-2.0
PKG_LICENSE_FILES:=

PKG_MAINTAINER:=John Crispin <blogic@openwrt.org>

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/cmake.mk

TARGET_LDFLAGS += $(if $(CONFIG_USE_GLIBC),-lrt)

define Package/ubox
  SECTION:=base
  CATEGORY:=Base system
  DEPENDS:=+libubox +ubusd +ubus +libubus +libuci +USE_GLIBC:librt
  TITLE:=OpenWrt system helper toolbox
endef

define Package/logd
  SECTION:=base
  CATEGORY:=Base system
  DEPENDS:=+libubox +libubus +libblobmsg-json +USE_GLIBC:librt
  TITLE:=OpenWrt system log implementation
endef

define Package/ubox/install
	$(INSTALL_DIR) $(1)/sbin $(1)/usr/sbin $(1)/lib/
	$(INSTALL_DIR) $(1)/sbin $(1)/usr/sbin $(1)/lib/ $(1)/usr/bin

	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/sbin/{kmodloader,validate_data} $(1)/sbin/
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/getrandom $(1)/usr/bin/
	$(INSTALL_DATA) $(PKG_INSTALL_DIR)/usr/lib/libvalidate.so $(1)/lib

	$(LN) ../../sbin/kmodloader $(1)/usr/sbin/rmmod
	$(LN) ../../sbin/kmodloader $(1)/usr/sbin/insmod
	$(LN) ../../sbin/kmodloader $(1)/usr/sbin/lsmod
	$(LN) ../../sbin/kmodloader $(1)/usr/sbin/modinfo
	$(LN) ../../sbin/kmodloader $(1)/usr/sbin/modprobe
endef

define Package/logd/install
	$(INSTALL_DIR) $(1)/sbin $(1)/etc/init.d/

	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/sbin/{logd,logread} $(1)/sbin/
	$(INSTALL_BIN) ./files/log.init $(1)/etc/init.d/log
endef

$(eval $(call BuildPackage,ubox))
$(eval $(call BuildPackage,logd))
