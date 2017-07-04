#
# Copyright (C) 2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

define Profile/R0
	NAME:=irz_mt00 board
endef

define Profile/R2
	NAME:=irz_mt02 board
	DEPENDS:= +kmod-rtc-pcf85063 +kmod-sdhci-mt7620
endef

define Profile/S-TERRA
	NAME:=S-Terra board
	DEPENDS:= +kmod-rtc-pcf85063
	FEATURES+=nand
endef

define Profile/RX
	NAME:="irz_mtxx boards composite profile"
endef

$(eval $(call Profile,R0))
$(eval $(call Profile,R2))
$(eval $(call Profile,S-TERRA))
$(eval $(call Profile,RX))
