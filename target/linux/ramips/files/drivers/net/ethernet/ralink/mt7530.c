/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2013 John Crispin <blogic@openwrt.org>
 */

#include <linux/if.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/netlink.h>
#include <linux/bitops.h>
#include <net/genetlink.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/lockdep.h>
#include <linux/workqueue.h>
#include <linux/of_device.h>

#include "mt7530.h"

#ifdef CONFIG_SOC_MT7621
#define MT7620A_GSW_REG_PIAC	0x0004
#else
#define MT7620A_GSW_REG_PIAC	0x7004
#endif

#define MT7530_CPU_PORT		6
#define MT7530_NUM_PORTS	8
#define MT7530_NUM_VLANS	16
#define MT7530_MAX_VID		4095
#define MT7530_MIN_VID		0

#define GSW_MDIO_ACCESS		BIT(31)
#define GSW_MDIO_READ		BIT(19)
#define GSW_MDIO_WRITE		BIT(18)
#define GSW_MDIO_START		BIT(16)
#define GSW_MDIO_ADDR_SHIFT	20
#define GSW_MDIO_REG_SHIFT	25
#define GSW_REG_PHY_TIMEOUT	(5 * HZ)
/* registers */
#define REG_ESW_VLAN_VTCR		0x90
#define REG_ESW_VLAN_VAWD1		0x94
#define REG_ESW_VLAN_VAWD2		0x98
#define REG_ESW_VLAN_VTIM(x)	(0x100 + 4 * ((x) / 2))

#define REG_ESW_VLAN_VAWD1_IVL_MAC	BIT(30)
#define REG_ESW_VLAN_VAWD1_VTAG_EN	BIT(28)
#define REG_ESW_VLAN_VAWD1_VALID	BIT(0)

/* vlan egress mode */
enum {
	ETAG_CTRL_UNTAG	= 0,
	ETAG_CTRL_TAG	= 2,
	ETAG_CTRL_SWAP	= 1,
	ETAG_CTRL_STACK	= 3,
};

#define REG_ESW_PORT_PCR(x)	(0x2004 | ((x) << 8))
#define REG_ESW_PORT_PVC(x)	(0x2010 | ((x) << 8))
#define REG_ESW_PORT_PPBV1(x)	(0x2014 | ((x) << 8))
#define GSW_REG_GPC1		0x7014
#define REG_HWTRAP		0x7804

#define MIB_DESC(_s , _o, _n)   \
	{                       \
		.size = (_s),   \
		.offset = (_o), \
		.name = (_n),   \
	}

struct mt7xxx_mib_desc {
	unsigned int size;
	unsigned int offset;
	const char *name;
};

#define MT7621_MIB_COUNTER_BASE	0x4000
#define MT7621_MIB_COUNTER_PORT_OFFSET	0x100
#define MT7621_STATS_TDPC	0x00
#define MT7621_STATS_TCRC	0x04
#define MT7621_STATS_TUPC	0x08
#define MT7621_STATS_TMPC	0x0C
#define MT7621_STATS_TBPC	0x10
#define MT7621_STATS_TCEC	0x14
#define MT7621_STATS_TSCEC	0x18
#define MT7621_STATS_TMCEC	0x1C
#define MT7621_STATS_TDEC	0x20
#define MT7621_STATS_TLCEC	0x24
#define MT7621_STATS_TXCEC	0x28
#define MT7621_STATS_TPPC	0x2C
#define MT7621_STATS_TL64PC	0x30
#define MT7621_STATS_TL65PC	0x34
#define MT7621_STATS_TL128PC	0x38
#define MT7621_STATS_TL256PC	0x3C
#define MT7621_STATS_TL512PC	0x40
#define MT7621_STATS_TL1024PC	0x44
#define MT7621_STATS_TOC	0x48
#define MT7621_STATS_RDPC	0x60
#define MT7621_STATS_RFPC	0x64
#define MT7621_STATS_RUPC	0x68
#define MT7621_STATS_RMPC	0x6C
#define MT7621_STATS_RBPC	0x70
#define MT7621_STATS_RAEPC	0x74
#define MT7621_STATS_RCEPC	0x78
#define MT7621_STATS_RUSPC	0x7C
#define MT7621_STATS_RFEPC	0x80
#define MT7621_STATS_ROSPC	0x84
#define MT7621_STATS_RJEPC	0x88
#define MT7621_STATS_RPPC	0x8C
#define MT7621_STATS_RL64PC	0x90
#define MT7621_STATS_RL65PC	0x94
#define MT7621_STATS_RL128PC	0x98
#define MT7621_STATS_RL256PC	0x9C
#define MT7621_STATS_RL512PC	0xA0
#define MT7621_STATS_RL1024PC	0xA4
#define MT7621_STATS_ROC	0xA8
#define MT7621_STATS_RDPC_CTRL	0xB0
#define MT7621_STATS_RDPC_ING	0xB4
#define MT7621_STATS_RDPC_ARL	0xB8

static const struct mt7xxx_mib_desc mt7621_mibs[] = {
	MIB_DESC(1, MT7621_STATS_TDPC, "TxDrop"),
	MIB_DESC(1, MT7621_STATS_TCRC, "TxCRC"),
	MIB_DESC(1, MT7621_STATS_TUPC, "TxUni"),
	MIB_DESC(1, MT7621_STATS_TMPC, "TxMulti"),
	MIB_DESC(1, MT7621_STATS_TBPC, "TxBroad"),
	MIB_DESC(1, MT7621_STATS_TCEC, "TxCollision"),
	MIB_DESC(1, MT7621_STATS_TSCEC, "TxSingleCol"),
	MIB_DESC(1, MT7621_STATS_TMCEC, "TxMultiCol"),
	MIB_DESC(1, MT7621_STATS_TDEC, "TxDefer"),
	MIB_DESC(1, MT7621_STATS_TLCEC, "TxLateCol"),
	MIB_DESC(1, MT7621_STATS_TXCEC, "TxExcCol"),
	MIB_DESC(1, MT7621_STATS_TPPC, "TxPause"),
	MIB_DESC(1, MT7621_STATS_TL64PC, "Tx64Byte"),
	MIB_DESC(1, MT7621_STATS_TL65PC, "Tx65Byte"),
	MIB_DESC(1, MT7621_STATS_TL128PC, "Tx128Byte"),
	MIB_DESC(1, MT7621_STATS_TL256PC, "Tx256Byte"),
	MIB_DESC(1, MT7621_STATS_TL512PC, "Tx512Byte"),
	MIB_DESC(1, MT7621_STATS_TL1024PC, "Tx1024Byte"),
	MIB_DESC(2, MT7621_STATS_TOC, "TxByte"),
	MIB_DESC(1, MT7621_STATS_RDPC, "RxDrop"),
	MIB_DESC(1, MT7621_STATS_RFPC, "RxFiltered"),
	MIB_DESC(1, MT7621_STATS_RUPC, "RxUni"),
	MIB_DESC(1, MT7621_STATS_RMPC, "RxMulti"),
	MIB_DESC(1, MT7621_STATS_RBPC, "RxBroad"),
	MIB_DESC(1, MT7621_STATS_RAEPC, "RxAlignErr"),
	MIB_DESC(1, MT7621_STATS_RCEPC, "RxCRC"),
	MIB_DESC(1, MT7621_STATS_RUSPC, "RxUnderSize"),
	MIB_DESC(1, MT7621_STATS_RFEPC, "RxFragment"),
	MIB_DESC(1, MT7621_STATS_ROSPC, "RxOverSize"),
	MIB_DESC(1, MT7621_STATS_RJEPC, "RxJabber"),
	MIB_DESC(1, MT7621_STATS_RPPC, "RxPause"),
	MIB_DESC(1, MT7621_STATS_RL64PC, "Rx64Byte"),
	MIB_DESC(1, MT7621_STATS_RL65PC, "Rx65Byte"),
	MIB_DESC(1, MT7621_STATS_RL128PC, "Rx128Byte"),
	MIB_DESC(1, MT7621_STATS_RL256PC, "Rx256Byte"),
	MIB_DESC(1, MT7621_STATS_RL512PC, "Rx512Byte"),
	MIB_DESC(1, MT7621_STATS_RL1024PC, "Rx1024Byte"),
	MIB_DESC(2, MT7621_STATS_ROC, "RxByte"),
	MIB_DESC(1, MT7621_STATS_RDPC_CTRL, "RxCtrlDrop"),
	MIB_DESC(1, MT7621_STATS_RDPC_ING, "RxIngDrop"),
	MIB_DESC(1, MT7621_STATS_RDPC_ARL, "RxARLDrop")
};

enum {
	/* Global attributes. */
	MT7530_ATTR_ENABLE_VLAN,
};

struct mt7530_port_entry {
	u16	pvid;
	u8	disabled;
};

struct mt7530_vlan_entry {
	u16	vid;
	u8	member;
	u8	etags;
};

struct mt7530_priv {
	void __iomem		*base;
	struct mii_bus		*bus;
	struct switch_dev	swdev;

	bool			global_vlan_enable;
	struct mt7530_vlan_entry	vlan_entries[MT7530_NUM_VLANS];
	struct mt7530_port_entry	port_entries[MT7530_NUM_PORTS];
};

struct mt7530_mapping {
	char	*name;
	u16	pvids[MT7530_NUM_PORTS];
	u8	members[MT7530_NUM_VLANS];
	u8	etags[MT7530_NUM_VLANS];
	u16	vids[MT7530_NUM_VLANS];
} mt7530_defaults[] = {
	{
		.name = "llllw",
		.pvids = { 1, 1, 1, 1, 2, 1, 1 },
		.members = { 0, 0x6f, 0x50 },
		.etags = { 0, 0x40, 0x40 },
		.vids = { 0, 1, 2 },
	}, {
		.name = "wllll",
		.pvids = { 2, 1, 1, 1, 1, 1, 1 },
		.members = { 0, 0x7e, 0x41 },
		.etags = { 0, 0x40, 0x40 },
		.vids = { 0, 1, 2 },
	},
};



struct mt7530_mapping*
mt7530_find_mapping(struct device_node *np)
{
	const char *map;
	int i;

	if (of_property_read_string(np, "ralink,port-map", &map))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(mt7530_defaults); i++)
		if (!strcmp(map, mt7530_defaults[i].name))
			return &mt7530_defaults[i];

	return NULL;
}

static void
mt7530_apply_mapping(struct mt7530_priv *mt7530, struct mt7530_mapping *map)
{
	int i = 0;

	for (i = 0; i < MT7530_NUM_PORTS; i++)
		mt7530->port_entries[i].pvid = map->pvids[i];

	for (i = 0; i < MT7530_NUM_VLANS; i++) {
		mt7530->vlan_entries[i].member = map->members[i];
		mt7530->vlan_entries[i].etags = map->etags[i];
		mt7530->vlan_entries[i].vid = map->vids[i];
	}
}

static int
mt7530_reset_switch(struct switch_dev *dev)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	int i;

	memset(priv->port_entries, 0, sizeof(priv->port_entries));
	memset(priv->vlan_entries, 0, sizeof(priv->vlan_entries));

	/* set default vid of each vlan to the same number of vlan, so the vid
	 * won't need be set explicitly.
	 */
	for (i = 0; i < MT7530_NUM_VLANS; i++) {
		priv->vlan_entries[i].vid = 0xFFFF;
	}

	return 0;
}

static int
mt7530_get_vlan_enable(struct switch_dev *dev,
			   const struct switch_attr *attr,
			   struct switch_val *val)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);

	val->value.i = priv->global_vlan_enable;

	return 0;
}

static int
mt7530_set_vlan_enable(struct switch_dev *dev,
			   const struct switch_attr *attr,
			   struct switch_val *val)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);

	priv->global_vlan_enable = val->value.i != 0;

	return 0;
}

static u32
mt7530_r32(struct mt7530_priv *priv, u32 reg)
{
	u32 val;
	if (priv->bus) {
		u16 high, low;

		mdiobus_write(priv->bus, 0x1f, 0x1f, (reg >> 6) & 0x3ff);
		low = mdiobus_read(priv->bus, 0x1f, (reg >> 2) & 0xf);
		high = mdiobus_read(priv->bus, 0x1f, 0x10);

		return (high << 16) | (low & 0xffff);
	}

	val = ioread32(priv->base + reg);
	pr_debug("MT7530 MDIO Read [%04x]=%08x\n", reg, val);

	return val;
}

static void
mt7530_w32(struct mt7530_priv *priv, u32 reg, u32 val)
{
	if (priv->bus) {
		mdiobus_write(priv->bus, 0x1f, 0x1f, (reg >> 6) & 0x3ff);
		mdiobus_write(priv->bus, 0x1f, (reg >> 2) & 0xf,  val & 0xffff);
		mdiobus_write(priv->bus, 0x1f, 0x10, val >> 16);
		return;
	}

	pr_debug("MT7530 MDIO Write[%04x]=%08x\n", reg, val);
	iowrite32(val, priv->base + reg);
}

static int mt7530_mii_busy_wait(struct mt7530_priv *priv)
{
	unsigned long t_start = jiffies;

	while (1) {
		if (!(mt7530_r32(priv, MT7620A_GSW_REG_PIAC) & GSW_MDIO_ACCESS))
			return 0;
		if (time_after(jiffies, t_start + GSW_REG_PHY_TIMEOUT)) {
			break;
		}
	}

	printk(KERN_ERR "mdio: MDIO timeout\n");
	return -1;
}

static u32 _mt7530_mii_write(struct mt7530_priv *priv, u32 phy_addr, u32 phy_register,
				u32 write_data)
{
	if (mt7530_mii_busy_wait(priv))
		return -1;

	write_data &= 0xffff;

	mt7530_w32(priv, MT7620A_GSW_REG_PIAC, GSW_MDIO_ACCESS | GSW_MDIO_START | GSW_MDIO_WRITE |
		(phy_register << GSW_MDIO_REG_SHIFT) |
		(phy_addr << GSW_MDIO_ADDR_SHIFT) | write_data);

	if (mt7530_mii_busy_wait(priv))
		return -1;

	return 0;
}

static u32 _mt7530_mii_read(struct mt7530_priv *priv, int phy_addr, int phy_reg)
{
	u32 d;

	if (mt7530_mii_busy_wait(priv))
		return 0xffff;

	mt7530_w32(priv, MT7620A_GSW_REG_PIAC, GSW_MDIO_ACCESS | GSW_MDIO_START | GSW_MDIO_READ |
		(phy_reg << GSW_MDIO_REG_SHIFT) |
		(phy_addr << GSW_MDIO_ADDR_SHIFT));

	if (mt7530_mii_busy_wait(priv))
		return 0xffff;

	d = mt7530_r32(priv, MT7620A_GSW_REG_PIAC) & 0xffff;

	return d;
}


static void
mt7530_vtcr(struct mt7530_priv *priv, u32 cmd, u32 val)
{
	int i;

	mt7530_w32(priv, REG_ESW_VLAN_VTCR, BIT(31) | (cmd << 12) | val);

	for (i = 0; i < 20; i++) {
		u32 val = mt7530_r32(priv, REG_ESW_VLAN_VTCR);

		if ((val & BIT(31)) == 0)
			break;

		udelay(1000);
	}
	if (i == 20)
		printk("mt7530: vtcr timeout\n");
}

static int
mt7530_get_port_pvid(struct switch_dev *dev, int port, int *val)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);

	if (port >= MT7530_NUM_PORTS)
		return -EINVAL;

	*val = mt7530_r32(priv, REG_ESW_PORT_PPBV1(port));
	*val &= 0xfff;

	return 0;
}

static int
mt7530_set_port_pvid(struct switch_dev *dev, int port, int pvid)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);

	if (port >= MT7530_NUM_PORTS)
		return -EINVAL;

	if (pvid < MT7530_MIN_VID || pvid > MT7530_MAX_VID)
		return -EINVAL;

	priv->port_entries[port].pvid = pvid;

	return 0;
}

static int
mt7530_get_vlan_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	u32 member;
	u32 etags;
	int i;

	val->len = 0;

	if (val->port_vlan < 0 || val->port_vlan >= MT7530_NUM_VLANS)
		return -EINVAL;

	mt7530_vtcr(priv, 0, val->port_vlan);

	member = mt7530_r32(priv, REG_ESW_VLAN_VAWD1);
	member >>= 16;
	member &= 0xff;

	etags = mt7530_r32(priv, REG_ESW_VLAN_VAWD2);

	for (i = 0; i < MT7530_NUM_PORTS; i++) {
		struct switch_port *p;
		int etag;

		if (!(member & BIT(i)))
			continue;

		p = &val->value.ports[val->len++];
		p->id = i;

		etag = (etags >> (i * 2)) & 0x3;

		if (etag == ETAG_CTRL_TAG)
			p->flags |= BIT(SWITCH_PORT_FLAG_TAGGED);
		else if (etag != ETAG_CTRL_UNTAG)
			printk("vlan egress tag control neither untag nor tag.\n");
	}

	return 0;
}

static int
mt7530_set_vlan_ports(struct switch_dev *dev, struct switch_val *val)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	u8 member = 0;
	u8 etags = 0;
	int i;

	if (val->port_vlan < 0 || val->port_vlan >= MT7530_NUM_VLANS ||
			val->len > MT7530_NUM_PORTS)
		return -EINVAL;
	if( priv->vlan_entries[val->port_vlan].vid == 0xffff ) {
		priv->vlan_entries[val->port_vlan].vid = val->port_vlan;
	}
	for (i = 0; i < val->len; i++) {
		struct switch_port *p = &val->value.ports[i];

		if (p->id >= MT7530_NUM_PORTS)
			return -EINVAL;

		member |= BIT(p->id);

		if (p->flags & BIT(SWITCH_PORT_FLAG_TAGGED))
			etags |= BIT(p->id);
	}
	priv->vlan_entries[val->port_vlan].member = member;
	priv->vlan_entries[val->port_vlan].etags = etags;

	return 0;
}

static int
mt7530_set_vid(struct switch_dev *dev, const struct switch_attr *attr,
		struct switch_val *val)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	int vlan;
	u16 vid;

	vlan = val->port_vlan;
	vid = (u16)val->value.i;

	if (vlan < 0 || vlan >= MT7530_NUM_VLANS)
		return -EINVAL;

	if (vid < MT7530_MIN_VID || vid > MT7530_MAX_VID)
		return -EINVAL;

	priv->vlan_entries[vlan].vid = vid;
	return 0;
}

static int
mt7530_get_vid(struct switch_dev *dev, const struct switch_attr *attr,
		struct switch_val *val)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	u32 vid;
	int vlan;

	vlan = val->port_vlan;

	vid = mt7530_r32(priv, REG_ESW_VLAN_VTIM(vlan));
	if (vlan & 1)
		vid = vid >> 12;
	vid &= 0xfff;

	val->value.i = vid;
	return 0;
}

static int
mt7530_apply_config(struct switch_dev *dev)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	int i, j;

	if (!priv->global_vlan_enable) {
		for (i = 0; i < MT7530_NUM_PORTS; i++)
			mt7530_w32(priv, REG_ESW_PORT_PCR(i), 0x00ff0000);

		for (i = 0; i < MT7530_NUM_PORTS; i++)
			mt7530_w32(priv, REG_ESW_PORT_PVC(i), 0x810000c0);

		return 0;
	}

	/* set all ports as security mode */
	for (i = 0; i < MT7530_NUM_PORTS; i++)
		mt7530_w32(priv, REG_ESW_PORT_PCR(i), 0x00ff0003);

	/* set all ports as user port */
	for (i = 0; i < MT7530_NUM_PORTS; i++)
		mt7530_w32(priv, REG_ESW_PORT_PVC(i), 0x81000000);

	for (i = 0; i < MT7530_NUM_VLANS; i++) {
		u16 vid = priv->vlan_entries[i].vid;
		u8 member = priv->vlan_entries[i].member;
		u8 etags = priv->vlan_entries[i].etags;
		u32 val;
		if( vid == 0xFFFF )
		    continue;

		/* vid of vlan */
		val = mt7530_r32(priv, REG_ESW_VLAN_VTIM(i));
		if (i % 2 == 0) {
			val &= 0xfff000;
			val |= vid;
		} else {
			val &= 0xfff;
			val |= (vid << 12);
		}
		mt7530_w32(priv, REG_ESW_VLAN_VTIM(i), val);

		/* vlan port membership */
		if (member)
			mt7530_w32(priv, REG_ESW_VLAN_VAWD1, REG_ESW_VLAN_VAWD1_IVL_MAC |
				REG_ESW_VLAN_VAWD1_VTAG_EN | (member << 16) |
				REG_ESW_VLAN_VAWD1_VALID);
		else
			mt7530_w32(priv, REG_ESW_VLAN_VAWD1, 0);

		/* egress mode */
		val = 0;
		for (j = 0; j < MT7530_NUM_PORTS; j++) {
			if (etags & BIT(j)) {
				val |= ETAG_CTRL_TAG << (j * 2);
			}
			else {
				if( member & (1 << j) ) {
					pr_info("port:%d, vid: %d\n", j, vid);
					priv->port_entries[j].pvid = vid;
				}
				val |= ETAG_CTRL_UNTAG << (j * 2);
			}
		}
		mt7530_w32(priv, REG_ESW_VLAN_VAWD2, val);

		/* write to vlan table */
		mt7530_vtcr(priv, 1, i);
	}

	/* Port Default PVID */
	for (i = 0; i < MT7530_NUM_PORTS; i++) {
		u32 val;
		val = mt7530_r32(priv, REG_ESW_PORT_PPBV1(i));
		val &= ~0xfff;
		val |= priv->port_entries[i].pvid;
		mt7530_w32(priv, REG_ESW_PORT_PPBV1(i), val);
	}

	return 0;
}

static int
mt7530_set_port_link(struct switch_dev *dev,  int port,
			struct switch_port_link *link)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	u32 pmcr, anar, cr;
	if (port < 0 || port >= MT7530_NUM_PORTS)
		return -EINVAL;

	pmcr = mt7530_r32(priv, 0x3000 + (0x100 * port));
	anar = _mt7530_mii_read(priv, port, 0x04);

	cr = _mt7530_mii_read(priv, port, 0x00);
	cr |= (1 << 15);
	_mt7530_mii_write(priv, port, 0x00, cr);

	anar &= ~(0xf << 5);
	if (link->aneg) {
		pmcr &= ~(1 << 15); // disable force mode
		anar |= (0xf << 5);
	} else {
		pmcr |= ~(1 << 15); // enable force mode

		if (link->duplex)
			pmcr |= (1 << 1);
		else 
			pmcr &= ~(1 << 1);

		pmcr &= ~(3 << 2);
		switch (link->speed) {
		case SWITCH_PORT_SPEED_10:
			if (link->duplex) {
			    anar |= (1 << 6);
			} else {
			    anar |= (1 << 5);
			}
			break;
		case SWITCH_PORT_SPEED_100:
			pmcr |= (1 << 2);
			if (link->duplex) {
			    anar |= (1 << 8);
			} else {
			    anar |= (1 << 7);
			}
			break;
		case SWITCH_PORT_SPEED_1000:
			pmcr |= (2 << 2);
			break;
		default:
			return -ENOTSUPP;
		}
	}

	mt7530_w32(priv, 0x3000 + (0x100 * port), pmcr);
	_mt7530_mii_write(priv, port, 0x04, anar);

	cr &= ~(1 << 15);
	_mt7530_mii_write(priv, port, 0x00, cr);

	return 0;
}

static int
mt7530_get_port_link(struct switch_dev *dev,  int port,
			struct switch_port_link *link)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	u32 speed, pmsr;

	if (port < 0 || port >= MT7530_NUM_PORTS)
		return -EINVAL;

	if (priv->port_entries[port].disabled)
		return -EINVAL;

	pmsr = mt7530_r32(priv, 0x3008 + (0x100 * port));

	link->link = pmsr & 1;
	link->duplex = (pmsr >> 1) & 1;
	speed = (pmsr >> 2) & 3;

	switch (speed) {
	case 0:
		link->speed = SWITCH_PORT_SPEED_10;
		break;
	case 1:
		link->speed = SWITCH_PORT_SPEED_100;
		break;
	case 2:
	case 3: /* forced gige speed can be 2 or 3 */
		link->speed = SWITCH_PORT_SPEED_1000;
		break;
	default:
		link->speed = SWITCH_PORT_SPEED_UNKNOWN;
		break;
	}

	return 0;
}

static int
mt7530_get_port_disabled(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	u32 port = val->port_vlan;

	if (port < 0 || port >= MT7530_NUM_PORTS)
		return -EINVAL;

	val->value.i = priv->port_entries[port].disabled;

	return 0;
}

static int
mt7530_set_port_disabled(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	u32 port = val->port_vlan;

	if (port < 0 || port >= MT7530_NUM_PORTS)
		return -EINVAL;

	u16 ctl;
	ctl = _mt7530_mii_read(priv, port, 0);

	if( val->value.i ) {
		priv->port_entries[port].disabled = 1;
		ctl |= (1 << 11);
	} else {
		priv->port_entries[port].disabled = 0;
		ctl &= ~(1 << 11);
	}
	_mt7530_mii_write(priv, port, 0, ctl);

	return 0;
}



static const struct switch_attr mt7530_global[] = {
	{
		.type = SWITCH_TYPE_INT,
		.name = "enable_vlan",
		.description = "VLAN mode (1:enabled)",
		.max = 1,
		.id = MT7530_ATTR_ENABLE_VLAN,
		.get = mt7530_get_vlan_enable,
		.set = mt7530_set_vlan_enable,
	},
};

static u64 get_mib_counter(struct mt7530_priv *priv, int i, int port)
{
	unsigned int port_base;
	u64 t;

	port_base = MT7621_MIB_COUNTER_BASE +
		    MT7621_MIB_COUNTER_PORT_OFFSET * port;

	t = mt7530_r32(priv, port_base + mt7621_mibs[i].offset);
	if (mt7621_mibs[i].size == 2) {
		u64 hi;

		hi = mt7530_r32(priv, port_base + mt7621_mibs[i].offset + 4);
		t |= hi << 32;
	}

	return t;
}

static int mt7621_sw_get_port_mib(struct switch_dev *dev,
				  const struct switch_attr *attr,
				  struct switch_val *val)
{
	static char buf[4096];
	struct mt7530_priv *priv = container_of(dev, struct mt7530_priv, swdev);
	int i, len = 0;

	if (val->port_vlan >= MT7530_NUM_PORTS)
		return -EINVAL;

	len += snprintf(buf + len, sizeof(buf) - len,
			"Port %d MIB counters\n", val->port_vlan);

	for (i = 0; i < sizeof(mt7621_mibs) / sizeof(*mt7621_mibs); ++i) {
		u64 counter;
		len += snprintf(buf + len, sizeof(buf) - len,
				"%-11s: ", mt7621_mibs[i].name);
		counter = get_mib_counter(priv, i, val->port_vlan);
		len += snprintf(buf + len, sizeof(buf) - len, "%llu\n",
				counter);
	}

	val->value.s = buf;
	val->len = len;
	return 0;
}

static const struct switch_attr mt7621_port[] = {
	{
		.type = SWITCH_TYPE_STRING,
		.name = "mib",
		.description = "Get MIB counters for port",
		.get = mt7621_sw_get_port_mib,
		.set = NULL,
	},
};

static const struct switch_attr mt7530_port[] = {
	{
		.type = SWITCH_TYPE_INT,
		.name = "port_disabled",
		.description = "Port disabled state",
		.get = mt7530_get_port_disabled,
		.set = mt7530_set_port_disabled,
	},
};

static const struct switch_attr mt7530_vlan[] = {
	{
		.type = SWITCH_TYPE_INT,
		.name = "vid",
		.description = "VLAN ID (0-4094)",
		.set = mt7530_set_vid,
		.get = mt7530_get_vid,
		.max = 4094,
	},
};

static const struct switch_dev_ops mt7621_ops = {
	.attr_global = {
		.attr = mt7530_global,
		.n_attr = ARRAY_SIZE(mt7530_global),
	},
	.attr_port = {
		.attr = mt7621_port,
		.n_attr = ARRAY_SIZE(mt7621_port),
	},
	.attr_vlan = {
		.attr = mt7530_vlan,
		.n_attr = ARRAY_SIZE(mt7530_vlan),
	},
	.get_vlan_ports = mt7530_get_vlan_ports,
	.set_vlan_ports = mt7530_set_vlan_ports,
	.get_port_pvid = mt7530_get_port_pvid,
	.set_port_pvid = mt7530_set_port_pvid,
	.get_port_link = mt7530_get_port_link,
	.apply_config = mt7530_apply_config,
	.reset_switch = mt7530_reset_switch,
};

static const struct switch_dev_ops mt7530_ops = {
	.attr_global = {
		.attr = mt7530_global,
		.n_attr = ARRAY_SIZE(mt7530_global),
	},
	.attr_port = {
		.attr = mt7530_port,
		.n_attr = ARRAY_SIZE(mt7530_port),
	},
	.attr_vlan = {
		.attr = mt7530_vlan,
		.n_attr = ARRAY_SIZE(mt7530_vlan),
	},
	.get_vlan_ports = mt7530_get_vlan_ports,
	.set_vlan_ports = mt7530_set_vlan_ports,
	.get_port_pvid = mt7530_get_port_pvid,
	.set_port_pvid = mt7530_set_port_pvid,
	.get_port_link = mt7530_get_port_link,
	.set_port_link = mt7530_set_port_link,
	.apply_config = mt7530_apply_config,
	.reset_switch = mt7530_reset_switch,
};

int
mt7530_probe(struct device *dev, void __iomem *base, struct mii_bus *bus, int vlan)
{
	struct switch_dev *swdev;
	struct mt7530_priv *mt7530;
	struct mt7530_mapping *map;
	int ret;

	mt7530 = devm_kzalloc(dev, sizeof(struct mt7530_priv), GFP_KERNEL);
	if (!mt7530)
		return -ENOMEM;

	mt7530->base = base;
	mt7530->bus = bus;
	mt7530->global_vlan_enable = vlan;

	swdev = &mt7530->swdev;
	if (bus) {
		swdev->alias = "mt7530";
		swdev->name = "mt7530";
	} else if (IS_ENABLED(CONFIG_SOC_MT7621)) {
		swdev->alias = "mt7621";
		swdev->name = "mt7621";
	} else {
		swdev->alias = "mt7620";
		swdev->name = "mt7620";
	}
	swdev->cpu_port = MT7530_CPU_PORT;
	swdev->ports = MT7530_NUM_PORTS;
	swdev->vlans = MT7530_NUM_VLANS;
	if (IS_ENABLED(CONFIG_SOC_MT7621))
		swdev->ops = &mt7621_ops;
	else
		swdev->ops = &mt7530_ops;

	ret = register_switch(swdev, NULL);
	if (ret) {
		dev_err(dev, "failed to register mt7530\n");
		return ret;
	}

	dev->platform_data = swdev;

	map = mt7530_find_mapping(dev->of_node);
	if (map)
		mt7530_apply_mapping(mt7530, map);
	mt7530_apply_config(swdev);

	/* magic vodoo */
	if (!IS_ENABLED(CONFIG_SOC_MT7621) && bus && mt7530_r32(mt7530, REG_HWTRAP) !=  0x1117edf) {
	        dev_info(dev, "fixing up MHWTRAP register - bootloader probably played with it\n");
		mt7530_w32(mt7530, REG_HWTRAP, 0x1117edf);
	}
	dev_info(dev, "loaded %s driver\n", swdev->name);

	return 0;
}
