/*
 * Marvell 88E61xx switch driver
 *
 * Copyright (c) 2014 Claudio Leite <leitec@staticky.com>
 * Copyright (c) 2014 Nikita Nazarenko <nnazarenko@radiofid.com>
 *
 * Based on code (c) 2008 Felix Fietkau <nbd@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/delay.h>
#include <linux/switch.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "mvsw61xx.h"

extern u64 uevent_next_seqnum(void);

MODULE_DESCRIPTION("Marvell 88E61xx Switch driver");
MODULE_AUTHOR("Claudio Leite <leitec@staticky.com>");
MODULE_AUTHOR("Nikita Nazarenko <nnazarenko@radiofid.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mvsw61xx");

/*
 * Register access is done through direct or indirect addressing,
 * depending on how the switch is physically connected.
 *
 * Direct addressing: all port and global registers directly
 *   accessible via an address/register pair
 *
 * Indirect addressing: switch is mapped at a single address,
 *   port and global registers accessible via a single command/data
 *   register pair
 */

static int
mvsw61xx_wait_mask_raw(struct mii_bus *bus, int addr,
		int reg, u16 mask, u16 val)
{
	int i = 100;
	u16 r;

	do {
		r = bus->read(bus, addr, reg);
		if ((r & mask) == val)
			return 0;
	} while (--i > 0);

	return -ETIMEDOUT;
}

static u16
r16(struct mii_bus *bus, bool indirect, int base_addr, int addr, int reg)
{
	u16 ind_addr;

	if (!indirect)
		return bus->read(bus, addr, reg);

	/* Indirect read: First, make sure switch is free */
	mvsw61xx_wait_mask_raw(bus, base_addr, MV_INDIRECT_REG_CMD,
			MV_INDIRECT_INPROGRESS, 0);

	/* Load address and request read */
	ind_addr = MV_INDIRECT_READ | (addr << MV_INDIRECT_ADDR_S) | reg;
	bus->write(bus, base_addr, MV_INDIRECT_REG_CMD,
			ind_addr);

	/* Wait until it's ready */
	mvsw61xx_wait_mask_raw(bus, base_addr, MV_INDIRECT_REG_CMD,
			MV_INDIRECT_INPROGRESS, 0);

	/* Read the requested data */
	return bus->read(bus, base_addr, MV_INDIRECT_REG_DATA);
}

static void
w16(struct mii_bus *bus, bool indirect, int base_addr, int addr,
		int reg, u16 val)
{
	u16 ind_addr;

	if (!indirect) {
		bus->write(bus, addr, reg, val);
		return;
	}

	/* Indirect write: First, make sure switch is free */
	mvsw61xx_wait_mask_raw(bus, base_addr, MV_INDIRECT_REG_CMD,
			MV_INDIRECT_INPROGRESS, 0);

	/* Load the data to be written */
	bus->write(bus, base_addr, MV_INDIRECT_REG_DATA, val);

	/* Wait again for switch to be free */
	mvsw61xx_wait_mask_raw(bus, base_addr, MV_INDIRECT_REG_CMD,
			MV_INDIRECT_INPROGRESS, 0);

	/* Load address, and issue write command */
	ind_addr = MV_INDIRECT_WRITE | (addr << MV_INDIRECT_ADDR_S) | reg;
	bus->write(bus, base_addr, MV_INDIRECT_REG_CMD,
			ind_addr);
}

/* swconfig support */

static inline u16
sr16(struct switch_dev *dev, int addr, int reg)
{
	struct mvsw61xx_state *state = get_state(dev);

	return r16(state->bus, state->is_indirect, state->base_addr, addr, reg);
}

static inline void
sw16(struct switch_dev *dev, int addr, int reg, u16 val)
{
	struct mvsw61xx_state *state = get_state(dev);

	w16(state->bus, state->is_indirect, state->base_addr, addr, reg, val);
}

static int
mvsw61xx_wait_mask_s(struct switch_dev *dev, int addr,
		int reg, u16 mask, u16 val)
{
	int i = 100;
	u16 r;

	do {
		r = sr16(dev, addr, reg) & mask;
		if (r == val)
			return 0;
	} while (--i > 0);

	return -ETIMEDOUT;
}

int mvsw61xx_phy_read16(struct switch_dev *dev, int addr, u8 reg, u16 *value) {
	uint16_t cmd;

	if( mvsw61xx_wait_mask_s(dev, MV_GLOBAL2REG(SMI_PHY_CMD), 0x8000, 0) ){	return -ETIMEDOUT; }

	cmd = (1 << 15) | (1 << 12) | (0x2 << 10) | (addr << 5) | reg;
	sw16(dev, MV_GLOBAL2REG(SMI_PHY_CMD), cmd);
	if( mvsw61xx_wait_mask_s(dev, MV_GLOBAL2REG(SMI_PHY_CMD), 0x8000, 0) ){	return -ETIMEDOUT; }
	*value = sr16(dev, MV_GLOBAL2REG(SMI_PHY_DATA));

	return 0;
}

int mvsw61xx_phy_write16(struct switch_dev *dev, int addr, u8 reg, u16 value) {
	uint16_t cmd;
	if( mvsw61xx_wait_mask_s(dev, MV_GLOBAL2REG(SMI_PHY_CMD), 0x8000, 0) ){	return -ETIMEDOUT; }

	cmd = (1 << 15) | (1 << 12) | (0x1 << 10) | (addr << 5) | reg;
	sw16(dev, MV_GLOBAL2REG(SMI_PHY_DATA), value);
	sw16(dev, MV_GLOBAL2REG(SMI_PHY_CMD), cmd);

	if( mvsw61xx_wait_mask_s(dev, MV_GLOBAL2REG(SMI_PHY_CMD), 0x8000, 0) ){	return -ETIMEDOUT; }

	return 0;
}

static struct vlan_state* mvsw61xx_get_vlan_by_vid(struct switch_dev *dev, u16 vid) {
	int i;
	struct mvsw61xx_state *state = get_state(dev);
	struct vlan_state* result = NULL;

	for( i=1; i < state->last_vlan; i++ ) {
		if( state->vlans[i].vid == vid ){
			result = &state->vlans[i];
			break;
		}
	}

	return result;
}

static int
mvsw61xx_get_port_mask(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	char *buf = state->buf;
	int port, len, i;
	u16 reg;

	port = val->port_vlan;
	reg = sr16(dev, MV_PORTREG(VLANMAP, port)) & MV_PORTS_MASK;

	len = sprintf(buf, "0x%04x: ", reg);

	for (i = 0; i < MV_PORTS; i++) {
		if (reg & (1 << i))
			len += sprintf(buf + len, "%d ", i);
		else if (i == port)
			len += sprintf(buf + len, "(%d) ", i);
	}

	val->value.s = buf;

	return 0;
}

static int
mvsw61xx_get_port_qmode(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);

	val->value.i = state->ports[val->port_vlan].qmode;

	return 0;
}

static int
mvsw61xx_set_port_qmode(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);

	state->ports[val->port_vlan].qmode = val->value.i;

	return 0;
}

static int
mvsw61xx_get_port_pvid(struct switch_dev *dev, int port, int *val)
{
	struct mvsw61xx_state *state = get_state(dev);

	*val = state->ports[port].pvid;

	return 0;
}

static int
mvsw61xx_set_port_pvid(struct switch_dev *dev, int port, int val)
{
	struct mvsw61xx_state *state = get_state(dev);

	if (val < 0 || val >= MV_MAX_VLAN)
		return -EINVAL;

	state->ports[port].pvid = (u16)val;

	return 0;
}


static int
mvsw61xx_get_port_link(struct switch_dev *dev, int port,
		struct switch_port_link *link)
{
	u16 status, speed;

	status = sr16(dev, MV_PORTREG(STATUS, port));

	link->link = status & MV_PORT_STATUS_LINK;
	if (!link->link)
		return 0;

	link->duplex = status & MV_PORT_STATUS_FDX;

	speed = (status & MV_PORT_STATUS_SPEED_MASK) >>
			MV_PORT_STATUS_SPEED_SHIFT;

	switch (speed) {
	case MV_PORT_STATUS_SPEED_10:
		link->speed = SWITCH_PORT_SPEED_10;
		break;
	case MV_PORT_STATUS_SPEED_100:
		link->speed = SWITCH_PORT_SPEED_100;
		break;
	case MV_PORT_STATUS_SPEED_1000:
		link->speed = SWITCH_PORT_SPEED_1000;
		break;
	}

	return 0;
}

static int
mvsw61xx_set_port_link(struct switch_dev *dev, int port,
		struct switch_port_link *link)
{
	u16 reg,ctl,status, state, anar, ccr, gcr;
	reg = sr16(dev, MV_PORTREG(PHYCTL, port));

	mvsw61xx_phy_read16 (dev, port, 0x04, &anar);
	anar &= 0xfc1f;
	mvsw61xx_phy_read16(dev, port, 0x00, &ccr);
	mvsw61xx_phy_read16(dev, port, 0x09, &gcr);
	reg &= (0xc000); // clear all except rgmii timings

	uint16_t spd = 0;
	uint16_t spdg = 0;

	// speed not set, use auto
	if ( link->speed == SWITCH_PORT_SPEED_UNKNOWN ) {
		reg |= MV_PORT_STATUS_SPEED_AUTO;
		reg &= ~(1 << 2);
		reg &= ~(1 << 4);
		reg &= ~(1 << 6);
		gcr |= (0x3 << 8);
		spd = 0xF << 5;
	} else { // force
		if( link->speed == SWITCH_PORT_SPEED_10 ) {
			// set speed 10Mbps
			// set duplex
			// set autonegation settings
			spd = 1 << 5;
			gcr &= ~(0x3 << 8);
		} else if ( link->speed == SWITCH_PORT_SPEED_100 ) {
			reg |= 0x1;
			spd = 1 << 7;
			gcr &= ~(0x3 << 8);
		} else if (link->speed == SWITCH_PORT_SPEED_1000 ) {
			reg |= 0x2;
			if( link->duplex){
				gcr &= ~(1 << 8);
				gcr |= 1 << 9;
			} else {
				gcr |= 1 << 8;
				gcr &= ~(1 << 9);
			}

		} else { // unknown speed
			return -ENOTSUPP;
		}

		if (link->duplex) {
			reg |= (1 << 3);
			spd = spd << 1; // copper duplex
		} else {
			reg &= ~(1 << 3);
		}
		reg |= (1 << 2);					// force duplex
	}
	anar |= spd;
	ccr |= 1 << 9;

	status = sr16(dev, MV_PORTREG(CONTROL, port));
	state = status & 0x3;
	status &= ~(0x3);
	reg &= ~(1 << 4);
	sw16(dev, MV_PORTREG(CONTROL, port), status);	// disable port
	sw16(dev, MV_PORTREG(PHYCTL, port), reg);		// set port link
	status |= state;
	sw16(dev, MV_PORTREG(CONTROL, port), status);	// enable port
	mvsw61xx_phy_write16(dev, port, 0x4, anar);
	mvsw61xx_phy_write16(dev, port, 0x9, gcr);
	// reset autonegation sequence
	mvsw61xx_phy_write16(dev, port, 0x0, ccr);

	return 0;
}

static int
mvsw61xx_get_force_link(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 status;

	status = sr16(dev, MV_PORTREG(PHYCTL, val->port_vlan));
	val->value.i = (status & (1 << 4)) ? 1 : 0;

	return 0;
}

static int
mvsw61xx_get_port_status(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 status;

	status = sr16(dev, MV_PORTREG(STATUS, val->port_vlan));
	val->value.i = status;

	return 0;
}

static int
mvsw61xx_set_force_link(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 reg;
	reg = sr16(dev, MV_PORTREG(PHYCTL, val->port_vlan));

	if( val->value.i ) {
		reg |= 1 << 4;
		reg |= 1 << 5;
	} else {
		reg &= ~(1 << 4);
		reg &= ~(1 << 5);
	}

	sw16(dev, MV_PORTREG(PHYCTL, val->port_vlan), reg);
	return 0;
}



static int
mvsw61xx_get_port_phydet(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 status;

	status = sr16(dev, MV_PORTREG(STATUS, val->port_vlan));
	val->value.i = status & (1 << 12) ? 1 : 0;

	return 0;
}

static int
mvsw61xx_set_port_phydet(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 reg;
	reg = sr16(dev, MV_PORTREG(STATUS, val->port_vlan));

	if( val->value.i ) {
		reg |= 1 << 12;
	} else {
		reg &= ~(1 << 12);
	}

	sw16(dev, MV_PORTREG(STATUS, val->port_vlan), reg);
	return 0;
}


static int
mvsw61xx_get_fiber_control(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 reg, page;

	// set 1 page for access to fiber registers
	mvsw61xx_phy_read16(dev, 0xF, 22, &page);
	mvsw61xx_phy_write16(dev, 0xF, 22, 1 );
	

	mvsw61xx_phy_read16(dev, 0xF, 0, &reg);

	val->value.i = reg;

	mvsw61xx_phy_write16(dev, 0xF, 22, page);

	return 0;
}

static int
mvsw61xx_set_fiber_control(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 reg, page;

	// set 1 page for access to fiber registers
	mvsw61xx_phy_read16(dev, 0xF, 22, &page);
	mvsw61xx_phy_write16(dev, 0xF, 22, 1 );
	

	mvsw61xx_phy_write16(dev, 0xF, 0, val->value.i);

	mvsw61xx_phy_write16(dev, 0xF, 22, page);

	return 0;
}


static int
mvsw61xx_get_fiber_power(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 reg, page;

	// set 1 page for access to fiber registers
	mvsw61xx_phy_read16(dev, 0xF, 22, &page);
	mvsw61xx_phy_write16(dev, 0xF, 22, 1 );
	

	mvsw61xx_phy_read16(dev, 0xF, 0, &reg);

	val->value.i = (reg & (1 << 11)) ? 0 : 1;

	mvsw61xx_phy_write16(dev, 0xF, 22, page);

	return 0;
}

static int
mvsw61xx_set_fiber_power(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 reg, page;

	// set 1 page for access to fiber registers
	mvsw61xx_phy_read16(dev, 0xF, 22, &page);
	mvsw61xx_phy_write16(dev, 0xF, 22, 1 );
	
	mvsw61xx_phy_read16(dev, 0xF, 0, &reg);
	if( val->value.i ) {
		reg &= ~(1 << 11);
	} else {
		reg |= 1 << 11;
	}
	mvsw61xx_phy_write16(dev, 0xF, 0, reg);

	mvsw61xx_phy_write16(dev, 0xF, 22, page);

	return 0;
}

static int
mvsw61xx_get_fiber_status(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 reg, page;

	// set 1 page for access to fiber registers
	mvsw61xx_phy_read16(dev, 0xF, 22, &page);
	mvsw61xx_phy_write16(dev, 0xF, 22, 1 );
	

	mvsw61xx_phy_read16(dev, 0xF, 1, &reg);

	val->value.i = reg;

	mvsw61xx_phy_write16(dev, 0xF, 22, page);

	return 0;
}

static int
mvsw61xx_get_fiber_regs(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16  page;
	u16 reg;
	int i;
	struct mvsw61xx_state *state = get_state(dev);
	size_t len = 0;
	// set 1 page for access to fiber registers
	mvsw61xx_phy_read16(dev, 0xF, 22, &page);
	mvsw61xx_phy_write16(dev, 0xF, 22, 1 );
	
	len = snprintf(state->buf, state->buf_size, "\n00   01   02   03   04   05   06   07   08   09   0A   0B   0C   0D   0E   0F   10   11   12   13   14   15   16   17   18   19   1A   1B   1C   1D   1E   1F  \n");
	
	for( i=0; i < 32; i++ ){	
		mvsw61xx_phy_read16(dev, 0xF, i, &reg);
		len += snprintf( state->buf + len, state->buf_size - len, "%04X ", reg);
	}

	val->value.s = state->buf;
	val->len = len;

	mvsw61xx_phy_write16(dev, val->port_vlan, 22, page);

	return 0;
}

static int
mvsw61xx_set_fiber_status(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 reg, page;

	// set 1 page for access to fiber registers
	mvsw61xx_phy_read16(dev, 0xF, 22, &page);
	mvsw61xx_phy_write16(dev, 0xF, 22, 1);
	

	mvsw61xx_phy_write16(dev, 0xF, 1, val->value.i);

	mvsw61xx_phy_write16(dev, 0xF, 22, page);

	return 0;
}


static int mvsw61xx_get_vlan_ports(struct switch_dev *dev,
		struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	struct vlan_state* v;
	int i, j, mode, vno;

	vno = val->port_vlan;

	if (vno <= 0 || vno >= MV_MAX_VLAN)
		return -EINVAL;

	v = mvsw61xx_get_vlan_by_vid(dev, vno);
	if( v == NULL )
		return -ENOENT;

	for (i = 0, j = 0; i < dev->ports; i++) {
		if (v->mask & (1 << i)) {
			val->value.ports[j].id = i;

			mode = (v->port_mode >> (i * 4)) & 0xf;
			if (mode == MV_VTUCTL_EGRESS_TAGGED)
				val->value.ports[j].flags =
					(1 << SWITCH_PORT_FLAG_TAGGED);
			else
				val->value.ports[j].flags = 0;

			j++;
		}
	}

	val->len = j;

	return 0;
}

static int mvsw61xx_set_vlan_ports(struct switch_dev *dev,
		struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	int i, mode, pno, vno;
	struct vlan_state* v;
	vno = val->port_vlan;

	if (vno <= 0 || vno >= MV_MAX_VLAN)
		return -EINVAL;

	v = mvsw61xx_get_vlan_by_vid(dev, vno);
	if( v == NULL ) { // set new vlan
		if( state->last_vlan >= MV_VLANS ){
			pr_err("VLAN table is full\n");
			return -EINVAL;
		}
		v = &state->vlans[state->last_vlan];
		state->last_vlan++;
	}

	v->mask = 0;
	v->port_mode = 0;
	v->port_sstate = 0;

	v->vid = vno;

	for (i = 0; i < val->len; i++) {
		pno = val->value.ports[i].id;

		v->mask |= (1 << pno);
		if (val->value.ports[i].flags &
				(1 << SWITCH_PORT_FLAG_TAGGED))
			mode = MV_VTUCTL_EGRESS_TAGGED;
		else
			mode = MV_VTUCTL_EGRESS_UNTAGGED;

		v->port_mode |= mode << (pno * 4);
		v->port_sstate |=
			MV_STUCTL_STATE_FORWARDING << (pno * 4 + 2);
	}

	/*
	 * DISCARD is nonzero, so it must be explicitly
	 * set on ports not in the VLAN.
	 */
	for (i = 0; i < dev->ports; i++)
		if (!(v->mask & (1 << i)))
			v->port_mode |=
				MV_VTUCTL_DISCARD << (i * 4);

	return 0;
}

static int mvsw61xx_get_vlan_port_based(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	struct vlan_state* v;
	int vno = val->port_vlan;

	if (vno <= 0 || vno >= MV_MAX_VLAN)
		return -EINVAL;

	v = mvsw61xx_get_vlan_by_vid(dev, vno);
	if( v == NULL )
		return -ENOENT;

	if (v->port_based)
		val->value.i = 1;
	else
		val->value.i = 0;

	return 0;
}

static int mvsw61xx_set_vlan_port_based(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	struct vlan_state* v;
	int vno = val->port_vlan;

	if (vno <= 0 || vno >= MV_MAX_VLAN)
		return -EINVAL;

	v = mvsw61xx_get_vlan_by_vid(dev, vno);
	if( v == NULL )
		return -ENOENT;

	v = mvsw61xx_get_vlan_by_vid(dev, vno);
	if( v == NULL )
		return -ENOENT;

	if (val->value.i == 1)
		v->port_based = true;
	else
		v->port_based = false;

	return 0;
}

static int mvsw61xx_get_vid(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	struct vlan_state* v;
	int vno = val->port_vlan;

	if (vno <= 0 || vno >= MV_MAX_VLAN)
		return -EINVAL;

	v = mvsw61xx_get_vlan_by_vid(dev, vno);
	if( v == NULL )
		return -ENOENT;

	val->value.i = v->vid;

	return 0;
}

static int mvsw61xx_set_vid(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	int vno = val->port_vlan;
	struct vlan_state* v;
	if (vno <= 0 || vno >= MV_MAX_VLAN)
		return -EINVAL;

	v = mvsw61xx_get_vlan_by_vid(dev, vno);
	if( v == NULL )
		return -ENOENT;

	v->vid = val->value.i;

	return 0;
}

static int mvsw61xx_get_enable_vlan(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);

	val->value.i = state->vlan_enabled;

	return 0;
}

static int mvsw61xx_set_enable_vlan(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);

	state->vlan_enabled = val->value.i;

	return 0;
}

static int mvsw61xx_vtu_program(struct switch_dev *dev)
{
	struct mvsw61xx_state *state = get_state(dev);
	u16 v1, v2, s1, s2;
	int i;

	/* Flush */
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
			MV_VTUOP_INPROGRESS, 0);
	sw16(dev, MV_GLOBALREG(VTU_OP),
			MV_VTUOP_INPROGRESS | MV_VTUOP_PURGE);

	sw16(dev, MV_GLOBALREG(VTU_VID), MV_VTU_VID_VALID);
	sw16(dev, MV_GLOBALREG(VTU_DATA1), 0xCCCC);
	sw16(dev, MV_GLOBALREG(VTU_DATA2), 0xCCCC);
	sw16(dev, MV_GLOBALREG(VTU_DATA3), 0);
	sw16(dev, MV_GLOBALREG(VTU_FID), 0);
	sw16(dev, MV_GLOBALREG(VTU_SID), 0);
	sw16(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS | MV_VTUOP_STULOAD);
		mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS, 0);
	
	/* Write VLAN table */
	pr_info("apply vlan settings. last_vlan: %d\n", state->last_vlan);
	for (i = 1; i < state->last_vlan; i++) {
		pr_info("apply vlan %d vid: %d port_sstate: 0x%07X port_mode: 0x%07X\n", i, state->vlans[i].vid, state->vlans[i].port_sstate, state->vlans[i].port_mode );
		if (state->vlans[i].mask == 0 ||
				state->vlans[i].vid == 0 ||
				state->vlans[i].port_based == true)
			continue;

		mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS, 0);

		/* Write per-VLAN port state into STU */
		s1 = (u16) (state->vlans[i].port_sstate & 0xffff);
		s2 = (u16) ((state->vlans[i].port_sstate >> 16) & 0xffff);

		sw16(dev, MV_GLOBALREG(VTU_VID), MV_VTU_VID_VALID);
		sw16(dev, MV_GLOBALREG(VTU_SID), i);
		sw16(dev, MV_GLOBALREG(VTU_DATA1), s1);
		sw16(dev, MV_GLOBALREG(VTU_DATA2), s2);
		sw16(dev, MV_GLOBALREG(VTU_DATA3), 0);

		sw16(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS | MV_VTUOP_STULOAD);
		mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS, 0);

		/* Write VLAN information into VTU */
		v1 = (u16) (state->vlans[i].port_mode & 0xffff);
		v2 = (u16) ((state->vlans[i].port_mode >> 16) & 0xffff);

		sw16(dev, MV_GLOBALREG(VTU_VID),
				MV_VTU_VID_VALID | state->vlans[i].vid);
		sw16(dev, MV_GLOBALREG(VTU_SID), i);
//		sw16(dev, MV_GLOBALREG(VTU_FID), 0);
		sw16(dev, MV_GLOBALREG(VTU_FID), state->vlans[i].vid & 0xFFF);
		sw16(dev, MV_GLOBALREG(VTU_DATA1), v1);
		sw16(dev, MV_GLOBALREG(VTU_DATA2), v2);
		sw16(dev, MV_GLOBALREG(VTU_DATA3), 0);

		sw16(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS | MV_VTUOP_LOAD);
		mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS, 0);
		
	}

	return 0;
}

static void mvsw61xx_vlan_port_config(struct switch_dev *dev, struct vlan_state *v)
{
	struct mvsw61xx_state *state = get_state(dev);
	int i, mode;

	if( v == NULL )
		return;

	for (i = 0; i < dev->ports; i++) {
		if (!(v->mask & (1 << i))){
			printk(KERN_INFO "vid: %d, skip port %d\n", v->vid, i);
			continue;
		}

		mode = (v->port_mode >> (i * 4)) & 0xf;

		if(mode != MV_VTUCTL_EGRESS_TAGGED)
			state->ports[i].pvid = v->vid;

		if (v->port_based) {
			state->ports[i].mask |= v->mask;
			state->ports[i].fdb = v->vid;
		}
		else
			state->ports[i].qmode = MV_8021Q_MODE_SECURE;
	}
}

static int mvsw61xx_update_state(struct switch_dev *dev)
{
	struct mvsw61xx_state *state = get_state(dev);
	int i;
	u16 reg;

	if (!state->registered)
		return -EINVAL;

	/*
	 * Set 802.1q-only mode if vlan_enabled is true.
	 *
	 * Without this, even if 802.1q is enabled for
	 * a port/VLAN, it still depends on the port-based
	 * VLAN mask being set.
	 *
	 * With this setting, port-based VLANs are still
	 * functional, provided the VID is not in the VTU.
	 */
	reg = sr16(dev, MV_GLOBAL2REG(SDET_POLARITY));

	if (state->vlan_enabled)
		reg |= MV_8021Q_VLAN_ONLY;
	else
		reg &= ~MV_8021Q_VLAN_ONLY;

	sw16(dev, MV_GLOBAL2REG(SDET_POLARITY), reg);

	/*
	 * Set port-based VLAN masks on each port
	 * based only on VLAN definitions known to
	 * the driver (i.e. in state).
	 *
	 * This means any pre-existing port mapping is
	 * wiped out once our driver is initialized.
	 */
	for (i = 0; i < dev->ports; i++) {
		state->ports[i].mask = 0;
		state->ports[i].qmode = MV_8021Q_MODE_DISABLE;
	}

	for (i = 1; i < state->last_vlan; i++)
		mvsw61xx_vlan_port_config(dev, &state->vlans[i] );

	for (i = 0; i < dev->ports; i++) {
		reg = sr16(dev, MV_PORTREG(VLANID, i)) & ~MV_PVID_MASK;
		reg |= state->ports[i].pvid;
		sw16(dev, MV_PORTREG(VLANID, i), reg);

		state->ports[i].mask &= ~(1 << i);

		/* set default forwarding DB number and port mask */
		reg = sr16(dev, MV_PORTREG(CONTROL1, i)) & ~MV_FDB_HI_MASK;
		reg |= (state->ports[i].fdb >> MV_FDB_HI_SHIFT) &
			MV_FDB_HI_MASK;
		sw16(dev, MV_PORTREG(CONTROL1, i), reg);

		reg = ((state->ports[i].fdb & 0xf) << MV_FDB_LO_SHIFT) |
			state->ports[i].mask;
		sw16(dev, MV_PORTREG(VLANMAP, i), reg);

		reg = sr16(dev, MV_PORTREG(CONTROL2, i)) &
			~MV_8021Q_MODE_MASK;
		reg |= state->ports[i].qmode << MV_8021Q_MODE_SHIFT;
		sw16(dev, MV_PORTREG(CONTROL2, i), reg);
	}

	mvsw61xx_vtu_program(dev);

	return 0;
}

static int mvsw61xx_apply(struct switch_dev *dev)
{
	return mvsw61xx_update_state(dev);
}

static int mvsw61xx_reset(struct switch_dev *dev)
{
	struct mvsw61xx_state *state = get_state(dev);
	int i;
	u16 reg;

	/* Disable all ports before reset */
	for (i = 0; i < dev->ports; i++) {
		reg = sr16(dev, MV_PORTREG(CONTROL, i)) &
			~MV_PORTCTRL_FORWARDING;
		sw16(dev, MV_PORTREG(CONTROL, i), reg);
	}

	reg = sr16(dev, MV_GLOBALREG(CONTROL)) | MV_CONTROL_RESET;

	sw16(dev, MV_GLOBALREG(CONTROL), reg);
	if (mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(CONTROL),
				MV_CONTROL_RESET, 0) < 0)
		return -ETIMEDOUT;

	for (i = 0; i < dev->ports; i++) {
		state->ports[i].fdb = 0;
		state->ports[i].qmode = 0;
		state->ports[i].mask = 0;
		state->ports[i].pvid = 0;

		/* Force flow control off */
		reg = sr16(dev, MV_PORTREG(PHYCTL, i)) & ~MV_PHYCTL_FC_MASK;
		reg |= MV_PHYCTL_FC_DISABLE;
		sw16(dev, MV_PORTREG(PHYCTL, i), reg);

		/* Set port association vector */
		sw16(dev, MV_PORTREG(ASSOC, i), (1 << i));
	}

	for (i = 0; i < MV_VLANS; i++) {
		state->vlans[i].port_based = false;
		state->vlans[i].mask = 0;
		state->vlans[i].vid = 0;
		state->vlans[i].port_mode = 0;
		state->vlans[i].port_sstate = 0;
	}
	state->last_vlan = 1;
	state->vlan_enabled = 0;

	mvsw61xx_update_state(dev);

	/* Re-enable ports */
	for (i = 0; i < dev->ports; i++) {
		reg = sr16(dev, MV_PORTREG(CONTROL, i)) |
			MV_PORTCTRL_FORWARDING;
		sw16(dev, MV_PORTREG(CONTROL, i), reg);
	}

	return 0;
}

static int mvsw61xx_get_vtu_base(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 vop, vid, vd1, vd2, vd3, vfid, vsid;
	u32 ports, ret;
	size_t len = 0;
	struct mvsw61xx_state *state = get_state(dev);
	memset(state->buf, 0, state->buf_size);	
	
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS, 0);
	
	sw16(dev, MV_GLOBALREG(VTU_OP), 0);
	sw16(dev, MV_GLOBALREG(VTU_VID), 0);
	sw16(dev, MV_GLOBALREG(VTU_DATA1), 0);
	sw16(dev, MV_GLOBALREG(VTU_DATA2), 0);
	sw16(dev, MV_GLOBALREG(VTU_DATA3), 0);
	sw16(dev, MV_GLOBALREG(VTU_FID), 0);
	sw16(dev, MV_GLOBALREG(VTU_SID), 0);

	sw16(dev, MV_GLOBALREG(VTU_VID),
				0xFFF);
	val->len = 0;
	do {
		sw16(dev, MV_GLOBALREG(VTU_OP),
					MV_VTUOP_INPROGRESS | MV_VTUOP_VTU_GET_NEXT);
		ret = mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
					MV_VTUOP_INPROGRESS, 0);
		vop = sr16(dev, MV_GLOBALREG(VTU_OP));
		vid = sr16(dev, MV_GLOBALREG(VTU_VID));
		vd1 = sr16(dev, MV_GLOBALREG(VTU_DATA1));
		vd2 = sr16(dev, MV_GLOBALREG(VTU_DATA2));
		vd3 = sr16(dev, MV_GLOBALREG(VTU_DATA3));
		vfid = sr16(dev, MV_GLOBALREG(VTU_FID));
		vsid = sr16(dev, MV_GLOBALREG(VTU_SID));
		ports = (vd2 << 16) + vd1;
		printk(KERN_INFO "vid: %d, fid: %d, sid: %d,  dataregs: 0x%x\n"
				, (vid & 0xFFF), vfid, vsid, ports);
		if( (vid & MV_VTU_VID_VALID) && (state->buf_size - len > 0)) {
			len += snprintf(state->buf + len, (state->buf_size - len), 
				"vid: %d, fid: %d, sid: %d,  dataregs: 0x%X\n"
				, (vid & 0xFFF), vfid, vsid, ports);
		}
	} while( vid & MV_VTU_VID_VALID && ret == 0);

	val->value.s = state->buf;
	val->len = len;
	return 0;
}

static int mvsw61xx_get_stu_base(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 vop, vid, vd1, vd2, vd3, vfid, vsid;
	u32 ports, ret;
	size_t len = 0;
	struct mvsw61xx_state *state = get_state(dev);
	memset(state->buf, 0, state->buf_size);	
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS, 0);

	sw16(dev, MV_GLOBALREG(VTU_OP), 0);
	sw16(dev, MV_GLOBALREG(VTU_VID), 0);
	sw16(dev, MV_GLOBALREG(VTU_DATA1), 0);
	sw16(dev, MV_GLOBALREG(VTU_DATA2), 0);
	sw16(dev, MV_GLOBALREG(VTU_DATA3), 0);
	sw16(dev, MV_GLOBALREG(VTU_FID), 0);
	sw16(dev, MV_GLOBALREG(VTU_SID), 0);
	sw16(dev, MV_GLOBALREG(VTU_SID),
				0x3F);
	val->len = 0;
	do {
		sw16(dev, MV_GLOBALREG(VTU_OP),
					MV_VTUOP_INPROGRESS | MV_VTUOP_STU_GET_NEXT);
		ret = mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
					MV_VTUOP_INPROGRESS, 0);
		vop = sr16(dev, MV_GLOBALREG(VTU_OP));
		vid = sr16(dev, MV_GLOBALREG(VTU_VID));
		vd1 = sr16(dev, MV_GLOBALREG(VTU_DATA1));
		vd2 = sr16(dev, MV_GLOBALREG(VTU_DATA2));
		vd3 = sr16(dev, MV_GLOBALREG(VTU_DATA3));
		vfid = sr16(dev, MV_GLOBALREG(VTU_FID));
		vsid = sr16(dev, MV_GLOBALREG(VTU_SID));
		ports = (vd2 << 16) + vd1;
		printk(KERN_INFO "sid: %d, dataregs: 0x%x\n"
				, vsid, ports);
		if( (vid & MV_VTU_VID_VALID) && (state->buf_size - len > 0)) {
			len += snprintf(state->buf + len, (state->buf_size - len), 
				"sid: %d,  dataregs: 0x%X\n"
				, vsid, ports);
		}
	} while( vid & MV_VTU_VID_VALID && ret == 0);

	val->value.s = state->buf;
	val->len = len;
	return 0;
}
/*
static int mvsw61xx_get_atu_base(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 vop, vid, vd1, vd2, vd3, vfid, vsid;
	u32 ports, ret;
	size_t len = 0;
	struct mvsw61xx_state *state = get_state(dev);
	
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS, 0);

	sw16(dev, MV_GLOBALREG(VTU_VID),
				0xFFF);
	val->len = 0;
	do {
		sw16(dev, MV_GLOBALREG(VTU_OP),
					MV_VTUOP_INPROGRESS | MV_VTUOP_VTU_GET_NEXT);
		ret = mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
					MV_VTUOP_INPROGRESS, 0);
		vop = sr16(dev, MV_GLOBALREG(VTU_OP));
		vid = sr16(dev, MV_GLOBALREG(VTU_VID));
		vd1 = sr16(dev, MV_GLOBALREG(VTU_DATA1));
		vd2 = sr16(dev, MV_GLOBALREG(VTU_DATA2));
		vd3 = sr16(dev, MV_GLOBALREG(VTU_DATA3));
		vfid = sr16(dev, MV_GLOBALREG(VTU_FID));
		vsid = sr16(dev, MV_GLOBALREG(VTU_SID));
		ports = (vd2 << 16) + vd1;
		printk(KERN_INFO "vid: %d, fid: %d, sid: %d,  dataregs: 0x%x\n"
				, (vid & 0xFFF), vfid, vsid, ports);
		if( (vid & MV_VTU_VID_VALID) && (state->buf_size - len > 0)) {
			len += snprintf(state->buf + len, (state->buf_size - len), 
				"vid: %d, fid: %d, sid: %d,  dataregs: 0x%X\n"
				, (vid & 0xFFF), vfid, vsid, ports);
		}
	} while( vid & MV_VTU_VID_VALID && ret == 0);

	val->value.s = state->buf;
	val->len = len;
	return 0;
}
*/

static int mvsw61xx_get_vtu_violation(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	u16 vop, vid;	
	struct mvsw61xx_state *state = get_state(dev);
	
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS, 0);

	sw16(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS | MV_VTUOP_GET_VIOLATION);
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(VTU_OP),
				MV_VTUOP_INPROGRESS, 0);

	vop = sr16(dev, MV_GLOBALREG(VTU_OP));
	vid = sr16(dev, MV_GLOBALREG(VTU_VID));

	if( vop & (3 << 5) ) {
		if( vop & (1 << 5)) {
			val->len = snprintf(state->buf, state->buf_size, "VTU miss: vid %d, port %d\n", (vid & 0xfff), vop & 0xF);
		} else if ( vop & (1 << 6) ) {
			val->len = snprintf(state->buf, state->buf_size, "Member violation: vid %d, port %d\n", (vid & 0xfff), vop & 0xF);
		}
	} else {
		val->len = snprintf(state->buf, state->buf_size, "No violation");
	}
	val->value.s = state->buf;
	return 0;
}

static int mvsw61xx_get_regvalue(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	uint8_t reg = state->ports[val->port_vlan].reg;
	uint16_t result;
    mvsw61xx_phy_read16(dev, val->port_vlan, reg, &result);

	val->len = snprintf(state->buf, state->buf_size, "%d | 0x%X", reg, result);
	val->value.s = state->buf;

	return 0;
}

static int mvsw61xx_set_regvalue(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	int port = val->port_vlan;

	uint8_t reg;
	uint16_t value;

	if( strchr( val->value.s, ' ' ) == NULL ){ // have parameter
		sscanf(val->value.s, "%hhd", &reg);
		state->ports[port].reg = reg;
	} else {
		sscanf(val->value.s, "%hhd %hd", &reg, &value);
		state->ports[port].reg = reg;
		mvsw61xx_phy_write16(dev, port, reg, value);
	}

	return 0;
}

static int mvsw61xx_get_fiber_regvalue(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	uint8_t reg = state->reg;
	uint16_t result;
    mvsw61xx_phy_read16(dev, 0xF, reg, &result);

	val->len = snprintf(state->buf, state->buf_size, "%d | 0x%X", reg, result);
	val->value.s = state->buf;

	return 0;
}

static int mvsw61xx_set_fiber_regvalue(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);

	uint8_t reg;
	uint16_t value;

	if( strchr( val->value.s, ' ' ) == NULL ){ // have parameter
		sscanf(val->value.s, "%hhd", &reg);
		state->reg = reg;
	} else {
		sscanf(val->value.s, "%hhd %hd", &reg, &value);
		state->reg = reg;
		mvsw61xx_phy_write16(dev, 0xF, reg, value);
	}

	return 0;
}
enum {
	MVSW61XX_ENABLE_VLAN,
	MVSW61XX_LAN_DIODE,
	MVSW61XX_FIBER_SHOW,
	MVSW61XX_FIBER_CONTROL,
	MVSW61XX_FIBER_STATUS,
	MVSW61XX_FIBER_REGVALUE,
	MVSW61XX_FIBER_POWER,
	MVSW61XX_VTU_VIOLATION,
	MVSW61XX_VTU_BASE,
	MVSW61XX_STU_BASE,
};

enum {
	MVSW61XX_VLAN_PORT_BASED,
	MVSW61XX_VLAN_ID,
};

enum {
	MVSW61XX_PORT_MASK,
	MVSW61XX_PORT_QMODE,
	MVSW61XX_PORT_REGISTER,
	MVSW61XX_PORT_STAT_INGRESS,
	MVSW61XX_PORT_STAT_EGRESS,
	MVSW61XX_PORT_FORCE_LINK,
	MVSW61XX_PORT_STATUS,
	MVSW61XX_PORT_PHYDET,
};

struct mvsw61xx_counter mib_counters[] = {
	{ MVSW61XX_COUNTER_IN, 0x0, 2, "InGoodOctets"},
	{ MVSW61XX_COUNTER_IN, 0x2, 1, "InBad"},
	{ MVSW61XX_COUNTER_IN, 0x4, 1, "InUnicast"},
	{ MVSW61XX_COUNTER_IN, 0x6, 1, "InBroadcasts"},
	{ MVSW61XX_COUNTER_IN, 0x7, 1, "InMulticasts"},
	{ MVSW61XX_COUNTER_IN, 0x16, 1, "InPause"},
	{ MVSW61XX_COUNTER_IN, 0x18, 1, "InUndersize"},
	{ MVSW61XX_COUNTER_IN, 0x19, 1, "InFragments"},
	{ MVSW61XX_COUNTER_IN, 0x1A, 1, "InOversize"},
	{ MVSW61XX_COUNTER_IN, 0x1B, 1, "InJabber"},
	{ MVSW61XX_COUNTER_IN, 0x1C, 1, "InRxErr"},
	{ MVSW61XX_COUNTER_IN, 0x1D, 1, "InFCSErr"},
	{ MVSW61XX_COUNTER_OUT, 0xE, 2, "OutOctets"},
	{ MVSW61XX_COUNTER_OUT, 0x10, 1, "OutUnicasts"},
	{ MVSW61XX_COUNTER_OUT, 0x12, 1, "OutMulticasts"},
	{ MVSW61XX_COUNTER_OUT, 0x13, 1, "OutBroadcasts"},
	{ MVSW61XX_COUNTER_OUT, 0x15, 1, "OutPause"},
	{ MVSW61XX_COUNTER_OUT, 0x1E, 1, "Collisions"},
	{ MVSW61XX_COUNTER_OUT, 0x05, 1, "Deffered"},
	{ MVSW61XX_COUNTER_OUT, 0x14, 1, "Single"},
	{ MVSW61XX_COUNTER_OUT, 0x17, 1, "Multiple"},
	{ MVSW61XX_COUNTER_OUT, 0x11, 1, "Excessive"},
	{ MVSW61XX_COUNTER_OUT, 0x03, 1, "OutFCSErr"},
	{ MVSW61XX_COUNTER_OUT, 0x1F, 1, "Late"},
	{ MVSW61XX_COUNTER_HISTOGRAM, 0x8, 1, "LE64"},
	{ MVSW61XX_COUNTER_HISTOGRAM, 0x9, 1, "65-127"},
	{ MVSW61XX_COUNTER_HISTOGRAM, 0xA, 1, "128-255"},
	{ MVSW61XX_COUNTER_HISTOGRAM, 0xB, 1, "256-511"},
	{ MVSW61XX_COUNTER_HISTOGRAM, 0xC, 1, "512-1023"},
	{ MVSW61XX_COUNTER_HISTOGRAM, 0xD, 1, "GE1024"},
};

static int mvsw61xx_get_stat_ingress(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	int port;
	int len;
	int i;
	u16 reg;

	len = 0;
	port = val->port_vlan + 1;
	
	// capture counters for port
	reg = 0x0 | ( (port & 0xf) << 5 ) | (0x5 << 12) | (1 << 16);
	
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);
		
	sw16(dev, MV_GLOBALREG(STAT_CONTROL), reg);
	
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);

	// read ingress counters
	len = snprintf(state->buf + len, state->buf_size, "\n");
	for( i = 0; i < ARRAY_SIZE(mib_counters); i++ ) {
		struct mvsw61xx_counter *c = &mib_counters[i];
		u64 value = 0;
		if( c->type == MVSW61XX_COUNTER_OUT )
			continue;

		if( c->len == 1 ) {
			reg = c->ptr | ( (port & 0xf) << 5 ) | (0x4 << 12) | (1 << 15);
			if( c->type == MVSW61XX_COUNTER_HISTOGRAM)
				reg |= (0x1 << 10);
			sw16(dev, MV_GLOBALREG(STAT_CONTROL), reg);
			mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);

			reg = sr16(dev, MV_GLOBALREG(STATLO));
			value = reg;
			reg = sr16(dev, MV_GLOBALREG(STATHI));
			value = value + (reg << 16);

		} else if( c->len == 2 ) {
			reg = c->ptr | ( (port & 0xf) << 5 ) | (0x4 << 12) | (1 << 15);
			sw16(dev, MV_GLOBALREG(STAT_CONTROL), reg);
			mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);

			reg = sr16(dev, MV_GLOBALREG(STATLO));
			value = reg;
			reg = sr16(dev, MV_GLOBALREG(STATHI));
			value = value + (reg << 16);

			reg = (c->ptr + 1) | ( (port & 0xf) << 5 ) | (0x4 << 12) | (1 << 15);
			sw16(dev, MV_GLOBALREG(STAT_CONTROL), reg);
			mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);

			reg = sr16(dev, MV_GLOBALREG(STATLO));
			value = value + ((u64)reg << 32 );
			reg = sr16(dev, MV_GLOBALREG(STATHI));
			value = value + ((u64)reg << 48);
		}
		len += snprintf(state->buf + len, state->buf_size, "\t\t%s : %llu\n", c->name, value);
	}

	// format output string
	val->value.s = state->buf;
	val->len = len;
	return 0;
}

static int mvsw61xx_get_stat_egress(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);
	int port;
	int len;
	int i;
	u16 reg;

	len = 0;
	port = val->port_vlan + 1;
	
	// capture counters for port
	reg = 0x0 | ( (port & 0xf) << 5 ) | (0x5 << 12) | (1 << 16);
	
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);
		
	sw16(dev, MV_GLOBALREG(STAT_CONTROL), reg);
	
	mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);

	// read ingress counters
	len = snprintf(state->buf + len, state->buf_size, "\n");
	for( i = 0; i < ARRAY_SIZE(mib_counters); i++ ) {
		struct mvsw61xx_counter *c = &mib_counters[i];
		u64 value = 0;
		if( c->type == MVSW61XX_COUNTER_IN )
			continue;

		if( c->len == 1 ) {
			reg = c->ptr | ( (port & 0xf) << 5 ) | (0x4 << 12) | (1 << 15);
			if( c->type == MVSW61XX_COUNTER_HISTOGRAM)
				reg |= (0x1 << 10);
			sw16(dev, MV_GLOBALREG(STAT_CONTROL), reg);
			mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);

			reg = sr16(dev, MV_GLOBALREG(STATLO));
			value = reg;
			reg = sr16(dev, MV_GLOBALREG(STATHI));
			value = value + (reg << 16);

		} else if( c->len == 2 ) {
			reg = c->ptr | ( (port & 0xf) << 5 ) | (0x4 << 12) | (1 << 15);
			sw16(dev, MV_GLOBALREG(STAT_CONTROL), reg);
			mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);

			reg = sr16(dev, MV_GLOBALREG(STATLO));
			value = reg;
			reg = sr16(dev, MV_GLOBALREG(STATHI));
			value = value + (reg << 16);

			reg = (c->ptr + 1) | ( (port & 0xf) << 5 ) | (0x4 << 12) | (1 << 15);
			sw16(dev, MV_GLOBALREG(STAT_CONTROL), reg);
			mvsw61xx_wait_mask_s(dev, MV_GLOBALREG(STAT_CONTROL), (1 << 15), 0);

			reg = sr16(dev, MV_GLOBALREG(STATLO));
			value = (u64)value + ((u64)reg << 32);
			reg = sr16(dev, MV_GLOBALREG(STATHI));
			value = (u64)value + ((u64)reg << 48);
		}
		len += snprintf(state->buf + len, state->buf_size, "\t\t%s : %llu\n", c->name, value);
	}

	// format output string
	val->value.s = state->buf;
	val->len = len;
	return 0;
}

static int mvsw61xx_get_vlan_lan_diode(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	struct mvsw61xx_state *state = get_state(dev);

	if( state == NULL )
		return -1;

	val->value.i = state->vlan_diode;

	return 0;
}

static int mvsw61xx_set_vlan_lan_diode(struct switch_dev *dev,
		const struct switch_attr *attr, struct switch_val *val)
{
	int i;
	u16 vlan_mask = 0xffff;
	u16 data;

	struct mvsw61xx_state *state = get_state(dev);
	int vlan = val->value.i;

	for( i=0; i < MV_VLANS; i++ ) {
		if( state->vlans[i].vid == vlan ) {
			vlan_mask = state->vlans[i].mask & 0x1F;
			break;
		}
	}

	if( vlan_mask == 0xffff )
		goto vlan_not_found;

	/* 
	set Port0 special function
	register index 0x7
	
	LED control format: 15 - update, 14-12 - pointer, 0-10 - data
	*/
	data = (1 << 15) |
			(0x7 << 12) |
			(vlan_mask & 0x7ff);
	
	sw16(dev, MV_PORTREG(LED_CONTROL, 0), data);

	state->vlan_diode = val->value.i;
	return 0;
	
vlan_not_found:
	return 0;
}



static const struct switch_attr mvsw61xx_global[] = {
	[MVSW61XX_ENABLE_VLAN] = {
		.id = MVSW61XX_ENABLE_VLAN,
		.type = SWITCH_TYPE_INT,
		.name = "enable_vlan",
		.description = "Enable 802.1q VLAN support",
		.get = mvsw61xx_get_enable_vlan,
		.set = mvsw61xx_set_enable_vlan,
	},
	[MVSW61XX_LAN_DIODE] = {
		.id = MVSW61XX_LAN_DIODE,
		.type = SWITCH_TYPE_INT,
		.name = "vlan_lan_diode",
		.description = "VLAN number which ports are used for lan diode blinking",
		.get = mvsw61xx_get_vlan_lan_diode,
		.set = mvsw61xx_set_vlan_lan_diode,
	},

	[MVSW61XX_FIBER_SHOW] = {
		.id = MVSW61XX_FIBER_SHOW,
		.type = SWITCH_TYPE_STRING,
		.description = "Fiber registers",
		.name = "fiber_registers",
		.get = mvsw61xx_get_fiber_regs,
		.set = NULL
	},
	[MVSW61XX_FIBER_CONTROL] = {
		.id = MVSW61XX_FIBER_CONTROL,
		.type = SWITCH_TYPE_INT,
		.description = "Fiber control register",
		.name = "fiber_control_reg",
		.get = mvsw61xx_get_fiber_control,
		.set = mvsw61xx_set_fiber_control,
	},
	[MVSW61XX_FIBER_STATUS] = {
		.id = MVSW61XX_FIBER_STATUS,
		.type = SWITCH_TYPE_INT,
		.description = "Fiber status register",
		.name = "fiber_status_reg",
		.get = mvsw61xx_get_fiber_status,
		.set = mvsw61xx_set_fiber_status,
	},
	[MVSW61XX_FIBER_REGVALUE] = {
		.id = MVSW61XX_FIBER_REGVALUE,
		.type = SWITCH_TYPE_STRING,
		.description = "MII register and value",
		.name = "regvalue",
		.get = mvsw61xx_get_fiber_regvalue,
		.set = mvsw61xx_set_fiber_regvalue,
	},
	[MVSW61XX_FIBER_POWER] = {
		.id = MVSW61XX_FIBER_POWER,
		.type = SWITCH_TYPE_INT,
		.description = "Fiber power enable",
		.name = "fiber_power",
		.get = mvsw61xx_get_fiber_power,
		.set = mvsw61xx_set_fiber_power,
	},
	[MVSW61XX_VTU_VIOLATION] = {
		.id = MVSW61XX_VTU_VIOLATION,
		.type = SWITCH_TYPE_STRING,
		.description = "VTU violation",
		.name = "vtu_violation",
		.get = mvsw61xx_get_vtu_violation ,
		.set = NULL,
	},
	[MVSW61XX_VTU_BASE] = {
		.id = MVSW61XX_VTU_BASE,
		.type = SWITCH_TYPE_STRING,
		.description = "VTU base",
		.name = "vtu_base",
		.get = mvsw61xx_get_vtu_base ,
		.set = NULL,
	},
	[MVSW61XX_STU_BASE] = {
		.id = MVSW61XX_STU_BASE,
		.type = SWITCH_TYPE_STRING,
		.description = "STU base",
		.name = "stu_base",
		.get = mvsw61xx_get_stu_base ,
		.set = NULL,
	},
};

static const struct switch_attr mvsw61xx_vlan[] = {
	[MVSW61XX_VLAN_PORT_BASED] = {
		.id = MVSW61XX_VLAN_PORT_BASED,
		.type = SWITCH_TYPE_INT,
		.name = "port_based",
		.description = "Use port-based (non-802.1q) VLAN only",
		.get = mvsw61xx_get_vlan_port_based,
		.set = mvsw61xx_set_vlan_port_based,
	},
	[MVSW61XX_VLAN_ID] = {
		.id = MVSW61XX_VLAN_ID,
		.type = SWITCH_TYPE_INT,
		.name = "vid",
		.description = "Get/set VLAN ID",
		.get = mvsw61xx_get_vid,
		.set = mvsw61xx_set_vid,
	},
};

static const struct switch_attr mvsw61xx_port[] = {
	[MVSW61XX_PORT_MASK] = {
		.id = MVSW61XX_PORT_MASK,
		.type = SWITCH_TYPE_STRING,
		.description = "Port-based VLAN mask",
		.name = "mask",
		.get = mvsw61xx_get_port_mask,
		.set = NULL,
	},
	[MVSW61XX_PORT_REGISTER] = {
		.id = MVSW61XX_PORT_REGISTER,
		.type = SWITCH_TYPE_STRING,
		.description = "MII register and value",
		.name = "regvalue",
		.get = mvsw61xx_get_regvalue,
		.set = mvsw61xx_set_regvalue,
	},
	[MVSW61XX_PORT_QMODE] = {
		.id = MVSW61XX_PORT_QMODE,
		.type = SWITCH_TYPE_INT,
		.description = "802.1q mode: 0=off/1=fallback/2=check/3=secure",
		.name = "qmode",
		.get = mvsw61xx_get_port_qmode,
		.set = mvsw61xx_set_port_qmode,
	},
	[MVSW61XX_PORT_STAT_INGRESS] = {
		.id = MVSW61XX_PORT_STAT_INGRESS,
		.type = SWITCH_TYPE_STRING,
		.description = "Port ingress counters",
		.name = "ingress_stat",
		.get = mvsw61xx_get_stat_ingress,
		.set = NULL,
	},
	[MVSW61XX_PORT_STAT_EGRESS] = {
		.id = MVSW61XX_PORT_STAT_EGRESS,
		.type = SWITCH_TYPE_STRING,
		.description = "Port egress counters",
		.name = "egress_stat",
		.get = mvsw61xx_get_stat_egress,
		.set = NULL,
	},
	[MVSW61XX_PORT_FORCE_LINK] = {
		.id = MVSW61XX_PORT_FORCE_LINK,
		.type = SWITCH_TYPE_INT,
		.description = "Port force link",
		.name = "force_link",
		.get = mvsw61xx_get_force_link,
		.set = mvsw61xx_set_force_link,
	},
	[MVSW61XX_PORT_STATUS] = {
		.id = MVSW61XX_PORT_STATUS,
		.type = SWITCH_TYPE_INT,
		.description = "Port status register",
		.name = "status_reg",
		.get = mvsw61xx_get_port_status,
		.set = NULL,
	},
	[MVSW61XX_PORT_PHYDET] = {
		.id = MVSW61XX_PORT_PHYDET,
		.type = SWITCH_TYPE_INT,
		.description = "Port PHY detection",
		.name = "phydet",
		.get = mvsw61xx_get_port_phydet,
		.set = mvsw61xx_set_port_phydet,
	},

};

static const struct switch_dev_ops mvsw61xx_ops = {
	.attr_global = {
		.attr = mvsw61xx_global,
		.n_attr = ARRAY_SIZE(mvsw61xx_global),
	},
	.attr_vlan = {
		.attr = mvsw61xx_vlan,
		.n_attr = ARRAY_SIZE(mvsw61xx_vlan),
	},
	.attr_port = {
		.attr = mvsw61xx_port,
		.n_attr = ARRAY_SIZE(mvsw61xx_port),
	},
	.get_port_link = mvsw61xx_get_port_link,
	.set_port_link = mvsw61xx_set_port_link,
	.get_port_pvid = mvsw61xx_get_port_pvid,
	.set_port_pvid = mvsw61xx_set_port_pvid,
	.get_vlan_ports = mvsw61xx_get_vlan_ports,
	.set_vlan_ports = mvsw61xx_set_vlan_ports,
	.phy_read16 = mvsw61xx_phy_read16,
	.phy_write16 = mvsw61xx_phy_write16,
	.apply_config = mvsw61xx_apply,
	.reset_switch = mvsw61xx_reset,
};

/* end swconfig stuff */

static void mvsw61xx_link_poll_work(struct work_struct *ugly) {
	struct mvsw61xx_state *state;
	int i;
	state = container_of(ugly, struct mvsw61xx_state, link_poll_work);
	
	for( i=0; i < MV_PORTS; i++ ){
	// get port status
		u16 status = sr16(&state->dev, MV_PORTREG(STATUS, i));
		u16 link, speed, duplex;
		link = status & MV_PORT_STATUS_LINK;
		speed = (status & MV_PORT_STATUS_SPEED_MASK) >> MV_PORT_STATUS_SPEED_SHIFT;
		switch (speed) {
		    case MV_PORT_STATUS_SPEED_10:
			speed = SWITCH_PORT_SPEED_10;
			break;
		    case MV_PORT_STATUS_SPEED_100:
			speed = SWITCH_PORT_SPEED_100;
			break;
		    case MV_PORT_STATUS_SPEED_1000:
			speed = SWITCH_PORT_SPEED_1000;
		    break;
		}
		duplex = status & MV_PORT_STATUS_FDX;
		if( state->ports[i].link_status != link ) {
			printk(KERN_DEBUG "Link changed. Port: %d, link: 0x%X/0x%X\n", i, link, state->ports[i].link_status);
			//make hotplug event
			state->ports[i].link_status = link;
			switch_create_link_event(&state->dev, i, link, duplex, speed);
		}
	}
	state->link_poll_timer.expires = round_jiffies(jiffies + HZ);
	add_timer(&state->link_poll_timer);
}

static void mvsw61xx_link_int_work(struct work_struct *ugly) {
	struct mvsw61xx_state *state;
	u16 reg;
	int i;
	state = container_of(ugly, struct mvsw61xx_state, link_poll_work);

	reg = sr16(&state->dev, MV_GLOBALREG(STATUS));

	// Handle DevInt
	if( reg & (1 << 7 ) ) {
		
		reg = sr16(&state->dev, MV_GLOBAL2REG(INT_SRC));

		// handle phy interrupts
		for( i=0; i < 5; i++ ) {
			if( reg & (1 << i ) ) {
				reg = sr16(&state->dev, MV_PHYREG(INTERRUPT_STATUS, i));
				if( reg & (1 << 10 ) ) {
					printk(KERN_DEBUG "LINK CHANGED\n");
				}
				if( reg & (1 << 13 ) ) {
					printk(KERN_DEBUG "DUPLEX CHANGED\n");
				}
				if( reg & (1 << 14 ) ) {
					printk(KERN_DEBUG "SPEED CHANGED\n");
				}
			}
		}

		// Handle SERDES interrupt
		if( reg & (1 << 11) ) {
		}
	}	
	// handle other dev interrupts
}

static void mvsw61xx_link_poll_timer(unsigned long _dst) {
	struct mvsw61xx_state *state = (struct mvsw61xx_state *) _dst;

	schedule_work(&state->link_poll_work);
}

static irqreturn_t mvsw61xx_irq_handler( int irq, void* data){
	struct mvsw61xx_state *state = data;

	schedule_work(&state->link_poll_work);

	return IRQ_HANDLED;
}

static int mvsw61xx_probe(struct platform_device *pdev)
{
	struct mvsw61xx_state *state;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *mdio;
	char *model_str;
	u32 val;
	int err;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->buf_size = PAGE_SIZE;
	state->buf = kzalloc(state->buf_size, GFP_KERNEL);

	if( !state->buf ){
		kfree(state);
		return -ENOMEM;
	}

	mdio = of_parse_phandle(np, "mii-bus", 0);
	if (!mdio) {
		dev_err(&pdev->dev, "Couldn't get MII bus handle\n");
		err = -ENODEV;
		goto out_err;
	}

	state->bus = of_mdio_find_bus(mdio);
	if (!state->bus) {
		dev_err(&pdev->dev, "Couldn't find MII bus from handle\n");
		err = -ENODEV;
		goto out_err;
	}

	state->is_indirect = of_property_read_bool(np, "is-indirect");

	if (state->is_indirect) {
		if (of_property_read_u32(np, "reg", &val)) {
			dev_err(&pdev->dev, "Switch address not specified\n");
			err = -ENODEV;
			goto out_err;
		}

		state->base_addr = val;
	} else {
		state->base_addr = MV_BASE;
	}

	state->model = r16(state->bus, state->is_indirect, state->base_addr,
				MV_PORTREG(IDENT, 0)) & MV_IDENT_MASK;

	switch(state->model) {
	case MV_IDENT_VALUE_6171:
		model_str = MV_IDENT_STR_6171;
		break;
	case MV_IDENT_VALUE_6172:
		model_str = MV_IDENT_STR_6172;
		break;
	case MV_IDENT_VALUE_6176:
		model_str = MV_IDENT_STR_6176;
		break;
	default:
		dev_err(&pdev->dev, "No compatible switch found at 0x%02x\n",
				state->base_addr);
		err = -ENODEV;
		goto out_err;
	}

	platform_set_drvdata(pdev, state);
	dev_info(&pdev->dev, "Found %s at %s:%02x\n", model_str,
			state->bus->id, state->base_addr);

	dev_info(&pdev->dev, "Using %sdirect addressing\n",
			(state->is_indirect ? "in" : ""));

	if (of_property_read_u32(np, "cpu-port-0", &val)) {
		dev_err(&pdev->dev, "CPU port not set\n");
		err = -ENODEV;
		goto out_err;
	}

	state->cpu_port0 = val;

	if (!of_property_read_u32(np, "cpu-port-1", &val))
		state->cpu_port1 = val;
	else
		state->cpu_port1 = -1;

	state->int_gpio=0;
	INIT_WORK(&state->link_poll_work, mvsw61xx_link_poll_work);
	init_timer(&state->link_poll_timer);
	state->link_poll_timer.data = (unsigned long)state;
	state->link_poll_timer.function = mvsw61xx_link_poll_timer;
	state->link_poll_timer.expires = round_jiffies(jiffies + HZ);
	add_timer(&state->link_poll_timer);

	state->dev.vlans = MV_MAX_VLAN;
	state->dev.cpu_port = state->cpu_port0;
	state->dev.ports = MV_PORTS;
	state->dev.name = model_str;
	state->dev.ops = &mvsw61xx_ops;
	state->dev.alias = dev_name(&pdev->dev);
	state->last_vlan = 1;

	err = register_switch(&state->dev, NULL);
	if (err < 0)
		goto out_err;

	state->registered = true;

	return 0;
out_err:
	kfree(state);
	return err;
}

static int
mvsw61xx_remove(struct platform_device *pdev)
{
	struct mvsw61xx_state *state = platform_get_drvdata(pdev);

	if (state->registered)
		unregister_switch(&state->dev);

	kfree(state);

	return 0;
}

static const struct of_device_id mvsw61xx_match[] = {
	{ .compatible = "marvell,88e6171" },
	{ .compatible = "marvell,88e6172" },
	{ .compatible = "marvell,88e6176" },
	{ }
};
MODULE_DEVICE_TABLE(of, mvsw61xx_match);

static struct platform_driver mvsw61xx_driver = {
	.probe = mvsw61xx_probe,
	.remove = mvsw61xx_remove,
	.driver = {
		.name = "mvsw61xx",
		.of_match_table = of_match_ptr(mvsw61xx_match),
		.owner = THIS_MODULE,
	},
};

static int __init mvsw61xx_module_init(void)
{
	return platform_driver_register(&mvsw61xx_driver);
}
late_initcall(mvsw61xx_module_init);

static void __exit mvsw61xx_module_exit(void)
{
	platform_driver_unregister(&mvsw61xx_driver);
}
module_exit(mvsw61xx_module_exit);
