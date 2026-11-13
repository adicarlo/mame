// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT100 graphics port: the graphics connector (J2)

    The connector (TM-003 8.1) carries the 8080's data bus with a write
    strobe for port E2, GRAPHICS FLAG L back to the flag buffer (port 42
    bit 2), INIT, horizontal and vertical blank, and the two video inputs
    GRAPHICS 1 IN L and GRAPHICS 2 IN L, which light a dot dim, normal or
    bright where the VT100's own video is black.

***************************************************************************/

#include "emu.h"
#include "vt100_gport.h"


//**************************************************************************
//  SLOT
//**************************************************************************

DEFINE_DEVICE_TYPE(VT100_GPORT_SLOT, vt100_gport_slot_device, "vt100_gport_slot", "DEC VT100 graphics connector")

vt100_gport_slot_device::vt100_gport_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, VT100_GPORT_SLOT, tag, owner, clock)
	, device_single_card_slot_interface<device_vt100_gport_interface>(mconfig, *this)
	, m_screen(*this, finder_base::DUMMY_TAG)
	, m_card(nullptr)
{
}

void vt100_gport_slot_device::device_start()
{
	m_card = get_card_device();
}

void vt100_gport_slot_device::write(u8 data)
{
	if (m_card)
		m_card->write(data);
}

int vt100_gport_slot_device::flag_r()
{
	// pulled up on the terminal controller
	return m_card ? m_card->flag_r() : 1;
}

int vt100_gport_slot_device::scan(int line, u8 *dots)
{
	return m_card ? m_card->scan(line, dots) : NONE;
}


device_vt100_gport_interface::device_vt100_gport_interface(const machine_config &mconfig, device_t &device)
	: device_interface(device, "vt100gfx")
	, m_slot(dynamic_cast<vt100_gport_slot_device *>(device.owner()))
{
}


void vt100_gport_options(device_slot_interface &device)
{
	device.option_add("m7071", VT105_WG);
}
