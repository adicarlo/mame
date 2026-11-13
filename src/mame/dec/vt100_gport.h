// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT100 graphics port: the graphics connector (J2)

***************************************************************************/

#ifndef MAME_DEC_VT100_GPORT_H
#define MAME_DEC_VT100_GPORT_H

#pragma once

#include "screen.h"


class device_vt100_gport_interface;

class vt100_gport_slot_device : public device_t, public device_single_card_slot_interface<device_vt100_gport_interface>
{
public:
	// what a card shows on one scan line
	enum format_t : int
	{
		NONE = 0,
		RECTANGULAR,    // 512 dots at 10.4 MHz
		SQUARE          // 512 dots at 12.4 MHz, all on screen (TM-003 figure 6-19)
	};

	template <typename T, typename U>
	vt100_gport_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, const char *dflt, U &&screen_tag)
		: vt100_gport_slot_device(mconfig, tag, owner, 0)
	{
		set_options(std::forward<T>(opts), dflt, false);
		m_screen.set_tag(std::forward<U>(screen_tag));
	}

	vt100_gport_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// GRAPHIC WRITE L strobe from port E2
	void write(u8 data);

	// GRAPHICS FLAG L: high (1) when nothing is fitted or the card is busy
	int flag_r();

	// GRAPHICS 1/2 IN L for scan line 'line', counted from the end of
	// vertical blank: 512 dots, nonzero = lit
	int scan(int line, u8 *dots); // a format_t, for the video device's callback

protected:
	virtual void device_start() override ATTR_COLD;

private:
	friend class device_vt100_gport_interface;

	required_device<screen_device> m_screen;
	device_vt100_gport_interface *m_card;
};


class device_vt100_gport_interface : public device_interface
{
public:
	virtual void write(u8 data) = 0;
	virtual int flag_r() = 0;
	virtual vt100_gport_slot_device::format_t scan(int line, u8 *dots) = 0;

protected:
	device_vt100_gport_interface(const machine_config &mconfig, device_t &device);

	screen_device &screen() { return *m_slot->m_screen; }

private:
	vt100_gport_slot_device *const m_slot;
};


DECLARE_DEVICE_TYPE(VT100_GPORT_SLOT, vt100_gport_slot_device)
DECLARE_DEVICE_TYPE(VT105_WG, device_vt100_gport_interface)

void vt100_gport_options(device_slot_interface &device);

#endif // MAME_DEC_VT100_GPORT_H
