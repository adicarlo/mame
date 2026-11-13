// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT100 alternate character ROM socket

***************************************************************************/

#ifndef MAME_DEC_VT100_ALTCHAR_H
#define MAME_DEC_VT100_ALTCHAR_H

#pragma once


class device_vt100_altchar_interface;

// The second character generator socket: E9 on the VT100 terminal
// controller, E4 on the VT101/VT102 board.  The video logic reads it in
// place of the standard ROM for characters with the ALT CHAR SET attribute.
class vt100_altchar_slot_device : public device_t, public device_single_card_slot_interface<device_vt100_altchar_interface>
{
public:
	template <typename T>
	vt100_altchar_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, const char *dflt)
		: vt100_altchar_slot_device(mconfig, tag, owner, 0)
	{
		set_options(std::forward<T>(opts), dflt, false);
	}

	vt100_altchar_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// one byte of the 2K socket
	u8 read(offs_t offset);

protected:
	virtual void device_start() override ATTR_COLD;

private:
	device_vt100_altchar_interface *m_card;
};


class device_vt100_altchar_interface : public device_interface
{
public:
	virtual u8 read(offs_t offset) = 0;

protected:
	device_vt100_altchar_interface(const machine_config &mconfig, device_t &device);
};


DECLARE_DEVICE_TYPE(VT100_ALTCHAR_SLOT, vt100_altchar_slot_device)

void vt100_altchar_options(device_slot_interface &device);

#endif // MAME_DEC_VT100_ALTCHAR_H
