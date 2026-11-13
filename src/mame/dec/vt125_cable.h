// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT125: the 24-pin cable between the mono board (54-14277 J1, on
    the graphics connector) and the paddle board (54-14275 J4, in the STP
    connector), MP01053 MB7 and paddle sheet 2

    It carries the mono board 8085's I/O data bus, A0-A2 and the IN COM L
    and OUT COM L strobes (ports 08-0F) to the paddle's three 8251As and
    baud rate generator; the 3 and 6 MHz clocks and RESET OUT; and back
    READ READY and TRANS READY (8085 RST 6.5 and 5.5), GRAPHICS FLAG and
    CLK SHIFT.  The two boards are separate options on separate
    connectors, so the terminal joins them once both are in.

***************************************************************************/

#ifndef MAME_DEC_VT125_CABLE_H
#define MAME_DEC_VT125_CABLE_H

#pragma once


class vt125_mono_cable_interface;

// the paddle board's end
class vt125_paddle_cable_interface
{
public:
	virtual ~vt125_paddle_cable_interface() = default;

	virtual void connect(vt125_mono_cable_interface &mono) = 0;

	// IN COM L / OUT COM L with A2-A0
	virtual u8 in_com(offs_t offset) = 0;
	virtual void out_com(offs_t offset, u8 data) = 0;
};

// the mono board's end
class vt125_mono_cable_interface
{
public:
	virtual ~vt125_mono_cable_interface() = default;

	virtual void connect(vt125_paddle_cable_interface &paddle) = 0;

	virtual void read_ready_w(int state) = 0;
	virtual void trans_ready_w(int state) = 0;
	virtual void graphics_flag_w(int state) = 0;
	virtual void clk_shift_w(int state) = 0;
};

#endif // MAME_DEC_VT125_CABLE_H
