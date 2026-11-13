// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC M7071 waveform generator (VT105;  EK-VT105-TM-001, print set MP00642,
    CS-M7071-0-1 rev N).  Charts, not drawing: two graphs of one 8-bit Y
    per X (512 X), each optionally shaded down to 0 or to a baseline,
    markers on the graphs, and full height vertical and full width
    horizontal lines.  Nothing is stored per dot: each scan line compares
    the RAMs with a Y counter that counts down from 229 (rectangular,
    10.4 MHz dots) or 239 (square, 12.4 MHz) from vertical blank (sheet 5,
    E34/E28).  No ROM or PROM on the board.

    The VT100 firmware sends everything between ESC 1 and ESC 2 to port E2
    and polls GRAPHICS FLAG between bytes; the board decodes the commands
    (programmer's card, T105 figure C-1).  Control bytes are 10xxxxx, data
    bytes 01xxxxx; each command takes pairs of data bytes, low bits first,
    and repeats for each further pair.  The decoder (sheet 1, E65/E72)
    sees DB3, DB0, DB1, DB2; DB4 selects unit 1, a second board, which W3
    leaves out.

    One open collector gate drives both video lines (sheet 4, E87, to XE90
    pins 15 and 10), so every graphics dot is bright.

    GRAPHICS FLAG L goes high from each completed data pair until the
    board has written it (PREWRITE, sheet 6 E58): the write waits for the
    end of the dot sweep on a line outside blanking and at least 12 us
    after the load (E53).  The initialize bit holds it high for the 1.7 ms
    of DIS CLR (sheet 1, E85).

    TODO:
    - the interactive test (register 1 T bit)
    - ODD EVEN(1) H, which gates the write (sheet 6, E46): every line is
      assumed
    - the second unit (P..\ commands)

***************************************************************************/

#include "emu.h"
#include "vt100_gport.h"

#include <algorithm>


//**************************************************************************
//  M7071 WAVEFORM GENERATOR
//**************************************************************************

namespace {

class vt105_wg_device : public device_t, public device_vt100_gport_interface
{
public:
	vt105_wg_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, VT105_WG, tag, owner, clock)
		, device_vt100_gport_interface(mconfig, *this)
	{
	}

	virtual void write(u8 data) override;
	virtual int flag_r() override;
	virtual vt100_gport_slot_device::format_t scan(int line, u8 *dots) override;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	// decoder outputs (E72)
	enum : u8
	{
		CMD_NOP = 0,    // @: baseline
		CMD_X,          // H
		CMD_ENABLE0,    // A
		CMD_ENABLE1,    // I
		CMD_GRAPH0,     // B
		CMD_GRAPH1,     // J
		CMD_CUR0,       // C
		CMD_CUR1,       // K
		CMD_HORZ,       // D
		CMD_VERT,       // L
		CMD_NONE = 0xff
	};

	// enable register 0, first byte (E77) and second byte (E70)
	static constexpr u8 EN0_DISPLAY = 0x01, EN0_GRAPH0 = 0x02, EN0_GRAPH1 = 0x04, EN0_HIST0 = 0x08, EN0_HIST1 = 0x10;
	static constexpr u8 EN0_BASECTRL = 0x01, EN0_BASE0 = 0x02, EN0_BASE1 = 0x04, EN0_STRIP = 0x08, EN0_DUAL = 0x10;
	// enable register 1, first byte (E62, initialize) and second byte
	static constexpr u8 EN1_HORZ = 0x01, EN1_VERT = 0x02, EN1_CUR0 = 0x04, EN1_CUR1 = 0x08, EN1_INIT = 0x10;
	static constexpr u8 EN1_SQUARE = 0x01, EN1_TEST = 0x02;

	// write side: the vertical/cursor RAM bits
	static constexpr u8 VC_VERT = 0x01, VC_CUR0 = 0x02, VC_CUR1 = 0x04;

	void execute(u8 lo, u8 hi);
	void busy_until_write();
	TIMER_CALLBACK_MEMBER(write_done);
	TIMER_CALLBACK_MEMBER(clear_done);

	emu_timer *m_write_timer;
	emu_timer *m_clear_timer;

	u8 m_graph[2][512];     // Y per X (93L422s, sheet 2)
	u8 m_vc[512];           // vertical line and cursor bits per X (sheet 3)
	u8 m_horz[256];         // horizontal line per Y (3106 E48, sheet 5)
	u8 m_base[2];           // baselines (E41, E40)
	u16 m_add;              // write pointer (sheet 6 E36/E37/E39)
	u8 m_en0a, m_en0b;      // enable register 0
	u8 m_en1a, m_en1b;      // enable register 1
	u8 m_cmd;               // last control byte, decoded
	bool m_first;           // next data byte is the first of a pair
	u8 m_lo;                // first data byte of the pair
	bool m_prewrite;
	bool m_dis_clr;
	bool m_strip_ff;        // strip enable flip-flop: set by the pointer's overflow
};

void vt105_wg_device::device_start()
{
	m_write_timer = timer_alloc(FUNC(vt105_wg_device::write_done), this);
	m_clear_timer = timer_alloc(FUNC(vt105_wg_device::clear_done), this);

	std::fill(std::begin(m_graph[0]), std::end(m_graph[0]), 0);
	std::fill(std::begin(m_graph[1]), std::end(m_graph[1]), 0);
	std::fill(std::begin(m_vc), std::end(m_vc), 0);
	std::fill(std::begin(m_horz), std::end(m_horz), 0);
	m_base[0] = m_base[1] = 0;
	m_add = 0;

	save_item(NAME(m_graph));
	save_item(NAME(m_vc));
	save_item(NAME(m_horz));
	save_item(NAME(m_base));
	save_item(NAME(m_add));
	save_item(NAME(m_en0a));
	save_item(NAME(m_en0b));
	save_item(NAME(m_en1a));
	save_item(NAME(m_en1b));
	save_item(NAME(m_cmd));
	save_item(NAME(m_first));
	save_item(NAME(m_lo));
	save_item(NAME(m_prewrite));
	save_item(NAME(m_dis_clr));
	save_item(NAME(m_strip_ff));
}

void vt105_wg_device::device_reset()
{
	// INIT L clears the decoder and the enable registers (sheet 1: E65,
	// E77, E62, E60 CLR; sheet 5: E70, E58); RAM keeps its contents
	m_en0a = m_en0b = 0;
	m_en1a = m_en1b = 0;
	m_cmd = CMD_NONE;
	m_first = true;
	m_lo = 0;
	m_prewrite = false;
	m_dis_clr = false;
	m_strip_ff = false;
	m_write_timer->adjust(attotime::never);
	m_clear_timer->adjust(attotime::never);
}

void vt105_wg_device::write(u8 data)
{
	switch (data & 0x60)
	{
	case 0x40:
		// control: decoder inputs DB3, DB0, DB1, DB2 (sheet 1, E65 to E72)
		if (BIT(data, 4))
			m_cmd = CMD_NONE;   // unit 1
		else
		{
			u8 const sel = bitswap<4>(data, 2, 1, 0, 3);
			m_cmd = (sel < 10) ? sel : CMD_NONE;
		}
		m_first = true;
		break;

	case 0x20:
		// data
		if (m_first)
			m_lo = data & 0x1f;
		else
			execute(m_lo, data & 0x1f);
		m_first = !m_first;
		break;

	default:
		// neither (E64/E74): ignored.  D7 alone pulses COPY L to TB1 for a
		// hard copy unit, which isn't modelled.
		break;
	}
}

void vt105_wg_device::execute(u8 lo, u8 hi)
{
	u16 const x = lo | ((hi & 0x0f) << 5);
	u8 const y = lo | ((hi & 0x07) << 5);
	bool const w = BIT(hi, 4);

	switch (m_cmd)
	{
	case CMD_NOP:
		m_base[BIT(m_en0b, 0)] = y;
		break;

	case CMD_X:
		m_add = x;
		break;

	case CMD_ENABLE0:
		m_en0a = lo;
		m_en0b = hi;
		break;

	case CMD_ENABLE1:
		m_en1a = lo;
		m_en1b = hi;
		if (BIT(lo, 4) && !m_dis_clr)
		{
			// DIS CLR: E85, 1.7 ms
			m_dis_clr = true;
			std::fill(std::begin(m_graph[0]), std::end(m_graph[0]), 0);
			std::fill(std::begin(m_graph[1]), std::end(m_graph[1]), 0);
			std::fill(std::begin(m_vc), std::end(m_vc), 0);
			std::fill(std::begin(m_horz), std::end(m_horz), 0);
			m_clear_timer->adjust(attotime::from_usec(1700));
		}
		break;

	case CMD_GRAPH0:
	case CMD_GRAPH1:
		// dual strip: graph 1's load counts the pointer back first, so a B
		// pair and a J pair land at the same X (T105 6.14.2, figure 6-18)
		if ((m_cmd == CMD_GRAPH1) && (m_en0b & EN0_DUAL))
			m_add = (m_add - 1) & 0x1ff;
		m_graph[m_cmd - CMD_GRAPH0][m_add] = y;
		m_add = (m_add + 1) & 0x1ff;
		if (!m_add && (m_en0b & (EN0_STRIP | EN0_DUAL)))
			m_strip_ff = true;  // OVERFLOW: the picture starts moving
		break;

	case CMD_CUR0:
	case CMD_CUR1:
	case CMD_VERT:
	{
		// these carry their own X, which also loads the pointer (LD XDATA,
		// T105 figure 6-12); they don't advance it (figure 6-18, though
		// 6.7 lists WRITE DATA among the count sources)
		u8 const bit = (m_cmd == CMD_VERT) ? VC_VERT : (m_cmd == CMD_CUR0) ? VC_CUR0 : VC_CUR1;
		m_add = x;
		m_vc[x] = w ? (m_vc[x] | bit) : (m_vc[x] & ~bit);
		break;
	}

	case CMD_HORZ:
		m_horz[y] = w ? 1 : 0;
		break;

	default:
		return;
	}

	busy_until_write();
}

// The pair is latched now and written at the end of the dot sweep of the
// first line outside blanking at least 12 us later (sheet 6, E53).
void vt105_wg_device::busy_until_write()
{
	m_prewrite = true;

	screen_device &scr = screen();
	int const sweep_end = scr.visible_area().right();
	int const lines = scr.visible_area().bottom() + 1;
	attotime const line = scr.scan_period();
	attotime const hold = attotime::from_usec(12);

	// the end of the sweep on the current line, which may have passed
	int const vpos = scr.vpos();
	attotime until = scr.time_until_pos(vpos, sweep_end);
	if (until > line)
		until -= scr.frame_period();

	for (int i = 0; i <= scr.height(); i++)
	{
		int const v = (vpos + i) % scr.height();
		if (v < lines && until >= hold)
			break;
		until += line;
	}
	m_write_timer->adjust(until);
}

TIMER_CALLBACK_MEMBER(vt105_wg_device::write_done)
{
	m_prewrite = false;
}

TIMER_CALLBACK_MEMBER(vt105_wg_device::clear_done)
{
	m_dis_clr = false;
}

int vt105_wg_device::flag_r()
{
	return (m_prewrite || m_dis_clr) ? 1 : 0;
}

vt100_gport_slot_device::format_t vt105_wg_device::scan(int line, u8 *dots)
{
	bool const square = BIT(m_en1b, 0);
	int const lines = square ? 240 : 230;
	if (!(m_en0a & EN0_DISPLAY) || line < 0 || line >= lines)
		return vt100_gport_slot_device::NONE;

	u8 const y = lines - 1 - line;     // the Y counter counts down to 0 at the bottom
	bool const hline = (m_en1a & EN1_HORZ) && m_horz[y];

	// strip chart: once the pointer has overflowed, the sweep starts at it
	// rather than at 0, so the newest point is at the right and the trace
	// moves left as points are added (T105 6.14.1)
	bool const strip = (m_en0b & (EN0_STRIP | EN0_DUAL)) && m_strip_ff;
	u16 const start = strip ? m_add : 0;

	// dual strip ends the line one dot early, hiding the switch between
	// the graphs (T105 6.14.2)
	int const count = (m_en0b & EN0_DUAL) ? 511 : 512;

	std::fill_n(dots, 512, 0);
	for (int i = 0; i < count; i++)
	{
		u16 const x = (start + i) & 0x1ff;
		bool lit = false;
		for (int g = 0; g < 2; g++)
		{
			u8 const v = m_graph[g][x];
			bool const cur = BIT(m_en1a, 2 + g) && BIT(m_vc[x], 1 + g);

			// the graph, and the markers: the 16 lines whose upper 4 bits
			// match the graph's (T105 6.12)
			if (BIT(m_en0a, 1 + g) && (v == y))
				lit = true;
			if (cur && ((v >> 4) == (y >> 4)))
				lit = true;

			// shading lights every other dot, the two graphs on opposite
			// dot times (TIME(0)/TIME(1), E80/E81); which graph has which
			// phase isn't stated.  With its baseline enabled a graph is
			// shaded between the graph and the baseline, the baseline
			// included (T105 figure 6-14); otherwise the histogram bit
			// shades it down to the bottom.
			if (BIT(i, 0) == g)
			{
				u8 const b = m_base[g];
				if (BIT(m_en0b, 1 + g))
				{
					if ((y == b) || ((y < b) && (v < y)) || ((y > b) && (v > y)))
						lit = true;
				}
				if (BIT(m_en0a, 3 + g) && (v > y))
					lit = true;
			}
		}

		// vertical lines on scans with Y0 set (T105 figure 6-15),
		// horizontal lines on every other dot (the 320 ns gate)
		if ((m_en1a & EN1_VERT) && (m_vc[x] & VC_VERT) && BIT(y, 0))
			lit = true;
		if (hline && !BIT(i, 0))
			lit = true;

		dots[i] = lit ? 1 : 0;
	}
	return square ? vt100_gport_slot_device::SQUARE : vt100_gport_slot_device::RECTANGULAR;
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(VT105_WG, device_vt100_gport_interface, vt105_wg_device, "vt105_wg", "DEC M7071 waveform generator (VT105)")
