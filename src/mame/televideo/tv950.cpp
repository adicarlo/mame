// license:BSD-3-Clause
// copyright-holders:R. Belmont
/***************************************************************************

    2013-09-10 Skeleton driver for Televideo TV950
    2016-07-30 Preliminary not-so-skeleton driver

    TODO:
    - Bidirectional communications mode
    - Smooth scroll: the row latch at 9000 (counter A60 supplies RA0-3 to
      the character ROMs on the real board) is not used yet; RA comes from
      the CRTC, which is right only for full 10-line rows

    Video: the 6545 never draws a whole frame.  The VIA's T2 counts HSYNCs
    and its IRQ is the 6502's NMI; each NMI reads VIA port A, whose CA2
    handshake resets the CRTC, then loads R12/R13 with the next row's
    address and T2 with the row's height.  So every character row comes
    from wherever the firmware's row table points -- display RAM, or work
    RAM for the 25th (status) line -- which is how lines scroll or stay
    locked independently.  The CRTC's own frame (R4=2, R6=1) only runs
    through in the vertical blank, which is where VSYNC and the cursor
    blink count happen.  The driver records each displayed line's address
    and raster at DE and draws the frame from that record.

    Hardware:
    6502A CPU
    6545 CRTC
    6522A VIA, wired to count HSYNCs and to enable the 6502 to pull RESET on the CRTC
    3x 6551 ACIA  1 for the keyboard, 1 for the modem port, 1 for the printer port

    VIA hookup (see schematics):
    PA3 = beep?
    PA5 = inverse video
    PA6 = IRQ out: the NMI handler drives it low after the last row to ask
          for the frame's housekeeping (cursor position, blink, ...), which
          the IRQ handler runs while it is low; DDRA is all outputs
    PA7 = force blank
    PB6 = Hblank in
    PB7 = out speaker
    CA1 = reset CRTC in
    CA2 = reset CRTC out
    CB2 = blink timer

    IRQ = ACIAs (all 3 ORed together)
    NMI = 6522 VIA's IRQ line

    http://www.bitsavers.org/pdf/televideo/950/Model_950_Terminal_Theory_of_Operation_26Jan1981.pdf
    http://www.bitsavers.org/pdf/televideo/950/2002100_Model_950_Maintenance_Manual_Nov1983.pdf
    http://www.bitsavers.org/pdf/televideo/950/B300002-001_Model_950_Operators_Manual_Feb81.pdf

****************************************************************************/

#include "emu.h"
#include "bus/rs232/rs232.h"
#include "cpu/m6502/m6502.h"
#include "machine/input_merger.h"
#include "machine/6522via.h"
#include "machine/mos6551.h"
#include "tv950kb.h"
#include "video/mc6845.h"
#include "screen.h"


namespace {

#define MASTER_CLOCK XTAL(23'814'000)

class tv950_state : public driver_device
{
public:
	tv950_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_via(*this, "via")
	, m_mainirq(*this, "mainirq")
		, m_crtc(*this, "crtc")
		, m_uart(*this, "a%uuart", 49U)
		, m_keyboard(*this, "keyboard")
		, m_vram(*this, "vram")
		, m_ram(*this, "mainram")
		, m_gfx(*this, "graphics")
		, m_dsw(*this, "DSW%u", 0U)
	{ }

	void tv950(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void via_a_w(uint8_t data);
	void via_b_w(uint8_t data);
	uint8_t via_b_r();
	MC6845_ON_UPDATE_ADDR_CHANGED(crtc_update_addr);
	void crtc_de_w(int state);
	void crtc_vs_w(int state);
	void via_cb2_w(int state);
	TIMER_CALLBACK_MEMBER(crtc_reset_release);
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	void row_addr_w(uint8_t data);
	void via_crtc_reset_w(int state);

	void tv950_mem(address_map &map) ATTR_COLD;

	// what the CRTC put out on one displayed scan line
	struct scanline
	{
		uint16_t addr;
		uint8_t ra;
		int16_t cursor;
	};
	static constexpr int VISIBLE_LINES = 250;  // 25 rows of 10

	uint8_t m_via_row = 0;
	bool m_bow = false;            // PA5: black on white
	bool m_force_blank = false;    // PA7
	bool m_blink = false;          // CB2: blink-attribute phase
	scanline m_lines[2][VISIBLE_LINES];
	int m_fill = 0;                // buffer being recorded; the other is shown
	int m_line_count = 0;
	int m_shown_count = 0;
	emu_timer *m_reset_timer = nullptr;

	required_device<m6502_device> m_maincpu;
	required_device<via6522_device> m_via;
	required_device<input_merger_device> m_mainirq;
	required_device<r6545_1_device> m_crtc;
	required_device_array<mos6551_device, 3> m_uart;
	required_device<tv950kb_device> m_keyboard;
	required_shared_ptr<uint8_t> m_vram;
	required_shared_ptr<uint8_t> m_ram;
	required_region_ptr<uint16_t> m_gfx;
	required_ioport_array<4> m_dsw;

	int m_row_addr = 0;
};

void tv950_state::machine_start()
{
	m_uart[0]->write_dcd(0);
	m_uart[0]->write_dsr(0);
	m_uart[2]->write_cts(0);

	m_reset_timer = timer_alloc(FUNC(tv950_state::crtc_reset_release), this);

	save_item(NAME(m_via_row));
	save_item(NAME(m_bow));
	save_item(NAME(m_force_blank));
	save_item(NAME(m_blink));
	save_item(NAME(m_row_addr));
}

void tv950_state::tv950_mem(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x07ff).ram().share("mainram");
	map(0x2000, 0x3fff).ram().share("vram"); // VRAM
	map(0x8100, 0x8100).rw(m_crtc, FUNC(r6545_1_device::status_r), FUNC(r6545_1_device::address_w));
	map(0x8101, 0x8101).rw(m_crtc, FUNC(r6545_1_device::register_r), FUNC(r6545_1_device::register_w));
	map(0x9000, 0x9000).w(FUNC(tv950_state::row_addr_w));
	map(0x9300, 0x9303).rw(m_uart[0], FUNC(mos6551_device::read), FUNC(mos6551_device::write)); // CS0 = AB9
	map(0x9500, 0x9503).rw(m_uart[2], FUNC(mos6551_device::read), FUNC(mos6551_device::write)); // CS0 = AB10
	map(0x9900, 0x9903).rw(m_uart[1], FUNC(mos6551_device::read), FUNC(mos6551_device::write)); // CS0 = AB11
	map(0xb100, 0xb10f).m(m_via, FUNC(via6522_device::map));
	map(0xe000, 0xffff).rom().region("maincpu", 0);
}


/* Input ports */
static INPUT_PORTS_START( tv950 )
	PORT_START("DSW0")
	PORT_DIPNAME( 0x01, 0x00, "S01")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x01, DEF_STR( Off ))
	PORT_DIPNAME( 0x02, 0x00, "S02")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x02, DEF_STR( Off ))
	PORT_DIPNAME( 0x04, 0x00, "S03")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x04, DEF_STR( Off ))
	PORT_DIPNAME( 0x08, 0x00, "S04")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x08, DEF_STR( Off ))
	PORT_DIPNAME( 0x10, 0x00, "S05")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x10, DEF_STR( Off ))
	PORT_DIPNAME( 0x20, 0x20, "S06")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x20, DEF_STR( Off ))
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("DSW1")
	PORT_DIPNAME( 0x01, 0x00, "S07")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x01, DEF_STR( Off ))
	PORT_DIPNAME( 0x02, 0x00, "S08")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x02, DEF_STR( Off ))
	PORT_DIPNAME( 0x04, 0x00, "S09")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x04, DEF_STR( Off ))
	PORT_DIPNAME( 0x08, 0x00, "S10")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x08, DEF_STR( Off ))
	PORT_DIPNAME( 0x10, 0x00, "S11")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x10, DEF_STR( Off ))
	PORT_DIPNAME( 0x20, 0x20, "S12")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x20, DEF_STR( Off ))
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("DSW2")
	PORT_DIPNAME( 0x01, 0x00, "S13")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x01, DEF_STR( Off ))
	PORT_DIPNAME( 0x02, 0x00, "S14")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x02, DEF_STR( Off ))
	PORT_DIPNAME( 0x04, 0x00, "S15")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x04, DEF_STR( Off ))
	PORT_DIPNAME( 0x08, 0x00, "S16")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x08, DEF_STR( Off ))
	PORT_DIPNAME( 0x10, 0x00, "S17")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x10, DEF_STR( Off ))
	PORT_DIPNAME( 0x20, 0x20, "S18")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x20, DEF_STR( Off ))
	PORT_BIT( 0xc0, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("DSW3")
	PORT_DIPNAME( 0x01, 0x00, "S19")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x01, DEF_STR( Off ))
	PORT_DIPNAME( 0x02, 0x00, "S20")
	PORT_DIPSETTING(    0x00, DEF_STR( On ))
	PORT_DIPSETTING(    0x02, DEF_STR( Off ))
	PORT_BIT( 0xfc, IP_ACTIVE_LOW, IPT_UNUSED )
INPUT_PORTS_END


void tv950_state::machine_reset()
{
	m_via_row = 0;
	m_line_count = 0;
	m_shown_count = 0;
}

// R8 selects transparent memory addressing, which needs a handler
MC6845_ON_UPDATE_ADDR_CHANGED( tv950_state::crtc_update_addr )
{
}

// a displayed line begins: note where its characters come from
void tv950_state::crtc_de_w(int state)
{
	if (state && m_line_count < VISIBLE_LINES)
		m_lines[m_fill][m_line_count++] = { m_crtc->line_address(), m_crtc->get_ra(), m_crtc->cursor_x() };
}

// VSYNC comes once a frame, in the blank between the last row and the first
void tv950_state::crtc_vs_w(int state)
{
	if (state)
	{
		m_shown_count = m_line_count;
		m_fill ^= 1;
		m_line_count = 0;
	}
}

// CA2 low pulls the CRTC's RESET; A64/A71 release it seven character clocks
// later, and that release is CA1's falling edge (PCR selects it), which
// ends the handshake and raises CA2 again
void tv950_state::via_crtc_reset_w(int state)
{
	if (!state)
	{
		m_crtc->reset_w(0);
		m_via->write_ca1(1);
		m_reset_timer->adjust(attotime::from_ticks(7, MASTER_CLOCK / 14));
	}
}

TIMER_CALLBACK_MEMBER(tv950_state::crtc_reset_release)
{
	m_crtc->reset_w(1);
	m_via->write_ca1(0);
}

void tv950_state::via_cb2_w(int state)
{
	m_blink = state;
}

void tv950_state::row_addr_w(uint8_t data)
{
	m_row_addr = data;
}

void tv950_state::via_a_w(uint8_t data)
{
	m_via_row = ~data & 15;
	m_bow = BIT(data, 5);
	m_mainirq->in_w<3>(!BIT(data, 6));
	m_force_blank = BIT(data, 7);
	// PA4 to do
}

void tv950_state::via_b_w(uint8_t data)
{
	// bit 3 of m_via_row must be active as well?
	m_keyboard->rx_w(!BIT(data, 7));
}

uint8_t tv950_state::via_b_r()
{
	uint8_t data = 0xff;
	for (int n = 0; n < 4; n++)
		if (BIT(m_via_row, n))
			data &= m_dsw[n]->read();
	return data;
}

// Draw the last complete frame from the recorded lines.  The CRTC addresses
// the CPU's memory: display RAM at 2000-3FFF, work RAM (the status line)
// below.  A character code 90-9F is an attribute: it takes a cell, shown as
// a half-intensity space, and holds from there to the end of the screen in
// display order (A18/A19 carry it from row to row).  Its low bits are
// blank, blink, reverse and underline; half intensity is bit 14 of the
// glyph word, per character.
uint32_t tv950_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	rgb_t const bright = rgb_t::white();
	rgb_t const dim(0xa0, 0xa0, 0xa0);
	scanline const *const lines = m_lines[m_fill ^ 1];
	uint8_t attr = 0, row_attr = 0;

	for (int y = 0; y < VISIBLE_LINES; y++)
	{
		bool const draw = y >= cliprect.top() && y <= cliprect.bottom();
		if (y >= m_shown_count || m_force_blank)
		{
			if (draw)
				bitmap.plot_box(0, y, 80 * 14, 1, rgb_t::black());
			continue;
		}

		scanline const &line = lines[y];
		if (line.ra == 0)
			row_attr = attr;
		attr = row_attr;

		uint32_t *p = &bitmap.pix(y);
		for (int x = 0; x < 80; x++)
		{
			uint16_t const addr = (line.addr + x) & 0x3fff;
			uint8_t const chr = BIT(addr, 13) ? m_vram[addr & 0x1fff] : m_ram[addr & 0x07ff];
			uint16_t glyph = m_gfx[chr << 4 | (line.ra & 15)];
			uint8_t cell_attr = attr;
			if ((chr & 0xf0) == 0x90)
			{
				cell_attr = attr & chr;
				attr = chr & 0x0f;
				glyph = 1 << 14;
			}

			bool const hide = BIT(cell_attr, 0) || (BIT(cell_attr, 1) && m_blink);
			uint16_t dots = hide ? 0 : (glyph & 0x3fff);
			if (BIT(cell_attr, 3) && line.ra == 9)
				dots = 0x3fff;
			// flip-flop A31 delays the CRTC's CURSOR one character (DEL CURSOR)
			bool const cursor = line.cursor >= 0 && x == line.cursor + 1;
			if (BIT(cell_attr, 2) ^ cursor ^ m_bow)
				dots ^= 0x3fff;

			if (!draw)
				continue;
			rgb_t const fg = BIT(glyph, 14) ? dim : bright;
			for (int i = 0; i < 14; i++)
				*p++ = BIT(dots, 13 - i) ? fg : rgb_t::black();
		}
	}
	return 0;
}

void tv950_state::tv950(machine_config &config)
{
	/* basic machine hardware */
	M6502(config, m_maincpu, MASTER_CLOCK/14);
	m_maincpu->set_addrmap(AS_PROGRAM, &tv950_state::tv950_mem);

	input_merger_device &mainirq(INPUT_MERGER_ANY_HIGH(config, "mainirq")); // open collector
	mainirq.output_handler().set_inputline(m_maincpu, m6502_device::IRQ_LINE);

	screen_device &screen(SCREEN(config, "screen"));
	// 105 characters of 14 dots per line (R0), 25 rows of 10 lines plus a
	// 20-line vertical blank: about 16.2 kHz and 60 Hz
	screen.set_raw(MASTER_CLOCK, 105 * 14, 0, 80 * 14, 270, 0, VISIBLE_LINES);
	screen.set_screen_update(FUNC(tv950_state::screen_update));

	// there are many 6845 CRTC submodels, the Theory of Operation manual references the Rockwell R6545-1 specificially.
	R6545_1(config, m_crtc, MASTER_CLOCK/14);
	m_crtc->set_screen("screen");
	m_crtc->set_show_border_area(false);
	m_crtc->set_char_width(14);
	m_crtc->set_on_update_addr_change_callback(FUNC(tv950_state::crtc_update_addr));
	m_crtc->out_de_callback().set(FUNC(tv950_state::crtc_de_w));
	m_crtc->out_hsync_callback().set(m_via, FUNC(via6522_device::write_pb6)).invert();
	m_crtc->out_vsync_callback().set(FUNC(tv950_state::crtc_vs_w));
	m_crtc->set_screen(nullptr);

	MOS6522(config, m_via, MASTER_CLOCK/14);
	m_via->irq_handler().set_inputline(m_maincpu, M6502_NMI_LINE);
	m_via->writepa_handler().set(FUNC(tv950_state::via_a_w));
	m_via->writepb_handler().set(FUNC(tv950_state::via_b_w));
	m_via->readpb_handler().set(FUNC(tv950_state::via_b_r));
	m_via->ca2_handler().set(FUNC(tv950_state::via_crtc_reset_w));
	m_via->cb2_handler().set(FUNC(tv950_state::via_cb2_w));

	MOS6551(config, m_uart[0], MASTER_CLOCK/14).set_xtal(MASTER_CLOCK/13); // for keyboard
	m_uart[0]->irq_handler().set("mainirq", FUNC(input_merger_device::in_w<0>));

	MOS6551(config, m_uart[1], MASTER_CLOCK/14).set_xtal(MASTER_CLOCK/13); // for main port
	m_uart[1]->irq_handler().set("mainirq", FUNC(input_merger_device::in_w<1>));
	m_uart[1]->dtr_handler().set("p3", FUNC(rs232_port_device::write_dtr));
	m_uart[1]->rts_handler().set("p3", FUNC(rs232_port_device::write_rts));
	m_uart[1]->txd_handler().set("p3", FUNC(rs232_port_device::write_txd));

	MOS6551(config, m_uart[2], MASTER_CLOCK/14).set_xtal(MASTER_CLOCK/13); // for printer port
	m_uart[2]->irq_handler().set("mainirq", FUNC(input_merger_device::in_w<2>));
	m_uart[2]->txd_handler().set("p4", FUNC(rs232_port_device::write_txd)); // to pin 3 (RXD)
	m_uart[2]->rts_handler().set("p4", FUNC(rs232_port_device::write_rts)); // to pin 5 (CTS)
	m_uart[2]->dtr_handler().set("p4", FUNC(rs232_port_device::write_dtr)); // to pin 6 (DSR)

	TV950_KEYBOARD(config, m_keyboard);
	m_keyboard->tx_cb().set(m_uart[0], FUNC(mos6551_device::write_rxd));

	rs232_port_device &p3(RS232_PORT(config, "p3", default_rs232_devices, nullptr)); // main port
	p3.dsr_handler().set(m_uart[1], FUNC(mos6551_device::write_dsr));
	p3.dcd_handler().set(m_uart[1], FUNC(mos6551_device::write_dcd));
	p3.rxd_handler().set(m_uart[1], FUNC(mos6551_device::write_rxd));
	p3.cts_handler().set(m_uart[1], FUNC(mos6551_device::write_cts));

	rs232_port_device &p4(RS232_PORT(config, "p4", default_rs232_devices, nullptr)); // printer port
	p4.dsr_handler().set(m_uart[2], FUNC(mos6551_device::write_dsr)); // from pin 20 (DTR)
	p4.rxd_handler().set(m_uart[2], FUNC(mos6551_device::write_rxd)); // from pin 2 (TXD)
}

/* ROM definition */
ROM_START( tv950 )
	ROM_REGION(0x2000, "maincpu", 0)
	ROM_LOAD( "180000-001a_a41_eb17.bin", 0x001000, 0x001000, CRC(b7187cc5) SHA1(41cc8fd51661314e03ee7e00cc1e206e9a694d92) )
	ROM_LOAD( "180000-007a_a42_67d3.bin", 0x000000, 0x001000, CRC(3ef2e6fb) SHA1(21ccfd2b50c37b715eed67671b82faa4d75fc6bb) )

	ROM_REGION16_LE(0x2000, "graphics", 0)
	ROM_LOAD16_BYTE( "180000-002a_a33_9294.bin", 0x000001, 0x001000, CRC(eaf4f346) SHA1(b4c531626846f3f055ddc086ac24fdb1b34f3f8e) )
	ROM_LOAD16_BYTE( "180000-003a_a32_7ebf.bin", 0x000000, 0x001000, CRC(783ca0b6) SHA1(1cec9a9a56ef5795809f7ca7cd2e3f61b27e698d) )

	ROM_REGION(0x10000, "user1", 0)
	// came with "tv950.zip"
	ROM_LOAD( "180000-43i.a25", 0x0000, 0x1000, CRC(ac6f0bfc) SHA1(2a3863700405fbb9e510613559d78fceee3544e8) )
	ROM_LOAD( "180000-44i.a20", 0x0000, 0x1000, CRC(db91a727) SHA1(e94724ed1a563fb846f4203ae6523ee6b4c6577f) )
	// came with "tv950kbd.zip"
	ROM_LOAD( "1800000-003a.a32", 0x0000, 0x1000, CRC(eaef0138) SHA1(7198851299fce07c95d18e32cbfbe936c0dbec2a) )
	ROM_LOAD( "1800000-002a.a33", 0x0000, 0x1000, CRC(856dd85c) SHA1(e2570017e098b0e1ead7749e9c2ac40be2367433) )
ROM_END

} // anonymous namespace


/* Driver */
//    YEAR  NAME   PARENT  COMPAT  MACHINE  INPUT  CLASS        INIT        COMPANY      FULLNAME                            FLAGS
COMP( 1981, tv950, 0,      0,      tv950,   tv950, tv950_state, empty_init, "TeleVideo", "Model 950 Video Display Terminal", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND )
