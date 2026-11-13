// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT100 Standard Terminal Port (STP) connector and options

    The STP is J3 on the terminal controller: 20 pairs of contacts in
    series with the communication port.  Each pair joins a terminal
    controller signal to the matching line signal, and shorts when the
    connector is empty.  An option breaks the pairs it controls; the usual
    arrangement is two UARTs, a "local" one talking to the terminal's 8251
    and a "line" one talking to the host, with the modem lines passed
    through.  An option also gets LOCAL CLK (16 x 15,734 baud, from the
    video timing) to clock the terminal's 8251 and its own local UART, and
    raises OPTION PRESENT, which the firmware reads from the flag buffer.

    Every signal on the connector is modelled (TM-003 7.5.3, MP-00633
    sheet 3).  TC INIT is bit 5 of the brightness D/A latch (MP-00633 sheet
    4): high while the terminal initialises, at power-up, on RIS and before
    a confidence test.

    Source: VT100 technical manual EK-VT100-TM-003, chapter 7 (STP Option
    Interface Guide), 4.1.5.18 and 4.3.15.

***************************************************************************/

#include "emu.h"
#include "vt100_stp.h"

#include "bus/rs232/rs232.h"
#include "cpu/i8085/i8085.h"
#include "machine/com8116.h"
#include "machine/i8251.h"
#include "machine/input_merger.h"


//**************************************************************************
//  SLOT
//**************************************************************************

DEFINE_DEVICE_TYPE(VT100_STP_SLOT, vt100_stp_slot_device, "vt100_stp_slot", "DEC VT100 Standard Terminal Port")

vt100_stp_slot_device::vt100_stp_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, VT100_STP_SLOT, tag, owner, clock)
	, device_single_card_slot_interface<device_vt100_stp_interface>(mconfig, *this)
	, m_out(*this)
	, m_card(nullptr)
{
}

// Found before any device starts, so a pin written while the terminal's
// devices start already reaches the card rather than the empty connector.
void vt100_stp_slot_device::device_resolve_objects()
{
	m_card = get_card_device();
}

void vt100_stp_slot_device::device_start()
{
}

// A closed pair joins pin 2n-1 to pin 2n.  EXT TCLK, EXT RCLK, LOCAL CLK and
// TC INIT have nothing on the other side of their pairs, so they go nowhere.
void vt100_stp_slot_device::join(u32 pin, int state)
{
	m_out[(pin & 1) ? (pin + 1) : (pin - 1)](state);
}


device_vt100_stp_interface::device_vt100_stp_interface(const machine_config &mconfig, device_t &device)
	: device_interface(device, "vt100stp")
	, m_slot(dynamic_cast<vt100_stp_slot_device *>(device.owner()))
{
}


//**************************************************************************
//  VT1XX-AC PRINTER PORT
//**************************************************************************

/***************************************************************************

    The VT1XX-AC printer port option adds a serial printer port.  It takes
    the STP and comes with its own terminal firmware (23-095E2, 23-096E2,
    23-139E2, 23-140E2), which adds SET-UP C for the printer settings.  The
    technical manual gives only its installation and SET-UP: factory
    switch settings 1 and 5 on, 2-4 off (5-69).

    The board layout here is read from its firmware, 23-029E4:
    - an 8085 with the ROM at 0000-1FFF (a 4K program, see the ROM) and
      RAM at 2400-27FF
    - three 8251s, all set up as x16 clock, 8 data bits, no parity, 1 stop:
      80/90 the local side (the terminal's 8251), A0/B0 the line (host)
      side, C0/D0 the printer.  At power-up the firmware waits for an XON
      on the local side, which the terminal sends when it starts, then
      sends it DECID (ESC Z) to identify it.  It copies the local 8251's DSR
      (the terminal's DTR) to the line 8251's DTR.  The line's DSR is taken
      to pass straight through to the terminal (inferred).
    - the printer speed: the firmware writes its SET-UP C speed code (the
      VT100's 0-15 rate table; 6 = 300 baud, the default) by executing
      OUT <E0 + code>: the code is in the address, not the data (the first
      one goes out with A = 25h).  A baud rate generator taking A0-A3 at
      E0-EF is assumed; its part isn't known, so the terminal's own
      COM5016-013 is used.
    - interrupts: receiver ready on RST 6.5 and transmitter ready on RST
      5.5, which the firmware enables (SIM 0Ch).  Each 8251's RTS output
      gates its transmitter ready: the firmware stops a channel by setting
      RTS (output low) and starts it by clearing it, and without the gate
      the idle transmitters would hold RST 5.5 for good.  Both inferred.
    - a timer on RST 7.5: it ends an escape sequence from the host (40
      ticks after ESC) before passing it on, and times the BREAK it sends
      the host when the terminal sends one: 220 ticks, or 2800 if the
      terminal has dropped DTR (a long break).  The VT1XX-AC user guide
      gives 0.2333 s and 3.5 s, each +/-10%, which only a tick of 857-889 Hz
      meets.  LOCAL CLK / 288 (scan line rate / 18, 874 Hz) is assumed:
      0.252 s and 3.20 s.

    The printer connector carries only TXD, RXD (XON/XOFF from the
    printer), DSR (the printer's DTR, its ready line) and DTR, which is on
    whenever the terminal is (EK-VT1AC-UG-002, chapter 6).  While DSR is off
    the firmware sends the printer nothing and carries on where it stopped
    when DSR returns.

    TODO:
    - RAM: the firmware tests and uses only 2400-27FF (1K); whether
      2000-23FF exists or mirrors it isn't known
    - the switches: the firmware reads no port for them
    - an "intelligent" STP board, maybe an older version or a prototype:
      an expansion board with a processor and DMA, a RAM/send buffer for
      the printer port.  Two banks of two EPROMs (2K or 4K each, jumpers
      W2/W3 and W4/W5) and two PROMs, none dumped: 23-003E3 (E10),
      23-004E3 (E4), 23-005E3 (E9), 23-006E3 (E3); PROM A 23-312A1 (E26,
      8085 I/O decode?), PROM B 23-313A1 (E15, ROM and RAM page select?)

***************************************************************************/

namespace {

class vt100_stp_ac_device : public device_t, public device_vt100_stp_interface
{
public:
	vt100_stp_ac_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, VT100_STP_AC, tag, owner, clock)
		, device_vt100_stp_interface(mconfig, *this)
		, m_cpu(*this, "cpu")
		, m_line(*this, "line")
		, m_local(*this, "local")
		, m_prt(*this, "prtuart")
		, m_brg(*this, "brg")
		, m_rxint(*this, "rxint")
		, m_printer(*this, "printer")
	{
	}

	virtual void stp_w(u32 pin, int state) override;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset_after_children() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

private:
	required_device<i8085a_cpu_device> m_cpu;
	required_device<i8251_device> m_line;
	required_device<i8251_device> m_local;
	required_device<i8251_device> m_prt;
	required_device<com8116_device> m_brg;
	required_device<input_merger_device> m_rxint;
	required_device<rs232_port_device> m_printer;

	u16 m_tick = 0; // LOCAL CLK divider for the RST 7.5 timer
	u8 m_local_clk = 0;
	u8 m_txrdy = 0; // transmitter ready, one bit per 8251
	u8 m_rts = 0;   // RTS output level, one bit per 8251

	template <u32 N> void txrdy_w(int state) { m_txrdy = (m_txrdy & ~(1 << N)) | (state << N); txint_update(); }
	template <u32 N> void rts_w(int state) { m_rts = (m_rts & ~(1 << N)) | (state << N); txint_update(); }
	void txint_update();
	void local_clk_w(int state);

	void brg_w(offs_t offset, u8 data);

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;
};

ROM_START( vt100_stp_ac )
	ROM_REGION(0x2000, "cpu", 0)
	// The program is 4K: its power-up checksum covers 0000-0FFF (at 00DC) and
	// comes out right, and nothing refers above 0E80.  1000-17FF repeats
	// 0800-0FFF and 1800-1FFF is empty, from dumping it as an 8K part (A11 to
	// pin 18, A12 to pin 21, A13 to pin 20).
	ROM_LOAD( "23-029e4.stp.e14", 0x0000, 0x2000, CRC(da55c62b) SHA1(261b02b774d57253d1dedecab8ca0e368c2a96cd)) // Label: "S 8218 // C43020 // 23029E4 (C) DEC // TP02" @E14
ROM_END

const tiny_rom_entry *vt100_stp_ac_device::device_rom_region() const
{
	return ROM_NAME(vt100_stp_ac);
}

void vt100_stp_ac_device::mem_map(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x1fff).rom().region("cpu", 0);
	map(0x2000, 0x27ff).ram();
}

void vt100_stp_ac_device::io_map(address_map &map)
{
	map(0x80, 0x80).rw(m_local, FUNC(i8251_device::data_r), FUNC(i8251_device::data_w));
	map(0x90, 0x90).rw(m_local, FUNC(i8251_device::status_r), FUNC(i8251_device::control_w));
	map(0xa0, 0xa0).rw(m_line, FUNC(i8251_device::data_r), FUNC(i8251_device::data_w));
	map(0xb0, 0xb0).rw(m_line, FUNC(i8251_device::status_r), FUNC(i8251_device::control_w));
	map(0xc0, 0xc0).rw(m_prt, FUNC(i8251_device::data_r), FUNC(i8251_device::data_w));
	map(0xd0, 0xd0).rw(m_prt, FUNC(i8251_device::status_r), FUNC(i8251_device::control_w));
	map(0xe0, 0xef).w(FUNC(vt100_stp_ac_device::brg_w));
}

void vt100_stp_ac_device::brg_w(offs_t offset, u8 data)
{
	// the rate code is A0-A3; one speed for both directions (SET-UP C has a
	// single T/R setting)
	m_brg->str_w(offset);
	m_brg->stt_w(offset);
}

void vt100_stp_ac_device::local_clk_w(int state)
{
	// RST 7.5 timer: a square wave at LOCAL CLK / 288 (assumed, see above)
	if (state && !m_local_clk)
	{
		if (++m_tick == 288)
			m_tick = 0;
		if (m_tick == 0 || m_tick == 144)
			m_cpu->set_input_line(I8085_RST75_LINE, m_tick ? CLEAR_LINE : ASSERT_LINE);
	}
	m_local_clk = state;

	// LOCAL CLK clocks both ends of the local link
	m_local->write_txc(state);
	m_local->write_rxc(state);
	slot().drive(vt100_stp_slot_device::INTERNAL_TCLK, state);
	slot().drive(vt100_stp_slot_device::INTERNAL_RCLK, state);
}

void vt100_stp_ac_device::stp_w(u32 pin, int state)
{
	using stp = vt100_stp_slot_device;
	switch (pin)
	{
	case stp::TC_TXD:    m_local->write_rxd(state); break;
	case stp::TC_DTR:    m_local->write_dsr(state); break;
	case stp::TC_TCLK:   m_line->write_txc(state); break;
	case stp::TC_RCLK:   m_line->write_rxc(state); break;
	case stp::LOCAL_CLK: local_clk_w(state); break;
	case stp::LINE_RXD:  m_line->write_rxd(state); break;
	case stp::LINE_DSR:  m_line->write_dsr(state); slot().join(pin, state); break;

	// TC INIT holds the card in reset (assumed: TM-003 7.4.2.4 asks options
	// to reset on it).  The 8251s are reset with the 8085: the firmware
	// writes their mode words (00D4, 018D, 0192) without the software reset
	// sequence, so it expects them fresh from a hardware reset.
	case stp::TC_INIT:
		m_cpu->set_input_line(INPUT_LINE_RESET, state ? ASSERT_LINE : CLEAR_LINE);
		if (state)
		{
			m_line->reset();
			m_local->reset();
			m_prt->reset();
		}
		break;

	// the modem lines and the speed lines pass through (assumed)
	default:             slot().join(pin, state); break;
	}
}

void vt100_stp_ac_device::txint_update()
{
	m_cpu->set_input_line(I8085_RST55_LINE, (m_txrdy & m_rts) ? ASSERT_LINE : CLEAR_LINE);
}

void vt100_stp_ac_device::device_start()
{
	save_item(NAME(m_tick));
	save_item(NAME(m_local_clk));
	save_item(NAME(m_txrdy));
	save_item(NAME(m_rts));

	// CTS isn't known to be wired on the board; hold it asserted so that the
	// transmitters run
	m_line->write_cts(0);
	m_local->write_cts(0);
	m_prt->write_cts(0);
}

void vt100_stp_ac_device::device_reset_after_children()
{
	// the printer connector's DTR is on whenever the terminal is.  Not in
	// device_start: a device on the connector that answers DTR (a loopback
	// plug) has not started yet then, and would never assert DSR.  After the
	// children, so that resetting the device on the connector doesn't undo it
	m_printer->write_dtr(0);
}

void vt100_stp_ac_device::device_add_mconfig(machine_config &config)
{
	I8085A(config, m_cpu, 4.9152_MHz_XTAL);
	m_cpu->set_addrmap(AS_PROGRAM, &vt100_stp_ac_device::mem_map);
	m_cpu->set_addrmap(AS_IO, &vt100_stp_ac_device::io_map);

	INPUT_MERGER_ANY_HIGH(config, m_rxint).output_handler().set_inputline(m_cpu, I8085_RST65_LINE);

	I8251(config, m_line, 4.9152_MHz_XTAL / 2);
	m_line->txd_handler().set(FUNC(vt100_stp_ac_device::drive_w<vt100_stp_slot_device::LINE_TXD>));
	m_line->dtr_handler().set(FUNC(vt100_stp_ac_device::drive_w<vt100_stp_slot_device::LINE_DTR>));
	m_line->rxrdy_handler().set(m_rxint, FUNC(input_merger_device::in_w<0>));
	m_line->txrdy_handler().set(FUNC(vt100_stp_ac_device::txrdy_w<0>));
	m_line->rts_handler().set(FUNC(vt100_stp_ac_device::rts_w<0>));

	I8251(config, m_local, 4.9152_MHz_XTAL / 2);
	m_local->txd_handler().set(FUNC(vt100_stp_ac_device::drive_w<vt100_stp_slot_device::TC_RXD>));
	m_local->rxrdy_handler().set(m_rxint, FUNC(input_merger_device::in_w<1>));
	m_local->txrdy_handler().set(FUNC(vt100_stp_ac_device::txrdy_w<1>));
	m_local->rts_handler().set(FUNC(vt100_stp_ac_device::rts_w<1>));

	I8251(config, m_prt, 4.9152_MHz_XTAL / 2);
	m_prt->txd_handler().set(m_printer, FUNC(rs232_port_device::write_txd));
	m_prt->rxrdy_handler().set(m_rxint, FUNC(input_merger_device::in_w<2>));
	m_prt->txrdy_handler().set(FUNC(vt100_stp_ac_device::txrdy_w<2>));
	m_prt->rts_handler().set(FUNC(vt100_stp_ac_device::rts_w<2>));

	COM5016_013(config, m_brg, XTAL(24'883'200) / 9);
	m_brg->fr_handler().set(m_prt, FUNC(i8251_device::write_rxc));
	m_brg->ft_handler().set(m_prt, FUNC(i8251_device::write_txc));

	RS232_PORT(config, m_printer, default_rs232_devices, nullptr);
	m_printer->rxd_handler().set(m_prt, FUNC(i8251_device::write_rxd));
	m_printer->dsr_handler().set(m_prt, FUNC(i8251_device::write_dsr));
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(VT100_STP_AC, device_vt100_stp_interface, vt100_stp_ac_device, "vt100_stp_ac", "DEC VT1XX-AC printer port")


//**************************************************************************
//  VT103 STP CONNECTOR CARD
//**************************************************************************

/***************************************************************************

    The VT103 STP connector card (5414350, MP00731 page 67; EK-VT103-UG-001
    3.2.2) makes the terminal the console of the LSI-11 built into the
    VT103.  It has no parts besides its connectors:
    - J1 (console): the terminal's TXD and RXD, LOCAL CLK and, through
      jumper W1, TC INIT.  A 10-wire cable takes it to a console channel
      of the LSI-11's serial card (a DLV11-J or MXV11).  The STP signals
      are at EIA levels (the terminal's 1488/1489 sit on its side of the
      STP, MP-00633 sheet 3), so J1 is an RS-232 port.
    - LOCAL CLK clocks the terminal's 8251 both ways, so the console runs
      at about 15,734 baud whatever SET-UP says (UG 3.2.2: "approximately
      15K baud").  The serial card takes its own clock from J1.
    - J2 and J3: the line side of every pair, and the terminal's transmit
      clock, for a cable to a second serial channel, which then has the
      terminal's EIA connector.  Nothing is fitted there in this model, so
      the EIA connector is dead, as it is with no cable on J2/J3.
    - The terminal's handshake inputs (CTS, DSR, CD, RI, SPDI) and its RTS,
      DTR and SPDS outputs go nowhere; the 1489s read the open inputs as
      off.  OPTION PRESENT (pin 34) is not grounded, so it reads high.

    Not modelled: W1 (TC INIT to J1 pin L, for resetting the LSI-11) and
    the clocks on J1 and J2, which nothing here takes.  The serial devices
    that plug into J1 (null_modem, pty) run at fixed rates, none within
    8% of 15,734 baud, so only a loopback plug talks to it yet.

***************************************************************************/

namespace {

class vt100_stp_vt103_device : public device_t, public device_vt100_stp_interface
{
public:
	vt100_stp_vt103_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, VT100_STP_VT103, tag, owner, clock)
		, device_vt100_stp_interface(mconfig, *this)
		, m_console(*this, "console")
	{
	}

	virtual void stp_w(u32 pin, int state) override;

protected:
	virtual void device_start() override ATTR_COLD { }
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:
	required_device<rs232_port_device> m_console;
};

void vt100_stp_vt103_device::stp_w(u32 pin, int state)
{
	using stp = vt100_stp_slot_device;
	switch (pin)
	{
	case stp::TC_TXD:
		m_console->write_txd(state);
		break;
	case stp::LOCAL_CLK:
		slot().drive(stp::INTERNAL_TCLK, state);
		slot().drive(stp::INTERNAL_RCLK, state);
		break;
	default:
		// everything else ends on J2/J3, or isn't wired
		break;
	}
}

void vt100_stp_vt103_device::device_reset()
{
	using stp = vt100_stp_slot_device;

	// open inputs on the terminal side read as off
	slot().drive(stp::TC_CTS, 1);
	slot().drive(stp::TC_DSR, 1);
	slot().drive(stp::TC_CD, 1);
	slot().drive(stp::TC_RI, 1);
	slot().drive(stp::TC_SPDI, 1);

	// and the EIA connector's outputs are not driven: idle
	slot().drive(stp::LINE_TXD, 1);
	slot().drive(stp::LINE_RTS, 1);
	slot().drive(stp::LINE_DTR, 1);
	slot().drive(stp::LINE_SPDS, 1);
}

void vt100_stp_vt103_device::device_add_mconfig(machine_config &config)
{
	RS232_PORT(config, m_console, default_rs232_devices, nullptr);
	m_console->rxd_handler().set(FUNC(vt100_stp_vt103_device::drive_w<vt100_stp_slot_device::TC_RXD>));
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(VT100_STP_VT103, device_vt100_stp_interface, vt100_stp_vt103_device, "vt100_stp_vt103", "DEC VT103 STP connector card")


void vt100_stp_options(device_slot_interface &device)
{
	device.option_add("vt1xxac", VT100_STP_AC);
	device.option_add("vt103", VT100_STP_VT103);
	device.option_add("vt18x", VT100_STP_VT18X);
}
