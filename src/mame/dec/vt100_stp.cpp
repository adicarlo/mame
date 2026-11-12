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

    Only the pairs an option here breaks are modelled: TXD, RXD, DTR, DSR
    and the terminal 8251's clocks.  RTS, CTS, CD, RI and the speed lines
    still go straight between the terminal and the RS-232 port.  TC INIT
    isn't modelled; options reset with the machine.

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
	, m_tc_rxd(*this)
	, m_tc_dsr(*this)
	, m_int_tclk(*this)
	, m_int_rclk(*this)
	, m_line_txd(*this)
	, m_line_dtr(*this)
	, m_card(nullptr)
{
}

void vt100_stp_slot_device::device_resolve_objects()
{
	m_card = get_card_device();
}

// With no option the contact pairs short, joining each terminal signal to
// its line signal; LOCAL CLK goes nowhere.

void vt100_stp_slot_device::tc_txd_w(int state)
{
	if (m_card)
		m_card->tc_txd_w(state);
	else
		m_line_txd(state);
}

void vt100_stp_slot_device::tc_dtr_w(int state)
{
	if (m_card)
		m_card->tc_dtr_w(state);
	else
		m_line_dtr(state);
}

void vt100_stp_slot_device::tc_tclk_w(int state)
{
	if (m_card)
		m_card->tc_tclk_w(state);
	else
		m_int_tclk(state);
}

void vt100_stp_slot_device::tc_rclk_w(int state)
{
	if (m_card)
		m_card->tc_rclk_w(state);
	else
		m_int_rclk(state);
}

void vt100_stp_slot_device::local_clk_w(int state)
{
	if (m_card)
		m_card->local_clk_w(state);
}

void vt100_stp_slot_device::line_rxd_w(int state)
{
	if (m_card)
		m_card->line_rxd_w(state);
	else
		m_tc_rxd(state);
}

void vt100_stp_slot_device::line_dsr_w(int state)
{
	if (m_card)
		m_card->line_dsr_w(state);
	else
		m_tc_dsr(state);
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
    - an 8085 with the ROM at 0000-1FFF and RAM at 2000-27FF
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
      ticks after ESC) before passing it on, and times BREAK on the local
      link (220 ticks short, 2800 long).  LOCAL CLK / 256 (about 983 Hz) is
      assumed: it makes the short break 0.224 s against the VT100's
      0.2333 s, but the long one 2.85 s against 3.5 s.

    TODO:
    - check the 23-029E4 dump: in 2K blocks it reads A, B, B, empty
    - the switches: the firmware reads no port for them
    - the printer port's handshake lines

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
	{
	}

	virtual void tc_txd_w(int state) override { m_local->write_rxd(state); }
	virtual void tc_dtr_w(int state) override { m_local->write_dsr(state); }
	virtual void tc_tclk_w(int state) override { m_line->write_txc(state); }
	virtual void tc_rclk_w(int state) override { m_line->write_rxc(state); }
	virtual void local_clk_w(int state) override;
	virtual void line_rxd_w(int state) override { m_line->write_rxd(state); }
	virtual void line_dsr_w(int state) override { m_line->write_dsr(state); tc_dsr(state); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

private:
	required_device<i8085a_cpu_device> m_cpu;
	required_device<i8251_device> m_line;
	required_device<i8251_device> m_local;
	required_device<i8251_device> m_prt;
	required_device<com8116_device> m_brg;

	u8 m_tick = 0; // LOCAL CLK divider for the RST 7.5 timer
	int m_local_clk = 0;
	u8 m_txrdy = 0; // transmitter ready, one bit per 8251
	u8 m_rts = 0;   // RTS output level, one bit per 8251

	template <unsigned N> void txrdy_w(int state) { m_txrdy = (m_txrdy & ~(1 << N)) | (state << N); txint_update(); }
	template <unsigned N> void rts_w(int state) { m_rts = (m_rts & ~(1 << N)) | (state << N); txint_update(); }
	void txint_update();

	void brg_w(offs_t offset, u8 data);

	// 8251 outputs to the connector
	void line_txd_w(int state) { line_txd(state); }
	void line_dtr_w(int state) { line_dtr(state); }
	void local_txd_w(int state) { tc_rxd(state); }

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;
};

ROM_START( vt100_stp_ac )
	ROM_REGION(0x2000, "cpu", 0)
	// It may be in the wrong order: it was dumped with A11 to pin 18, A12 to
	// pin 21 and A13 to pin 20, which may not be the right assignments.  At
	// worst its eight parts are out of order.
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
	// RST 7.5 timer: a square wave at LOCAL CLK / 256 (assumed, see above)
	if (state && !m_local_clk)
	{
		m_tick++;
		if (!(m_tick & 0x7f))
			m_cpu->set_input_line(I8085_RST75_LINE, BIT(m_tick, 7) ? ASSERT_LINE : CLEAR_LINE);
	}
	m_local_clk = state;

	// LOCAL CLK clocks both ends of the local link
	m_local->write_txc(state);
	m_local->write_rxc(state);
	int_tclk(state);
	int_rclk(state);
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

void vt100_stp_ac_device::device_add_mconfig(machine_config &config)
{
	I8085A(config, m_cpu, 4.9152_MHz_XTAL);
	m_cpu->set_addrmap(AS_PROGRAM, &vt100_stp_ac_device::mem_map);
	m_cpu->set_addrmap(AS_IO, &vt100_stp_ac_device::io_map);

	INPUT_MERGER_ANY_HIGH(config, "rxint").output_handler().set_inputline(m_cpu, I8085_RST65_LINE);

	I8251(config, m_line, 4.9152_MHz_XTAL / 2);
	m_line->txd_handler().set(FUNC(vt100_stp_ac_device::line_txd_w));
	m_line->dtr_handler().set(FUNC(vt100_stp_ac_device::line_dtr_w));
	m_line->rxrdy_handler().set("rxint", FUNC(input_merger_device::in_w<0>));
	m_line->txrdy_handler().set(FUNC(vt100_stp_ac_device::txrdy_w<0>));
	m_line->rts_handler().set(FUNC(vt100_stp_ac_device::rts_w<0>));

	I8251(config, m_local, 4.9152_MHz_XTAL / 2);
	m_local->txd_handler().set(FUNC(vt100_stp_ac_device::local_txd_w));
	m_local->rxrdy_handler().set("rxint", FUNC(input_merger_device::in_w<1>));
	m_local->txrdy_handler().set(FUNC(vt100_stp_ac_device::txrdy_w<1>));
	m_local->rts_handler().set(FUNC(vt100_stp_ac_device::rts_w<1>));

	I8251(config, m_prt, 4.9152_MHz_XTAL / 2);
	m_prt->txd_handler().set("printer", FUNC(rs232_port_device::write_txd));
	m_prt->rxrdy_handler().set("rxint", FUNC(input_merger_device::in_w<2>));
	m_prt->txrdy_handler().set(FUNC(vt100_stp_ac_device::txrdy_w<2>));
	m_prt->rts_handler().set(FUNC(vt100_stp_ac_device::rts_w<2>));

	COM5016_013(config, m_brg, XTAL(24'883'200) / 9);
	m_brg->fr_handler().set(m_prt, FUNC(i8251_device::write_rxc));
	m_brg->ft_handler().set(m_prt, FUNC(i8251_device::write_txc));

	rs232_port_device &printer(RS232_PORT(config, "printer", default_rs232_devices, nullptr));
	printer.rxd_handler().set(m_prt, FUNC(i8251_device::write_rxd));
	printer.dsr_handler().set(m_prt, FUNC(i8251_device::write_dsr));
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(VT100_STP_AC, device_vt100_stp_interface, vt100_stp_ac_device, "vt100_stp_ac", "DEC VT1XX-AC printer port")


void vt100_stp_options(device_slot_interface &device)
{
	device.option_add("vt1xxac", VT100_STP_AC);
}
