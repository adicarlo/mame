// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT18X: Z80 CP/M computer option for the VT100 (VT180)

    Two boards.  The control module (54-15152) sits in the VT100's expansion
    backplane but takes only power from it; a ribbon cable joins it to the
    paddle board (50-15149), which plugs into the STP.  The VT100 becomes
    the console: its 8251 talks to the VT18X console 8251, and its
    communication connector J4 becomes the VT18X general purpose port.
    The VT18X carries its own communication port (J1), and the paddle board
    a printer port.

    Source: EK-VT18X-TM-001 (VT180 technical manual, Feb 83), section 6.6.
    Page numbers are the manual's.

    Control module (6-53 to 6-99):
    - Z80A at 4 MHz, 64K of DRAM.  The 8K ROM overlays 0000-1FFF for reads
      until port 98 bit 0 is set; writes always go to RAM (inferred: the
      ROM is only an output-enable device on /RD, Fig 6-24).  One wait
      state on a ROM read (6-54).
    - 8251As: 40/41 console, 48/49 printer, 50/51 general purpose
      ("term 1"), 58/59 communication.  The console is clocked by LOCAL
      CLK from the VT100 (16 x 15,734 baud; the firmware sets it to x16),
      the printer and general port by a baud rate generator set at port
      90, the communication port by the VT100's transmit and receive
      clocks, taken from the STP (its SET-UP B speeds), or by the modem.
    - interrupts: IM 0.  Each 8251's RxRDY, and a 75 Hz real time clock,
      pull the Z80's INT line.  At acknowledge a buffer puts RST 18 on the
      bus while the clock is pending, otherwise RST 10 (Fig 6-23).  The
      clock is cleared by reading port 90.
    - WD1793 at 78-7B, 1 MHz, MFM only, with four single-sided 48 tpi
      drives.  The Z80 polls its DRQ and INTRQ at port 80; there is no DMA
      and neither interrupts (Fig 6-29).

    Ports:
    80 W  bits 1-0 drive select, 2 DDEN (0 = MFM), 3 motor on
       R  bits 3-0 as written, 4 Z80 INT (0 = pending), 5 drive ready
          (grounded through W6, so 0), 6 1793 INTRQ, 7 1793 DRQ
    88 R  0 ROM disabled, 1 secondary RTS and 2 speed select (as written to
          port 90), 3 ring indicator, 4 CTS, 5 speed indicator, 6 carrier
          detect (the communication port's), 7 power-up INIT (cleared by
          reading port 90)
    90 W  bits 2-0 printer speed, 5-3 general port speed (75, 150, 300,
          600, 1200, 2400, 4800, 9600), 6 secondary RTS, 7 speed select
       R  clears the real time clock interrupt and bit 7 of port 88
    98 W  0 ROM disable, 1 loopback, 2 communication clocks from the modem,
          7-4 LEDs 4-1 (0 = on); cleared by INIT
    A0 R  diagnostic: 0 track 0, 1 manufacturing test (1 in use), 2
          direction, 3 write gate, 4 write data latch, 5 MOTOR ON L (0 =
          on: the self-test requires 0 with the motor on, though Table 6-34
          reads as active high), 7-6 the selected drive

    Paddle board (Fig 6-28, 6-100):
    - STP 2 (the VT100's TXD) to the console RxD, the console TxD to STP 6
    - STP 16 (the VT100's DTR) to the console DSR
    - STP 36 LOCAL CLK to the console clocks and to STP 27 and 29, the
      VT100 8251's clocks
    - STP 28 and 30 (the VT100's baud rate generator) to the communication
      8251's clocks
    - the general port: TxD to STP 1, RxD from STP 5, DSR from STP 13; STP
      7 and 15 (RTS and DTR on J4) pulled up, so always on
    - STP 38 INIT to the control module's reset
    - the printer connector: TxD, RxD, DSR, and DTR always on

    The drive motors run while port 80 bit 3 is set or the 1793 has the
    head loaded (Fig 6-29), so with bit 3 clear they stop when the 1793
    unloads the head, 15 index pulses (3 s) after its last command: the
    three second delay of Table 6-30.  The diskette diagnostic
    (BJ-T040B-MV) checks the motor is off 3.2 s after a head-loading
    restore.  Drive READY L is unused: W6 grounds it on the board (Table
    6-12), so the 1793 always sees ready and port 80 bit 5 reads 0.  The
    1 s one-shot E63 in Fig 6-29 times head settling: the 1793's HLT rises
    when it times out after HLD rises, or at once with port 80 bit 3 set
    (E62).  Fig 6-29 doesn't show which way E63's output runs; the
    diagnostic does: after a head-loading restore it wants status bit 5
    (HLD and HLT) clear at 0.5 s and set by 2 s.

    Fig 6-26 also shows each 8251's CLK on CTL 2 MHz, and the loopback
    muxes on the receive side only: each transmitter still drives its line
    while looped back, so the self-test's console characters reach the
    VT100's screen.

    Assumed where the manual is silent:
    - the 8251s' CTS inputs are held asserted (not shown on Fig 6-26 or
      6-28)
    - the baud rate generator's part isn't named: its rates are taken as
      exact, x16
    - INIT resets the Z80 as well as the 8251s, 1793, ROM latch and LEDs
    - the communication port's status bits read 1 when asserted

    Not emulated: the extra wait states on interrupt acknowledge, and port
    A0's write data latch (bit 4; the diagnostics don't read it).

***************************************************************************/

#include "emu.h"
#include "vt100_stp.h"

#include "bus/rs232/rs232.h"
#include "cpu/z80/z80.h"
#include "imagedev/floppy.h"
#include "machine/clock.h"
#include "machine/i8251.h"
#include "machine/input_merger.h"
#include "machine/wd_fdc.h"
#include "softlist_dev.h"

#include "formats/vt180_dsk.h"


namespace {

class vt100_stp_vt18x_device : public device_t, public device_vt100_stp_interface
{
public:
	vt100_stp_vt18x_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, VT100_STP_VT18X, tag, owner, clock)
		, device_vt100_stp_interface(mconfig, *this)
		, m_cpu(*this, "cpu")
		, m_rom(*this, "rom")
		, m_ram(*this, "ram", 0x10000, ENDIANNESS_LITTLE)
		, m_rom_view(*this, "rom_view")
		, m_con(*this, "con")
		, m_prt(*this, "prt")
		, m_gen(*this, "gen")
		, m_com(*this, "com")
		, m_int(*this, "int")
		, m_prt_clk(*this, "prt_clk")
		, m_gen_clk(*this, "gen_clk")
		, m_fdc(*this, "fdc")
		, m_floppy(*this, "fdc:%u", 0U)
		, m_comm(*this, "comm")
		, m_printer(*this, "printer")
		, m_leds(*this, "vt18x_led%u", 1U)
		, m_e22(*this, "E22")
	{
	}

	virtual void stp_w(unsigned pin, int state) override;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_reset_after_children() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

public:
	DECLARE_INPUT_CHANGED_MEMBER(e22_changed);

private:
	static void floppy_formats(format_registration &fr);

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 rom_r(offs_t offset);
	u8 drive_r();
	void drive_w(u8 data);
	u8 status_r();
	u8 rtc_clear_r();
	void speed_w(u8 data);
	void diag_w(u8 data);
	u8 diag_r();

	int irq_ack(device_t &device, int irqline);
	TIMER_CALLBACK_MEMBER(rtc_tick);
	TIMER_CALLBACK_MEMBER(head_settled);
	void motor_update();
	void hld_w(int state);

	void init_board();
	void con_rxd_update();
	void gen_rxd_update();
	void prt_rxd_update();
	void com_rxd_update();
	void com_clocks_update();
	void comm_txd_update();
	void comm_rts_update();
	void comm_spds_update();

	required_device<z80_device> m_cpu;
	required_region_ptr<u8> m_rom;
	memory_share_creator<u8> m_ram;
	memory_view m_rom_view;
	required_device<i8251_device> m_con;
	required_device<i8251_device> m_prt;
	required_device<i8251_device> m_gen;
	required_device<i8251_device> m_com;
	required_device<input_merger_device> m_int;
	required_device<clock_device> m_prt_clk;
	required_device<clock_device> m_gen_clk;
	required_device<fd1793_device> m_fdc;
	required_device_array<floppy_connector, 4> m_floppy;
	required_device<rs232_port_device> m_comm;
	required_device<rs232_port_device> m_printer;
	output_finder<4> m_leds;
	required_ioport m_e22;

	emu_timer *m_rtc_timer;

	u8 m_drive;         // port 80 latch, bits 3-0
	u8 m_speed;         // port 90 latch
	u8 m_diag;          // port 98 latch
	bool m_rtc;         // real time clock interrupt pending
	bool m_irq;         // the Z80's INT line
	bool m_power_init;  // port 88 bit 7
	bool m_hld;
	bool m_dirc;         // 1793 DIRC: stepping in, toward the spindle
	bool m_wg;           // 1793 write gate
	bool m_motor;
	bool m_settled;      // E63 has timed out since HLD last rose
	emu_timer *m_settle_timer;

	// inputs to the receive side, for the loopback
	int m_tc_txd, m_line_rxd, m_comm_rxd, m_printer_rxd;
	int m_con_txd, m_gen_txd, m_prt_txd, m_com_txd;
	int m_com_rts;
	int m_tc_tclk, m_tc_rclk;
};


ROM_START( vt18x )
	ROM_REGION(0x2000, "rom", 0)
	ROM_LOAD( "23-021e3-00.bin", 0x0000, 0x1000, CRC(a2a575d2) SHA1(47a2c40aaec89e8476240f25515d75ab157f2911))
	ROM_LOAD( "23-017e3-00.bin", 0x1000, 0x1000, CRC(4bdd2398) SHA1(84f288def6c143a2d2ed9dedf947c862c66bb18e))
ROM_END

const tiny_rom_entry *vt100_stp_vt18x_device::device_rom_region() const
{
	return ROM_NAME(vt18x);
}

// E22, standard settings from Table 2-1
INPUT_PORTS_START( vt18x )
	PORT_START("E22")
	PORT_DIPNAME(0x001, 0x001, "RTS (J1 pin 4)") PORT_DIPLOCATION("E22:1") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt100_stp_vt18x_device::e22_changed), 0)
	PORT_DIPSETTING(    0x000, DEF_STR(Off))
	PORT_DIPSETTING(    0x001, DEF_STR(On))
	PORT_DIPNAME(0x002, 0x002, "CTS (J1 pin 5)") PORT_DIPLOCATION("E22:2")
	PORT_DIPSETTING(    0x000, DEF_STR(Off))
	PORT_DIPSETTING(    0x002, DEF_STR(On))
	PORT_DIPNAME(0x004, 0x000, "Secondary CTS (J1 pin 13)") PORT_DIPLOCATION("E22:3")
	PORT_DIPSETTING(    0x000, DEF_STR(Off))
	PORT_DIPSETTING(    0x004, DEF_STR(On))
	PORT_DIPNAME(0x008, 0x008, "Speed indicator (J1 pin 12)") PORT_DIPLOCATION("E22:4")
	PORT_DIPSETTING(    0x000, DEF_STR(Off))
	PORT_DIPSETTING(    0x008, DEF_STR(On))
	PORT_DIPNAME(0x010, 0x010, "Secondary RTS (J1 pin 11)") PORT_DIPLOCATION("E22:5")
	PORT_DIPSETTING(    0x000, DEF_STR(Off))
	PORT_DIPSETTING(    0x010, DEF_STR(On))
	PORT_DIPNAME(0x020, 0x020, "TXD (J1 pin 2)") PORT_DIPLOCATION("E22:6") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt100_stp_vt18x_device::e22_changed), 0)
	PORT_DIPSETTING(    0x000, DEF_STR(Off))
	PORT_DIPSETTING(    0x020, DEF_STR(On))
	PORT_DIPNAME(0x040, 0x000, "Secondary TXD (J1 pin 14)") PORT_DIPLOCATION("E22:7")
	PORT_DIPSETTING(    0x000, DEF_STR(Off))
	PORT_DIPSETTING(    0x040, DEF_STR(On))
	PORT_DIPNAME(0x080, 0x080, "Secondary RTS (J1 pin 19)") PORT_DIPLOCATION("E22:8")
	PORT_DIPSETTING(    0x000, DEF_STR(Off))
	PORT_DIPSETTING(    0x080, DEF_STR(On))
	PORT_DIPUNUSED_DIPLOC(0x100, 0x000, "E22:9")
	PORT_DIPNAME(0x200, 0x000, "Speed select (J1 pin 23)") PORT_DIPLOCATION("E22:10") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(vt100_stp_vt18x_device::e22_changed), 0)
	PORT_DIPSETTING(    0x000, DEF_STR(Off))
	PORT_DIPSETTING(    0x200, DEF_STR(On))
INPUT_PORTS_END

ioport_constructor vt100_stp_vt18x_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(vt18x);
}


void vt100_stp_vt18x_device::mem_map(address_map &map)
{
	map(0x2000, 0xffff).lrw8(
			NAME([this] (offs_t offset) { return m_ram[0x2000 + offset]; }),
			NAME([this] (offs_t offset, u8 data) { m_ram[0x2000 + offset] = data; }));
	map(0x0000, 0x1fff).view(m_rom_view);
	m_rom_view[0](0x0000, 0x1fff).r(FUNC(vt100_stp_vt18x_device::rom_r));
	m_rom_view[0](0x0000, 0x1fff).lw8(NAME([this] (offs_t offset, u8 data) { m_ram[offset] = data; }));
	m_rom_view[1](0x0000, 0x1fff).lrw8(
			NAME([this] (offs_t offset) { return m_ram[offset]; }),
			NAME([this] (offs_t offset, u8 data) { m_ram[offset] = data; }));
}

void vt100_stp_vt18x_device::io_map(address_map &map)
{
	map.global_mask(0xff);
	map.unmap_value_high();
	map(0x40, 0x41).rw(m_con, FUNC(i8251_device::read), FUNC(i8251_device::write));
	map(0x48, 0x49).rw(m_prt, FUNC(i8251_device::read), FUNC(i8251_device::write));
	map(0x50, 0x51).rw(m_gen, FUNC(i8251_device::read), FUNC(i8251_device::write));
	map(0x58, 0x59).rw(m_com, FUNC(i8251_device::read), FUNC(i8251_device::write));
	map(0x78, 0x7b).rw(m_fdc, FUNC(fd1793_device::read), FUNC(fd1793_device::write));
	map(0x80, 0x80).rw(FUNC(vt100_stp_vt18x_device::drive_r), FUNC(vt100_stp_vt18x_device::drive_w));
	map(0x88, 0x88).r(FUNC(vt100_stp_vt18x_device::status_r));
	map(0x90, 0x90).rw(FUNC(vt100_stp_vt18x_device::rtc_clear_r), FUNC(vt100_stp_vt18x_device::speed_w));
	map(0x98, 0x98).w(FUNC(vt100_stp_vt18x_device::diag_w));
	map(0xa0, 0xa0).r(FUNC(vt100_stp_vt18x_device::diag_r));
}

// the ROM is read with one wait state (6-54)
u8 vt100_stp_vt18x_device::rom_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		m_cpu->adjust_icount(-1);
	return m_rom[offset];
}


u8 vt100_stp_vt18x_device::drive_r()
{
	return m_drive
		| (m_irq ? 0x00 : 0x10)
		// 0x20 drive ready: grounded, 0
		| (m_fdc->intrq_r() ? 0x40 : 0x00)
		| (m_fdc->drq_r() ? 0x80 : 0x00);
}

void vt100_stp_vt18x_device::drive_w(u8 data)
{
	m_drive = data & 0x0f;
	m_fdc->set_floppy(m_floppy[data & 3]->get_device());
	m_fdc->dden_w(BIT(data, 2));
	motor_update();
}

u8 vt100_stp_vt18x_device::status_r()
{
	return (BIT(m_diag, 0) ? 0x01 : 0x00)
		| (BIT(m_speed, 6) ? 0x02 : 0x00)
		| (BIT(m_speed, 7) ? 0x04 : 0x00)
		| (m_comm->ri_r() ? 0x00 : 0x08)
		| ((BIT(m_e22->read(), 1) && !m_comm->cts_r()) ? 0x10 : 0x00)
		| ((BIT(m_e22->read(), 3) && !m_comm->si_r()) ? 0x20 : 0x00)
		| (m_comm->dcd_r() ? 0x00 : 0x40)
		| (m_power_init ? 0x80 : 0x00);
}

u8 vt100_stp_vt18x_device::rtc_clear_r()
{
	if (!machine().side_effects_disabled())
	{
		m_rtc = false;
		m_power_init = false;
		m_int->in_w<4>(0);
	}
	return 0xff;
}

void vt100_stp_vt18x_device::speed_w(u8 data)
{
	static constexpr u32 rates[8] = { 75, 150, 300, 600, 1200, 2400, 4800, 9600 };

	m_speed = data;
	m_prt_clk->set_unscaled_clock(rates[data & 7] * 16);
	m_gen_clk->set_unscaled_clock(rates[(data >> 3) & 7] * 16);
	comm_spds_update();
}

// E22, the communication switches (EK-VT18X-IN-002 Table 2-1, p.19): each
// connects one signal between the card and J1.  Open, a line the card drives
// is left off (mark) at J1, and one it reads is seen off -- inferred, as an
// open EIA receiver input reads mark.  Switches 3, 5, 7 and 8 carry the
// secondary channel (J1 pins 13, 11, 14, 19), which the RS-232 port doesn't
// have; 9 is unused.
void vt100_stp_vt18x_device::comm_txd_update()
{
	m_comm->write_txd(BIT(m_e22->read(), 5) ? m_com_txd : 1);
}

void vt100_stp_vt18x_device::comm_rts_update()
{
	m_comm->write_rts(BIT(m_e22->read(), 0) ? m_com_rts : 1);
}

// port 90 bit 7 set = speed select on (polarity inferred)
void vt100_stp_vt18x_device::comm_spds_update()
{
	m_comm->write_spds((BIT(m_e22->read(), 9) && BIT(m_speed, 7)) ? 0 : 1);
}

INPUT_CHANGED_MEMBER(vt100_stp_vt18x_device::e22_changed)
{
	comm_txd_update();
	comm_rts_update();
	comm_spds_update();
}

void vt100_stp_vt18x_device::diag_w(u8 data)
{
	m_diag = data;
	m_rom_view.select(BIT(data, 0));
	for (int i = 0; i < 4; i++)
		m_leds[i] = BIT(data, 4 + i) ? 0 : 1;
	con_rxd_update();
	gen_rxd_update();
	prt_rxd_update();
	com_rxd_update();
	com_clocks_update();
}

u8 vt100_stp_vt18x_device::diag_r()
{
	floppy_image_device *floppy = m_floppy[m_drive & 3]->get_device();
	return ((floppy && !floppy->trk00_r()) ? 0x01 : 0x00)
		| 0x02
		| (m_dirc ? 0x00 : 0x04)
		| (m_wg ? 0x00 : 0x08)
		| (m_motor ? 0x00 : 0x20)
		| ((m_drive & 3) << 6);
}


int vt100_stp_vt18x_device::irq_ack(device_t &device, int irqline)
{
	return m_rtc ? 0xdf : 0xd7; // RST 18, RST 10
}

TIMER_CALLBACK_MEMBER(vt100_stp_vt18x_device::rtc_tick)
{
	m_rtc = true;
	m_int->in_w<4>(1);
}

void vt100_stp_vt18x_device::hld_w(int state)
{
	// HLD rising fires E63 (through E60, DREADY being held true)
	if (state && !m_hld)
	{
		m_settled = false;
		m_settle_timer->adjust(attotime::from_seconds(1));
	}
	m_hld = state;
	motor_update();
}

TIMER_CALLBACK_MEMBER(vt100_stp_vt18x_device::head_settled)
{
	m_settled = true;
	motor_update();
}

// Fig 6-29: the E48 pair pulls MOTOR ON L (J2 16, common to the drives)
// while HLD is set or port 80 bit 3 is.  E62 gives the 1793 HLT when
// bit 3 is set or E63 has timed out.
void vt100_stp_vt18x_device::motor_update()
{
	bool const motor = BIT(m_drive, 3) || m_hld;
	if (motor != m_motor)
	{
		m_motor = motor;
		for (auto &f : m_floppy)
			if (f->get_device())
				f->get_device()->mon_w(motor ? 0 : 1);
	}
	m_fdc->hlt_w((BIT(m_drive, 3) || m_settled) ? 1 : 0);
}


// Port 98 bit 1 loops the console and communication transmitters back to
// their own receivers, and crosses the printer and general port.
void vt100_stp_vt18x_device::con_rxd_update()
{
	m_con->write_rxd(BIT(m_diag, 1) ? m_con_txd : m_tc_txd);
}

void vt100_stp_vt18x_device::gen_rxd_update()
{
	m_gen->write_rxd(BIT(m_diag, 1) ? m_prt_txd : m_line_rxd);
}

void vt100_stp_vt18x_device::prt_rxd_update()
{
	m_prt->write_rxd(BIT(m_diag, 1) ? m_gen_txd : m_printer_rxd);
}

void vt100_stp_vt18x_device::com_rxd_update()
{
	m_com->write_rxd(BIT(m_diag, 1) ? m_com_txd : m_comm_rxd);
}

void vt100_stp_vt18x_device::com_clocks_update()
{
	if (BIT(m_diag, 2))
	{
		m_com->write_txc(m_comm->txc_r());
		m_com->write_rxc(m_comm->rxc_r());
	}
	else
	{
		m_com->write_txc(m_tc_tclk);
		m_com->write_rxc(m_tc_rclk);
	}
}


void vt100_stp_vt18x_device::stp_w(unsigned pin, int state)
{
	using stp = vt100_stp_slot_device;
	switch (pin)
	{
	case stp::TC_TXD:
		m_tc_txd = state;
		con_rxd_update();
		break;
	case stp::TC_DTR:
		m_con->write_dsr(state);
		break;
	case stp::LOCAL_CLK:
		m_con->write_txc(state);
		m_con->write_rxc(state);
		slot().drive(stp::INTERNAL_TCLK, state);
		slot().drive(stp::INTERNAL_RCLK, state);
		break;
	case stp::TC_TCLK:
		m_tc_tclk = state;
		if (!BIT(m_diag, 2))
			m_com->write_txc(state);
		break;
	case stp::TC_RCLK:
		m_tc_rclk = state;
		if (!BIT(m_diag, 2))
			m_com->write_rxc(state);
		break;
	case stp::LINE_RXD:
		m_line_rxd = state;
		gen_rxd_update();
		break;
	case stp::LINE_DSR:
		m_gen->write_dsr(state);
		break;
	case stp::TC_INIT:
		m_cpu->set_input_line(INPUT_LINE_RESET, state ? ASSERT_LINE : CLEAR_LINE);
		if (state)
			init_board();
		break;
	default:
		// the rest aren't wired on the paddle board
		break;
	}
}

// INIT (CTL RESET L): the 8251s, the 1793, the ROM latch and the LEDs
void vt100_stp_vt18x_device::init_board()
{
	m_con->reset();
	m_prt->reset();
	m_gen->reset();
	m_com->reset();
	m_fdc->soft_reset();
	diag_w(0);
}


void vt100_stp_vt18x_device::device_start()
{

	m_rtc_timer = timer_alloc(FUNC(vt100_stp_vt18x_device::rtc_tick), this);
	// W6 grounds READY L, so the 1793 always sees the drive ready (6-78)
	m_fdc->set_force_ready(true);
	m_settle_timer = timer_alloc(FUNC(vt100_stp_vt18x_device::head_settled), this);

	m_drive = 0;
	m_speed = 0;
	m_diag = 0;
	m_rtc = false;
	m_irq = false;
	m_power_init = true;
	m_hld = false;
	m_dirc = false;
	m_wg = false;
	m_motor = false;
	m_settled = false;
	m_tc_txd = m_line_rxd = m_comm_rxd = m_printer_rxd = 1;
	m_con_txd = m_gen_txd = m_prt_txd = m_com_txd = 1;
	m_com_rts = 1;
	m_tc_tclk = m_tc_rclk = 1;

	save_item(NAME(m_drive));
	save_item(NAME(m_speed));
	save_item(NAME(m_diag));
	save_item(NAME(m_rtc));
	save_item(NAME(m_irq));
	save_item(NAME(m_power_init));
	save_item(NAME(m_hld));
	save_item(NAME(m_dirc));
	save_item(NAME(m_wg));
	save_item(NAME(m_motor));
	save_item(NAME(m_settled));
	save_item(NAME(m_tc_txd));
	save_item(NAME(m_line_rxd));
	save_item(NAME(m_comm_rxd));
	save_item(NAME(m_printer_rxd));
	save_item(NAME(m_con_txd));
	save_item(NAME(m_gen_txd));
	save_item(NAME(m_prt_txd));
	save_item(NAME(m_com_txd));
	save_item(NAME(m_com_rts));
	save_item(NAME(m_tc_tclk));
	save_item(NAME(m_tc_rclk));

	// CTS isn't shown wired on any of the 8251s
	m_con->write_cts(0);
	m_prt->write_cts(0);
	m_gen->write_cts(0);
	m_com->write_cts(0);

	// 75 Hz, from the baud rate generator
	m_rtc_timer->adjust(attotime::from_hz(75), 0, attotime::from_hz(75));
}

void vt100_stp_vt18x_device::device_reset()
{
	using stp = vt100_stp_slot_device;

	// the terminal side's handshake inputs are open: off
	slot().drive(stp::TC_CTS, 1);
	slot().drive(stp::TC_DSR, 1);
	slot().drive(stp::TC_CD, 1);
	slot().drive(stp::TC_RI, 1);
	slot().drive(stp::TC_SPDI, 1);

	// RTS and DTR on the VT100's J4 are pulled up on the paddle board; the
	// speed select is open
	slot().drive(stp::LINE_RTS, 0);
	slot().drive(stp::LINE_DTR, 0);
	slot().drive(stp::LINE_SPDS, 1);

	m_drive = 0;
	m_fdc->set_floppy(m_floppy[0]->get_device());
	m_fdc->dden_w(0);
	motor_update();
	speed_w(0);
	diag_w(0);
}

void vt100_stp_vt18x_device::device_reset_after_children()
{
	// the printer connector's DTR is always on.  After the children, so that
	// a device on the connector that answers DTR has started and been reset
	m_printer->write_dtr(0);
	m_printer->write_rts(0);
}


void vt100_stp_vt18x_device::floppy_formats(format_registration &fr)
{
	fr.add_mfm_containers();
	fr.add(FLOPPY_VT180_FORMAT);
}

static void vt18x_floppies(device_slot_interface &device)
{
	device.option_add("525ssdd", FLOPPY_525_SSDD);
}

void vt100_stp_vt18x_device::device_add_mconfig(machine_config &config)
{
	Z80(config, m_cpu, 4_MHz_XTAL);
	m_cpu->set_addrmap(AS_PROGRAM, &vt100_stp_vt18x_device::mem_map);
	m_cpu->set_addrmap(AS_IO, &vt100_stp_vt18x_device::io_map);
	m_cpu->set_irq_acknowledge_callback(FUNC(vt100_stp_vt18x_device::irq_ack));

	INPUT_MERGER_ANY_HIGH(config, m_int).output_handler().set([this] (int state) { m_irq = state; m_cpu->set_input_line(INPUT_LINE_IRQ0, state); });

	I8251(config, m_con, 2_MHz_XTAL);
	m_con->txd_handler().set([this] (int state) { m_con_txd = state; slot().drive(vt100_stp_slot_device::TC_RXD, state); con_rxd_update(); });
	m_con->rxrdy_handler().set(m_int, FUNC(input_merger_device::in_w<0>));

	I8251(config, m_prt, 2_MHz_XTAL);
	m_prt->txd_handler().set([this] (int state) { m_prt_txd = state; m_printer->write_txd(state); gen_rxd_update(); });
	m_prt->rxrdy_handler().set(m_int, FUNC(input_merger_device::in_w<1>));

	I8251(config, m_gen, 2_MHz_XTAL);
	m_gen->txd_handler().set([this] (int state) { m_gen_txd = state; slot().drive(vt100_stp_slot_device::LINE_TXD, state); prt_rxd_update(); });
	m_gen->rxrdy_handler().set(m_int, FUNC(input_merger_device::in_w<2>));

	I8251(config, m_com, 2_MHz_XTAL);
	m_com->txd_handler().set([this] (int state) { m_com_txd = state; comm_txd_update(); com_rxd_update(); });
	m_com->dtr_handler().set(m_comm, FUNC(rs232_port_device::write_dtr));
	m_com->rts_handler().set([this] (int state) { m_com_rts = state; comm_rts_update(); });
	m_com->rxrdy_handler().set(m_int, FUNC(input_merger_device::in_w<3>));

	CLOCK(config, m_prt_clk, 75 * 16);
	m_prt_clk->signal_handler().set(m_prt, FUNC(i8251_device::write_txc));
	m_prt_clk->signal_handler().append(m_prt, FUNC(i8251_device::write_rxc));

	CLOCK(config, m_gen_clk, 75 * 16);
	m_gen_clk->signal_handler().set(m_gen, FUNC(i8251_device::write_txc));
	m_gen_clk->signal_handler().append(m_gen, FUNC(i8251_device::write_rxc));

	FD1793(config, m_fdc, 1_MHz_XTAL);
	m_fdc->hld_wr_callback().set(FUNC(vt100_stp_vt18x_device::hld_w));
	m_fdc->dirc_wr_callback().set([this] (int state) { m_dirc = state; });
	m_fdc->wg_wr_callback().set([this] (int state) { m_wg = state; });
	for (auto &f : m_floppy)
		FLOPPY_CONNECTOR(config, f, vt18x_floppies, "525ssdd", vt100_stp_vt18x_device::floppy_formats).enable_sound(true);

	SOFTWARE_LIST(config, "flop_list").set_original("vt180");

	RS232_PORT(config, m_comm, default_rs232_devices, nullptr);
	m_comm->rxd_handler().set([this] (int state) { m_comm_rxd = state; com_rxd_update(); });
	m_comm->dsr_handler().set(m_com, FUNC(i8251_device::write_dsr));
	m_comm->txc_handler().set([this] (int state) { if (BIT(m_diag, 2)) m_com->write_txc(state); });
	m_comm->rxc_handler().set([this] (int state) { if (BIT(m_diag, 2)) m_com->write_rxc(state); });

	RS232_PORT(config, m_printer, default_rs232_devices, nullptr);
	m_printer->rxd_handler().set([this] (int state) { m_printer_rxd = state; prt_rxd_update(); });
	m_printer->dsr_handler().set(m_prt, FUNC(i8251_device::write_dsr));
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(VT100_STP_VT18X, device_vt100_stp_interface, vt100_stp_vt18x_device, "vt100_stp_vt18x", "DEC VT18X Z80 CP/M option")
