// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC TU58 DECtape II

    Two DC100 cartridge drives and a serial controller, talking DEC's Radial
    Serial Protocol to a host over RS-232 (or RS-422/423, jumpered).  Sold
    on its own and built into the VT103 and the consoles of the
    VAX-11/730 and /750.

    Sources: TU58 DECtape II Technical Manual EK-0TU58-TM-001 (TM); TU58
    Engineering Specification, Jun 1978 (ES); serial controller 5413489
    schematic (VT103 print set MP00731, pages 58-61, and the copy in
    NF6X's tu58firmware repository); firmware 23-089E2 and Eric Smith's
    disassembly of it.

    Controller:
    - 8085, 4.9152 MHz crystal (2.4576 MHz CLK)
    - 2K ROM at 0000
    - 8155: 256 bytes of RAM at 2000 (mirrored to 2FFF), ports at I/O
      20-25.  PA in, PB out, PC out, and the timer, clocked from CLK
    - 6402 UART at I/O 08 (any port with A3 set).  Hard-wired 8N1.  It has
      no status register: DR goes to RST 6.5, TBRE to RST 5.5 and FE to
      TRAP (a break from the host resets the protocol), and the firmware
      reads them back with RIM.  Each direction's rate is a wire-wrap
      jumper to a divider of CLK; 38,400 is the factory setting
    - RD STROBE on RST 7.5, RD DATA on SID, WRT DATA from SOD

    PA (in):  0 WRT PERMIT L   1 VEL TP H   2 B CART L   3 A CART L
              4 RUN TP L       5 OE (UART overrun; unused by 23-089E2)
              6 SW BOOT L      7 MARK H
    PB (out): 0 RUN L          1 WRT ENB H  2 REVERSE H  3 SEL DRV B L
              4 SEL TRK 0 L    5 GAIN REDUCE L  6 ERASE ENA L  7 30IPS H
    PC (out): 0 BOOT L (low: hold the line in break)  2 LED (high: on)
              1, 3, 4: test points

    Tape (TM 1.3, ES 1.3; layout from the firmware where they differ):
    - 2 tracks of 1024 records of 128 bytes, both written forward.  A
      record's header holds its number on the cartridge, track 1 adding
      1024.  The firmware maps the host's 512-byte blocks to records:
      record R = 4 x block + n, at physical record R rotated left one bit
      in 11 (so track = bit 0 of R, ...).  The image file is the 256K of
      logical blocks, in block order, as other TU58 emulators keep it.
    - Ratio encoding, one bit cell per 1/800 in (41.7 us at 30 in/s): the
      write current goes high at the start of each cell and low a quarter
      (0) or three quarters (1) of the way through.  The read electronics
      integrate each cell and give RD DATA at the next rising edge, with
      RD STROBE.
    - Marks are cells four times as long.  The mark finder (two LS107s
      clocked by the 8155 timer, which the firmware runs at about 1.9
      normal cells) sets MARK H at a strobe that ends a cell longer than
      two timer periods.  Before each record but the first on a track is
      an interrecord mark of 16 mark cells, 1010...; the zone at the start
      of the tape is mark cells of zeros (BOT) and at the end ones (EOT).
    - Motion: one motor per drive under a velocity servo (30 or 60 in/s,
      either direction), started by RUN L.  A run timeout stops the motor
      400 ms after the last fall of RUN L.  The firmware brakes by
      reversing the motor and waiting for the tachometer to stop changing.

    Assumed, not found in the documents:
    - acceleration 1000 in/s^2, either way (no figure in the TM; this
      reaches 30 in/s within the firmware's 34.5 ms spin-up wait)
    - the tachometer changes every 7.03 thou of tape (2133 Hz at 30 in/s,
      from the servo's quarter-period delay: 18 stages of CLK/16)
    - RUN L high cancels the run timeout as well as stopping the motor
    - the zone lengths, 10 in each of mark cells, and a cartridge going in
      half an inch short of the end of BOT.  The firmware bounds them: a
      search backwards spins up for 52 ms at 60 in/s, finds BOT, goes on
      46 ms and brakes, all within the zone (about 6200 cells); then it
      must reach the first record forwards at 30 in/s before the run
      timeout (400 ms, about 9000 cells)
    - the gaps within a record and the record length (1322 cells, the
      ES's figure): the firmware looks for the data sync from about 49
      cells after the header, so it is put at 56, where the firmware
      writes it
    - the jumpered rates: CLK divided by powers of two

    TODO:
    - erase, gain reduce, and reading at the wrong speed or while
      accelerating, which the real read electronics can't do
    - the LED

***************************************************************************/

#include "emu.h"
#include "tu58.h"

#include "cpu/i8085/i8085.h"
#include "imagedev/magtape.h"
#include "machine/ay31015.h"
#include "machine/clock.h"
#include "machine/i8155.h"

#include <cmath>

#define LOG_MOTION (1U << 1)
#define LOG_WRITE  (1U << 2)

//#define VERBOSE (LOG_GENERAL | LOG_MOTION | LOG_WRITE)
#include "logmacro.h"

#define LOGMOTION(...) LOGMASKED(LOG_MOTION, __VA_ARGS__)
#define LOGWRITE(...)  LOGMASKED(LOG_WRITE, __VA_ARGS__)


//**************************************************************************
//  CARTRIDGE AND DRIVE
//**************************************************************************

// Positions are in bit cells of 1/800 in from the physical start of tape.
class tu58_cartridge_device : public microtape_image_device
{
public:
	static constexpr int RECORDS = 1024;           // per track
	static constexpr int IMAGE_SIZE = 2 * RECORDS * 128;
	static constexpr int MARK_LEN = 4;
	static constexpr s64 BOT_LEN = 2000 * MARK_LEN;
	static constexpr s64 EOT_LEN = 2000 * MARK_LEN;
	static constexpr s64 LOAD_POS = BOT_LEN - 400;  // where a cartridge goes in
	static constexpr int IRM_LEN = 16 * MARK_LEN;  // 16 mark cells
	static constexpr int REC_BITS = 1258;          // normal cells after the mark
	static constexpr s64 REC_LEN = IRM_LEN + REC_BITS;
	static constexpr s64 TRACK_LEN = BOT_LEN + RECORDS * REC_LEN + EOT_LEN;
	static constexpr double CELLS_PER_INCH = 800.0;
	static constexpr double ACCEL = 1000.0 * CELLS_PER_INCH;   // assumed
	static constexpr double TACH_CELLS = CELLS_PER_INCH * 30.0 / (2 * 2133.0);

	tu58_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// device_image_interface implementation
	virtual std::pair<std::error_condition, std::string> call_load() override;
	virtual std::pair<std::error_condition, std::string> call_create(int format_type, util::option_resolution *format_options) override;
	virtual void call_unload() override;
	virtual const char *file_extensions() const noexcept override { return "dsk,tu58,img"; }
	virtual const char *image_interface() const noexcept override { return "tu58"; }

	bool present() const { return m_present; }
	bool write_protected() const { return !m_present || is_readonly(); }

	// motion
	void set_setpoint(double cells_per_s);
	void rebase();
	double pos();
	bool stalled() const { return m_stalled; }

	enum event_kind : u8 { NONE, BOUNDARY, SETPOINT, REVERSAL };
	struct event
	{
		attotime when;   // from now; never if nothing will happen
		u8 kind;         // a cell boundary, the set point reached, or speed zero
		s64 pos;         // the boundary
		int dir;         // the direction it is crossed in
	};
	event next_event();
	void arrive(const event &ev);

	// the bit in the cell ending at a boundary crossed in direction dir
	int bit_before(int track, s64 boundary, int dir);

	// a record written from the bit stream: the record under pos
	void write_record(int track, double pos, const u8 *data, u16 checksum);

protected:
	virtual void device_start() override ATTR_COLD;

private:
	struct cell { s64 start; int len; int bit; };

	cell cell_at(int track, s64 pos);
	int record_bit(int track, int record, int bit);
	u16 checksum(int track, int record);
	u32 image_offset(int track, int record) const;
	void state_at(const attotime &t, double &p, double &v) const;

	bool m_present;
	std::vector<u8> m_data;
	std::vector<s32> m_checksum;   // per physical record: -1 to compute from the data

	attotime m_t0;
	double m_p0;
	double m_v0;
	double m_sp;
	bool m_stalled;
};

DEFINE_DEVICE_TYPE(TU58_CARTRIDGE, tu58_cartridge_device, "tu58_cartridge", "DEC TU58 cartridge drive")

tu58_cartridge_device::tu58_cartridge_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: microtape_image_device(mconfig, TU58_CARTRIDGE, tag, owner, clock)
	, m_present(false)
	, m_data(IMAGE_SIZE, 0)
	, m_checksum(2 * RECORDS, -1)
	, m_t0(attotime::zero)
	, m_p0(LOAD_POS)
	, m_v0(0.0)
	, m_sp(0.0)
	, m_stalled(false)
{
}

void tu58_cartridge_device::device_start()
{
	save_item(NAME(m_present));
	save_item(NAME(m_data));
	save_item(NAME(m_checksum));
	save_item(NAME(m_t0));
	save_item(NAME(m_p0));
	save_item(NAME(m_v0));
	save_item(NAME(m_sp));
	save_item(NAME(m_stalled));
}

std::pair<std::error_condition, std::string> tu58_cartridge_device::call_load()
{
	std::fill(m_data.begin(), m_data.end(), 0);
	std::fill(m_checksum.begin(), m_checksum.end(), -1);
	if (length() > IMAGE_SIZE)
		return std::make_pair(image_error::INVALIDLENGTH, "Image is larger than a TU58 cartridge (262,144 bytes)");
	fseek(0, SEEK_SET);
	fread(m_data.data(), length());

	// a cartridge goes in rewound, as the firmware leaves it: just past BOT
	rebase();
	m_p0 = LOAD_POS;
	m_v0 = 0.0;
	m_stalled = false;
	m_present = true;
	return std::make_pair(std::error_condition(), std::string());
}

std::pair<std::error_condition, std::string> tu58_cartridge_device::call_create(int format_type, util::option_resolution *format_options)
{
	std::vector<u8> blank(IMAGE_SIZE, 0);
	if (fwrite(blank.data(), IMAGE_SIZE) != IMAGE_SIZE)
		return std::make_pair(image_error::UNSPECIFIED, std::string());
	return call_load();
}

void tu58_cartridge_device::call_unload()
{
	m_present = false;
}

u32 tu58_cartridge_device::image_offset(int track, int record) const
{
	// undo the firmware's rotate: physical record R' -> logical record R
	const u32 p = record + track * RECORDS;
	return (((p >> 1) | ((p & 1) << 10)) & 0x7ff) * 128;
}

u16 tu58_cartridge_device::checksum(int track, int record)
{
	s32 &c = m_checksum[track * RECORDS + record];
	if (c >= 0)
		return c;

	// end-around-carry sum of the little-endian words, seeded with the
	// record's number on the cartridge (23-089E2 0527)
	u32 sum = record + track * RECORDS;
	const u8 *d = &m_data[image_offset(track, record)];
	for (int i = 0; i < 128; i += 2)
	{
		sum += d[i] | (d[i + 1] << 8);
		if (sum > 0xffff)
			sum = (sum & 0xffff) + 1;
	}
	return sum;
}

// The normal cells of a record, after its mark (record bits, LSB first,
// low byte first):
//    0-15   header sync: 15 zeros and a one
//   16-31   record number
//   32-47   its complement
//   48-103  data sync: 55 zeros and a one
//  104-1127 128 data bytes
// 1128-1143 checksum
// 1144-1159 the record number again (23-089E2 writes it after the checksum)
// 1160-     zeros to the next mark
int tu58_cartridge_device::record_bit(int track, int record, int bit)
{
	const u16 number = record + track * RECORDS;
	if (bit < 15)
		return 0;
	if (bit == 15)
		return 1;
	if (bit < 32)
		return BIT(number, bit - 16);
	if (bit < 48)
		return BIT(~number, bit - 32);
	if (bit < 103)
		return 0;
	if (bit == 103)
		return 1;
	if (bit < 1128)
		return BIT(m_data[image_offset(track, record) + ((bit - 104) >> 3)], (bit - 104) & 7);
	if (bit < 1144)
		return BIT(checksum(track, record), bit - 1128);
	if (bit < 1160)
		return BIT(number, bit - 1144);
	return 0;
}

tu58_cartridge_device::cell tu58_cartridge_device::cell_at(int track, s64 pos)
{
	if (pos < BOT_LEN)
		return cell{ pos - (pos % MARK_LEN), MARK_LEN, 0 };

	const s64 rel = pos - BOT_LEN;
	const s64 record = rel / REC_LEN;
	if (record >= RECORDS)
	{
		const s64 eot = pos - (BOT_LEN + RECORDS * REC_LEN);
		return cell{ pos - (eot % MARK_LEN), MARK_LEN, 1 };
	}

	const s64 offset = rel % REC_LEN;
	if (offset < IRM_LEN)
	{
		// no mark before the first record on a track: zeros instead
		if (record == 0)
			return cell{ pos, 1, 0 };
		const s64 start = pos - (offset % MARK_LEN);
		return cell{ start, MARK_LEN, ((offset / MARK_LEN) & 1) ? 0 : 1 };
	}
	return cell{ pos, 1, record_bit(track, record, offset - IRM_LEN) };
}

int tu58_cartridge_device::bit_before(int track, s64 boundary, int dir)
{
	// Reading backwards gives the same bits: the strobe comes at each rising
	// edge, and a cell played backwards is still high for a quarter or three
	// quarters of the time between rising edges.
	return cell_at(track, (dir > 0) ? (boundary - 1) : boundary).bit;
}

void tu58_cartridge_device::write_record(int track, double pos, const u8 *data, u16 sum)
{
	const s64 rel = s64(pos) - BOT_LEN;
	if (rel < 0 || rel >= RECORDS * REC_LEN)
	{
		LOGWRITE("write outside the records at %.0f, ignored\n", pos);
		return;
	}
	const int record = rel / REC_LEN;
	const u32 offset = image_offset(track, record);
	std::copy_n(data, 128, &m_data[offset]);
	m_checksum[track * RECORDS + record] = -1;
	if (checksum(track, record) != sum)
		m_checksum[track * RECORDS + record] = sum;
	LOGWRITE("wrote track %d record %d (image offset %05x)%s\n", track, record, offset,
			(m_checksum[track * RECORDS + record] >= 0) ? ", bad checksum" : "");

	if (!is_readonly())
	{
		fseek(offset, SEEK_SET);
		fwrite(data, 128);
	}
}

void tu58_cartridge_device::state_at(const attotime &t, double &p, double &v) const
{
	if (m_stalled)
	{
		p = m_p0;
		v = 0.0;
		return;
	}

	const double dt = (t - m_t0).as_double();
	const double diff = m_sp - m_v0;
	const double a = (diff > 0) ? ACCEL : (diff < 0) ? -ACCEL : 0.0;
	const double ta = (a != 0.0) ? (std::abs(diff) / ACCEL) : 0.0;
	if (dt < ta)
	{
		v = m_v0 + a * dt;
		p = m_p0 + m_v0 * dt + 0.5 * a * dt * dt;
	}
	else
	{
		v = m_sp;
		p = m_p0 + m_v0 * ta + 0.5 * a * ta * ta + m_sp * (dt - ta);
	}
	p = std::clamp(p, 0.0, double(TRACK_LEN));
}

void tu58_cartridge_device::rebase()
{
	const attotime now = machine().time();
	double p, v;
	state_at(now, p, v);
	m_t0 = now;
	m_p0 = p;
	m_v0 = v;
}

double tu58_cartridge_device::pos()
{
	double p, v;
	state_at(machine().time(), p, v);
	return p;
}

void tu58_cartridge_device::set_setpoint(double cells_per_s)
{
	rebase();
	m_sp = cells_per_s;
	// at an end of the tape, only pulling it back moves it
	if (m_stalled && ((m_p0 <= 0.0 && m_sp > 0.0) || (m_p0 >= TRACK_LEN && m_sp < 0.0)))
	{
		m_stalled = false;
		m_v0 = 0.0;
	}
	LOGMOTION("%s: setpoint %.1f in/s at %.1f in/s, cell %.1f\n", tag(), m_sp / CELLS_PER_INCH, m_v0 / CELLS_PER_INCH, m_p0);
}

tu58_cartridge_device::event tu58_cartridge_device::next_event()
{
	rebase();
	event ev{ attotime::never, NONE, 0, 0 };
	if (m_stalled)
		return ev;

	const double v = m_v0, p = m_p0;
	const double diff = m_sp - v;
	const double a = (diff > 0) ? ACCEL : (diff < 0) ? -ACCEL : 0.0;
	if (v == 0.0 && a == 0.0)
		return ev;

	// the acceleration changes when the set point is reached, and the
	// motion reverses when the speed passes zero
	double seg = (a != 0.0) ? (std::abs(diff) / ACCEL) : 1e9;
	u8 seg_kind = SETPOINT;
	if (v != 0.0 && a != 0.0 && ((v > 0) != (a > 0)) && (std::abs(v) / ACCEL) < seg)
	{
		seg = std::abs(v) / ACCEL;
		seg_kind = REVERSAL;
	}

	const int dir = (v > 0) ? 1 : (v < 0) ? -1 : ((a > 0) ? 1 : -1);

	// the next boundary ahead
	s64 b;
	if (dir > 0)
	{
		const s64 at = std::min(s64(std::floor(p)), TRACK_LEN - 1);
		const cell c = cell_at(0, at);
		b = c.start + c.len;
	}
	else
	{
		const s64 at = std::max(s64(std::ceil(p)) - 1, s64(0));
		b = cell_at(0, at).start;
	}
	b = std::clamp<s64>(b, 0, TRACK_LEN);
	if ((dir > 0 && p >= TRACK_LEN) || (dir < 0 && p <= 0.0))
	{
		m_stalled = true;
		m_v0 = 0.0;
		return ev;
	}

	// time to cover the distance, accelerating along the motion
	const double d = std::abs(double(b) - p);
	const double sv = std::abs(v);
	const double sa = a * dir;
	double t;
	if (sa == 0.0)
		t = d / sv;
	else
	{
		const double disc = sv * sv + 2.0 * sa * d;
		t = (disc < 0.0) ? 1e9 : ((std::sqrt(disc) - sv) / sa);
	}

	if (t <= seg)
	{
		ev.when = attotime::from_double(t);
		ev.kind = BOUNDARY;
		ev.pos = b;
		ev.dir = dir;
	}
	else
	{
		ev.when = attotime::from_double(seg);
		ev.kind = seg_kind;
	}
	return ev;
}

void tu58_cartridge_device::arrive(const event &ev)
{
	rebase();
	// land exactly, or rounding leaves the next event due at once, forever
	if (ev.kind == SETPOINT)
		m_v0 = m_sp;
	else if (ev.kind == REVERSAL)
		m_v0 = 0.0;
	if (ev.kind != BOUNDARY)
		return;
	m_p0 = double(ev.pos);
	if (ev.pos <= 0 || ev.pos >= TRACK_LEN)
	{
		LOGMOTION("%s: ran off the %s of the tape\n", tag(), ev.pos ? "end" : "start");
		m_v0 = 0.0;
		m_stalled = true;
	}
}


//**************************************************************************
//  CONTROLLER
//**************************************************************************

namespace {

class dec_tu58_device : public device_t, public device_rs232_port_interface
{
public:
	dec_tu58_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	virtual void input_txd(int state) override { m_uart->write_si(state); }

	DECLARE_INPUT_CHANGED_MEMBER(rate_changed) { update_rates(); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_reset_after_children() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	required_device<i8085a_cpu_device> m_cpu;
	required_device<i8155_device> m_pio;
	required_device<ay31015_device> m_uart;
	required_device<clock_device> m_rxclk;
	required_device<clock_device> m_txclk;
	required_device_array<tu58_cartridge_device, 2> m_drive;
	required_ioport m_jumpers;
	required_ioport m_switches;

	emu_timer *m_strobe_timer;
	emu_timer *m_run_timer;

	tu58_cartridge_device::event m_event;
	u8 m_pb;
	u8 m_pc;
	int m_sid;
	int m_mark;
	int m_tmr;
	int m_tmr_edges;
	int m_running;
	int m_so;
	int m_oe;

	// the bit stream from SOD while WRT ENB is high
	bool m_writing;
	int m_wr_drive;
	int m_wr_track;
	double m_wr_pos;
	std::vector<std::pair<attotime, int>> m_wr_edges;

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);

	u8 pa_r();
	void pb_w(u8 data);
	void pc_w(u8 data);
	void tmr_w(int state);
	void so_w(int state);
	int sid_r() { return m_sid; }
	void sod_w(int state);

	int selected() const { return BIT(m_pb, 3) ? 0 : 1; }
	int track() const { return BIT(m_pb, 4) ? 1 : 0; }

	void update_rates();
	void update_motors();
	void plan();
	void finish_write();

	TIMER_CALLBACK_MEMBER(strobe);
	TIMER_CALLBACK_MEMBER(run_timeout);
};

dec_tu58_device::dec_tu58_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, DEC_TU58, tag, owner, clock)
	, device_rs232_port_interface(mconfig, *this)
	, m_cpu(*this, "cpu")
	, m_pio(*this, "pio")
	, m_uart(*this, "uart")
	, m_rxclk(*this, "rxclk")
	, m_txclk(*this, "txclk")
	, m_drive(*this, "drive%u", 0U)
	, m_jumpers(*this, "JUMPERS")
	, m_switches(*this, "SWITCHES")
	, m_strobe_timer(nullptr)
	, m_run_timer(nullptr)
	, m_pb(0xff)
	, m_pc(0xff)
	, m_sid(0)
	, m_mark(0)
	, m_tmr(1)
	, m_tmr_edges(0)
	, m_running(0)
	, m_so(1)
	, m_oe(0)
	, m_writing(false)
	, m_wr_drive(0)
	, m_wr_track(0)
	, m_wr_pos(0.0)
{
}

ROM_START( tu58 )
	ROM_REGION(0x800, "cpu", 0)
	ROM_DEFAULT_BIOS("089e2")
	ROM_SYSTEM_BIOS(0, "089e2", "23-089E2 (VT103)")
	ROMX_LOAD("23-089e2-00.e1", 0x000, 0x800, CRC(8614dd4c) SHA1(1b554e6c98bddfc6bc48d81c990deea43cf9df7f), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "294e2", "23-294E2")
	ROMX_LOAD("23-294e2-00.e1", 0x000, 0x800, CRC(2f6f51f8) SHA1(49f7b274c342dabac14983e1ba42ca46bda30741), ROM_BIOS(1))
	ROM_SYSTEM_BIOS(2, "389e2", "23-389E2")
	ROMX_LOAD("23-389e2-00.e1", 0x000, 0x800, CRC(b25a9978) SHA1(4fc781e94719339cfc5c2d1753a08e50e12472fa), ROM_BIOS(2))
ROM_END

const tiny_rom_entry *dec_tu58_device::device_rom_region() const
{
	return ROM_NAME(tu58);
}

INPUT_PORTS_START( tu58 )
	PORT_START("JUMPERS")
	PORT_CONFNAME(0x0f, 0x08, "Receive rate") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(dec_tu58_device::rate_changed), 0)
	PORT_CONFSETTING(0x00, "150")
	PORT_CONFSETTING(0x01, "300")
	PORT_CONFSETTING(0x02, "600")
	PORT_CONFSETTING(0x03, "1200")
	PORT_CONFSETTING(0x04, "2400")
	PORT_CONFSETTING(0x05, "4800")
	PORT_CONFSETTING(0x06, "9600")
	PORT_CONFSETTING(0x07, "19200")
	PORT_CONFSETTING(0x08, "38400")
	PORT_CONFNAME(0xf0, 0x80, "Transmit rate") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(dec_tu58_device::rate_changed), 0)
	PORT_CONFSETTING(0x00, "150")
	PORT_CONFSETTING(0x10, "300")
	PORT_CONFSETTING(0x20, "600")
	PORT_CONFSETTING(0x30, "1200")
	PORT_CONFSETTING(0x40, "2400")
	PORT_CONFSETTING(0x50, "4800")
	PORT_CONFSETTING(0x60, "9600")
	PORT_CONFSETTING(0x70, "19200")
	PORT_CONFSETTING(0x80, "38400")

	PORT_START("SWITCHES")
	PORT_CONFNAME(0x01, 0x01, "Boot switch")
	PORT_CONFSETTING(0x00, DEF_STR(On))
	PORT_CONFSETTING(0x01, DEF_STR(Off))
INPUT_PORTS_END

ioport_constructor dec_tu58_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(tu58);
}

void dec_tu58_device::mem_map(address_map &map)
{
	map(0x0000, 0x07ff).rom().region("cpu", 0);
	map(0x2000, 0x20ff).mirror(0x0f00).rw(m_pio, FUNC(i8155_device::memory_r), FUNC(i8155_device::memory_w));
}

void dec_tu58_device::io_map(address_map &map)
{
	map(0x00, 0xff).rw(FUNC(dec_tu58_device::io_r), FUNC(dec_tu58_device::io_w));
}

// The 8155 is selected by port 2x (A13 high, A12, A14, A15 low) and the UART
// by A3 alone, so ports 28-2F reach both.
u8 dec_tu58_device::io_r(offs_t offset)
{
	u8 data = 0xff;
	if ((offset & 0xf0) == 0x20)
		data &= m_pio->io_r(offset & 7);
	if (BIT(offset, 3))
	{
		// RRD puts the received character on the bus and DRR clears DR
		data &= m_uart->receive();
		if (!machine().side_effects_disabled())
		{
			m_uart->write_rdav(0);
			m_uart->write_rdav(1);
		}
	}
	return data;
}

void dec_tu58_device::io_w(offs_t offset, u8 data)
{
	if ((offset & 0xf0) == 0x20)
		m_pio->io_w(offset & 7, data);
	if (BIT(offset, 3))
		m_uart->transmit(data);
}

u8 dec_tu58_device::pa_r()
{
	tu58_cartridge_device &drive = *m_drive[selected()];
	const int tach = int(std::floor(drive.pos() / tu58_cartridge_device::TACH_CELLS)) & 1;

	return (drive.write_protected() ? 0x01 : 0x00)
			| (tach << 1)
			| (m_drive[1]->present() ? 0x00 : 0x04)
			| (m_drive[0]->present() ? 0x00 : 0x08)
			| (m_running ? 0x00 : 0x10)
			| (m_oe << 5)
			| ((m_switches->read() & 0x01) << 6)
			| (m_mark << 7);
}

void dec_tu58_device::pb_w(u8 data)
{
	const u8 old = m_pb;
	m_pb = data;

	// RUN L: each fall starts the motor and restarts the run timeout
	if (BIT(old, 0) && !BIT(data, 0))
	{
		m_running = 1;
		m_run_timer->adjust(attotime::from_msec(400));
	}
	else if (BIT(data, 0))
	{
		m_running = 0;
		m_run_timer->enable(false);
	}

	if (!BIT(old, 1) && BIT(data, 1))
	{
		m_writing = true;
		m_wr_drive = selected();
		m_wr_track = track();
		m_wr_pos = m_drive[m_wr_drive]->pos();
		m_wr_edges.clear();
	}
	else if (BIT(old, 1) && !BIT(data, 1))
		finish_write();

	if ((old ^ data) & 0x9d)
		update_motors();
}

void dec_tu58_device::pc_w(u8 data)
{
	const u8 old = m_pc;
	m_pc = data;
	if (BIT(old ^ data, 0))
		output_rxd(BIT(data, 0) ? m_so : 0);
	if (BIT(old ^ data, 2))
		LOG("LED %s\n", BIT(data, 2) ? "on" : "off");
}

void dec_tu58_device::so_w(int state)
{
	m_so = state;
	output_rxd(BIT(m_pc, 0) ? state : 0);
}

void dec_tu58_device::tmr_w(int state)
{
	// the mark finder counts falling edges of TIMER OUT between strobes
	if (m_tmr && !state)
		m_tmr_edges++;
	m_tmr = state;
}

void dec_tu58_device::sod_w(int state)
{
	if (m_writing)
		m_wr_edges.emplace_back(machine().time(), state);
}

void dec_tu58_device::update_rates()
{
	// CLK / 4 is 16 x 38,400; the slower rates are further halvings
	const u32 jumpers = m_jumpers->read();
	m_rxclk->set_unscaled_clock((4.9152_MHz_XTAL / 8).value() >> (8 - std::min<u32>(jumpers & 0x0f, 8)));
	m_txclk->set_unscaled_clock((4.9152_MHz_XTAL / 8).value() >> (8 - std::min<u32>(jumpers >> 4, 8)));
}

void dec_tu58_device::update_motors()
{
	const double speed = (BIT(m_pb, 7) ? 30.0 : 60.0) * tu58_cartridge_device::CELLS_PER_INCH;
	for (int i = 0; i < 2; i++)
	{
		tu58_cartridge_device &drive = *m_drive[i];
		const bool on = m_running && !BIT(m_pb, 0) && selected() == i && drive.present();
		drive.set_setpoint(on ? (BIT(m_pb, 2) ? -speed : speed) : 0.0);
	}
	plan();
}

void dec_tu58_device::plan()
{
	m_event = m_drive[selected()]->next_event();
	m_strobe_timer->adjust(m_event.when);
}

TIMER_CALLBACK_MEMBER(dec_tu58_device::strobe)
{
	tu58_cartridge_device &drive = *m_drive[selected()];
	drive.arrive(m_event);
	if (m_event.kind == tu58_cartridge_device::BOUNDARY && !drive.stalled())
	{
		// RD DATA is clocked on the strobe; MARK H too, from the timer edges
		// since the last one
		m_sid = drive.bit_before(track(), m_event.pos, m_event.dir);
		m_mark = (m_tmr_edges >= 2) ? 1 : 0;
		m_tmr_edges = 0;
		m_cpu->set_input_line(I8085_RST75_LINE, ASSERT_LINE);
		m_cpu->set_input_line(I8085_RST75_LINE, CLEAR_LINE);
	}
	plan();
}

TIMER_CALLBACK_MEMBER(dec_tu58_device::run_timeout)
{
	LOGMOTION("run timeout\n");
	m_running = 0;
	update_motors();
}

void dec_tu58_device::finish_write()
{
	m_writing = false;

	// decode the ratio encoding: each cell runs from one rising edge to the
	// next, and is a one if high for more than half of it
	std::vector<u8> bits;
	attotime rise = attotime::never, fall = attotime::never;
	attotime period = attotime::zero;
	int level = 0;
	for (const auto &[t, state] : m_wr_edges)
	{
		if (state && !level)
		{
			if (!rise.is_never() && !fall.is_never())
			{
				period = t - rise;
				bits.push_back(((fall - rise).as_double() * 2 > period.as_double()) ? 1 : 0);
			}
			rise = t;
			fall = attotime::never;
		}
		else if (!state && level)
			fall = t;
		level = state;
	}
	if (!rise.is_never() && !fall.is_never() && period != attotime::zero)
		bits.push_back(((fall - rise).as_double() * 2 > period.as_double()) ? 1 : 0);
	m_wr_edges.clear();

	// the data sync is the first one
	auto sync = std::find(bits.begin(), bits.end(), 1);
	if (sync == bits.end() || (bits.end() - sync) < 1 + 1024 + 16)
	{
		LOGWRITE("write of %u bits has no complete record, ignored\n", unsigned(bits.size()));
		return;
	}
	++sync;
	u8 data[128];
	for (int i = 0; i < 128; i++)
	{
		data[i] = 0;
		for (int b = 0; b < 8; b++)
			data[i] |= *sync++ << b;
	}
	u16 sum = 0;
	for (int b = 0; b < 16; b++)
		sum |= *sync++ << b;

	m_drive[m_wr_drive]->write_record(m_wr_track, m_wr_pos, data, sum);
}

void dec_tu58_device::device_start()
{
	m_strobe_timer = timer_alloc(FUNC(dec_tu58_device::strobe), this);
	m_run_timer = timer_alloc(FUNC(dec_tu58_device::run_timeout), this);

	save_item(NAME(m_event.when));
	save_item(NAME(m_event.kind));
	save_item(NAME(m_event.pos));
	save_item(NAME(m_event.dir));
	save_item(NAME(m_pb));
	save_item(NAME(m_pc));
	save_item(NAME(m_sid));
	save_item(NAME(m_mark));
	save_item(NAME(m_tmr));
	save_item(NAME(m_tmr_edges));
	save_item(NAME(m_running));
	save_item(NAME(m_so));
	save_item(NAME(m_oe));
	save_item(NAME(m_writing));
	save_item(NAME(m_wr_drive));
	save_item(NAME(m_wr_track));
	save_item(NAME(m_wr_pos));
}

void dec_tu58_device::device_reset()
{
	// 8 data bits, no parity, 1 stop bit; the control register always loaded
	m_uart->write_swe(0);
	m_uart->write_np(1);
	m_uart->write_tsb(0);
	m_uart->write_nb1(1);
	m_uart->write_nb2(1);
	m_uart->write_eps(1);
	m_uart->write_cs(1);
	m_uart->write_rdav(1);
	update_rates();

	m_pb = 0xff;
	m_pc = 0xff;
	m_running = 0;
	m_mark = 0;
	m_tmr_edges = 0;
	m_writing = false;
	m_run_timer->enable(false);
	update_motors();

	output_rxd(m_so);
}

void dec_tu58_device::device_reset_after_children()
{
	// the UART's reset sets its status pins without calling back, so give
	// the CPU their levels
	m_cpu->set_input_line(I8085_RST55_LINE, m_uart->tbmt_r() ? ASSERT_LINE : CLEAR_LINE);
	m_cpu->set_input_line(I8085_RST65_LINE, m_uart->dav_r() ? ASSERT_LINE : CLEAR_LINE);
	m_cpu->set_input_line(I8085_TRAP_LINE, m_uart->fe_r() ? ASSERT_LINE : CLEAR_LINE);
	m_oe = m_uart->or_r();
}

void dec_tu58_device::device_add_mconfig(machine_config &config)
{
	I8085A(config, m_cpu, 4.9152_MHz_XTAL);
	m_cpu->set_addrmap(AS_PROGRAM, &dec_tu58_device::mem_map);
	m_cpu->set_addrmap(AS_IO, &dec_tu58_device::io_map);
	m_cpu->in_sid_func().set(FUNC(dec_tu58_device::sid_r));
	m_cpu->out_sod_func().set(FUNC(dec_tu58_device::sod_w));

	I8155(config, m_pio, 4.9152_MHz_XTAL / 2);
	m_pio->in_pa_callback().set(FUNC(dec_tu58_device::pa_r));
	m_pio->out_pb_callback().set(FUNC(dec_tu58_device::pb_w));
	m_pio->out_pc_callback().set(FUNC(dec_tu58_device::pc_w));
	m_pio->out_to_callback().set(FUNC(dec_tu58_device::tmr_w));

	AY31015(config, m_uart); // 6402
	m_uart->set_auto_rdav(false);
	m_uart->write_so_callback().set(FUNC(dec_tu58_device::so_w));
	m_uart->write_dav_callback().set_inputline(m_cpu, I8085_RST65_LINE);
	m_uart->write_tbmt_callback().set_inputline(m_cpu, I8085_RST55_LINE);
	m_uart->write_fe_callback().set_inputline(m_cpu, I8085_TRAP_LINE);
	m_uart->write_or_callback().set([this] (int state) { m_oe = state; });

	CLOCK(config, m_rxclk, 4.9152_MHz_XTAL / 8);
	m_rxclk->signal_handler().set(m_uart, FUNC(ay31015_device::write_rcp));
	CLOCK(config, m_txclk, 4.9152_MHz_XTAL / 8);
	m_txclk->signal_handler().set(m_uart, FUNC(ay31015_device::write_tcp));

	TU58_CARTRIDGE(config, m_drive[0]);
	TU58_CARTRIDGE(config, m_drive[1]);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(DEC_TU58, device_rs232_port_interface, dec_tu58_device, "dec_tu58", "DEC TU58 DECtape II")
