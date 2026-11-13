// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT125 mono board (54-14277), on the VT100 graphics connector

    The VT125's graphics processor: an 8085 with 24K of mask ROM and 16K of
    DRAM, a vector generator drawing into a 768 x 256 x 2-plane bit map
    (240 lines shown), and an output map driving the VT100's GRAPHICS 1/2
    IN and three analog outputs for a colour monitor.  All its serial ports
    are 8251As on the paddle board in the STP connector, reached over a
    24-pin cable (vt125_cable.h): the host line, the printer port, and the
    link to the VT100's own 8251.  It runs entirely from the VT100's
    24.0734 MHz video clock (graphics connector pin 10, TM-003 8.1): no
    crystal of its own.

    Print set MP01053 (VT125 field maintenance), sheets MB1-MB13, pdf
    pages 20-32.  Memory and I/O decode from MB7 (E40, E81) and MB9 (E92).

    TODO:
    - the vector generator (MB2, MB4-MB6), bit map (MB9-MB13) and video
      out; the GO / DONE handshake completes at once for now
    - the five PROMs only this board has (23-067B1, 068B1, 069B1, 070B1,
      582A2) are undumped
    - status bits D1, D2 (MEM DATA A/B) and D6 (CLK OUT H) read 0
    - interrupt acknowledge cycles take no wait state here

***************************************************************************/

#include "emu.h"
#include "vt100_gport.h"
#include "vt125_cable.h"

#include "cpu/i8085/i8085.h"

#define LOG_VG (1U << 1)

//#define VERBOSE (LOG_GENERAL | LOG_VG)
#include "logmacro.h"

#include <cmath>

#define LOGVG(...) LOGMASKED(LOG_VG, __VA_ARGS__)


namespace {

class vt125_mono_device : public device_t, public device_vt100_gport_interface, public vt125_mono_cable_interface
{
public:
	vt125_mono_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// device_vt100_gport_interface
	virtual void write(u8 data) override { }
	virtual int flag_r() override { return m_graphics_flag; }
	virtual vt100_gport_slot_device::format_t scan(int line, u8 *dots) override { return vt100_gport_slot_device::NONE; }
	virtual void init_w(int state) override;

	// vt125_mono_cable_interface
	virtual void connect(vt125_paddle_cable_interface &paddle) override { m_paddle = &paddle; }
	virtual void read_ready_w(int state) override;
	virtual void trans_ready_w(int state) override;
	virtual void graphics_flag_w(int state) override { m_graphics_flag = state; }
	virtual void clk_shift_w(int state) override { m_clk_shift = state; }

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	required_device<i8085a_cpu_device> m_cpu;

	vt125_paddle_cable_interface *m_paddle;
	memory_passthrough_handler m_wait_taps[3];
	emu_timer *m_reset_timer;
	TIMER_CALLBACK_MEMBER(reset_release) { m_cpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE); }

	u8 m_init;              // INIT H from the VT100
	u8 m_graphics_flag;     // GRAPHICS FLAG L to the VT100, from the paddle
	u8 m_clk_shift;         // CLK SHIFT from the paddle

	u8 m_mode;              // MODE 0/1 (E56), from A1-A0 of the EXEC write
	u16 m_x, m_y;           // vector generator position, 12 bits each
	u8 m_error;
	u8 m_file[4];           // vector generator register file (E59/E76)
	u8 m_pattern_ram[16];   // E89, 16 x 4 (only BD0/BD1 written)
	u8 m_offset[6];         // ports 10-15 (E51)
	u8 m_output_map[4];     // E46/E58, ports 18-1B

	void mem_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;

	void vblank(screen_device &screen, bool state);

	u8 status_r();
	u8 in_com_r(offs_t offset);
	void exec_w(offs_t offset, u8 data);
	void out_com_w(offs_t offset, u8 data);
	void offset_w(offs_t offset, u8 data);
	void output_map_w(offs_t offset, u8 data);
	void x_lo_w(u8 data) { m_x = (m_x & 0xf00) | data; }
	void x_hi_w(u8 data) { m_x = (m_x & 0x0ff) | ((data & 0x0f) << 8); }
	void y_lo_w(u8 data) { m_y = (m_y & 0xf00) | data; }
	void y_hi_w(u8 data) { m_y = (m_y & 0x0ff) | ((data & 0x0f) << 8); }
	void error_w(u8 data) { m_error = data; }
	void pattern_ram_w(offs_t offset, u8 data) { m_pattern_ram[offset] = data & 0x03; }
	void file_w(offs_t offset, u8 data) { m_file[offset] = data; }
};


vt125_mono_device::vt125_mono_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, VT125_MONO, tag, owner, clock)
	, device_vt100_gport_interface(mconfig, *this)
	, m_cpu(*this, "cpu")
	, m_paddle(nullptr)
	, m_init(0)
	, m_graphics_flag(1)
	, m_clk_shift(0)
	, m_mode(0)
	, m_x(0)
	, m_y(0)
	, m_error(0)
{
	std::fill(std::begin(m_file), std::end(m_file), 0);
	std::fill(std::begin(m_pattern_ram), std::end(m_pattern_ram), 0);
	std::fill(std::begin(m_offset), std::end(m_offset), 0);
	std::fill(std::begin(m_output_map), std::end(m_output_map), 0);
}


ROM_START( vt125_mono )
	ROM_REGION(0x8000, "cpu", ROMREGION_ERASEFF)
	ROM_LOAD("23-043e4-00.e22", 0x0000, 0x2000, CRC(0b0ca731) SHA1(e46f5b9f85338c3485e50f0b3864d8d41c531189)) // MK36000 mask ROM
	ROM_LOAD("23-044e4-00.e23", 0x2000, 0x2000, CRC(75f76c16) SHA1(376031c2c81fddbfb19c6983af5608e5c89e6100))
	ROM_LOAD("23-045e4-00.e24", 0x4000, 0x2000, CRC(e33fcff5) SHA1(f4d6cdb1c20f6703bbea18c51c36677d205b9522))
	// E25 (6000-7FFF) is an empty socket "for future options" (assembly drawing)

	ROM_REGION(0x100, "dir", 0) // vector direction, as on the VK100
	ROM_LOAD("23-059b1.e41", 0x0000, 0x0100, CRC(4b63857a) SHA1(3217247d983521f0b0499b5c4ef6b5de9844c465))

	ROM_REGION(0x100, "trans", 0) // X address translation, as on the VK100
	ROM_LOAD("23-060b1.e60", 0x0000, 0x0100, CRC(198317fc) SHA1(00e97104952b3fbe03a4f18d800d608b837d10ae))

	ROM_REGION(0x500, "proms", ROMREGION_ERASEFF)
	ROM_LOAD("23-067b1.e135", 0x0000, 0x0100, NO_DUMP) // 82S135: CPU DRAM cycle sequencer (MB8)
	ROM_LOAD("23-068b1.e64", 0x0100, 0x0100, NO_DUMP) // 82S135: horizontal timing (MB2)
	ROM_LOAD("23-069b1.e66", 0x0200, 0x0100, NO_DUMP) // 82S135: bit map RAM cycles (MB2)
	ROM_LOAD("23-070b1.e71", 0x0300, 0x0100, NO_DUMP) // 82S135: vector generator sequencer (MB2)
	ROM_LOAD("23-582a2.e93", 0x0400, 0x0100, NO_DUMP) // 82S129: bit map write enables (MB13)
ROM_END

const tiny_rom_entry *vt125_mono_device::device_rom_region() const
{
	return ROM_NAME(vt125_mono);
}


// MB9: E92 decodes A15-A13.  ROMs E22-E25 read at 0000-7FFF; the DRAM is
// selected at 8000-BFFF.  Nothing drives the bus for reads at C000-FFFF.
void vt125_mono_device::mem_map(address_map &map)
{
	map(0x0000, 0x7fff).rom().region("cpu", 0);
	map(0x8000, 0xbfff).ram();
}

// MB7: I/O decodes A7-A0 only.  Reads: E81 section A on A4-A3, so A7-A5
// and A2-A0 are ignored except as the paddle's A2-A0.  Writes with A7 = 0:
// E81 section B on A4-A3; with A7 = 1: E40 on A6-A4, A3-A0 ignored.
void vt125_mono_device::io_map(address_map &map)
{
	map.global_mask(0xff);

	map(0x00, 0x07).mirror(0xe0).r(FUNC(vt125_mono_device::status_r));
	map(0x08, 0x0f).mirror(0xe0).r(FUNC(vt125_mono_device::in_com_r));

	map(0x00, 0x07).mirror(0x60).w(FUNC(vt125_mono_device::exec_w));       // EXEC
	map(0x08, 0x0f).mirror(0x60).w(FUNC(vt125_mono_device::out_com_w));    // OUT COM
	map(0x10, 0x17).mirror(0x60).w(FUNC(vt125_mono_device::offset_w));     // LD OFFSET
	map(0x18, 0x1f).mirror(0x60).w(FUNC(vt125_mono_device::output_map_w)); // WR RAM
	map(0x80, 0x80).mirror(0x0f).w(FUNC(vt125_mono_device::x_lo_w));
	map(0x90, 0x90).mirror(0x0f).w(FUNC(vt125_mono_device::x_hi_w));
	map(0xa0, 0xa0).mirror(0x0f).w(FUNC(vt125_mono_device::y_lo_w));
	map(0xb0, 0xb0).mirror(0x0f).w(FUNC(vt125_mono_device::y_hi_w));
	map(0xc0, 0xc0).mirror(0x0f).w(FUNC(vt125_mono_device::error_w));
	map(0xd0, 0xdf).w(FUNC(vt125_mono_device::pattern_ram_w));             // LD RAM
	map(0xf0, 0xf3).mirror(0x0c).w(FUNC(vt125_mono_device::file_w));       // LD FILE
}


void vt125_mono_device::device_add_mconfig(machine_config &config)
{
	// X1/X2 from the 6 MHz taps of E19, the 24.0734 MHz HS CLK divided by
	// 4 (MB1); CLK OUT is half that
	I8085A(config, m_cpu, 24.0734_MHz_XTAL / 4);
	m_cpu->set_addrmap(AS_PROGRAM, &vt125_mono_device::mem_map);
	m_cpu->set_addrmap(AS_IO, &vt125_mono_device::io_map);
}

void vt125_mono_device::device_start()
{
	screen().register_vblank_callback(vblank_state_delegate(&vt125_mono_device::vblank, this));

	// READY (MB8 E140, E95; MB9 E92): E140 latches ALE and drops READY a
	// clock later unless E92 decodes a memory cycle at 8000-BFFF.  So the
	// program DRAM runs at full speed and every other bus cycle -- the mask
	// ROMs, C000-FFFF and all I/O -- takes one wait state (inferred from
	// the circuit; the 8085A data sheet's READY timing wasn't to hand).
	// Taps see opcode fetches too, so they cover every ROM cycle.
	m_reset_timer = timer_alloc(FUNC(vt125_mono_device::reset_release), this);

	auto wait = [this] (offs_t, u8 &, u8) { m_cpu->adjust_icount(-1); };
	address_space &prog = m_cpu->space(AS_PROGRAM);
	address_space &io = m_cpu->space(AS_IO);
	m_wait_taps[0] = prog.install_read_tap(0x0000, 0x7fff, "rom_wait", wait);
	m_wait_taps[1] = prog.install_readwrite_tap(0xc000, 0xffff, "hi_wait", wait, wait);
	m_wait_taps[2] = io.install_readwrite_tap(0x00, 0xff, "io_wait", wait, wait);

	save_item(NAME(m_init));
	save_item(NAME(m_graphics_flag));
	save_item(NAME(m_clk_shift));
	save_item(NAME(m_mode));
	save_item(NAME(m_x));
	save_item(NAME(m_y));
	save_item(NAME(m_error));
	save_item(NAME(m_file));
	save_item(NAME(m_pattern_ram));
	save_item(NAME(m_offset));
	save_item(NAME(m_output_map));
}

void vt125_mono_device::device_reset()
{
	m_mode = 0;

	// Power-up reset: RST IN is R27 560K / C112 1uF (MB7), against the
	// VT100's 8224 RESIN on R2 10K / C10 39uF (MP-00633 BV6).  Both are
	// Schmitt inputs with only bounds given (8085AH 0.8-2.4 V, 8224
	// 0.8-2.6 V); taking both to switch at 1.6 V, mid-band, the 8085 comes
	// out of reset ~54 ms after the 8080 (inferred).  That matters: the
	// firmware's link queries go out a fixed ~0.3 s after INIT H falls or
	// its self-test ends, and the VT100's 8251 receiver is only enabled
	// ~0.33 s after INIT falls.  Released with the 8080 at the same time,
	// the 8085 finishes its self-test before INIT falls and the queries are
	// lost (self-test error 08).
	m_cpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	m_reset_timer->adjust(attotime::from_double(
			560e3 * 1e-6 * std::log(5.0 / (5.0 - 1.6)) - 10e3 * 39e-6 * std::log(5.0 / (5.0 - 1.6))));
}


// INIT H from the VT100 is the 8085's TRAP (MB7); the board itself is reset
// only at power-up (RESET IN from its own RC network)
void vt125_mono_device::init_w(int state)
{
	m_init = state ? 1 : 0;
	m_cpu->set_input_line(I8085_TRAP_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}

// V BLANK H (VERT BLANK L from the VT100, inverted by E78) is RST 7.5,
// which latches on the rising edge, the start of vertical blank
void vt125_mono_device::vblank(screen_device &screen, bool state)
{
	m_cpu->set_input_line(I8085_RST75_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}

// READ READY and TRANS READY: the paddle's three 8251s' RxRDY and TxRDY,
// each wire-ORed (MB7, paddle sheet 2)
void vt125_mono_device::read_ready_w(int state)
{
	m_cpu->set_input_line(I8085_RST65_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}

void vt125_mono_device::trans_ready_w(int state)
{
	m_cpu->set_input_line(I8085_RST55_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}


// RD STAT (E35).  The bit order is read from the drop nesting on MB7, which
// is hard to follow; the firmware polls D0 and D7 as INIT and GO.
u8 vt125_mono_device::status_r()
{
	return (m_init ? 0x01 : 0x00)
			| (screen().vblank() ? 0x08 : 0x00)
			| 0x30
			| 0x80; // GO L: the vector generator is idle
}

u8 vt125_mono_device::in_com_r(offs_t offset)
{
	// pulled up when the cable isn't connected (inferred)
	return m_paddle ? m_paddle->in_com(offset) : 0xff;
}

void vt125_mono_device::out_com_w(offs_t offset, u8 data)
{
	if (m_paddle)
		m_paddle->out_com(offset, data);
}

// EXEC: A1-A0 load MODE 0/1 (E56), D0 sets GO (E38)
void vt125_mono_device::exec_w(offs_t offset, u8 data)
{
	m_mode = offset & 3;
	if (BIT(data, 0))
		LOGVG("%s: GO mode %d at %d,%d file %02x %02x %02x %02x error %02x\n", machine().describe_context(),
				m_mode, m_x, m_y, m_file[0], m_file[1], m_file[2], m_file[3], m_error);
}

// E51: 10 minor offset, 11 horizontal offset, 12 vertical offset, 13
// pattern (E28), 14 pattern multiplier (E88), 15 CLR CLK SHIFT
void vt125_mono_device::offset_w(offs_t offset, u8 data)
{
	if (offset < std::size(m_offset))
		m_offset[offset] = data;
}

// the output map: four 8-bit entries, two 74LS670s (MB1)
void vt125_mono_device::output_map_w(offs_t offset, u8 data)
{
	m_output_map[offset & 3] = data;
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(VT125_MONO, device_vt100_gport_interface, vt125_mono_device, "vt125_mono", "DEC VT125 mono board (graphics processor)")
