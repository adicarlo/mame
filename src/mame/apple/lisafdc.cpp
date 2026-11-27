// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

// The LISA FDC subsystem

#include "emu.h"
#include "lisafdc.h"

// Read command
// lisa 1: 136a
//   87 88 00 00 00 00 00 00 (wait until boot)
//   86 08 00 00 00 00 00 00
//   85 ff 00 00 00 00 00 00
//   81 00 00 00 00 00 00 00
//   85 cc 00 00 00 00 00 00
//   81 00 00 00 00 00 00 00

// lisa 2: 149a
//   86 80 00 00 00 00 00 00
//   85 ff 00 00 00 00 00 00
//   81 00 00 00 00 00 00 00

// lisa2 floppy failure
//    1db3 = write 10 at +b (fail!)
// -> 1d8a =
//    bp 1d7a

// macxl: 149e
//   87 88 00 00 00 00 00 00 (wait until boot)
//   86 80 00 00 00 00 00 00
//   85 ff 00 00 00 00 00 00
//   81 00 80 00 00 00 00 00
//   85 cc 80 00 00 00 00 00

// 050-4008-L_IWM_floppy_pg4.pdf

// 54321 (0=value)
// 00000 ph0
// 00001 ph1
// 00010 ph2
// 00011 ph3
// 00100 !hds
// 00101 enable/disable lss clocking (4MHz exactly)
// 00110 prom a2
// 00111 prom a3 & wrq
// 01000 dr0
// 01001 dr1
// 01010 !mt0
// 01011 !mt1
// 01100 dis
// 01101 nc
// 01110 dskdiag
// 01111 fdir

// latch read, ma0=0 && ma45=00


DEFINE_DEVICE_TYPE(LISAFDC,  lisa_fdc_device,  "lisafdc",  "Lisa 1 FDC subsystem")
DEFINE_DEVICE_TYPE(LISA2FDC, lisa2_fdc_device, "lisa2fdc", "Lisa 2 FDC subsystem")
DEFINE_DEVICE_TYPE(MACXLFDC, macxl_fdc_device, "macxlfdc", "Mac XL/Lisa 210 FDC subsystem")

ROM_START( lisa_fdc )
	ROM_REGION(0x100, "state", 0)
	ROM_LOAD("341-0172-a", 0x000, 0x0100, CRC(223f3917) SHA1(c7d32f091c3a02a0f8159bf51f6c810d23bd7b9f)) /* Very similar to the diskii's 341-0028 */

	ROM_REGION(0x1000, "cpu", 0)
	// note: other ?prototype? revisions of this rom for the lisa probably exist as well
	ROM_LOAD("341-0138-f", 0x0000, 0x1000, CRC(edd8d560) SHA1(872211d21386cd9625b3735d7682e2b2ecff05b4) )
ROM_END

ROM_START( lisa2_fdc )
	ROM_REGION(0x100, "state", 0)
	ROM_LOAD("341-0172-a", 0x000, 0x0100, CRC(223f3917) SHA1(c7d32f091c3a02a0f8159bf51f6c810d23bd7b9f)) /* Very similar to the diskii's 341-0028 */

	ROM_REGION(0x1000, "cpu",0)
	ROM_LOAD("341-0290-b", 0x0000, 0x1000, CRC(bc6364f1) SHA1(f3164923330a51366a06d9d8a4a01ec7b0d3a8aa)) // 341-0290-B LISA 2/5 Disk Rom (ioa8), supports profile on external port
ROM_END

ROM_START( macxl_fdc )
	ROM_REGION(0x1000, "cpu", 0)
	ROM_DEFAULT_BIOS("apple")

	ROM_SYSTEM_BIOS(0, "apple", "LISA 2/10 Disk Rom")
	ROMX_LOAD("341-0281-d", 0x0000, 0x1000, CRC(e343fe74) SHA1(a0e484ead2d2315fca261f39fff2f211ff61b0ef), ROM_BIOS(0)) // 341-0281-D LISA 2/10 Disk Rom (io88), supports widget on internal port

	ROM_SYSTEM_BIOS(1, "sun", "Sun Mac XL Disk rom for 800k drives (Rev C, from Goodwill XL)")
	ROMX_LOAD("341-8003-c", 0x0000, 0x1000, CRC(8c67959a) SHA1(aa446b0c4acb4cb6c9d0adfbbea900fb8c04c1e9), ROM_BIOS(1)) // 341-8003-C Sun Mac XL Disk rom for 800k drives (Rev C, from Goodwill XL) (io88800k)
	// Note: there are two earlier/alternate versions of this rom as well which are dumped */
ROM_END

const tiny_rom_entry *lisa_fdc_device::device_rom_region() const
{
	return ROM_NAME( lisa_fdc );
}

const tiny_rom_entry *lisa2_fdc_device::device_rom_region() const
{
	return ROM_NAME( lisa2_fdc );
}

// The Lisa 2/5's Sony drive takes the speed byte as the same PWM value
// the Mac XL board writes to its speed register: both firmwares start
// with $FF, $D5, $D0 (inferred; the converter board isn't documented)
void lisa2_fdc_device::speed_w(u8 data)
{
	m_pwm = data;
	update_pwm();
}

void lisa2_fdc_device::update_sel()
{
	lisa_original_fdc_device::update_sel();
	update_pwm();
}

const tiny_rom_entry *macxl_fdc_device::device_rom_region() const
{
	return ROM_NAME( macxl_fdc );
}

lisa_base_fdc_device::lisa_base_fdc_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, type, tag, owner, clock),
	m_cpu(*this, "cpu"),
	m_nvram(*this, "ram"),
	m_ram(*this, "ram"),
	m_floppy(*this, "%d", 0U),
	m_diag_cb(*this),
	m_fdir_cb(*this),
	m_cur_floppy(nullptr)
{
}

lisa_original_fdc_device::lisa_original_fdc_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	lisa_base_fdc_device(mconfig, type, tag, owner, clock),
	m_lss_rom(*this, "state")
{
}

lisa_fdc_device::lisa_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	lisa_original_fdc_device(mconfig, LISAFDC, tag, owner, clock)
{
}

lisa2_fdc_device::lisa2_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	lisa_original_fdc_device(mconfig, LISA2FDC, tag, owner, clock)
{
}

macxl_fdc_device::macxl_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	lisa_base_fdc_device(mconfig, MACXLFDC, tag, owner, clock),
	m_iwm(*this, "iwm")
{
}

void lisa_original_fdc_device::device_add_mconfig(machine_config &config)
{
	M6504(config, m_cpu, 16_MHz_XTAL / 8);
	m_cpu->set_addrmap(AS_PROGRAM, &lisa_original_fdc_device::map);

	NVRAM(config, m_nvram);
}

void lisa_fdc_device::device_add_mconfig(machine_config &config)
{
	lisa_original_fdc_device::device_add_mconfig(config);
	applefdintf_device::add_twiggy(config, m_floppy[0]);
	applefdintf_device::add_twiggy(config, m_floppy[1]);
}

void lisa2_fdc_device::device_add_mconfig(machine_config &config)
{
	lisa_original_fdc_device::device_add_mconfig(config);
	applefdintf_device::add_35_nc(config, m_floppy[0]);
	applefdintf_device::add_35_sd(config, m_floppy[1]);
}

void macxl_fdc_device::device_add_mconfig(machine_config &config)
{
	M6504(config, m_cpu, 16_MHz_XTAL / 8);
	m_cpu->set_addrmap(AS_PROGRAM, &macxl_fdc_device::map);

	NVRAM(config, m_nvram);

	IWM(config, m_iwm, 16_MHz_XTAL / 2);
	m_iwm->devsel_cb().set(FUNC(macxl_fdc_device::devsel_w));
	m_iwm->phases_cb().set(FUNC(macxl_fdc_device::phases_w));

	applefdintf_device::add_35_nc(config, m_floppy[0]);
	applefdintf_device::add_35_sd(config, m_floppy[1]);
}

void lisa_base_fdc_device::device_start()
{
	save_item(NAME(m_hds));
	save_item(NAME(m_sel));
	save_item(NAME(m_phases));
	save_item(NAME(m_dis));
	save_item(NAME(m_pwm));
}

void lisa_base_fdc_device::device_reset()
{
	m_sel = 0;
	m_hds = 0;
	m_phases = 0;
	m_dis = false;
	m_pwm = 0;
	update_sel();
}

u8 lisa_base_fdc_device::ram_r(offs_t offset)
{
	return m_ram[offset];
}

void lisa_base_fdc_device::ram_w(offs_t offset, u8 data)
{
	m_ram[offset] = data;
}

void lisa_base_fdc_device::update_sel()
{
	if(m_sel == 1)
		m_cur_floppy = m_floppy[0]->get_device();
	else if(m_sel == 2)
		m_cur_floppy = m_floppy[1]->get_device();
	else
		m_cur_floppy = nullptr;

	if(m_cur_floppy) {
		m_cur_floppy->ss_w(m_hds);
		m_cur_floppy->seek_phase_w(m_phases);
	}
}

void lisa_base_fdc_device::update_phases()
{
	if(m_cur_floppy)
		m_cur_floppy->seek_phase_w(m_phases);
}

void lisa_original_fdc_device::map(address_map &map)
{
	map(0x0000, 0x03ff).ram().share(m_ram);

	map(0x0400, 0x040f).rw(FUNC(lisa_original_fdc_device::latch_r), FUNC(lisa_original_fdc_device::latch_w));

	map(0x0410, 0x0410).lr8(NAME([this]() -> u8 { dr1_0();  return 0; })).lw8(NAME([this](u8) { dr1_0();  }));
	map(0x0411, 0x0411).lr8(NAME([this]() -> u8 { dr1_1();  return 0; })).lw8(NAME([this](u8) { dr1_1();  }));
	map(0x0412, 0x0412).lr8(NAME([this]() -> u8 { dr2_0();  return 0; })).lw8(NAME([this](u8) { dr2_0();  }));
	map(0x0413, 0x0413).lr8(NAME([this]() -> u8 { dr2_1();  return 0; })).lw8(NAME([this](u8) { dr2_1();  }));
	map(0x0414, 0x0414).lr8(NAME([this]() -> u8 { mt0_0();  return 0; })).lw8(NAME([this](u8) { mt0_0();  }));
	map(0x0415, 0x0415).lr8(NAME([this]() -> u8 { mt0_1();  return 0; })).lw8(NAME([this](u8) { mt0_1();  }));
	map(0x0416, 0x0416).lr8(NAME([this]() -> u8 { mt1_0();  return 0; })).lw8(NAME([this](u8) { mt1_0();  }));
	map(0x0417, 0x0417).lr8(NAME([this]() -> u8 { speed_clock(); return 0; })).lw8(NAME([this](u8) { speed_clock(); }));
	map(0x0418, 0x0418).lr8(NAME([this]() -> u8 { dis_0();  return 0; })).lw8(NAME([this](u8) { dis_0();  }));
	map(0x0419, 0x0419).lr8(NAME([this]() -> u8 { dis_1();  return 0; })).lw8(NAME([this](u8) { dis_1();  }));
	map(0x041c, 0x041c).lr8(NAME([this]() -> u8 { diag_0(); return 0; })).lw8(NAME([this](u8) { diag_0(); }));
	map(0x041d, 0x041d).lr8(NAME([this]() -> u8 { diag_1(); return 0; })).lw8(NAME([this](u8) { diag_1(); }));
	map(0x041e, 0x041e).lr8(NAME([this]() -> u8 { fdir_0(); return 0; })).lw8(NAME([this](u8) { fdir_0(); }));
	map(0x041f, 0x041f).lr8(NAME([this]() -> u8 { fdir_1(); return 0; })).lw8(NAME([this](u8) { fdir_1(); }));

	map(0x1000, 0x1fff).rom().region("cpu", 0);
}

void macxl_fdc_device::map(address_map &map)
{
	map(0x0000, 0x03ff).ram().share(m_ram);

	map(0x0800, 0x080f).rw(m_iwm, FUNC(iwm_device::read), FUNC(iwm_device::write));
	map(0x0810, 0x0810).lr8(NAME([this]() -> u8 { stop_0(); return 0; })).lw8(NAME([this](u8) { stop_0(); }));
	map(0x0811, 0x0811).lr8(NAME([this]() -> u8 { stop_1(); return 0; })).lw8(NAME([this](u8) { stop_1(); }));
	map(0x0814, 0x0814).lr8(NAME([this]() -> u8 { mt0_0();  return 0; })).lw8(NAME([this](u8) { mt0_0();  }));
	map(0x0815, 0x0815).lr8(NAME([this]() -> u8 { mt0_1();  return 0; })).lw8(NAME([this](u8) { mt0_1();  }));
	map(0x0816, 0x0816).lr8(NAME([this]() -> u8 { mt1_0();  return 0; })).lw8(NAME([this](u8) { mt1_0();  }));
	map(0x0817, 0x0817).lr8(NAME([this]() -> u8 { mt1_1();  return 0; })).lw8(NAME([this](u8) { mt1_1();  }));
	map(0x0818, 0x0818).lr8(NAME([this]() -> u8 { dis_0();  return 0; })).lw8(NAME([this](u8) { dis_0();  }));
	map(0x0819, 0x0819).lr8(NAME([this]() -> u8 { dis_1();  return 0; })).lw8(NAME([this](u8) { dis_1();  }));
	map(0x081a, 0x081a).lr8(NAME([this]() -> u8 { hds_0();  return 0; })).lw8(NAME([this](u8) { hds_0();  }));
	map(0x081b, 0x081b).lr8(NAME([this]() -> u8 { hds_1();  return 0; })).lw8(NAME([this](u8) { hds_1();  }));
	map(0x081c, 0x081c).lr8(NAME([this]() -> u8 { diag_0(); return 0; })).lw8(NAME([this](u8) { diag_0(); }));
	map(0x081d, 0x081d).lr8(NAME([this]() -> u8 { diag_1(); return 0; })).lw8(NAME([this](u8) { diag_1(); }));
	map(0x081e, 0x081e).lr8(NAME([this]() -> u8 { fdir_0(); return 0; })).lw8(NAME([this](u8) { fdir_0(); }));
	map(0x081f, 0x081f).lr8(NAME([this]() -> u8 { fdir_1(); return 0; })).lw8(NAME([this](u8) { fdir_1(); }));
	map(0x0820, 0x0820).w(FUNC(macxl_fdc_device::pwm_w));

	map(0x1000, 0x1fff).rom().region("cpu", 0);
}

void macxl_fdc_device::stop_0()
{
	logerror("stop 0\n");
}

void macxl_fdc_device::stop_1()
{
	logerror("stop 1\n");
}

void lisa_base_fdc_device::mt0_0()
{
	logerror("mt0 0\n");
}

void lisa_base_fdc_device::mt0_1()
{
	logerror("mt0 1\n");
}

void lisa_base_fdc_device::mt1_0()
{
	logerror("mt1 0\n");
}

void lisa_base_fdc_device::mt1_1()
{
	logerror("mt1 1\n");
}

// DIS locks the 68000 out of the shared RAM (HW manual Sep82 6.2.2)
void lisa_base_fdc_device::dis_0()
{
	m_dis = false;
}

void lisa_base_fdc_device::dis_1()
{
	m_dis = true;
}

void lisa_base_fdc_device::hds_0()
{
	m_hds = 0;
	if(m_cur_floppy)
		m_cur_floppy->ss_w(m_hds);
}

void lisa_base_fdc_device::hds_1()
{
	m_hds = 1;
	if(m_cur_floppy)
		m_cur_floppy->ss_w(m_hds);
}

void lisa_base_fdc_device::diag_0()
{
	m_diag_cb(0);
}

void lisa_base_fdc_device::diag_1()
{
	m_diag_cb(1);
}

void lisa_base_fdc_device::fdir_0()
{
	m_fdir_cb(0);
}

void lisa_base_fdc_device::fdir_1()
{
	m_fdir_cb(1);
}

// The original I/O board's disk state machine (HW manual Sep82 6.2.4,
// 6.3.5): the 341-0172 PROM, a state latch and an LS323 shift register,
// clocked at 4 MHz (250 ns per state).  The PROM is the Disk II's 341-0028
// with a different write entry sequence and is wired the same way, so the
// sequencing below mirrors wozfdc's lss_sync.  PROM address: bit 1 =
// shift register MSB, bit 2 = A2, bit 3 = A3 (write), bit 4 = read pulse
// (low at a flux transition), bits 0 and 5-7 = state; bit 7 also drives
// the write data.  Not modelled: the timing counter stalling during 68000
// accesses (it stalls the 6504 too).

// A flux transition is presented to the PROM for two states (500 ns).
// With one state, as wozfdc does at the Disk II's 2 MHz, some
// transitions fall where the read program doesn't look and bits are
// dropped (the 341-0290-B firmware reads track 0 headers wrong and
// reports error 23).  The width is inferred: the edge detector's clock
// isn't clear in the manual (6.2.3 names Q1 and Q2 on facing pages).
static constexpr u64 READ_PULSE_CYCLES = 2;

u64 lisa_original_fdc_device::time_to_lss_cycles(const attotime &tm) const
{
	return tm.as_ticks(16_MHz_XTAL / 4);
}

attotime lisa_original_fdc_device::lss_cycles_to_time(u64 cycles) const
{
	return attotime::from_ticks(cycles, 16_MHz_XTAL / 4);
}

void lisa_original_fdc_device::lss_sync(u64 extra_cycles)
{
	attotime next_flux = m_cur_floppy ? m_cur_floppy->get_next_transition(lss_cycles_to_time(m_lss_cycles - 1)) : attotime::never;
	u64 cycles_next_flux = next_flux != attotime::never ? time_to_lss_cycles(next_flux) : u64(-1);
	u64 cycles_next_flux_down = cycles_next_flux != u64(-1) ? cycles_next_flux + READ_PULSE_CYCLES : u64(-1);

	if(m_lss_cycles >= cycles_next_flux && m_lss_cycles < cycles_next_flux_down)
		m_lss_address &= ~0x10;
	else
		m_lss_address |= 0x10;

	const u64 cycles_limit = time_to_lss_cycles(machine().time()) + extra_cycles;
	while(m_lss_cycles < cycles_limit) {
		u64 cycles_next_trans = cycles_limit;
		if(cycles_next_trans > cycles_next_flux && m_lss_cycles < cycles_next_flux)
			cycles_next_trans = cycles_next_flux;
		if(cycles_next_trans > cycles_next_flux_down && m_lss_cycles < cycles_next_flux_down)
			cycles_next_trans = cycles_next_flux_down;

		while(m_lss_cycles < cycles_next_trans) {
			const u8 opcode = m_lss_rom[m_lss_address];

			if(m_lss_address & 0x08) {
				if(m_lss_write_line != bool(m_lss_address & 0x80)) {
					m_lss_write_line = !m_lss_write_line;
					if(m_cur_floppy)
						m_cur_floppy->write_flux_change(lss_cycles_to_time(m_lss_cycles));
				}
			}

			m_lss_address = (m_lss_address & 0x1e) | (opcode & 0xc0) | ((opcode & 0x20) >> 5) | ((opcode & 0x10) << 1);
			switch(opcode & 0x0f) {
			case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7:
				m_lss_data = 0x00;
				break;
			case 0x8: case 0xc:
				break;
			case 0x9:
				m_lss_data <<= 1;
				break;
			case 0xa: case 0xe:
				// Sense: the drive's sense line shifts in at the top
				m_lss_data = (m_lss_data >> 1) | (m_cur_floppy && m_cur_floppy->wpt_r() ? 0x80 : 0x00);
				break;
			case 0xb: case 0xf:
				m_lss_data = m_lss_load_data;
				break;
			case 0xd:
				m_lss_data = (m_lss_data << 1) | 0x01;
				break;
			}
			if(m_lss_data & 0x80)
				m_lss_address |= 0x02;
			else
				m_lss_address &= ~0x02;
			m_lss_cycles++;
		}

		if(m_lss_cycles == cycles_next_flux)
			m_lss_address &= ~0x10;
		else if(m_lss_cycles == cycles_next_flux_down) {
			m_lss_address |= 0x10;
			next_flux = m_cur_floppy ? m_cur_floppy->get_next_transition(lss_cycles_to_time(m_lss_cycles)) : attotime::never;
			cycles_next_flux = next_flux != attotime::never ? time_to_lss_cycles(next_flux) : u64(-1);
			cycles_next_flux_down = cycles_next_flux != u64(-1) ? cycles_next_flux + READ_PULSE_CYCLES : u64(-1);
		}
	}
}

// $0400-$040F: the LS259 at C-2 (A3-A1 pick the bit, A0 is the value):
// phases 0-3, HDS, state machine clock select, A2, A3/WRQ.  Reads of even
// addresses also put the shift register on the data bus.
void lisa_original_fdc_device::latch_control(offs_t offset)
{
	const bool state = offset & 1;
	switch(offset >> 1) {
	case 0: case 1: case 2: case 3:
		if(state)
			m_phases |= 1 << (offset >> 1);
		else
			m_phases &= ~(1 << (offset >> 1));
		update_phases();
		break;
	case 4:
		// The latch holds /HDS ("00100 !hds" above).  On the Lisa 2/5 the
		// firmware's Sony commands only decode that way: $134F is MotorOn,
		// $1331 reads NotTrack0, $1405 reads NoDrive then DoubleSide.
		if(state)
			hds_0();
		else
			hds_1();
		break;
	case 5:
		// "enable/disable lss clocking (4MHz exactly)": the firmware sets it
		// around writes; the effect isn't documented, so it is only kept
		m_lss_clock_sel = state;
		break;
	case 6:
		if(state)
			m_lss_address |= 0x04;
		else
			m_lss_address &= ~0x04;
		break;
	case 7:
		if(state && !(m_lss_address & 0x08)) {
			m_lss_address |= 0x08;
			if(m_cur_floppy) {
				m_cur_floppy->write_start(machine().time());
				m_cur_floppy->set_write_splice(machine().time());
			}
		} else if(!state && (m_lss_address & 0x08)) {
			m_lss_address &= ~0x08;
			if(m_cur_floppy)
				m_cur_floppy->write_end(machine().time());
		}
		break;
	}
}

u8 lisa_original_fdc_device::latch_r(offs_t offset)
{
	if(machine().side_effects_disabled())
		return (offset & 1) ? 0x00 : m_lss_data;

	lss_sync();
	latch_control(offset);
	if(offset & 1)
		return 0x00;

	// The state machine runs faster than the 6504 and gets one more
	// cycle in before the data is sampled (as wozfdc)
	lss_sync(1);
	return m_lss_data;
}

void lisa_original_fdc_device::latch_w(offs_t offset, u8 data)
{
	lss_sync();
	latch_control(offset);
	m_lss_load_data = data;
}

// The 6504 clocks a speed byte out to the drive, MSB first: the data on
// PH0, a clock on each MT1/ set (HW manual Sep82 Fig. 9-2: "Phase0/motor
// data", "Motor Data Clock").  Taken eight bits at a time; the drive end
// has no framing signal, so that is inferred from the firmware always
// sending whole bytes ($13B0 in 341-0290-B).
void lisa_original_fdc_device::speed_clock()
{
	m_speed_shift = (m_speed_shift << 1) | (m_phases & 1);
	if(++m_speed_bits == 8) {
		m_speed_bits = 0;
		speed_w(m_speed_shift);
	}
}

void lisa_original_fdc_device::update_sel()
{
	if(m_lss_rom.target())
		lss_sync();
	lisa_base_fdc_device::update_sel();
}

void lisa_original_fdc_device::device_start()
{
	lisa_base_fdc_device::device_start();
	m_lss_cycles = 0;
	m_lss_address = 0;
	m_lss_data = 0;
	m_lss_load_data = 0;
	m_lss_write_line = false;
	m_lss_clock_sel = false;
	m_speed_shift = 0;
	m_speed_bits = 0;
	save_item(NAME(m_speed_shift));
	save_item(NAME(m_speed_bits));
	save_item(NAME(m_lss_cycles));
	save_item(NAME(m_lss_address));
	save_item(NAME(m_lss_data));
	save_item(NAME(m_lss_load_data));
	save_item(NAME(m_lss_write_line));
	save_item(NAME(m_lss_clock_sel));
}

void lisa_original_fdc_device::device_reset()
{
	lisa_base_fdc_device::device_reset();
	m_lss_cycles = time_to_lss_cycles(machine().time());
	m_lss_address &= ~0x0e;
	m_lss_data = 0;
	m_lss_write_line = false;
	m_lss_clock_sel = false;
	m_speed_bits = 0;
}

void lisa_original_fdc_device::dr1_0()
{
	m_sel &= ~1;
	update_sel();
}

void lisa_original_fdc_device::dr1_1()
{
	m_sel |= 1;
	update_sel();
}

void lisa_original_fdc_device::dr2_0()
{
	m_sel &= ~2;
	update_sel();
}

void lisa_original_fdc_device::dr2_1()
{
	m_sel |= 2;
	update_sel();
}

void macxl_fdc_device::phases_w(uint8_t phases)
{
	m_phases = phases;
	update_phases();
}

void macxl_fdc_device::devsel_w(uint8_t devsel)
{
	m_sel = devsel;
	update_sel();
}

void macxl_fdc_device::update_sel()
{
	lisa_base_fdc_device::update_sel();
	m_iwm->set_floppy(m_cur_floppy);
	update_pwm();
}

void macxl_fdc_device::device_start()
{
	lisa_base_fdc_device::device_start();
}

void macxl_fdc_device::device_reset()
{
	lisa_base_fdc_device::device_reset();
}

void lisa_base_fdc_device::update_pwm()
{
	if(!m_cur_floppy)
		return;
	// Use a ramp somewhat similar to the macs
	// The documentation requires:
	// - duty cycle of 9.4%, 305 < rpm < 380 (middle 342.5, rom expectation 361)
	// - duty cycle of 91%,  625 < rpm < 780 (middle 702.5, rom expectation 723.5)
	// - linear between these two points

	// Assume 0 = duty cycle of 100%, ff = duty cycle of 0%

	float duty_cycle = (255 - m_pwm) / 255.0;
	float rpm = (duty_cycle - 0.094) * (723.5 - 361) / (0.91 - 0.094) + 361;

	logerror("rpm %02x -> %f (%s)\n", m_pwm, rpm, machine().describe_context());
	m_cur_floppy->set_rpm(rpm);
}

void macxl_fdc_device::pwm_w(u8 data)
{
	m_pwm = data;
	update_pwm();
}
