// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

// The LISA MMU

#include "emu.h"
#include "lisammu.h"

DEFINE_DEVICE_TYPE(LISAMMU, lisa_mmu_device, "lisammu", "Lisa MMU")

lisa_mmu_device::lisa_mmu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, LISAMMU, tag, owner, clock),
	device_memory_interface(mconfig, *this),
	m_nmi_cb(*this),
	m_maincpu(*this, finder_base::DUMMY_TAG),
	m_video(*this, finder_base::DUMMY_TAG),
	m_ram_config("ram", ENDIANNESS_BIG, 16, 24),
	m_io_config("io", ENDIANNESS_BIG, 16, 24),
	m_special_io_config("special_io", ENDIANNESS_BIG, 16, 24),
	m_cpu_space_config("cpu_space", ENDIANNESS_BIG, 16, 24, 0, address_map_constructor(FUNC(lisa_mmu_device::default_autovectors_map), this))
{
}

lisa_mmu_device::mmu::mmu(lisa_mmu_device &dev, m68000_device *maincpu, address_space &ram, address_space &io, address_space &special_io, address_space &cpu_space) : m_dev(dev), m_maincpu(maincpu), m_a_ram(ram), m_a_io(io), m_a_special_io(special_io), m_a_cpu_space(cpu_space)
{
	ram.specific(m_ram);
	io.specific(m_io);
	special_io.specific(m_special_io);
	cpu_space.specific(m_cpu_space);

	for(int i=0; i != 128*4; i++) {
		m_base_address[i] = 0;
		m_limit_address[i] = 0;
		m_mode[i] = INVALID;
	}

	for(int i=0; i != 128; i++) {
		m_base_address[i + 128*4] = i << 17;
		m_limit_address[i + 128*4] = (i+1) << 17;
		m_mode[i + 128*4] = SPECIAL;
	}

	m_seg = 0;
	m_super = true;
	m_setup = true;

	recompute_slot();
}

const u8 lisa_mmu_device::mode_table[16] = {
	INVALID, INVALID, INVALID, INVALID,
	MAIN_RO_STACK, MAIN_RO, MAIN_RW_STACK, MAIN_RW,
	INVALID, IO, INVALID, INVALID,
	INVALID, INVALID, INVALID, SPECIAL
};

void lisa_mmu_device::mmu::setup_entry(int entry, u16 sor, u16 slr)
{
	m_base_address[entry] = (sor & 0xfff) << 9;
	m_mode[entry] = mode_table[(slr & 0xf00) >> 8];
	if(m_mode[entry] == MAIN_RO_STACK || m_mode[entry] == MAIN_RW_STACK)
		m_limit_address[entry] = m_base_address[entry] + ((0xff - (slr & 0xff)) << 9);
	else
		m_limit_address[entry] = m_base_address[entry] + ((0x100 - (slr & 0xff)) << 9);
}

u16 lisa_mmu_device::mmu::read_program(offs_t addr, u16 mem_mask)
{
	return read(addr, mem_mask);
}

u16 lisa_mmu_device::mmu::read_data(offs_t addr, u16 mem_mask)
{
	return read(addr, mem_mask);
}

void lisa_mmu_device::mmu::write_program(offs_t addr, u16 data, u16 mem_mask)
{
	write(addr, data, mem_mask);
}

void lisa_mmu_device::mmu::write_data(offs_t addr, u16 data, u16 mem_mask)
{
	write(addr, data, mem_mask);
}

u16 lisa_mmu_device::mmu::read(offs_t addr, u16 mem_mask)
{
	u64 cyc = m_maincpu->total_cycles();
	if(cyc & 3)
		if(m_maincpu->access_before_time((cyc|3) + 1, cyc))
			return 0;

	int slot = m_current_base | ((addr & 0xffffff) >> 17);
 retry:
	offs_t newaddr = m_base_address[slot] + (addr & 0x01ffff);
	switch(m_mode[slot]) {
	case INVALID:
	default:
		m_maincpu->trigger_bus_error();
		return 0;

	case MAIN_RO: case MAIN_RW:
		if(newaddr >= m_limit_address[slot]) {
			m_maincpu->trigger_bus_error();
			return 0;
		}
		m_dev.ram_access(newaddr, mem_mask, false);
		return m_ram.read_interruptible(newaddr, mem_mask);

	case MAIN_RO_STACK: case MAIN_RW_STACK:
		if(newaddr < m_limit_address[slot]) {
			m_maincpu->trigger_bus_error();
			return 0;
		}
		m_dev.ram_access(newaddr, mem_mask, false);
		return m_ram.read_interruptible(newaddr, mem_mask);

	case IO:
		if(newaddr >= m_limit_address[slot]) {
			m_maincpu->trigger_bus_error();
			return 0;
		}
		return m_io.read_interruptible(newaddr, mem_mask);

	case SPECIAL:
		if(slot >= 4*128) {
			if(addr & 0x4000) {
				slot &= 0x7f;
				goto retry;
			}
		} else if(newaddr >= m_limit_address[slot]) {
			m_maincpu->trigger_bus_error();
			return 0;
		}
		return m_special_io.read_interruptible(newaddr, mem_mask);
	}
}

void lisa_mmu_device::mmu::write(offs_t addr, u16 data, u16 mem_mask)
{
	u64 cyc = m_maincpu->total_cycles();
	if(cyc & 3)
		if(m_maincpu->access_before_time((cyc|3) + 1, cyc))
			return;

	write_access(addr, data, mem_mask);
	m_dev.write_cycle();
}

void lisa_mmu_device::mmu::write_access(offs_t addr, u16 data, u16 mem_mask)
{
	int slot = m_current_base | ((addr & 0xffffff) >> 17);
 retry:
	offs_t newaddr = m_base_address[slot] + (addr & 0x01ffff);
	switch(m_mode[slot]) {
	case INVALID: case MAIN_RO: case MAIN_RO_STACK:
	default:
		m_maincpu->trigger_bus_error();
		return;

	case MAIN_RW:
		if(newaddr >= m_limit_address[slot]) {
			m_maincpu->trigger_bus_error();
			return;
		}
		m_ram.write_interruptible(newaddr, data, mem_mask);
		m_dev.ram_access(newaddr, mem_mask, true);
		break;

	case MAIN_RW_STACK:
		if(newaddr < m_limit_address[slot]) {
			m_maincpu->trigger_bus_error();
			return;
		}
		m_ram.write_interruptible(newaddr, data, mem_mask);
		m_dev.ram_access(newaddr, mem_mask, true);
		break;

	case IO:
		if(newaddr >= m_limit_address[slot]) {
			m_maincpu->trigger_bus_error();
			return;
		}
		m_io.write_interruptible(newaddr, data, mem_mask);
		break;

	case SPECIAL:
		if(slot >= 4*128) {
			if(addr & 0x4000) {
				slot &= 0x7f;
				goto retry;
			}
		} else if(newaddr >= m_limit_address[slot]) {
			m_maincpu->trigger_bus_error();
			return;
		}
		m_special_io.write_interruptible(newaddr, data, mem_mask);
		break;
	}
}

u16 lisa_mmu_device::mmu::read_cpu(offs_t addr, u16 mem_mask)
{
	return m_cpu_space.read_interruptible(addr, mem_mask);
}

void lisa_mmu_device::mmu::recompute_slot()
{
	if(m_setup)
		m_current_base = 128*4;
	else if(m_super)
		m_current_base = 0;
	else
		m_current_base = m_seg << 7;
}

void lisa_mmu_device::mmu::set_seg(u8 seg)
{
	m_seg = seg;
	recompute_slot();
}

void lisa_mmu_device::mmu::set_setup(bool setup)
{
	m_setup = setup;
	recompute_slot();
}

void lisa_mmu_device::mmu::set_super(bool super)
{
	m_super = super;
	recompute_slot();
}

bool lisa_mmu_device::mmu::translate(int spacenum, int intention, offs_t &address, address_space *&target_space)
{
	if(spacenum == m68000_device::AS_CPU_SPACE) {
		target_space = &m_a_cpu_space;
		return true;
	}

	if(spacenum != AS_PROGRAM)
		return false;

	int slot = m_current_base | (address >> 17);
	offs_t addr = address;
 retry:
	address = m_base_address[slot] + (addr & 0x01ffff);
	switch(m_mode[slot]) {
	case INVALID:
	default:
		return false;

	case MAIN_RW: case MAIN_RO:
		if(address >= m_limit_address[slot])
			return false;
		target_space = &m_a_ram;
		return true;

	case MAIN_RW_STACK: case MAIN_RO_STACK:
		if(address < m_limit_address[slot])
			return false;
		target_space = &m_a_ram;
		return true;

	case IO:
		if(address >= m_limit_address[slot])
			return false;
		target_space = &m_a_io;
		return true;

	case SPECIAL:
		if(slot >= 4*128) {
			if(addr & 0x4000) {
				slot &= 0x7f;
				goto retry;
			}
		} else if(address >= m_limit_address[slot])
			return false;
		target_space = &m_a_special_io;
		return true;
	}
}

void lisa_mmu_device::device_start()
{
	m_mmu = std::make_unique<mmu>(*this, m_maincpu.target(), space(AS_RAM), space(AS_IO), space(AS_SPECIAL_IO), space(m68000_device::AS_CPU_SPACE));
	m_maincpu->set_current_mmu(m_mmu.get());

	std::fill(m_sor.begin(), m_sor.end(), 0x0000);
	std::fill(m_slr.begin(), m_slr.end(), 0x0000);

	// Parity bits live in the RAM, so they survive a reset
	m_bad_parity.resize(0x200000 / 32, 0);
	m_bad_parity_count = 0;
	m_error_address = 0;

	save_item(NAME(m_sor));
	save_item(NAME(m_slr));
	save_item(NAME(m_bad_parity));
	save_item(NAME(m_bad_parity_count));
	save_item(NAME(m_error_address));
	save_item(NAME(m_diag1));
	save_item(NAME(m_diag2));
	save_item(NAME(m_soft_mask));
	save_item(NAME(m_soft_error));
	save_item(NAME(m_hard_mask));
	save_item(NAME(m_hard_error));

	m_seg = 0;
}

void lisa_mmu_device::device_reset()
{
	m_mmu->set_setup(true);
	m_mmu->set_seg(0);
	m_seg = 0;

	// Reset clears the control latch: DIAG1/DIAG2 off, both error
	// detectors masked (which also clears their latches)
	m_diag1 = m_diag2 = false;
	m_soft_mask = m_hard_mask = true;
	m_soft_error = m_hard_error = false;
	update_nmi();
}

device_memory_interface::space_config_vector lisa_mmu_device::memory_space_config() const
{
	return space_config_vector {
	std::make_pair(AS_RAM, &m_ram_config),
	std::make_pair(AS_IO, &m_io_config),
	std::make_pair(AS_SPECIAL_IO, &m_special_io_config),
	std::make_pair(m68000_device::AS_CPU_SPACE, &m_cpu_space_config),
	};
}

void lisa_mmu_device::sor_w(offs_t address, u16 data, u16 mem_mask)
{
	int slot = (m_seg << 7) | (address >> 16);
	COMBINE_DATA(m_sor.data() + slot);
	m_mmu->setup_entry(slot, m_sor[slot], m_slr[slot]);
}

u16 lisa_mmu_device::sor_r(offs_t address)
{
	int slot = (m_seg << 7) | (address >> 16);
	return (m_sor[slot] & 0xfff) | (m_video->sn_r() ? 0x8000 : 0);
}

void lisa_mmu_device::slr_w(offs_t address, u16 data, u16 mem_mask)
{
	int slot = (m_seg << 7) | (address >> 16);
	COMBINE_DATA(m_slr.data() + slot);
	m_mmu->setup_entry(slot, m_sor[slot], m_slr[slot]);
}

u16 lisa_mmu_device::slr_r(offs_t address)
{
	int slot = (m_seg << 7) | (address >> 16);
	return (m_slr[slot] & 0xfff) | (m_video->sn_r() ? 0x8000 : 0);
}

u16 lisa_mmu_device::status_r()
{
	// Bits 0 and 1 are the soft and hard error latches, active low.
	// Bit 3 is the bus timeout latch (BUST/, active low): set when AS/
	// stays asserted for ~56 us with no device answering, cleared by
	// reading F000.  Every access completes here, so it always reads 1.
	return m_video->status_r() | (m_soft_error ? 0x00 : 0x01) | (m_hard_error ? 0x00 : 0x02) | 0x08;
}

void lisa_mmu_device::setup_1()
{
	m_mmu->set_setup(true);
}

void lisa_mmu_device::setup_0()
{
	m_mmu->set_setup(false);
}

void lisa_mmu_device::seg1_1()
{
	m_seg |= 1;
	m_mmu->set_seg(m_seg);
}

void lisa_mmu_device::seg1_0()
{
	m_seg &= ~1;
	m_mmu->set_seg(m_seg);
}

void lisa_mmu_device::seg2_1()
{
	m_seg |= 2;
	m_mmu->set_seg(m_seg);
}

void lisa_mmu_device::seg2_0()
{
	m_seg &= ~2;
	m_mmu->set_seg(m_seg);
}

// Error logic (CPU schematic 050-4009 sheets 3 and 5).  DIAG1 and
// DIAG2 pull SFER/ and HDER/ low on every CPU write cycle, memory or
// I/O.  A memory board also pulls HDER/ low when it reads a byte with
// bad parity, and it stores bad parity for every byte written while
// DIAG2 is set.  SFER/ and HDER/ set the LS279 soft and hard error
// latches unless their mask bit holds them clear, and either latch
// raises the level 7 interrupt.

void lisa_mmu_device::diag1_0()
{
	m_diag1 = false;
}

void lisa_mmu_device::diag1_1()
{
	m_diag1 = true;
}

void lisa_mmu_device::diag2_0()
{
	m_diag2 = false;
}

void lisa_mmu_device::diag2_1()
{
	m_diag2 = true;
}

// SFMSK/ asserted: soft error detection off, latch cleared
void lisa_mmu_device::serr_1()
{
	m_soft_mask = true;
	if(m_soft_error) {
		m_soft_error = false;
		update_nmi();
	}
}

void lisa_mmu_device::serr_0()
{
	m_soft_mask = false;
}

// HDMSK/ asserted: hard error detection off, latch cleared
void lisa_mmu_device::herr_1()
{
	m_hard_mask = true;
	if(m_hard_error) {
		m_hard_error = false;
		update_nmi();
	}
}

void lisa_mmu_device::herr_0()
{
	m_hard_mask = false;
}

u16 lisa_mmu_device::parity_error_address_r()
{
	// Physical A20-A6 in bits 15-1.  Bit 0 flags a video cycle; video
	// fetches aren't seen here, so it reads 0.
	return (m_error_address >> 5) & 0xfffe;
}

void lisa_mmu_device::update_nmi()
{
	m_nmi_cb(m_soft_error || m_hard_error);
}

void lisa_mmu_device::latch_hard_error()
{
	if(!m_hard_mask && !m_hard_error) {
		m_hard_error = true;
		update_nmi();
	}
}

void lisa_mmu_device::write_cycle()
{
	if(m_diag1 && !m_soft_mask && !m_soft_error) {
		m_soft_error = true;
		update_nmi();
	}
	if(m_diag2)
		latch_hard_error();
}

void lisa_mmu_device::ram_access(offs_t address, u16 mem_mask, bool write)
{
	// The error address register (F000) takes the address of every
	// memory cycle until an error is latched, then holds it
	address &= 0x1ffffe;
	if(!m_soft_error && !m_hard_error)
		m_error_address = address;

	if(!m_diag2 && !m_bad_parity_count)
		return;

	// One parity bit per byte: UDS selects the even byte, LDS the odd one
	for(int i = 0; i != 2; i++) {
		if(!(mem_mask & (i ? 0x00ff : 0xff00)))
			continue;
		const u32 byte = address | i;
		u32 &bits = m_bad_parity[byte >> 5];
		const u32 bit = 1U << (byte & 31);
		if(!write) {
			if(bits & bit)
				latch_hard_error();
		} else if(m_diag2) {
			if(!(bits & bit)) {
				bits |= bit;
				m_bad_parity_count++;
			}
		} else if(bits & bit) {
			bits &= ~bit;
			m_bad_parity_count--;
		}
	}
}

void lisa_mmu_device::default_autovectors_map(address_map &map)
{
	offs_t mask = 0xffffff - 0xf;
	map(mask + 0x3, mask + 0x3)/*.before_time(*this, FUNC(m68000_device::vpa_sync)).after_delay(*this, FUNC(m68000_device::vpa_after))*/.lr8(NAME([] () -> u8 { return m68000_device::autovector(1); }));
	map(mask + 0x5, mask + 0x5)/*.before_time(*this, FUNC(m68000_device::vpa_sync)).after_delay(*this, FUNC(m68000_device::vpa_after))*/.lr8(NAME([] () -> u8 { return m68000_device::autovector(2); }));
	map(mask + 0x7, mask + 0x7)/*.before_time(*this, FUNC(m68000_device::vpa_sync)).after_delay(*this, FUNC(m68000_device::vpa_after))*/.lr8(NAME([] () -> u8 { return m68000_device::autovector(3); }));
	map(mask + 0x9, mask + 0x9)/*.before_time(*this, FUNC(m68000_device::vpa_sync)).after_delay(*this, FUNC(m68000_device::vpa_after))*/.lr8(NAME([] () -> u8 { return m68000_device::autovector(4); }));
	map(mask + 0xb, mask + 0xb)/*.before_time(*this, FUNC(m68000_device::vpa_sync)).after_delay(*this, FUNC(m68000_device::vpa_after))*/.lr8(NAME([] () -> u8 { return m68000_device::autovector(5); }));
	map(mask + 0xd, mask + 0xd)/*.before_time(*this, FUNC(m68000_device::vpa_sync)).after_delay(*this, FUNC(m68000_device::vpa_after))*/.lr8(NAME([] () -> u8 { return m68000_device::autovector(6); }));
	map(mask + 0xf, mask + 0xf)/*.before_time(*this, FUNC(m68000_device::vpa_sync)).after_delay(*this, FUNC(m68000_device::vpa_after))*/.lr8(NAME([] () -> u8 { return m68000_device::autovector(7); }));
}
