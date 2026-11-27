// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

// The LISA MMU
#ifndef MAME_APPLE_LISAMMU_H
#define MAME_APPLE_LISAMMU_H

#pragma once

#include "lisavideo.h"

#include "cpu/m68000/m68000.h"


class lisa_mmu_device : public device_t, public device_memory_interface {
public:
	enum {
		AS_RAM = AS_PROGRAM,
		AS_SPECIAL_IO = AS_DATA
	};

	// Address map flag for devices that answer with VPA/ instead of
	// DTACK/: the 68000 then runs a 6800-style cycle synchronised to E
	enum : u16 {
		VPA = 1
	};

	lisa_mmu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	template <typename T> void set_cpu(T &&tag) { m_maincpu.set_tag(std::forward<T>(tag)); }
	template <typename T> void set_video(T &&tag) { m_video.set_tag(std::forward<T>(tag)); }

	auto write_nmi() { return m_nmi_cb.bind(); }

	u16 sor_r(offs_t adr);
	void sor_w(offs_t adr, u16 data, u16 mem_mask);
	u16 slr_r(offs_t adr);
	void slr_w(offs_t adr, u16 data, u16 mem_mask);
	u16 status_r();

	u16 parity_error_address_r();
	void bus_timeout();

	void setup_1();
	void setup_0();
	void seg1_1();
	void seg1_0();
	void seg2_1();
	void seg2_0();
	void diag1_0();
	void diag1_1();
	void diag2_0();
	void diag2_1();
	void serr_0();
	void serr_1();
	void herr_0();
	void herr_1();

protected:
	void device_start() override ATTR_COLD;
	void device_reset() override ATTR_COLD;
	space_config_vector memory_space_config() const override ATTR_COLD;

private:
	enum : u8 {
		INVALID,
		MAIN_RO_STACK,
		MAIN_RO,
		MAIN_RW_STACK,
		MAIN_RW,
		IO,
		SPECIAL
	};

	static const u8 mode_table[16];

	class mmu : public m68000_device::mmu {
	public:
		mmu(lisa_mmu_device &dev, m68000_device *maincpu, address_space &ram, address_space &io, address_space &special_io, address_space &cpu_space);
		virtual ~mmu() = default;

		u16 read_program(offs_t addr, u16 mem_mask) override;
		void write_program(offs_t addr, u16 data, u16 mem_mask) override;
		u16 read_data(offs_t addr, u16 mem_mask) override;
		void write_data(offs_t addr, u16 data, u16 mem_mask) override;
		u16 read_cpu(offs_t addr, u16 mem_mask) override;
		void set_super(bool super) override;
		bool translate(int spacenum, int intention, offs_t &address, address_space *&target_space) override;

		void set_seg(u8 seg);
		void set_setup(bool setup);

		void setup_entry(int entry, u16 sor, u16 slr);

	private:
		lisa_mmu_device &m_dev;
		m68000_device *m_maincpu;
		address_space &m_a_ram, &m_a_io, &m_a_special_io, &m_a_cpu_space;
		memory_access<24, 1, 0, ENDIANNESS_BIG>::specific m_ram, m_io, m_special_io, m_cpu_space;

		std::array<u32, 128*(4+1)> m_base_address;
		std::array<u32, 128*(4+1)> m_limit_address;
		std::array<u8, 128*(4+1)> m_mode;

		u32 m_current_base;
		u8 m_seg;
		bool m_setup, m_super;

		inline u16 read(offs_t addr, u16 mem_mask);
		inline void write(offs_t addr, u16 data, u16 mem_mask);
		inline void write_access(offs_t addr, u16 data, u16 mem_mask);
		void recompute_slot();
	};

	devcb_write_line m_nmi_cb;

	required_device<m68000_device> m_maincpu;
	required_device<lisa_video_device> m_video;
	std::unique_ptr<mmu> m_mmu;

	address_space_config m_ram_config, m_io_config, m_special_io_config, m_cpu_space_config;

	std::array<u16, 128*4> m_sor, m_slr;

	std::vector<u32> m_bad_parity; // one bit per RAM byte written with DIAG2 set
	u32 m_bad_parity_count;
	emu_timer *m_video_parity_timer;
	u16 m_error_address;
	bool m_bus_timeout;
	u64 m_vpa_time;
	u8 m_seg;
	bool m_diag1, m_diag2;
	bool m_soft_mask, m_soft_error, m_hard_mask, m_hard_error;

	void default_autovectors_map(address_map &map);
	void ram_access(offs_t address, u16 mem_mask, bool write);
	void write_cycle(bool diag1, bool diag2);
	bool vpa_wait(offs_t address);
	u16 video_error_address() const;
	void latch_soft_error(u16 error_address);
	void latch_hard_error(u16 error_address);
	void update_nmi();
	TIMER_CALLBACK_MEMBER(video_parity_check);
};

DECLARE_DEVICE_TYPE(LISAMMU, lisa_mmu_device)

#endif
