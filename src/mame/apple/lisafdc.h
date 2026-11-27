// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

// The LISA FDC subsystem

#ifndef MAME_APPLE_LISAFDC_H
#define MAME_APPLE_LISAFDC_H

#pragma once

#include "cpu/m6502/m6504.h"
#include "machine/iwm.h"
#include "machine/nvram.h"

class lisa_base_fdc_device : public device_t {
public:
	u8 ram_r(offs_t offset);
	void ram_w(offs_t offset, u8 data);
	bool dis() const { return m_dis; }

	auto write_diag() { return m_diag_cb.bind(); }
	auto write_fdir() { return m_fdir_cb.bind(); }

protected:
	required_device<m6504_device> m_cpu;
	required_device<nvram_device> m_nvram;
	required_shared_ptr<u8> m_ram;
	required_device_array<floppy_connector, 2> m_floppy;
	devcb_write_line m_diag_cb;
	devcb_write_line m_fdir_cb;
	floppy_image_device *m_cur_floppy;
	u8 m_hds, m_sel, m_phases;
	bool m_dis;
	u8 m_pwm;

	lisa_base_fdc_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock = 0);

	void device_start() override ATTR_COLD;
	void device_reset() override ATTR_COLD;

	void mt0_0();
	void mt0_1();
	void mt1_0();
	void mt1_1();
	void dis_0();
	void dis_1();
	void hds_0();
	void hds_1();
	void diag_0();
	void diag_1();
	void fdir_0();
	void fdir_1();

	virtual void update_sel();
	void update_phases();
	void update_pwm();
};

class lisa_original_fdc_device : public lisa_base_fdc_device {
protected:
	lisa_original_fdc_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock = 0);
	void device_start() override ATTR_COLD;
	void device_reset() override ATTR_COLD;
	void device_add_mconfig(machine_config &config) override ATTR_COLD;
	void map(address_map &map) ATTR_COLD;

	// Disk state machine: 256x8 PROM, state latch and LS323 shift register
	required_region_ptr<u8> m_lss_rom;
	u64 m_lss_cycles;
	u8 m_lss_address;
	u8 m_lss_data;
	u8 m_lss_load_data;
	bool m_lss_write_line;
	bool m_lss_clock_sel;
	u8 m_speed_shift;
	u8 m_speed_bits;

	u8 latch_r(offs_t offset);
	void latch_w(offs_t offset, u8 data);
	void latch_control(offs_t offset);
	void lss_sync(u64 extra_cycles = 0);
	u64 time_to_lss_cycles(const attotime &tm) const;
	attotime lss_cycles_to_time(u64 cycles) const;
	virtual void update_sel() override;
	void speed_clock();
	virtual void speed_w(u8 data) { }

	void dr1_0();
	void dr1_1();
	void dr2_0();
	void dr2_1();
};

class lisa_fdc_device : public lisa_original_fdc_device {
public:
	lisa_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

protected:
	void device_add_mconfig(machine_config &config) override ATTR_COLD;
	const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
};

class lisa2_fdc_device : public lisa_original_fdc_device {
public:
	lisa2_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

protected:
	void device_add_mconfig(machine_config &config) override ATTR_COLD;
	const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void update_sel() override;
	virtual void speed_w(u8 data) override;
};

class macxl_fdc_device : public lisa_base_fdc_device {
public:
	macxl_fdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	void pwm_w(u8 data);

protected:
	required_device<iwm_device> m_iwm;

	void device_start() override ATTR_COLD;
	void device_reset() override ATTR_COLD;
	void device_add_mconfig(machine_config &config) override ATTR_COLD;
	const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	void map(address_map &map) ATTR_COLD;

	void stop_0();
	void stop_1();

	void devsel_w(u8 devsel);
	void phases_w(uint8_t phases);

	virtual void update_sel() override;
};

DECLARE_DEVICE_TYPE(LISAFDC,  lisa_fdc_device)
DECLARE_DEVICE_TYPE(LISA2FDC, lisa2_fdc_device)
DECLARE_DEVICE_TYPE(MACXLFDC, macxl_fdc_device)

#endif
