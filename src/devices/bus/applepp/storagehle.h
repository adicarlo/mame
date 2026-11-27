// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, Adam Di Carlo
/***************************************************************************

  HLE emulation of Profile & Widget

***************************************************************************/

#ifndef MAME_BUS_APPLEPP_STORAGEHLE_H
#define MAME_BUS_APPLEPP_STORAGEHLE_H

#pragma once

#include "applepp.h"

#include "imagedev/harddriv.h"

class applepp_storage_device : public device_t, public device_applepp_interface
{
public:
	virtual void pd_w(u8 data) override;

	virtual void pstrb_w(int level) override;
	virtual void prw_w(int level) override;
	virtual void pcmd_w(int level) override;
	virtual void pres_w(int level) override;

protected:
	// A block on the wire: 20 tag bytes, then 512 data bytes
	static constexpr unsigned BLOCK_SIZE = 532;
	static constexpr unsigned STATUS_SIZE = 4;

	enum : u8 {
		IDLE,       // waiting for CMD/
		HANDSHAKE,  // response byte offered, waiting for CMD/ to drop
		RECV_CMD,   // host strobes in the command bytes
		RECV_DATA,  // host strobes in a block to write
		SEND_DATA   // host strobes out the status (and block)
	};

	applepp_storage_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	void device_add_mconfig(machine_config &config) override ATTR_COLD;
	void device_start() override ATTR_COLD;
	void device_reset() override ATTR_COLD;

	TIMER_CALLBACK_MEMBER(respond);
	TIMER_CALLBACK_MEMBER(ready);

	std::error_condition image_load(device_image_interface &image);
	void image_unload(device_image_interface &image);
	void present(u8 data);
	void start_handshake();
	void accept_response();
	void read_block(u32 block);
	void write_block(u32 block);
	void build_spare_table(u8 *dest);
	void set_status(u8 s1, u8 s2, u8 s3);

	required_device<harddisk_image_device> m_image;
	emu_timer *m_respond_timer;
	emu_timer *m_ready_timer;

	int m_strb, m_rw, m_cmd, m_res;
	u8 m_state;
	u8 m_pd;  // the host's byte
	u8 m_out; // the drive's byte
	u8 m_response;
	bool m_nak;
	u16 m_index, m_count;
	u8 m_command[6];
	u8 m_buffer[STATUS_SIZE + BLOCK_SIZE]; // status, then the drive's block buffer
};

class applepp_profile_device : public applepp_storage_device
{
public:
	applepp_profile_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

class applepp_widget_device : public applepp_storage_device
{
public:
	applepp_widget_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


DECLARE_DEVICE_TYPE(APPLEPP_PROFILE, applepp_profile_device)
DECLARE_DEVICE_TYPE(APPLEPP_WIDGET,  applepp_widget_device)

#endif // MAME_BUS_APPLEPP_STORAGEHLE_H
