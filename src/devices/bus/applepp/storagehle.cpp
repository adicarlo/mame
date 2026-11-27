// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, Adam Di Carlo
/***************************************************************************

  HLE emulation of Profile & Widget

  The protocol follows Apple's "ProFile Communications Protocol"
  (bitsavers apple/disk/profile/Profile_Communication_Protocol.pdf).
  Line levels are as the Lisa sees them through its non-inverting
  buffers (I/O board 050-4008 sheet 3): CMD/, PSTRB/ and CRES/ are
  active low, BSY low means the drive has a response or is working,
  OCD low means a drive is attached.

  Every exchange starts with a handshake: the host asserts CMD/, the
  drive puts a response byte on the bus and asserts BSY, the host
  answers $55 and releases CMD/, and the drive releases BSY once it is
  ready for the next step.  The responses are $01 (send the command
  bytes), $02 (read done, send status and block), $03, $04 or $05 (send
  the block to write, write and verify, or write and spare) and $06 (do
  the write).  Bytes move one per PSTRB/ pulse.  Details follow the 5 MB
  firmware 341-0080-B (disassembly by Patrick Schaefer, bitsavers
  apple/disk/profile/firmware) where the 10 MB one doesn't differ.

  Images are hard disk images with 532-byte sectors: the 20 tag bytes,
  then the 512 data bytes, as they go over the wire.  A 5 MB ProFile has
  9728 blocks: chdman createhd -o profile.chd -chs 152,4,16 -ss 532

  Not modelled: the power-on spin-up and scan (about 70 s with BSY
  held), seek, rotation and interleave timing (one fixed access time),
  sparing and the retry count, the spare table update on the first
  command, the separate read and write buffers behind $FFFFFE.

***************************************************************************/

#include "emu.h"
#include "storagehle.h"

#include <bit>

DEFINE_DEVICE_TYPE(APPLEPP_PROFILE, applepp_profile_device, "profilehle", "Apple Profile drive HLE")
DEFINE_DEVICE_TYPE(APPLEPP_WIDGET,  applepp_widget_device,  "widgethle",  "Apple Widget drive HLE")

namespace {

// Inferred: the drive's Z8 answers the handshake in a few tens of
// microseconds, and a block access costs about half a revolution at
// 3600 rpm plus a short seek
const attotime RESPOND_TIME = attotime::from_usec(20);
const attotime ACCESS_TIME = attotime::from_msec(10);

}

applepp_storage_device::applepp_storage_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock),
	  device_applepp_interface(mconfig, *this),
	  m_image(*this, "image")
{
}

void applepp_storage_device::device_add_mconfig(machine_config &config)
{
	HARDDISK(config, m_image).set_device_load(FUNC(applepp_storage_device::image_load));
	m_image->set_device_unload(FUNC(applepp_storage_device::image_unload));
}

// Without an image there is no drive on the cable: OCD floats high
std::error_condition applepp_storage_device::image_load(device_image_interface &image)
{
	if(m_image->get_info().sectorbytes != BLOCK_SIZE)
		return image_error::INVALIDIMAGE;
	m_connector->pchk_set(0);
	return std::error_condition();
}

void applepp_storage_device::image_unload(device_image_interface &image)
{
	m_connector->pchk_set(1);
}

void applepp_storage_device::device_start()
{
	m_respond_timer = timer_alloc(FUNC(applepp_storage_device::respond), this);
	m_ready_timer = timer_alloc(FUNC(applepp_storage_device::ready), this);

	save_item(NAME(m_strb));
	save_item(NAME(m_rw));
	save_item(NAME(m_cmd));
	save_item(NAME(m_res));
	save_item(NAME(m_state));
	save_item(NAME(m_pd));
	save_item(NAME(m_out));
	save_item(NAME(m_response));
	save_item(NAME(m_nak));
	save_item(NAME(m_reset_status));
	save_item(NAME(m_index));
	save_item(NAME(m_count));
	save_item(NAME(m_command));
	save_item(NAME(m_buffer));

	m_strb = m_rw = m_cmd = m_res = 1;
	m_pd = 0;
	m_out = 0xff;
	std::fill(std::begin(m_buffer), std::end(m_buffer), 0);
}

void applepp_storage_device::device_reset()
{
	m_connector->pchk_set(m_image->exists() ? 0 : 1);
	m_connector->pbsy_set(1);
	present(0xff);

	m_respond_timer->adjust(attotime::never);
	m_ready_timer->adjust(attotime::never);
	m_state = IDLE;
	m_response = 0;
	m_nak = false;
	m_reset_status = false;
	m_index = m_count = 0;
	std::fill(std::begin(m_command), std::end(m_command), 0);
}

// Inferred: the drive drives PARITY/ for the bytes the host puts on the
// bus too (DR/W low), since the Lisa OS checks the parity latch after it
// writes
void applepp_storage_device::pd_w(u8 data)
{
	m_pd = data;
	if(!m_rw)
		m_connector->pparity_set((std::popcount(data) + 1) & 1);
}

void applepp_storage_device::prw_w(int level)
{
	if(m_rw == level)
		return;
	m_rw = level;

	// Back to driving the bus: the byte and its parity
	if(level)
		m_connector->pd_set(m_out);
}

void applepp_storage_device::present(u8 data)
{
	m_out = data;
	m_connector->pd_set(data);
}

void applepp_storage_device::pstrb_w(int level)
{
	if(m_strb == level)
		return;
	m_strb = level;

	// Bytes move at the end of the strobe: the host has written the
	// byte, or read the one on the bus
	if(!level || !m_res)
		return;

	switch(m_state) {
	case RECV_CMD:
		// Extra bytes are ignored
		if(m_index < std::size(m_command))
			m_command[m_index++] = m_pd;
		break;

	case RECV_DATA:
		// The firmware spots an overlong write by the fence bytes after
		// the buffer, which the 534th byte reaches
		if(m_index < BLOCK_SIZE)
			m_buffer[STATUS_SIZE + m_index] = m_pd;
		if(m_index < BLOCK_SIZE + 2)
			m_index++;
		break;

	case SEND_DATA:
		if(m_index + 1 < m_count)
			present(m_buffer[++m_index]);
		break;
	}
}

void applepp_storage_device::pcmd_w(int level)
{
	if(m_cmd == level)
		return;
	m_cmd = level;

	if(!m_res)
		return;

	if(!level) {
		// A drive still busy answers once it's done
		if(!m_ready_timer->enabled())
			start_handshake();

	} else if(m_state == HANDSHAKE) {
		m_respond_timer->adjust(attotime::never);
		if(m_pd == 0x55)
			accept_response();
		else {
			// Anything but $55 is a NAK: back to idle, which also drops
			// the reset bit (SendState clears status byte 3)
			m_nak = true;
			m_reset_status = false;
			m_state = IDLE;
			m_ready_timer->adjust(RESPOND_TIME);
		}
	}
}

void applepp_storage_device::start_handshake()
{
	// The response depends on where the host is in the exchange
	if(m_state == RECV_CMD) {
		switch(m_command[0]) {
		case 0x00: m_response = 0x02; break; // read
		case 0x01: m_response = 0x03; break; // write
		case 0x02: m_response = 0x04; break; // write and verify
		case 0x03: m_response = 0x05; break; // write and spare the block
		default:
			// An unknown command starts over (WarmStart2)
			logerror("unknown command %02x\n", m_command[0]);
			m_response = 0x01;
			break;
		}
	} else if(m_state == RECV_DATA)
		m_response = 0x06;
	else
		m_response = 0x01;

	m_state = HANDSHAKE;
	m_respond_timer->adjust(RESPOND_TIME);
}

void applepp_storage_device::pres_w(int level)
{
	if(m_res == level)
		return;
	m_res = level;

	// A reset after power-on is a warm start: the drive skips spin-up
	// and its scan, and reports the reset in the next status
	if(!level) {
		m_respond_timer->adjust(attotime::never);
		m_ready_timer->adjust(attotime::never);
		m_state = IDLE;
		m_nak = false;
		m_reset_status = true;
		m_connector->pbsy_set(1);
	}
}

TIMER_CALLBACK_MEMBER(applepp_storage_device::respond)
{
	present(m_response);
	m_connector->pbsy_set(0);
}

TIMER_CALLBACK_MEMBER(applepp_storage_device::ready)
{
	if(m_state == SEND_DATA)
		present(m_buffer[0]);
	m_connector->pbsy_set(1);
	if(!m_cmd && m_res)
		start_handshake();
}

void applepp_storage_device::accept_response()
{
	const u32 block = (m_command[1] << 16) | (m_command[2] << 8) | m_command[3];

	switch(m_response) {
	case 0x01:
		// A short command leaves the old bytes in place
		m_state = RECV_CMD;
		m_index = 0;
		m_ready_timer->adjust(RESPOND_TIME);
		break;

	case 0x02:
		// Accepting the command clears an earlier NAK
		m_nak = false;
		read_block(block);
		m_state = SEND_DATA;
		m_index = 0;
		m_count = STATUS_SIZE + BLOCK_SIZE;
		m_ready_timer->adjust(ACCESS_TIME);
		break;

	case 0x03:
	case 0x04:
	case 0x05:
		m_nak = false;
		m_index = 0;
		if(!tenmeg() && block != 0xfffffe && block >= blocks()) {
			// The 5 MB firmware drops a write to a bad block before the
			// data stage and just reports it
			set_status(0x00, 0x00, block == 0xffffff ? 0x00 : 0x40);
			m_state = SEND_DATA;
			m_count = STATUS_SIZE;
		} else
			m_state = RECV_DATA;
		// The drive seeks before it takes the data
		m_ready_timer->adjust(block == 0xfffffe ? RESPOND_TIME : ACCESS_TIME);
		break;

	case 0x06:
		write_block(block);
		m_state = SEND_DATA;
		m_index = 0;
		m_count = STATUS_SIZE;
		m_ready_timer->adjust(ACCESS_TIME);
		break;
	}
}

// Status byte 1: bit 7 NAK received, bit 6 write aborted, bit 5 spare
// table updated, bit 0 failed.  Byte 3: bit 7 drive reset, bit 6 invalid
// block number.  Byte 4 counts retries.  (DefsProFile5RW.inc)
void applepp_storage_device::set_status(u8 s1, u8 s2, u8 s3)
{
	m_buffer[0] = s1 | (m_nak ? 0x80 : 0x00);
	m_buffer[1] = s2;
	m_buffer[2] = s3 | (m_reset_status ? 0x80 : 0x00);
	m_buffer[3] = 0;
	m_nak = false;
	m_reset_status = false;
}

u32 applepp_storage_device::blocks() const
{
	if(!m_image->exists())
		return 0;
	const auto &info = m_image->get_info();
	return info.cylinders * info.heads * info.sectors;
}

bool applepp_storage_device::tenmeg() const
{
	return blocks() > 9728;
}

void applepp_storage_device::read_block(u32 block)
{
	u8 *const data = m_buffer + STATUS_SIZE;

	// $FFFFFF is the spare table, $FFFFFE the drive's buffer as it is
	if(block == 0xffffff) {
		build_spare_table(data);
		set_status(0x00, 0x00, 0x00);
	} else if(block == 0xfffffe)
		set_status(0x00, 0x00, 0x00);
	else if(!m_image->exists())
		set_status(0x01, 0x00, 0x00);
	else if(block >= blocks())
		// Only the 10 MB firmware also flags the operation as failed
		set_status(tenmeg() ? 0x01 : 0x00, 0x00, 0x40);
	else if(!m_image->read(block, data))
		set_status(0x01, 0x00, 0x00);
	else
		set_status(0x00, 0x00, 0x00);
}

void applepp_storage_device::write_block(u32 block)
{
	// An overrun also rereads the spare table, hence bit 5
	if(m_index > BLOCK_SIZE + 1)
		set_status(0x61, 0x00, 0x00);
	else if(block == 0xfffffe)
		set_status(0x00, 0x00, 0x00);
	else if(!m_image->exists())
		set_status(0x01, 0x00, 0x00);
	else if(block >= blocks())
		set_status(0x01, 0x00, 0x40);
	else if(!m_image->write(block, m_buffer + STATUS_SIZE))
		set_status(0x01, 0x00, 0x00);
	else
		set_status(0x00, 0x00, 0x00);
}

// The spare table starts at the first byte of the block (Lisa OS
// PROFILEASM reads the block count at offset 18 of the stream).  It
// lives on the disk, written by the formatter: the 5 MB name is padded
// with blanks, the 10 MB one with NULs, and the map area after the
// header is $FF (profile5LLF.asm, profile10LLF).  The firmware writes
// its own revision into it.
void applepp_storage_device::build_spare_table(u8 *dest)
{
	const bool ten = tenmeg();
	const u32 count = blocks();

	std::fill_n(dest, BLOCK_SIZE, 0);
	std::fill_n(dest + 26, 200, 0xff);
	u8 *p = std::copy_n(ten ? "PROFILE 10M\0\0" : "PROFILE      ", 13, dest);
	*p++ = 0x00; // device number
	*p++ = 0x00;
	*p++ = ten ? 0x10 : 0x00;
	*p++ = ten ? 0x04 : 0x03; // firmware revision, 4.04 or 3.98
	*p++ = ten ? 0x04 : 0x98;
	*p++ = count >> 16;
	*p++ = count >> 8;
	*p++ = count;
	*p++ = BLOCK_SIZE >> 8;
	*p++ = BLOCK_SIZE & 0xff;
	*p++ = 0x20; // spares
	*p++ = 0x00; // spares allocated
	*p++ = 0x00; // bad blocks
	// Both lists are empty: their FF FF FF ends are in the $FF fill
}

applepp_profile_device::applepp_profile_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: applepp_storage_device(mconfig, APPLEPP_PROFILE, tag, owner, clock)
{
}

applepp_widget_device::applepp_widget_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: applepp_storage_device(mconfig, APPLEPP_WIDGET, tag, owner, clock)
{
}
