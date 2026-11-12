// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT100 Standard Terminal Port (STP) connector

***************************************************************************/

#ifndef MAME_DEC_VT100_STP_H
#define MAME_DEC_VT100_STP_H

#pragma once


class device_vt100_stp_interface;

// The STP (J3 on the terminal controller) sits in series with the
// communication port: 20 pairs of shorting contacts, even pins on the
// terminal controller ("TC") side, odd pins on the line side (TM-003 7.5.3).
// With nothing plugged in each pair is closed; a card's finger opens every
// pair, and the card joins through those it doesn't use.  Signals are
// numbered by pin.  A pin that a signal enters the connector on is written
// with write<PIN>(); a pin that one leaves on is bound with pin_handler<PIN>().
class vt100_stp_slot_device : public device_t, public device_single_card_slot_interface<device_vt100_stp_interface>
{
public:
	enum : unsigned
	{
		LINE_TXD = 1,       // out to the EIA connector
		TC_TXD = 2,         // in from the TC 8251
		EXT_TCLK = 3,       // in from the EIA connector (modem timing), option only
		LINE_RXD = 5,       // in from the EIA connector
		TC_RXD = 6,         // out to the TC 8251
		LINE_RTS = 7,
		TC_RTS = 8,
		EXT_RCLK = 9,       // in from the EIA connector (modem timing), option only
		LINE_CTS = 11,
		TC_CTS = 12,        // to the modem buffer only; the 8251's CTS is grounded
		LINE_DSR = 13,
		TC_DSR = 14,
		LINE_DTR = 15,
		TC_DTR = 16,
		LINE_CD = 17,
		TC_CD = 18,
		LINE_RI = 19,
		TC_RI = 20,
		LINE_SPDS = 21,     // speed select (secondary RTS)
		TC_SPDS = 22,
		LINE_SPDI = 23,     // speed indicator (secondary CD)
		TC_SPDI = 24,
		INTERNAL_TCLK = 27, // out to the TC 8251's TXC
		TC_TCLK = 28,       // in from the TC baud rate generator
		INTERNAL_RCLK = 29, // out to the TC 8251's RXC
		TC_RCLK = 30,
		LOCAL_CLK = 36,     // in from the video timing (16 x 15,734 baud), option only
		TC_INIT = 38        // in from the TC, high while it initialises, option only
	};

	template <typename T>
	vt100_stp_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, const char *dflt)
		: vt100_stp_slot_device(mconfig, tag, owner, 0)
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
	}

	vt100_stp_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// a signal entering the connector on PIN, from the terminal or the line
	template <unsigned Pin> void write(int state);

	// a signal leaving the connector on PIN
	template <unsigned Pin> auto pin_handler() { return m_out[Pin].bind(); }

	// from the card: drive PIN, which must be one that signals leave on
	void drive(unsigned pin, int state) { m_out[pin](state); }

	// from the card: join PIN to its partner, as the empty connector does
	void join(unsigned pin, int state);

	// OPTION PRESENT H (pin 34): pulled up on the TC, grounded through
	// pin 33 by the closed pair, so high whenever a card is in
	bool present() const { return m_card != nullptr; }

protected:
	virtual void device_resolve_objects() override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;

private:
	devcb_write_line::array<41> m_out;
	device_vt100_stp_interface *m_card;
};


class device_vt100_stp_interface : public device_interface
{
public:
	// A signal entered the connector on PIN.  By default the card joins
	// each pair through; a card overrides this for the pins it uses.
	virtual void stp_w(unsigned pin, int state) { slot().join(pin, state); }

protected:
	device_vt100_stp_interface(const machine_config &mconfig, device_t &device);

	vt100_stp_slot_device &slot() const { return *m_slot; }

	// for binding a card's outputs to the pins signals leave on
	template <unsigned Pin> void drive_w(int state) { m_slot->drive(Pin, state); }

private:
	vt100_stp_slot_device *const m_slot;
};


template <unsigned Pin>
void vt100_stp_slot_device::write(int state)
{
	if (m_card)
		m_card->stp_w(Pin, state);
	else
		join(Pin, state);
}


DECLARE_DEVICE_TYPE(VT100_STP_SLOT, vt100_stp_slot_device)
DECLARE_DEVICE_TYPE(VT100_STP_AC, device_vt100_stp_interface)

void vt100_stp_options(device_slot_interface &device);

#endif // MAME_DEC_VT100_STP_H
