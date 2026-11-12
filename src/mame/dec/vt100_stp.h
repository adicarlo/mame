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
// terminal's communication port.  With nothing plugged in its contact pairs
// short together, so the terminal's 8251 talks straight to the host; an
// option breaks the pairs it wants to control.  The terminal controller
// ("TC") side and the line side of each pair are separate callbacks here.
class vt100_stp_slot_device : public device_t, public device_single_card_slot_interface<device_vt100_stp_interface>
{
public:
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

	// to the terminal controller
	auto tc_rxd_handler() { return m_tc_rxd.bind(); }
	auto tc_dsr_handler() { return m_tc_dsr.bind(); }
	auto int_tclk_handler() { return m_int_tclk.bind(); }   // clocks for the TC's own 8251
	auto int_rclk_handler() { return m_int_rclk.bind(); }

	// to the communication line
	auto line_txd_handler() { return m_line_txd.bind(); }
	auto line_dtr_handler() { return m_line_dtr.bind(); }

	// from the terminal controller
	void tc_txd_w(int state);
	void tc_dtr_w(int state);
	void tc_tclk_w(int state);   // the TC baud rate generator
	void tc_rclk_w(int state);
	void local_clk_w(int state); // 16 x 15,734 baud, from the video timing

	// from the communication line
	void line_rxd_w(int state);
	void line_dsr_w(int state);

	// from the option, to the terminal controller and the line
	void opt_tc_rxd_w(int state) { m_tc_rxd(state); }
	void opt_tc_dsr_w(int state) { m_tc_dsr(state); }
	void opt_int_tclk_w(int state) { m_int_tclk(state); }
	void opt_int_rclk_w(int state) { m_int_rclk(state); }
	void opt_line_txd_w(int state) { m_line_txd(state); }
	void opt_line_dtr_w(int state) { m_line_dtr(state); }

	// OPTION PRESENT H: the empty connector shorts it to ground
	bool present() const { return m_card != nullptr; }

protected:
	virtual void device_resolve_objects() override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;

private:
	devcb_write_line m_tc_rxd;
	devcb_write_line m_tc_dsr;
	devcb_write_line m_int_tclk;
	devcb_write_line m_int_rclk;
	devcb_write_line m_line_txd;
	devcb_write_line m_line_dtr;

	device_vt100_stp_interface *m_card;
};


class device_vt100_stp_interface : public device_interface
{
public:
	virtual void tc_txd_w(int state) = 0;
	virtual void tc_dtr_w(int state) = 0;
	virtual void tc_tclk_w(int state) = 0;
	virtual void tc_rclk_w(int state) = 0;
	virtual void local_clk_w(int state) = 0;
	virtual void line_rxd_w(int state) = 0;
	virtual void line_dsr_w(int state) = 0;

protected:
	device_vt100_stp_interface(const machine_config &mconfig, device_t &device);

	vt100_stp_slot_device &slot() const { return *m_slot; }

private:
	vt100_stp_slot_device *const m_slot;
};


DECLARE_DEVICE_TYPE(VT100_STP_SLOT, vt100_stp_slot_device)
DECLARE_DEVICE_TYPE(VT100_STP_AC, device_vt100_stp_interface)

void vt100_stp_options(device_slot_interface &device);

#endif // MAME_DEC_VT100_STP_H
