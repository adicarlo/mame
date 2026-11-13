// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT125 paddle board (54-14275), in the VT100's STP connector

    Three 8251As and a COM8116-003 baud rate generator on the mono board
    8085's I/O bus, reached over a 24-pin cable (vt125_cable.h), plus the
    EIA drivers.  The graphics processor sits between the host and the
    VT100 and passes on everything not meant for it (VT125 User Guide
    EK-VT125-UG-002 ch7, figs 7-5 to 7-8):

      ports  8251  job
      0A/0B  E11   auxiliary (printer) port, the paddle's own connector J3
      0C/0D  E8    computer port, through the STP to the VT100's EIA (or
                   20 mA) connector
      0E/0F  E3    the link to the VT100's own 8251
      08           baud rate generator for E11 only (OUT 08)

    Print set MP01053, paddle sheets 1-2 (pdf 15-16).  All three have CS
    grounded and C/D on A0.  E11's DTR is GRAPHICS FLAG to the VT100
    (through the mono board), E3's is CLK SHIFT.  The COM8116-003 runs from
    the mono board's 6.018 MHz (24.0734 MHz / 4), for which its divisors
    are made: the firmware's aux port speeds E6, E8, EB, EE come out at
    300, 1200, 2400, 9600 baud receive, 9600 transmit.

    Inferred, not traced (the E4 switch bank's wiring of E3 and E8 to the
    STP fingers is not legible on the scan):
    - which 8251 is which: from the firmware, which loopback-tests 0C/0D
      as the computer port and 0A/0B as the auxiliary port
    - E3 and E8 clocked by the VT100's baud rate generator (TC TCLK/RCLK),
      which also still clocks the VT100's own 8251
    - the handshake lines on the terminal side, and the VT100's DTR on
      E3's DSR (the firmware tests it before starting the link)

***************************************************************************/

#include "emu.h"
#include "vt100_stp.h"
#include "vt125_cable.h"

#include "bus/rs232/rs232.h"
#include "machine/com8116.h"
#include "machine/i8251.h"
#include "machine/input_merger.h"


namespace {

class vt125_paddle_device : public device_t, public device_vt100_stp_interface, public vt125_paddle_cable_interface
{
public:
	vt125_paddle_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// device_vt100_stp_interface
	virtual void stp_w(u32 pin, int state) override;

	// vt125_paddle_cable_interface
	virtual void connect(vt125_mono_cable_interface &mono) override { m_mono = &mono; }
	virtual u8 in_com(offs_t offset) override;
	virtual void out_com(offs_t offset, u8 data) override;

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_reset_after_children() override ATTR_COLD;

private:
	required_device<i8251_device> m_aux;    // E11
	required_device<i8251_device> m_comm;   // E8
	required_device<i8251_device> m_link;   // E3
	required_device<com8116_device> m_brg;  // E10
	required_device<rs232_port_device> m_printer;

	vt125_mono_cable_interface *m_mono;
};


vt125_paddle_device::vt125_paddle_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, VT125_PADDLE, tag, owner, clock)
	, device_vt100_stp_interface(mconfig, *this)
	, m_aux(*this, "e11")
	, m_comm(*this, "e8")
	, m_link(*this, "e3")
	, m_brg(*this, "e10")
	, m_printer(*this, "printer")
	, m_mono(nullptr)
{
}


u8 vt125_paddle_device::in_com(offs_t offset)
{
	// E7 section A on A2-A1; its first output isn't wired
	switch (offset >> 1)
	{
	case 1: return m_aux->read(offset & 1);
	case 2: return m_comm->read(offset & 1);
	case 3: return m_link->read(offset & 1);
	default: return 0xff;
	}
}

void vt125_paddle_device::out_com(offs_t offset, u8 data)
{
	// E7 section B: WRITE BAUD L loads both halves of E10 at once
	switch (offset >> 1)
	{
	case 0: m_brg->stt_str_w(data); break;
	case 1: m_aux->write(offset & 1, data); break;
	case 2: m_comm->write(offset & 1, data); break;
	case 3: m_link->write(offset & 1, data); break;
	}
}


void vt125_paddle_device::stp_w(u32 pin, int state)
{
	using stp = vt100_stp_slot_device;
	switch (pin)
	{
	case stp::TC_TXD:
		m_link->write_rxd(state);
		break;
	case stp::TC_DTR:
		// the firmware checks DSR on the link before talking to the VT100
		// (E3 status bit 7, at 189E), as the VT18X's console 8251 does
		m_link->write_dsr(state);
		break;
	case stp::LINE_RXD:
		m_comm->write_rxd(state);
		break;
	case stp::LINE_DSR:
		m_comm->write_dsr(state);
		break;
	case stp::TC_TCLK:
		m_link->write_txc(state);
		m_comm->write_txc(state);
		slot().drive(stp::INTERNAL_TCLK, state);
		break;
	case stp::TC_RCLK:
		m_link->write_rxc(state);
		m_comm->write_rxc(state);
		slot().drive(stp::INTERNAL_RCLK, state);
		break;
	default:
		// the rest aren't wired on the paddle board
		break;
	}
}


void vt125_paddle_device::device_add_mconfig(machine_config &config)
{
	// all three 8251s are clocked by the mono board's 3 MHz (J4 pin 9),
	// the 24.0734 MHz HS CLK divided by 8
	constexpr XTAL CLK_3MHZ = 24.0734_MHz_XTAL / 8;

	input_merger_device &read_ready(INPUT_MERGER_ANY_HIGH(config, "read_ready"));
	read_ready.output_handler().set([this] (int state) { if (m_mono) m_mono->read_ready_w(state); });
	input_merger_device &trans_ready(INPUT_MERGER_ANY_HIGH(config, "trans_ready"));
	trans_ready.output_handler().set([this] (int state) { if (m_mono) m_mono->trans_ready_w(state); });

	I8251(config, m_aux, CLK_3MHZ);
	m_aux->txd_handler().set(m_printer, FUNC(rs232_port_device::write_txd));
	m_aux->dtr_handler().set([this] (int state) { if (m_mono) m_mono->graphics_flag_w(state); });
	m_aux->rxrdy_handler().set("read_ready", FUNC(input_merger_device::in_w<0>));
	m_aux->txrdy_handler().set("trans_ready", FUNC(input_merger_device::in_w<0>));

	I8251(config, m_comm, CLK_3MHZ);
	m_comm->txd_handler().set([this] (int state) { slot().drive(vt100_stp_slot_device::LINE_TXD, state); });
	m_comm->dtr_handler().set([this] (int state) { slot().drive(vt100_stp_slot_device::LINE_DTR, state); });
	m_comm->rxrdy_handler().set("read_ready", FUNC(input_merger_device::in_w<1>));
	m_comm->txrdy_handler().set("trans_ready", FUNC(input_merger_device::in_w<1>));

	I8251(config, m_link, CLK_3MHZ);
	m_link->txd_handler().set([this] (int state) { slot().drive(vt100_stp_slot_device::TC_RXD, state); });
	m_link->dtr_handler().set([this] (int state) { if (m_mono) m_mono->clk_shift_w(state); });
	m_link->rxrdy_handler().set("read_ready", FUNC(input_merger_device::in_w<2>));
	m_link->txrdy_handler().set("trans_ready", FUNC(input_merger_device::in_w<2>));

	// E10 from the mono board's 6 MHz (J4 pin 18); fT to TXC, fR to RXC
	COM8116_003(config, m_brg, 24.0734_MHz_XTAL / 4);
	m_brg->ft_handler().set(m_aux, FUNC(i8251_device::write_txc));
	m_brg->fr_handler().set(m_aux, FUNC(i8251_device::write_rxc));

	RS232_PORT(config, m_printer, default_rs232_devices, nullptr);
	m_printer->rxd_handler().set(m_aux, FUNC(i8251_device::write_rxd));
	m_printer->dsr_handler().set(m_aux, FUNC(i8251_device::write_dsr));
}

void vt125_paddle_device::device_start()
{
}

void vt125_paddle_device::device_reset()
{
	using stp = vt100_stp_slot_device;

	// the terminal side's handshake inputs are open: off (inferred)
	slot().drive(stp::TC_CTS, 1);
	slot().drive(stp::TC_DSR, 1);
	slot().drive(stp::TC_CD, 1);
	slot().drive(stp::TC_RI, 1);
	slot().drive(stp::TC_SPDI, 1);

	// computer port RTS and speed select are on whenever the terminal is
	// on (User Guide table 7-1)
	slot().drive(stp::LINE_RTS, 0);
	slot().drive(stp::LINE_SPDS, 0);
}

void vt125_paddle_device::device_reset_after_children()
{
	// the printer connector's DTR and RTS are pulled up to +12 V: always on
	// (User Guide table 7-2).  The 8251s' CTS inputs are grounded.
	m_printer->write_dtr(0);
	m_printer->write_rts(0);
	m_aux->write_cts(0);
	m_comm->write_cts(0);
	m_link->write_cts(0);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(VT125_PADDLE, device_vt100_stp_interface, vt125_paddle_device, "vt125_paddle", "DEC VT125 paddle board")
