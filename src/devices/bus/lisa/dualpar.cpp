// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    Apple Lisa Parallel Interface Card (dual parallel card)

    Two parallel ports for ProFile hard disks (or other parallel devices),
    each on its own 6522, and a 2 KB boot ROM (341-0193-A) holding the
    card ID ($E002: bootable, status routine, icons; card ID 2), a
    self-test that the boot ROM runs at power-on, and the code that boots
    from a ProFile on either port.

    Everything below is from the card schematic 050-4027-A and the
    service manual 029-0176-A:
    - The ROM (2716, U3F) is selected by SLn/ and sits on D0-D7: ROM byte
      n is at low-decode offset 2n+1.  A12 is not decoded, so it repeats
      in the 8 KB window.
    - Both 6522s are selected by SHn/ and sit on D0-D7; A11 picks the
      port (lower J1 at 0, upper J2 at $800), A3-A6 are RS0-RS3, so
      register r is at high-decode offset $800*port + 8*r + 1.  A7-A10
      and A12 are not decoded.
    - The 6522s run at CPUCK/4 (two LS112 stages), 1.25 MHz.
    - Both IRQ/ outputs are wired together onto the slot's INTn/.
    - The card answers every access with DTACK/; VPA/ and IAKn/ are not
      connected.
    Per port:
    - PA0-7: data, through an LS245 whose direction is PB3 (1 = from the
      connector) and whose enable is PB2 (0 = on).  PB2 also enables the
      LS244 half that drives /PSTRB (from CA2), R/W (PB3) and /CMD (PB4)
      to the connector and buffers PARITY in.
    - PB0: OCD (open cable detect), PB1 and CA1: BSY, CB1: CHK.
    - PB6: PARITY from the connector.  (Table 1 of the service manual
      puts CRES on PB6 and the parity reset on PB5; the schematic has
      PARITY on PB6 and CRES on the PB5 net.)
    - Parity checker: an LS280 sums the VIA-side data byte and PARITY,
      and an LS109 wired as a D flip-flop latches the even output on
      the rising edge of CA2; Q drives CB2.  The flip-flop's reset is the
      PB5 net, which PB5 (as an output, low) and CRES (through a 7417)
      both pull low; PB5 as an input reads CRES back.
    - PB7 drives CRES through a 7417 (open collector); system RESET does
      too (jumper X1/X2), and resets both 6522s.

    Not modelled: the DTACK/ delay (the card holds DTACK/ off until phi2
    is phased to the access, about four CPU clocks), and the CHK line
    (the applepp connector does not carry it; CB1 is held high).

***************************************************************************/

#include "emu.h"
#include "dualpar.h"

#include "bus/applepp/applepp.h"
#include "machine/6522via.h"
#include "machine/input_merger.h"

#include <bit>


namespace {

class lisa_dualpar_device : public device_t, public device_lisa_exp_card_interface
{
public:
	lisa_dualpar_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual tiny_rom_entry const *device_rom_region() const override ATTR_COLD;

	virtual void install(address_space &space, offs_t low, offs_t high) override ATTR_COLD;

private:
	required_device_array<via6522_device, 2> m_via;
	required_device_array<applepp_connector, 2> m_port;
	required_region_ptr<u8> m_rom;

	u8 m_host_data[2];      // the 6522's port A outputs
	u8 m_drive_data[2];     // what the device drives on the connector
	u8 m_pb[2];             // the 6522's port B outputs (inputs read as 1)
	bool m_drive_parity[2]; // PARITY from the connector
	bool m_ca2[2];
	bool m_parity_error[2]; // the LS109's Q, to CB2

	void low_map(address_map &map) ATTR_COLD;
	void high_map(address_map &map) ATTR_COLD;

	bool enabled(int n) const { return !BIT(m_pb[n], 2); }
	bool reading(int n) const { return BIT(m_pb[n], 3); }
	bool cres(int n) const { return BIT(m_pb[n], 7); }
	bool parity_in(int n) const { return enabled(n) ? m_drive_parity[n] : true; }
	u8 via_side_data(int n) const { return (enabled(n) && reading(n)) ? m_drive_data[n] : m_host_data[n]; }

	void update_outputs(int n);
	void update_inputs(int n);

	template <int N> void pa_w(u8 data);
	template <int N> void pb_w(u8 data);
	template <int N> void ca2_w(int state);
	template <int N> void drive_pd_w(u8 data);
	template <int N> void drive_parity_w(int state);
};


ROM_START(lisa_dualpar)
	ROM_REGION(0x800, "rom", 0)
	ROM_LOAD("341-0193-a.bin", 0x000, 0x800, CRC(48c96d3e) SHA1(9e7f7dc042c9082662ef31c1900920e40a0894dd))
ROM_END


lisa_dualpar_device::lisa_dualpar_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, LISA_DUALPAR, tag, owner, clock)
	, device_lisa_exp_card_interface(mconfig, *this)
	, m_via(*this, "via%u", 0U)
	, m_port(*this, "port%u", 0U)
	, m_rom(*this, "rom")
{
}

tiny_rom_entry const *lisa_dualpar_device::device_rom_region() const
{
	return ROM_NAME(lisa_dualpar);
}

void lisa_dualpar_device::device_add_mconfig(machine_config &config)
{
	input_merger_device &irq(INPUT_MERGER_ANY_HIGH(config, "irq"));
	irq.output_handler().set([this] (int state) { irq_w(state); });

	// port0 is the lower connector (J1), port1 the upper (J2)
	for (int n = 0; n < 2; n++)
	{
		MOS6522(config, m_via[n], DERIVED_CLOCK(1, 4));
		APPLEPP_CONNECTOR(config, m_port[n], applepp_intf, nullptr);
	}
	m_via[0]->irq_handler().set("irq", FUNC(input_merger_device::in_w<0>));
	m_via[1]->irq_handler().set("irq", FUNC(input_merger_device::in_w<1>));

	m_via[0]->writepa_handler().set(FUNC(lisa_dualpar_device::pa_w<0>));
	m_via[1]->writepa_handler().set(FUNC(lisa_dualpar_device::pa_w<1>));
	m_via[0]->writepb_handler().set(FUNC(lisa_dualpar_device::pb_w<0>));
	m_via[1]->writepb_handler().set(FUNC(lisa_dualpar_device::pb_w<1>));
	m_via[0]->ca2_handler().set(FUNC(lisa_dualpar_device::ca2_w<0>));
	m_via[1]->ca2_handler().set(FUNC(lisa_dualpar_device::ca2_w<1>));

	m_port[0]->write_pd().set(FUNC(lisa_dualpar_device::drive_pd_w<0>));
	m_port[1]->write_pd().set(FUNC(lisa_dualpar_device::drive_pd_w<1>));
	m_port[0]->write_pparity().set(FUNC(lisa_dualpar_device::drive_parity_w<0>));
	m_port[1]->write_pparity().set(FUNC(lisa_dualpar_device::drive_parity_w<1>));
	m_port[0]->write_pchk().set(m_via[0], FUNC(via6522_device::write_pb0));
	m_port[1]->write_pchk().set(m_via[1], FUNC(via6522_device::write_pb0));
	m_port[0]->write_pbsy().set(m_via[0], FUNC(via6522_device::write_pb1));
	m_port[0]->write_pbsy().append(m_via[0], FUNC(via6522_device::write_ca1));
	m_port[1]->write_pbsy().set(m_via[1], FUNC(via6522_device::write_pb1));
	m_port[1]->write_pbsy().append(m_via[1], FUNC(via6522_device::write_ca1));
}

void lisa_dualpar_device::device_start()
{
	save_item(NAME(m_host_data));
	save_item(NAME(m_drive_data));
	save_item(NAME(m_pb));
	save_item(NAME(m_drive_parity));
	save_item(NAME(m_ca2));
	save_item(NAME(m_parity_error));

	for (int n = 0; n < 2; n++)
	{
		m_drive_data[n] = 0xff;
		m_drive_parity[n] = true;
	}
}

void lisa_dualpar_device::device_reset()
{
	// RESET clears both 6522s, so every port line is an input (high)
	for (int n = 0; n < 2; n++)
	{
		m_host_data[n] = 0xff;
		m_pb[n] = 0xff;
		m_ca2[n] = true;
		m_parity_error[n] = false;
		m_via[n]->write_cb1(1);
		update_outputs(n);
		update_inputs(n);
	}
}

void lisa_dualpar_device::install(address_space &space, offs_t low, offs_t high)
{
	space.install_device(low, low + 0x1fff, *this, &lisa_dualpar_device::low_map);
	space.install_device(high, high + 0x1fff, *this, &lisa_dualpar_device::high_map);
}

void lisa_dualpar_device::low_map(address_map &map)
{
	map(0x0000, 0x1fff).lr8(NAME([this] (offs_t offset) { return m_rom[offset & 0x7ff]; })).umask16(0x00ff);
	map(0x0000, 0x1fff).nopw();
}

void lisa_dualpar_device::high_map(address_map &map)
{
	// offset is the word address: A3-A6 are bits 2-5, A11 is bit 10
	map(0x0000, 0x1fff).lrw8(
			NAME([this] (offs_t offset) { return m_via[BIT(offset, 10)]->read(BIT(offset, 2, 4)); }),
			NAME([this] (offs_t offset, u8 data) { m_via[BIT(offset, 10)]->write(BIT(offset, 2, 4), data); })).umask16(0x00ff);
}

// Lines the card drives to the connector, from the 6522's outputs
void lisa_dualpar_device::update_outputs(int n)
{
	// LS245 and LS244 outputs float (read high) while PB2 is high
	m_port[n]->pd_w((enabled(n) && !reading(n)) ? m_host_data[n] : 0xff);
	m_port[n]->prw_w(enabled(n) ? reading(n) : 1);
	m_port[n]->pcmd_w(enabled(n) ? BIT(m_pb[n], 4) : 1);
	m_port[n]->pres_w(cres(n));
}

// Lines the 6522 sees
void lisa_dualpar_device::update_inputs(int n)
{
	m_via[n]->write_pa((enabled(n) && reading(n)) ? m_drive_data[n] : 0xff);
	m_via[n]->write_pb6(parity_in(n));

	const bool pb5_net = BIT(m_pb[n], 5) && cres(n);
	m_via[n]->write_pb5(pb5_net);
	if (!pb5_net)
		m_parity_error[n] = false;
	m_via[n]->write_cb2(m_parity_error[n]);
}

template <int N>
void lisa_dualpar_device::pa_w(u8 data)
{
	m_host_data[N] = data;
	update_outputs(N);
	update_inputs(N);
}

template <int N>
void lisa_dualpar_device::pb_w(u8 data)
{
	m_pb[N] = data;
	update_outputs(N);
	update_inputs(N);
}

template <int N>
void lisa_dualpar_device::ca2_w(int state)
{
	if (state && !m_ca2[N] && BIT(m_pb[N], 5) && cres(N))
		m_parity_error[N] = !((std::popcount(via_side_data(N)) + parity_in(N)) & 1);
	m_ca2[N] = state;
	m_port[N]->pstrb_w(enabled(N) ? state : 1);
	m_via[N]->write_cb2(m_parity_error[N]);
}

template <int N>
void lisa_dualpar_device::drive_pd_w(u8 data)
{
	m_drive_data[N] = data;
	update_inputs(N);
}

template <int N>
void lisa_dualpar_device::drive_parity_w(int state)
{
	m_drive_parity[N] = state;
	update_inputs(N);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(LISA_DUALPAR, device_lisa_exp_card_interface, lisa_dualpar_device, "lisa_dualpar", "Apple Lisa Parallel Interface Card")
