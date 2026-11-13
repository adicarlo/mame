// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT100 alternate character ROM socket

    The character generator is two 2K x 8 sockets: the standard set
    (23-018E2) and an alternate one, read in its place for characters with
    the ALT CHAR SET attribute (attribute RAM bit 3, so with the AVO or on
    the VT102).  The VT100 has the alternate socket at E9, the VT101/VT102
    board at E4.  An empty socket reads high, which shows as solid cells
    (TM-003 6.2.4).

    E9 takes a 2316 mask ROM (pin 18 /CS2, pin 20 /CS1, pin 21 /CS3) as
    fitted; removing W4 and fitting W5 ties pin 21 to +5 V for a 2716 EPROM
    instead (MP-00633).

    The only DEC alternate ROM known is 23-094E2, fitted to the VT100-WC to
    -WZ foreign language word processing terminals.  Its contents are
    inferred to be European accented characters; it is not dumped.

    Sources: VT100 technical manual EK-VT100-TM-003, 6.2.4; MP-00633 print
    set.

***************************************************************************/

#include "emu.h"
#include "vt100_altchar.h"


//**************************************************************************
//  SLOT
//**************************************************************************

DEFINE_DEVICE_TYPE(VT100_ALTCHAR_SLOT, vt100_altchar_slot_device, "vt100_altchar_slot", "DEC VT100 alternate character ROM socket")

vt100_altchar_slot_device::vt100_altchar_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, VT100_ALTCHAR_SLOT, tag, owner, clock)
	, device_single_card_slot_interface<device_vt100_altchar_interface>(mconfig, *this)
	, m_card(nullptr)
{
}

void vt100_altchar_slot_device::device_start()
{
	m_card = get_card_device();
}

u8 vt100_altchar_slot_device::read(offs_t offset)
{
	// an empty socket reads high
	return m_card ? m_card->read(offset) : 0xff;
}


device_vt100_altchar_interface::device_vt100_altchar_interface(const machine_config &mconfig, device_t &device)
	: device_interface(device, "vt100altchar")
{
}


namespace {

//**************************************************************************
//  23-094E2
//**************************************************************************

class vt100_altchar_23_094e2_device : public device_t, public device_vt100_altchar_interface
{
public:
	vt100_altchar_23_094e2_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, VT100_ALTCHAR_23_094E2, tag, owner, clock)
		, device_vt100_altchar_interface(mconfig, *this)
		, m_rom(*this, "rom")
	{
	}

	virtual u8 read(offs_t offset) override { return m_rom[offset & 0x7ff]; }

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD { }

private:
	required_region_ptr<u8> m_rom;
};

ROM_START( vt100_altchar_23_094e2 )
	ROM_REGION(0x0800, "rom", ROMREGION_ERASE00)
	ROM_LOAD( "23-094e2-00", 0x0000, 0x0800, NO_DUMP)
ROM_END

const tiny_rom_entry *vt100_altchar_23_094e2_device::device_rom_region() const
{
	return ROM_NAME(vt100_altchar_23_094e2);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(VT100_ALTCHAR_23_094E2, device_vt100_altchar_interface, vt100_altchar_23_094e2_device, "vt100_altchar_23_094e2", "DEC 23-094E2 alternate character ROM")


void vt100_altchar_options(device_slot_interface &device)
{
	device.option_add("23_094e2", VT100_ALTCHAR_23_094E2);
}
