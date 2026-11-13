// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    DEC VT100 Advanced Video Option (AVO), board 5413097 (VT1XX-AB)

    Plugs onto the terminal controller board's J1.  Adds 1K x 8 of screen
    RAM at 2C00-2FFF, 4K x 4 of attribute RAM at 3000-3FFF, and four 2K
    program ROM sockets plus an 8K one.

    The terminal controller decodes only A10-A13, so its ROM and RAM repeat
    every 16K.  The AVO decodes A13-A15 (E12, a 74LS138 enabled on every
    cycle) and asserts MEM DISABLE for all of 8000-BFFF (E16, open collector
    gates on its 8000-9FFF and A000-BFFF outputs), so that area is the AVO's
    whether or not its sockets are filled; the patch switches add blocks
    below 2000 only.  TM-003 6.2.5.3 reads as if MEM DISABLE depended on the
    jumpers; the schematic doesn't.  Its RAM is selected from the terminal
    controller's decode, so it repeats with the base RAM outside 8000-BFFF.

    Switch pack S1 (E19 on the schematic; jumpers W1-W8 on early boards,
    Wn = S1-(9-n)) sets the ROM decode; closed enables:
    - S1-8..S1-5 (W1-W4): 2K socket A, B, C or D also answers at 0000,
      0800, 1000 or 1800, in place of the base firmware (patch ROMs; E24's
      2Y outputs, through E16 to MEM DISABLE)
    - S1-4, S1-3, S1-2 (W5-W7): the 8K socket (E8) at 0000 (overlay), A000
      or 8000 (E12 Y0, Y5, Y4 onto 8K ROM EN L)
    - S1-1 (W8): E8 is a 2K ROM, socket D at 9800; open, E8 is the 8K
      socket
    The 2K sockets answer in the 8000/A000 block the 8K socket doesn't
    claim: A000 with S1-2, else 8000 (inferred from W6/W7 in TM-003 6.2.3).
    No 8K ROM is dumped, so the 8K socket always reads empty.  Pack S2 (E20,
    W9-W16) rewires E8's pins for a 2K or 8K part and isn't modelled.  The
    TM's settings (table 5-10): none for a VT100, S1-3 for the VT1XX-AC and
    VT100-WC..WK, S1-8 and S1-2 for the VT100-WA/WB.  The decode is read at
    reset.

    Sources: VT100 technical manual EK-VT100-TM-003, 4.2.5 and 6.2.3,
    tables 5-8 to 5-10, figure 5-34; MP-00633 print set, AVO schematic
    D-CS-5413097-0-1 rev F (switch legend).

    TODO:
    - pack S2, and ROMs for the patch and 8K placements

***************************************************************************/

#include "emu.h"
#include "vt100_avo.h"

#include <algorithm>
#include <iterator>


//**************************************************************************
//  SLOT
//**************************************************************************

DEFINE_DEVICE_TYPE(VT100_AVO_SLOT, vt100_avo_slot_device, "vt100_avo_slot", "DEC VT100 AVO connector")

vt100_avo_slot_device::vt100_avo_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, VT100_AVO_SLOT, tag, owner, clock)
	, device_single_card_slot_interface<device_vt100_avo_interface>(mconfig, *this)
	, m_space(*this, finder_base::DUMMY_TAG, -1)
	, m_card(nullptr)
{
}

void vt100_avo_slot_device::device_start()
{
	m_card = get_card_device();
}

u8 vt100_avo_slot_device::video_r(offs_t offset)
{
	// nothing answers without the AVO: attributes all inactive
	return m_card ? m_card->video_r(offset) : 0xff;
}


device_vt100_avo_interface::device_vt100_avo_interface(const machine_config &mconfig, device_t &device)
	: device_interface(device, "vt100avo")
	, m_slot(dynamic_cast<vt100_avo_slot_device *>(device.owner()))
{
}


//**************************************************************************
//  CARD
//**************************************************************************

namespace {

class vt100_avo_device : public device_t, public device_vt100_avo_interface
{
public:
	vt100_avo_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: vt100_avo_device(mconfig, VT100_AVO, tag, owner, clock)
	{
	}

	virtual u8 video_r(offs_t offset) override;

protected:
	vt100_avo_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, type, tag, owner, clock)
		, device_vt100_avo_interface(mconfig, *this)
		, m_rom(*this, "rom")
		, m_s1(*this, "S1")
		, m_patch{ { *this, "patch0" }, { *this, "patch1" }, { *this, "patch2" }, { *this, "patch3" } }
		, m_overlay(*this, "overlay")
	{
	}

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	u8 attr_r(offs_t offset) { return m_attr[offset] | 0xf0; }   // 4 bits wide
	void attr_w(offs_t offset, u8 data) { m_attr[offset] = data & 0x0f; }

	u8 socket_r(offs_t offset);

	optional_region_ptr<u8> m_rom;      // 2K sockets A-D (E21, E17, E13, E8)
	required_ioport m_s1;
	memory_view m_patch[4];             // 0000-1FFF in 2K blocks
	memory_view m_overlay;              // the 8K socket at 0000
	u8 m_decode;                        // S1, as read at reset

	u8 m_screen[0x400];
	u8 m_attr[0x1000];
};

class vt100_avo_ac_device : public vt100_avo_device
{
public:
	vt100_avo_ac_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: vt100_avo_device(mconfig, VT100_AVO_AC, tag, owner, clock)
	{
	}

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
};


INPUT_PORTS_START( vt100_avo )
	PORT_START("S1")
	PORT_DIPNAME(0x01, 0x00, "E8 as 2K ROM D at 9800") PORT_DIPLOCATION("S1:1")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x01, DEF_STR(On))
	PORT_DIPNAME(0x02, 0x00, "8K ROM at 8000") PORT_DIPLOCATION("S1:2")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x02, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x00, "8K ROM at A000") PORT_DIPLOCATION("S1:3")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x04, DEF_STR(On))
	PORT_DIPNAME(0x08, 0x00, "8K ROM over 0000") PORT_DIPLOCATION("S1:4")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x08, DEF_STR(On))
	PORT_DIPNAME(0x10, 0x00, "ROM D over 1800") PORT_DIPLOCATION("S1:5")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x10, DEF_STR(On))
	PORT_DIPNAME(0x20, 0x00, "ROM C over 1000") PORT_DIPLOCATION("S1:6")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x20, DEF_STR(On))
	PORT_DIPNAME(0x40, 0x00, "ROM B over 0800") PORT_DIPLOCATION("S1:7")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x40, DEF_STR(On))
	PORT_DIPNAME(0x80, 0x00, "ROM A over 0000") PORT_DIPLOCATION("S1:8")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x80, DEF_STR(On))
INPUT_PORTS_END

// the VT1XX-AC kit's AVO maps its 8K program at A000
INPUT_PORTS_START( vt100_avo_ac )
	PORT_INCLUDE(vt100_avo)
	PORT_MODIFY("S1")
	PORT_DIPNAME(0x04, 0x04, "8K ROM at A000") PORT_DIPLOCATION("S1:3")
	PORT_DIPSETTING(   0x00, DEF_STR(Off))
	PORT_DIPSETTING(   0x04, DEF_STR(On))
INPUT_PORTS_END

ioport_constructor vt100_avo_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(vt100_avo);
}

ioport_constructor vt100_avo_ac_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(vt100_avo_ac);
}


ROM_START( vt100_avo_ac )
	ROM_REGION(0x2000, "rom", ROMREGION_ERASEFF)
	// VT1XX-AC printer port kit ROMs; table 5-9 also lists 23-284E2 for A and
	// 23-185E2 for B.  On these masks pin 18 is CE, pins 20 and 21 /CE1 and /CE2.
	// Same ROMs seen on a "54-13097-00 // PN2280402L" AVO and on a
	// "PN1030385J-F" AVO, which has a jumper in place of the switches.
	ROM_LOAD( "23-186e2.avo.e21", 0x0000, 0x0800, CRC(1592dec1) SHA1(c4b8fc9fc0514e0cd46ad2de03abe72271ce460b)) // Label: "S 8218 // C69063 // 23186E2" @E21
	ROM_LOAD( "23-187e2.avo.e17", 0x0800, 0x0800, CRC(c6d72a41) SHA1(956f9eb945a250fd05c76100b38c0ba381ab8fde)) // Label: "S 8228 // C69062 // 23187E2" @E17
ROM_END

const tiny_rom_entry *vt100_avo_ac_device::device_rom_region() const
{
	return ROM_NAME(vt100_avo_ac);
}


void vt100_avo_device::device_start()
{
	std::fill_n(m_screen, std::size(m_screen), 0);
	std::fill_n(m_attr, std::size(m_attr), 0x0f);
	m_decode = 0;

	save_item(NAME(m_screen));
	save_item(NAME(m_attr));
	save_item(NAME(m_decode));

	address_space &mem = space();

	// RAM: selected through the terminal controller's decode, so it repeats
	// every 16K with the base RAM
	mem.install_ram(0x2c00, 0x2fff, 0xc000, m_screen);
	mem.install_readwrite_handler(0x3000, 0x3fff, 0, 0xc000, 0,
			read8sm_delegate(*this, FUNC(vt100_avo_device::attr_r)),
			write8sm_delegate(*this, FUNC(vt100_avo_device::attr_w)));

	// MEM DISABLE: 8000-BFFF is always the AVO's; the patch blocks are
	// views over the base firmware
	mem.install_read_handler(0x8000, 0xbfff, read8sm_delegate(*this, FUNC(vt100_avo_device::socket_r)));
	mem.unmap_write(0x8000, 0xbfff);
	for (int i = 0; i < 4; i++)
	{
		offs_t const base = i * 0x800;
		mem.install_view(base, base + 0x7ff, m_patch[i]);
		if (m_rom.found())
			m_patch[i][0].install_rom(base, base + 0x7ff, &m_rom[base]);
		else
			m_patch[i][0].unmap_read(base, base + 0x7ff);
	}
	mem.install_view(0x0000, 0x1fff, m_overlay);
	m_overlay[0].unmap_read(0x0000, 0x1fff); // no 8K ROM dumped
}

void vt100_avo_device::device_reset()
{
	u8 const s1 = m_decode = m_s1->read();
	for (int i = 0; i < 4; i++)
	{
		if (BIT(s1, 7 - i))
			m_patch[i].select(0);
		else
			m_patch[i].disable();
	}
	if (BIT(s1, 3))
		m_overlay.select(0);
	else
		m_overlay.disable();
}

// 8000-BFFF: the 8K socket where S1-2/S1-3 put it, the 2K sockets in the
// other block; socket D only with S1-1, as E8 is otherwise the 8K socket.
// Unselected and empty sockets read open bus.
u8 vt100_avo_device::socket_r(offs_t offset)
{
	u8 const s1 = m_decode;
	bool const high = BIT(offset, 13); // A000-BFFF
	bool const rom8k = high ? BIT(s1, 2) : BIT(s1, 1);
	if (rom8k || !m_rom.found())
		return 0xff;    // no 8K ROM dumped
	if (high != BIT(s1, 1))
		return 0xff;    // the 2K sockets are in the other block
	offs_t const rom = offset & 0x1fff;
	if (rom >= 0x1800 && !BIT(s1, 0))
		return 0xff;
	return m_rom[rom];
}

u8 vt100_avo_device::video_r(offs_t offset)
{
	if (offset < 0x1000)
		return m_screen[offset & 0x3ff];
	else
		return attr_r(offset & 0xfff);
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(VT100_AVO, device_vt100_avo_interface, vt100_avo_device, "vt100_avo", "DEC VT1XX-AB Advanced Video Option")
DEFINE_DEVICE_TYPE_PRIVATE(VT100_AVO_AC, device_vt100_avo_interface, vt100_avo_ac_device, "vt100_avo_ac", "DEC VT1XX-AB Advanced Video Option (VT1XX-AC ROMs)")

void vt100_avo_options(device_slot_interface &device)
{
	device.option_add("avo", VT100_AVO);
	device.option_add("avo_ac", VT100_AVO_AC);
}
