// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    Apple Lisa expansion slots

***************************************************************************/

#include "emu.h"
#include "slot.h"

#include "dualpar.h"

DEFINE_DEVICE_TYPE(LISA_EXP_SLOT, lisa_exp_slot_device, "lisa_exp_slot", "Apple Lisa expansion slot")


lisa_exp_slot_device::lisa_exp_slot_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, LISA_EXP_SLOT, tag, owner, clock)
	, device_single_card_slot_interface<device_lisa_exp_card_interface>(mconfig, *this)
	, m_space(*this, finder_base::DUMMY_TAG, -1)
	, m_irq_cb(*this)
	, m_base(0)
{
}

void lisa_exp_slot_device::device_start()
{
	device_lisa_exp_card_interface *const card = get_card_device();
	if (card)
		card->install(*m_space, m_base, m_base + 0x2000);
}


device_lisa_exp_card_interface::device_lisa_exp_card_interface(machine_config const &mconfig, device_t &device)
	: device_interface(device, "lisaexp")
	, m_slot(dynamic_cast<lisa_exp_slot_device *>(device.owner()))
{
}

void device_lisa_exp_card_interface::interface_validity_check(validity_checker &valid) const
{
	device_t *const owner(device().owner());
	if (owner && dynamic_cast<device_slot_interface *>(owner) && !dynamic_cast<lisa_exp_slot_device *>(owner))
		osd_printf_error("Device %s (%s) is not an Apple Lisa expansion slot\n", owner->tag(), owner->name());
}


void lisa_exp_cards(device_slot_interface &device)
{
	device.option_add("dualpar", LISA_DUALPAR);
}
