// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/***************************************************************************

    Apple Lisa expansion slots

    Each of the three slots has two 8 KB select lines into the CPU's I/O
    space: SLn/ (low decode, where the boot ROM looks for the card's ID
    and program ROM) and SHn/ (high decode).  Slot 1 is at 000000/002000,
    slot 2 at 004000/006000, slot 3 at 008000/00a000.  The bus is 16 bits
    wide; a card answers with DTACK/ (or VPA/ for 6800-style parts), and
    with no card the access runs into the CPU board's bus timeout.  Each
    slot has its own interrupt line: slot 1 is level 5, slot 2 level 4,
    slot 3 level 3 (Lisa Hardware Manual, Sep 1982, 2.6 and 3.4).

    Not modelled: the bus grant chain and DMA (BR, BGin/BGout, BGACK/,
    LDMA/), and interrupt acknowledge cycles answered by a card (IAKn/).

***************************************************************************/

#ifndef MAME_BUS_LISA_SLOT_H
#define MAME_BUS_LISA_SLOT_H

#pragma once


class device_lisa_exp_card_interface;

DECLARE_DEVICE_TYPE(LISA_EXP_SLOT, lisa_exp_slot_device)


class lisa_exp_slot_device : public device_t, public device_single_card_slot_interface<device_lisa_exp_card_interface>
{
public:
	// base: the slot's low decode in I/O space (0, 0x4000 or 0x8000)
	template <typename T, typename U>
	lisa_exp_slot_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock, T &&space_tag, int space_num, offs_t base, U &&opts, char const *dflt)
		: lisa_exp_slot_device(mconfig, tag, owner, clock)
	{
		set_space(std::forward<T>(space_tag), space_num);
		set_base(base);
		set_options(std::forward<U>(opts), dflt, false);
	}

	lisa_exp_slot_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock = 0U);

	template <typename T> void set_space(T &&tag, int spacenum) { m_space.set_tag(std::forward<T>(tag), spacenum); }
	void set_base(offs_t base) { m_base = base; }
	auto irq_cb() { return m_irq_cb.bind(); }

protected:
	virtual void device_start() override ATTR_COLD;

private:
	required_address_space m_space;
	devcb_write_line m_irq_cb;
	offs_t m_base;

	friend class device_lisa_exp_card_interface;
};


class device_lisa_exp_card_interface : public device_interface
{
protected:
	device_lisa_exp_card_interface(machine_config const &mconfig, device_t &device);

	virtual void interface_validity_check(validity_checker &valid) const override ATTR_COLD;

	// low: the SLn/ window, high: the SHn/ window, 8 KB each
	virtual void install(address_space &space, offs_t low, offs_t high) ATTR_COLD = 0;

	void irq_w(int state)
	{
		if (m_slot)
			m_slot->m_irq_cb(state);
	}

private:
	lisa_exp_slot_device *const m_slot;

	friend class lisa_exp_slot_device;
};


void lisa_exp_cards(device_slot_interface &device) ATTR_COLD;

#endif // MAME_BUS_LISA_SLOT_H
