// license:BSD-3-Clause
// copyright-holders:Brice Onken
// thanks-to:Patrick Mackinlay

/*
 * Sony NEWS DMAC3 DMA controller
 *
 * References:
 *  - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/dmac3reg.h
 *  - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/dmac3var.h
 *  - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/dmac3.c
 *  - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/spifi.c
 *
 * TODO:
 *  - Almost everything
 */

#include "emu.h"
#include "dmac3.h"

#define VERBOSE 1
#include "logmacro.h"

DEFINE_DEVICE_TYPE(DMAC3, dmac3_device, "dmac3", "Sony CXD8403Q DMA Controller")

dmac3_device::dmac3_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, DMAC3, tag, owner, clock), m_irq_handler(*this)
{
}

void dmac3_device::map_dma_ram(address_map &map)
{
	// Host platform configures the use of RAM at device attach
	map(0x0, map_ram_size - 1).ram();
}

uint32_t dmac3_device::csr_r(DMAC3_Controller controller)
{
	uint32_t val = m_controllers[controller].csr;
	LOG("dmac3-%d csr_r: 0x%x\n", controller, val);
	return val;
}
uint32_t dmac3_device::intr_r(DMAC3_Controller controller)
{
	uint32_t val = m_controllers[controller].intr;
	LOG("dmac3-%d intr_r: 0x%x\n", controller, val);
	return val;
}
uint32_t dmac3_device::length_r(DMAC3_Controller controller)
{
	uint32_t val = m_controllers[controller].length;
	LOG("dmac3-%d length_r: 0x%x\n", controller, val);
	return val;
}
uint32_t dmac3_device::address_r(DMAC3_Controller controller)
{
	uint32_t val = m_controllers[controller].address;
	LOG("dmac3-%d address_r: 0x%x\n", controller, val);
	return val;
}
uint32_t dmac3_device::conf_r(DMAC3_Controller controller)
{
	uint32_t val = m_controllers[controller].conf;
	LOG("dmac3-%d conf_r: 0x%x\n", controller, val);
	return val;
}

void dmac3_device::csr_w(DMAC3_Controller controller, uint32_t data)
{
	LOG("dmac3-%d csr_w: 0x%x\n", controller, data);
	m_controllers[controller].csr = data;
}

void dmac3_device::intr_w(DMAC3_Controller controller, uint32_t data)
{
	LOG("dmac3-%d intr_w: 0x%x\n", controller, data);
	auto intr_clear_bits = ~data & INTR_CLR_MASK; // Get 1s on bits to clear (TODO: is this right?)
	auto intr_enable_bits = data & INTR_EN_MASK; // Get 1s on bits to set
	LOG("dmac3-%d intr_w: intr_clear_bits = 0x%x intr_enable_bits = 0x%x\n", controller, intr_clear_bits, intr_enable_bits);
	m_controllers[controller].intr &= ~intr_clear_bits; // Clear interrupts;
	LOG("dmac3-%d intr_w: step 1: 0x%x\n", controller, m_controllers[controller].intr);
	m_controllers[controller].intr &= ~INTR_EN_MASK;	// Clear mask bits
	LOG("dmac3-%d intr_w: step 2: 0x%x\n", controller, m_controllers[controller].intr);
	m_controllers[controller].intr |= intr_enable_bits; // Set mask to new mask
	LOG("dmac3-%d intr_w: step 3: 0x%x\n", controller, m_controllers[controller].intr);

}

void dmac3_device::length_w(DMAC3_Controller controller, uint32_t data)
{
	LOG("dmac3-%d length_w: 0x%x\n", controller, data);
	m_controllers[controller].length = data;
}

void dmac3_device::address_w(DMAC3_Controller controller, uint32_t data)
{
	LOG("dmac3-%d address_w: 0x%x\n", controller, data);
	m_controllers[controller].address = data;
}

void dmac3_device::conf_w(DMAC3_Controller controller, uint32_t data)
{
	// Log is polluted with switching between SPIFI3 and regular mode
	// Will probably remove the if at some point, but we can mostly trust all 3
	// DMAC+SPIFI3 users (MROM, NEWS-OS, and NetBSD) to follow this correctly
	if(data != CONF_FASTACCESS && data != CONF_SLOWACCESS)
	{
		LOG("dmac3-%d conf_w: 0x%x\n", controller, data);
	}
	m_controllers[controller].conf = data;
}

void dmac3_device::check_irq()
{
	bool newIrq = false;
	
	// Scan each controller for an interrupt condition
	for (int controller = 0; controller < 2; ++controller)
	{
		// TODO: other interrupts
		uint32_t intr = m_controllers[controller].intr;
		newIrq |= ((intr | INTR_INT) > 0) && ((intr | INTR_INTEN) > 0);
	}

	if (m_irq != newIrq)
	{
		LOG("DMAC3 interrupt changed to %d!\n", newIrq);
		m_irq = newIrq;
		m_irq_handler(newIrq);
	}
}
