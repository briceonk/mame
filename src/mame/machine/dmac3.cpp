// license:BSD-3-Clause
// copyright-holders:Brice Onken,Tsubai Masanari
// thanks-to:Patrick Mackinlay

#include "emu.h"
#include "dmac3.h"

#define VERBOSE 1
#include "logmacro.h"

DEFINE_DEVICE_TYPE(DMAC3, dmac3_device, "dmac3", "Sony CXD8403Q DMA Controller")

dmac3_device::dmac3_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, DMAC3, tag, owner, clock), m_irq_handler(*this), m_dma_r(*this), m_dma_w(*this)
{
}

void dmac3_device::device_start() 
{
	m_irq_handler.resolve_safe();
	m_dma_r.resolve_all_safe(0);
	m_dma_w.resolve_all_safe();

	m_irq_check = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(dmac3_device::irq_check), this));
	m_dma_check = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(dmac3_device::dma_check), this));
}

void dmac3_device::map_dma_ram(address_map &map)
{
	// Host platform configures the use of RAM at device attach
	map(0x0, MAP_RAM_SIZE - 1).ram();
}

void dmac3_device::set_base_map_address(uint32_t base_map_address)
{
	BASE_MAP_ADDRESS = base_map_address;
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
	if(data & CSR_RESET)
	{
		LOG("dmac3-%d chip reset\n", controller);
		reset_controller(controller);
	}
	else
	{
		m_controllers[controller].csr = data;
	}
}

void dmac3_device::intr_w(DMAC3_Controller controller, uint32_t data)
{
	LOG("dmac3-%d intr_w: 0x%x\n", controller, data);
	auto intr_clear_bits = ~data & INTR_CLR_MASK; // Get 1s on bits to clear XXX This might be wrong - NetBSD writes 1s to clear
	auto intr_enable_bits = data & INTR_EN_MASK; // Get 1s on bits to set
	m_controllers[controller].intr &= ~intr_clear_bits; // Clear requested interrupt flags
	m_controllers[controller].intr &= ~INTR_EN_MASK;	// Clear all mask bits
	m_controllers[controller].intr |= intr_enable_bits; // Set mask bits to new mask
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

void dmac3_device::device_reset()
{
	// Reset both controllers
	reset_controller(DMAC3_Controller::CTRL0);
	reset_controller(DMAC3_Controller::CTRL1);
}

void dmac3_device::reset_controller(DMAC3_Controller controller)
{
	m_controllers[controller].csr = 0;
	m_controllers[controller].intr &= INTR_INT; // TODO: is the external interrupt bit preserved? I assume so...
	m_controllers[controller].length = 0;
	m_controllers[controller].address = 0;
	m_controllers[controller].conf = 0;
	m_irq_check->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(dmac3_device::irq_check)
{
	bool newIrq = false;
	
	// Scan each controller for an interrupt condition - if any of these are true, set IRQ.
	// If both controllers have no interrupt conditions, IRQ can be cleared.
	for (int controller = 0; controller < 2; ++controller)
	{
		uint32_t intr = m_controllers[controller].intr;
		newIrq |= ((intr & INTR_INT) > 0) && ((intr & INTR_INTEN) > 0); // External interrupt (SPIFI)
		newIrq |= ((intr & INTR_EOPI) > 0) && ((intr & INTR_EOPIE) > 0); // End-of-operation interrupt
		newIrq |= ((intr & INTR_DRQI) > 0) && ((intr & INTR_DRQIE) > 0); // DRQ interrupt (?)
		newIrq |= ((intr & INTR_TCI) > 0) && ((intr & INTR_TCIE) > 0); // Transfer count interrupt (?)
		newIrq |= (intr & (INTR_PERR)) > 0; // XXX DREQ, EOP?
	}

	if (m_irq != newIrq)
	{
		LOG("DMAC3 interrupt changed to %d!\n", newIrq);
		m_irq = newIrq;
		m_irq_handler(newIrq);
	}
}

template <dmac3_device::DMAC3_Controller controller>
auto dmac3_device::drq_w(int state)
{
	m_controllers[controller].drq = state != 0;
	m_dma_check->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(dmac3_device::dma_check)
{

}
