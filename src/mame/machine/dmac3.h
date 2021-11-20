// license:BSD-3-Clause
// copyright-holders:Brice Onken,Tsubai Masanari
// thanks-to:Patrick Mackinlay

/*
 * Sony CXD8403Q DMAC3 DMA controller
 *
 * Register definitions were derived from the NetBSD source code, copyright (c) 2000 Tsubai Masanari.
 *
 * References:
 *  - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/dmac3reg.h
 *  - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/dmac3var.h
 *  - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/dmac3.c
 *  - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/spifi.c
 */

#ifndef MAME_MACHINE_DMAC3_H
#define MAME_MACHINE_DMAC3_H

#pragma once

class dmac3_device : public device_t
{
public:
	dmac3_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	// DMAC3 has two controllers on-chip
	// The 5000X uses controllers 0 and 1 for SPIFI/SCSI bus 0 and 1 respectively
	enum dmac3_controller
	{
		CTRL0 = 0,
		CTRL1 = 1,
	};

	// Address map setup
	template <typename... T>
	void set_apbus_address_translator(T &&...args) { m_apbus_virt_to_phys_callback.set(std::forward<T>(args)...); }
	template <dmac3_controller controller>
	void map(address_map &map)
	{
		map(0x0, 0x3).rw(FUNC(dmac3_device::csr_r<controller>), FUNC(dmac3_device::csr_w<controller>));
		map(0x4, 0x7).rw(FUNC(dmac3_device::intr_r<controller>), FUNC(dmac3_device::intr_w<controller>));
		map(0x8, 0xb).rw(FUNC(dmac3_device::length_r<controller>), FUNC(dmac3_device::length_w<controller>));
		map(0xc, 0xf).rw(FUNC(dmac3_device::address_r<controller>), FUNC(dmac3_device::address_w<controller>));
		map(0x10, 0x13).rw(FUNC(dmac3_device::conf_r<controller>), FUNC(dmac3_device::conf_w<controller>));
	}

	// Signal routing
	template <typename T>
	void set_bus(T &&tag, int spacenum) { m_bus.set_tag(std::forward<T>(tag), spacenum); }
	template <dmac3_controller controller>
	auto dma_r_cb() { return m_dma_r[controller].bind(); }
	template <dmac3_controller controller>
	auto dma_w_cb() { return m_dma_w[controller].bind(); }
	auto irq_out() { return m_irq_handler.bind(); }

	template <dmac3_controller controller>
	void irq_w(int state)
	{
		if (state)
		{
			m_controllers[controller].intr |= INTR_INT;
		}
		else
		{
			m_controllers[controller].intr &= ~INTR_INT;
		}
		m_irq_check->adjust(attotime::zero);
	}

	template <dmac3_controller controller>
	void drq_w(int state)
	{
		m_controllers[controller].drq = (state != 0);
		m_dma_check->adjust(attotime::zero);
	}

protected:
	// Overrides from device_t
	virtual void device_start() override;
	virtual void device_reset() override;

	// Connections to other devices
	required_address_space m_bus;
	devcb_write_line m_irq_handler;
	devcb_read8::array<2> m_dma_r;  // XXX 32b? 64b?
	devcb_write8::array<2> m_dma_w; // XXX 32b? 64b?
	device_delegate<uint32_t(uint32_t)> m_apbus_virt_to_phys_callback;

	// Timers and interrupts
	emu_timer *m_irq_check;
	emu_timer *m_dma_check;
	bool m_irq = false;
	TIMER_CALLBACK_MEMBER(irq_check);
	TIMER_CALLBACK_MEMBER(dma_check);

	// Other methods
	void reset_controller(dmac3_controller controller);

	// DMAC3 has two controllers on-chip
	struct dmac3_register_file
	{
		uint32_t csr = 0;     // Status register
		uint32_t intr = 0;    // Interrupt status register
		uint32_t length = 0;  // Transfer count register
		uint32_t address = 0; // Starting byte offset
		uint32_t conf = 0;    // Transaction configuration register
		bool drq = false;     // XXX Is this something different from DREQ?
	} m_controllers[2];

	// Bitmasks for DMAC3 registers
	enum DMAC3_CSR_MASKS : uint32_t
	{
		CSR_SEND = 0x0000,
		CSR_ENABLE = 0x0001,
		CSR_RECV = 0x0002,
		CSR_RESET = 0x0004,
		CSR_APAD = 0x0008,
		CSR_MBURST = 0x0010,
		CSR_DBURST = 0x0020,
	};

	const uint32_t INTR_CLR_MASK = (INTR_INT | INTR_TCI | INTR_EOP | INTR_EOPI | INTR_DREQ | INTR_DRQI | INTR_PERR);
	const uint32_t INTR_EN_MASK = (INTR_INTEN | INTR_TCIE | INTR_EOPIE | INTR_DRQIE);
	enum DMAC3_INTR_MASKS : uint32_t
	{
		INTR_INT = 0x0001,
		INTR_INTEN = 0x0002,
		INTR_TCIE = 0x0020,
		INTR_TCI = 0x0040,
		INTR_EOP = 0x0100,
		INTR_EOPIE = 0x0200, // End of operation interrupt enable
		INTR_EOPI = 0x0400,
		INTR_DREQ = 0x1000,  // Is this just DRQ? Or is this for triggering DMA requests to the host?
		INTR_DRQIE = 0x2000, // Interrupt on DRQ enable?
		INTR_DRQI = 0x4000,
		INTR_PERR = 0x8000,
	};

	// I'm not clear yet on what IPER, DERR, MPER are signalling
	// NetBSD ignores IPER and MPER, but resets the DMAC if DERR is asserted during the interrupt routine
	// DCEN and PCEN are set by NetBSD during attach (along with FASTACCESS)
	enum DMAC3_CONF_MASKS : uint32_t
	{
		CONF_IPER = 0x8000,
		CONF_MPER = 0x4000,
		CONF_PCEN = 0x2000,
		CONF_DERR = 0x1000,
		CONF_DCEN = 0x0800,
		CONF_ODDP = 0x0200,
		CONF_WIDTH = 0x00ff,
		CONF_SLOWACCESS = 0x0020, // SPIFI access mode (see NetBSD source code)
		CONF_FASTACCESS = 0x0001, // DMAC3 access mode (see NetBSD source code)
	};

	// Register file accessors
	uint32_t csr_r(dmac3_controller controller);
	uint32_t intr_r(dmac3_controller controller);
	uint32_t length_r(dmac3_controller controller);
	uint32_t address_r(dmac3_controller controller);
	uint32_t conf_r(dmac3_controller controller);

	void csr_w(dmac3_controller controller, uint32_t data);
	void intr_w(dmac3_controller controller, uint32_t data);
	void length_w(dmac3_controller controller, uint32_t data);
	void address_w(dmac3_controller controller, uint32_t data);
	void conf_w(dmac3_controller controller, uint32_t data);

	// Templates as partial functions for register file accessors since they can be bound at compile time
	template <dmac3_controller controller>
	uint32_t csr_r() { return csr_r(controller); }
	template <dmac3_controller controller>
	uint32_t intr_r() { return intr_r(controller); }
	template <dmac3_controller controller>
	uint32_t length_r() { return length_r(controller); }
	template <dmac3_controller controller>
	uint32_t address_r() { return address_r(controller); }
	template <dmac3_controller controller>
	uint32_t conf_r() { return conf_r(controller); }

	template <dmac3_controller controller>
	void csr_w(uint32_t data) { csr_w(controller, data); }
	template <dmac3_controller controller>
	void intr_w(uint32_t data) { intr_w(controller, data); }
	template <dmac3_controller controller>
	void length_w(uint32_t data) { length_w(controller, data); }
	template <dmac3_controller controller>
	void address_w(uint32_t data) { address_w(controller, data); }
	template <dmac3_controller controller>
	void conf_w(uint32_t data) { conf_w(controller, data); }
};

DECLARE_DEVICE_TYPE(DMAC3, dmac3_device)

#endif // MAME_MACHINE_DMAC3
