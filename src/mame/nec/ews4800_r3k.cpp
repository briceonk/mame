// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * NEC EWS4800 systems.
 *
 * Sources:
 *  - http://www.jira-net.or.jp/vm/data/1993090101/1993090101knr/4-1-14.pdf
 *  - http://wiki.netbsd.org/ports/ews4800mips/
 *
 * TODO:
 *  - everything
 */

#include "emu.h"

// processors and memory
#include "cpu/mips/mips1.h"
#include "machine/ram.h"

// i/o devices
#include "machine/z80scc.h"
#include "machine/am79c90.h"
#include "machine/timekpr.h"
#include "machine/ncr5390.h"
#include "machine/mc146818.h"
#include "video/bt459.h"
#include "screen.h"

// busses and connectors
#include "machine/nscsi_bus.h"
#include "bus/nscsi/cd.h"
#include "bus/nscsi/hd.h"
#include "bus/rs232/rs232.h"

#include "debugger.h"

#define LOG_INTERRUPT (1U << 1)
#define LOG_ALL_INTERRUPT (1U << 2)

#define VERBOSE (LOG_GENERAL|LOG_INTERRUPT|LOG_ALL_INTERRUPT)
#include "logmacro.h"

namespace {

class ews4800_r3k_state : public driver_device
{
public:
	ews4800_r3k_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, "cpu")
		, m_ram(*this, "ram")
		// , m_debug_ram(*this, "debug_ram")
		// , m_debug_ram_2(*this, "debug_ram_2")
		// , m_debug_ram_3(*this, "debug_ram_3")
		// , m_rtc(*this, "rtc")
		, m_scc(*this, "scc%u", 0U)
		, m_serial(*this, "serial%u", 0U)
		, m_rtc(*this, "rtc")
		, m_screen(*this, "screen")
		, m_bt459(*this, "ramdac")
		, m_vram(*this, "vram%u", 0U)
		// , m_scsibus(*this, "scsi")
		// , m_scsi(*this, "scsi:7:ncr53c96")
		, m_net(*this, "net")
	{
	}

	// machine config
	void ews4800_210(machine_config &config);

	void init();

protected:
	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

	// address maps
	void cpu_map(address_map &map);

	// u16 lance_r(offs_t offset, u16 mem_mask = 0xffff);
	// void lance_w(offs_t offset, u16 data, u16 mem_mask = 0xffff);
	// void lance_irq(int state);
	// void scsi_irq(int state);
	// void scsi_drq_w(int state);

	void int_check();

private:
	// processors and memory
	required_device<r3000_device> m_cpu;
	required_device<ram_device> m_ram;
	// required_device<ram_device> m_debug_ram;
	// required_device<ram_device> m_debug_ram_2;
	// required_device<ram_device> m_debug_ram_3;

	// // i/o devices
	// required_device<mk48t08_device> m_rtc; // actually is a t18, which is equivalent per datasheet. MAME has the century and flags register set, but datasheet doesn't. Why?
	required_device_array<z80scc_device, 2> m_scc;
	required_device_array<rs232_port_device, 2> m_serial;

	required_device<mc146818_device> m_rtc;

	required_device<screen_device> m_screen;
	required_device<bt459_device> m_bt459;
	required_device_array<ram_device, 2> m_vram; // shadow vram for reshaping for RAMDAC on the fly while maintaing CPU access coherency
	
	// required_device<nscsi_bus_device> m_scsibus;
	// required_device<ncr53c94_device> m_scsi;
	required_device<am7990_device> m_net;

	// PICNIC interrupt controller
	bool m_int_state[6] = {false, false, false, false, false, false};
	uint8_t m_picnic_mask[6] = {0, 0, 0, 0, 0, 0};
	uint8_t m_picnic_status[6] = {0, 0, 0, 0, 0, 0}; // TODO: NMI
	static constexpr int interrupt_map[6] = {0, 1, 2, 3, 4, 5};

	// uint32_t asob_int_mask = 0;
	// uint32_t asob_int_status = 0;
	// uint32_t asob_dma_int = 0;
	// uint32_t asob_dma_status = 0; // Not 100% sure what this is called or what it does
	// uint32_t led_state = 0;
	// uint32_t error_code = 0;
	// uint32_t timer_register = 0; // ???
	// uint32_t clock_register = 0;
	// uint32_t vmechk = 0x0;

	// PICNIC interrupt controller
	// The EWS4800/310 appears to use the same interrupt controller as the TR2 board (EWS4800/350)
	// https://github.com/NetBSD/src/blob/trunk/sys/arch/ews4800mips/include/sbd_tr2.h
	// https://github.com/NetBSD/src/blob/trunk/sys/arch/ews4800mips/ews4800mips/tr2_intr.c
	void picnic_status_w(offs_t offset, uint8_t data);
	uint8_t picnic_status_r(offs_t offset);
	void picnic_mask_w(offs_t offset, uint8_t data);
	uint8_t picnic_mask_r(offs_t offset);
	// void generic_irq_w(uint8_t irq, uint8_t mask, int state);

	// // Hardware timer TIMER0 (10ms period)
	// // TODO: Is this programmable somehow? 100Hz is what the NetBSD kernel expects.
	// emu_timer *m_timer0_timer;
	// TIMER_CALLBACK_MEMBER(timer0_clock);
	// void timer0_w(offs_t offset, uint32_t data);

	// // Debug
	void patch_rom(address_map &map);
	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
};

// TIMER_CALLBACK_MEMBER(ews4800_r3k_state::dma_check)
// {
// 	LOG("scsi dma_check cmd = 0x%x drq = 0x%x\n", scsi_dma_cmd, scsi_drq);
// 	if ((scsi_dma_cmd & 0x7) == 0x7 && scsi_drq) // TODO: DMA out?
// 	{
// 		// get byte from DMA source
// 		const uint8_t data = m_scsi->dma_r();
// 		LOG("scsi dma r 0x%x = 0x%x tcount 0x%x\n", scsi_dma_addr, data, scsi_dma_tcount);
// 		m_cpu->space(0).write_byte(scsi_dma_addr, data);
// 		++scsi_dma_addr;
// 		--scsi_dma_tcount;
// 		if (scsi_dma_tcount == 0)
// 		{
// 			// TODO: set IRQ
// 		}
// 		else
// 		{
// 			m_dma_check->adjust(attotime::zero);
// 		}
// 	}
// }

// void ews4800_r3k_state::intc_clear_w(offs_t offset, uint32_t data)
// {
// 	// This is mostly speculation from how the monitor ROM tests the interrupt controller
// 	// Identify interrupt status byte
// 	if(data != 0x8000007c) // don't log timer interrupt
// 	{
// 		LOG("intc_clear w 0x%x - ", data);
// 	}
// 	uint32_t val = 0;
// 	uint32_t byteval = data & 0xf;
// 	if (byteval == 0xc)
// 	{
// 		val = 0x8;
// 	}
// 	else if (byteval == 0x8)
// 	{
// 		val = 0x4;
// 	}
// 	else if (byteval == 0x4)
// 	{
// 		val = 0x2;
// 	}
// 	else if (byteval == 0x0)
// 	{
// 		val = 0x1;
// 	}

// 	// Get 1 on bit to set or clear using the status byte
// 	val = val << (((data & 0xf0) >> 4) * 4);

// 	// set or clear?
// 	if (data & 0x80000000)
// 	{
// 		intc_status |= val;
// 	}
// 	else
// 	{
// 		intc_status &= ~val;
// 	}

// 	// Refresh interrupt state
// 	if (intc_status != 0x80000000) // don't log timer interrupt
// 	{
// 		LOG(" update status to 0x%x (%s)\n", intc_status, machine().describe_context());
// 	}
// 	int_check();
// }

// uint32_t ews4800_r3k_state::intc_status_r(offs_t offset)
// {
// 	LOG("intc_status r 0x%x (%s)\n", intc_status, machine().describe_context());
// 	return intc_status;
// }

// void ews4800_r3k_state::intc_mask_w(offs_t offset, uint32_t data)
// {
// 	LOG("intc_mask w 0x%x (%s)\n", data, machine().describe_context());
// 	intc_mask = data;
// 	int_check();
// }

// uint32_t ews4800_r3k_state::intc_mask_r(offs_t offset)
// {
// 	LOG("intc_mask r 0x%x (%s)\n", intc_mask, machine().describe_context());
// 	return intc_mask;
// }

// /*
//  * int_check
//  *
//  * Observes the platform state and updates the R4400's interrupt lines if needed.
//  */
// void ews4800_r3k_state::int_check()
// {
// 	for (int i = 0; i < 6; i++)
// 	{
// 		bool state = (intc_status & interrupt_masks[i]) && (intc_mask & interrupt_masks[i]);

// 		if (m_int_state[i] != state)
// 		{
// 			LOG("Setting CPU input line %d to %d\n", interrupt_map[i], state ? 1 : 0);
// 			m_int_state[i] = state;
// 			m_cpu->set_input_line(interrupt_map[i], state ? 1 : 0);
// 		}
// 	}
// }

// // TODO: fix name, I copied this from my NWS-5000X driver
// TIMER_CALLBACK_MEMBER(ews4800_r3k_state::timer0_clock)
// {
// 	intc_clear_w(0, 0x8000007c);
// }

uint8_t ews4800_r3k_state::picnic_status_r(offs_t offset)
{
	LOGMASKED(LOG_ALL_INTERRUPT, "picnic_status_r: PICNIC INT%d = 0x%x\n", offset, m_picnic_status[offset]);
	return m_picnic_status[offset];
}

void ews4800_r3k_state::picnic_status_w(offs_t offset, uint8_t data)
{
	LOGMASKED(LOG_INTERRUPT, "picnic_status_w: PICNIC INT%d = 0x%x\n", offset, data);
	m_picnic_status[offset] = data;
	int_check();
}

uint8_t ews4800_r3k_state::picnic_mask_r(offs_t offset)
{
	LOGMASKED(LOG_ALL_INTERRUPT, "picnic_mask_r: PICNIC MASK%d = 0x%x\n", offset, m_picnic_mask[offset]);
	return m_picnic_mask[offset];
}

void ews4800_r3k_state::picnic_mask_w(offs_t offset, uint8_t data)
{
	LOGMASKED(LOG_INTERRUPT, "picnic_mask_w: PICNIC MASK%d = 0x%x\n", offset, data);
	m_picnic_mask[offset] = data;
	int_check();
}

/*
void ews4800_r3k_state::generic_irq_w(uint8_t irq, uint8_t mask, int state)
{
	LOGMASKED(LOG_INTERRUPT, "generic_irq_w: PICNIC INT%d IRQ %d set to %d\n", irq, mask, state);
	if (state)
	{
		m_picnic_status[irq] |= mask;
	}
	else
	{
		m_picnic_status[irq] &= ~mask;
	}
	int_check();
}
*/

void ews4800_r3k_state::int_check()
{
	// The R3000 series has 6 bits in the interrupt register
	// These map to the 6 PICNIC interrupt groups
	for (int i = 0; i < 6; i++)
	{
		bool state = m_picnic_status[i] & m_picnic_mask[i]; // TODO: mask is specific bits? Assume so for now

		if (m_int_state[i] != state)
		{
			LOGMASKED(LOG_ALL_INTERRUPT, "Setting CPU input line %d to %d\n", interrupt_map[i], state ? 1 : 0);
			m_int_state[i] = state;
			m_cpu->set_input_line(interrupt_map[i], state ? 1 : 0);
		}
	}
}

void ews4800_r3k_state::machine_start()
{
	// m_timer0_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(ews4800_r3k_state::timer0_clock), this));
	// m_dma_check = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(ews4800_r3k_state::dma_check), this));
}

void ews4800_r3k_state::machine_reset()
{
}

void ews4800_r3k_state::init()
{
	// map the configured ram
	m_cpu->space(0).install_ram(0x00000000, m_ram->mask(), m_ram->pointer());
}

void ews4800_r3k_state::patch_rom(address_map &map)
{
	// patch out small portion of memory test that relies on cache
	map(0x1fc04704, 0x1fc04707).lr32(NAME([](){ return 0x0; })); 
	map(0x1fc047b0, 0x1fc047b3).lr32(NAME([](){ return 0x0; }));

	// Completely skip cache test, it hangs
	map(0x1fc04df4, 0x1fc04df7).lr32(NAME([](){ return 0x1420023e; })); // bne $r0 $r1 $bfc056f0
	map(0x1fc04df8, 0x1fc04dfb).lr32(NAME([](){ return 0x0;}));
}

void ews4800_r3k_state::cpu_map(address_map &map)
{
	map.unmap_value_low();
	map(0x1fc00000, 0x1fc3ffff).rom().region("eprom", 0);
	patch_rom(map);

	// PICNIC interrupt controller
	map(0x1b000000, 0x1b000013).rw(FUNC(ews4800_r3k_state::picnic_status_r), FUNC(ews4800_r3k_state::picnic_status_w)).umask32(0xff000000);
	map(0x1b001000, 0x1b001013).rw(FUNC(ews4800_r3k_state::picnic_mask_r), FUNC(ews4800_r3k_state::picnic_mask_w)).umask32(0xff000000);


	map(0x1b012000, 0x1b0120ff).rw(m_rtc, FUNC(mc146818_device::read_direct), FUNC(mc146818_device::write_direct)).umask32(0xff000000); // extends past this for rest of NVRAM??

	map(0x1b010000, 0x1b01000f)
		.rw(m_scc[0], FUNC(z80scc_device::ab_dc_r), FUNC(z80scc_device::ab_dc_w))
		.umask32(0xff000000);
	map(0x1b011000, 0x1b01100f).rw(m_scc[1], FUNC(z80scc_device::ab_dc_r), FUNC(z80scc_device::ab_dc_w)).umask32(0xff000000);

	// DMAC
	map(0x1fbe0060, 0x1fbe0067).ram();
	map(0x1fbe00a0, 0x1fbe00a7).ram();
	map(0x1fbe00e0, 0x1fbe00e7).ram();

	// GA (framebuffer)
	map(0x15f00e00, 0x15f00e03).lr32(NAME([]() { return 0x10;})); // fb present? monitor ROM needs this to use the right GA offsets for the RAMDAC and such
	map(0x10000000, 0x101fffff).noprw(); // TODO: what is this region? Does it provide some kind of view into VRAM?
		// .lrw8(NAME([this](offs_t offset){ return m_vram[0]->read(offset); }), NAME([this](offs_t offset, uint8_t data) { m_vram[0]->write(offset, data); }));

	map(0x10400000, 0x105fffff).lrw8(NAME([this](offs_t offset)
		{ 
			return m_vram[0]->read(offset);
		}), 
		NAME([this](offs_t offset, uint8_t data) 
		{
			m_vram[0]->write(offset, data);

			// Reshape data for RAMDAC
			int column = offset % 2048;
			int row = offset / 2048;
			if (column < 1280 && row < 1024)
			{
				 m_vram[1]->write(1280 * row + column, data);
			}
		}));
	
	map(0x10c00000, 0x10c7ffff).noprw(); // TODO: probably GA block write	
	map(0x15f00c50, 0x15f00c5f).m(m_bt459, FUNC(bt459_device::map)).umask32(0xff);
}

// u16 ews4800_r3k_state::lance_r(offs_t offset, u16 mem_mask)
// {
// 	return m_cpu->space(0).read_word(offset);
// }

// void ews4800_r3k_state::lance_w(offs_t offset, u16 data, u16 mem_mask)
// {
// 	m_cpu->space(0).write_word(offset, data);
// }

// void ews4800_r3k_state::lance_irq(int state)
// {
// 	LOG("Got IRQ from LANCE 0x%x\n", state);
// 	if(!state)
// 	{
// 		asob_dma_status |= 0x1;
// 	}
// 	else
// 	{
// 		asob_dma_status &= ~0x1;
// 	}
// }

// void ews4800_r3k_state::scsi_irq(int state)
// {
// 	LOG("Got IRQ from SCSI 0x%x\n", state);
// 	if(state)
// 	{
// 		asob_dma_status |= 0x2;
// 	}
// 	else
// 	{
// 		asob_dma_status &= ~0x2;
// 	}
// }

// void ews4800_r3k_state::scsi_drq_w(int state)
// {
// 	LOG("Got DRQ from SCSI 0x%x\n", state);
// 	scsi_drq = state;
// 	m_dma_check->adjust(attotime::zero);
// }

// static void ews4800_scsi_devices(device_slot_interface &device)
// {
// 	device.option_add("harddisk", NSCSI_HARDDISK);
// 	device.option_add("cdrom", NSCSI_CDROM);
// }

u32 ews4800_r3k_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
	{
		m_bt459->screen_update(screen, bitmap, cliprect, m_vram[1]->pointer());

		return 0;
	}

/*
 * irq  function
 *  1   fdd, printer
 *  2   ethernet, scsi
 *  3   vme?
 *  4   serial
 *  5   clock
 */
void ews4800_r3k_state::ews4800_210(machine_config &config)
{
	R3000(config, m_cpu, 25_MHz_XTAL, 32768, 32768); // size is based on unmapped accesses to icache/dcache
	m_cpu->set_addrmap(AS_PROGRAM, &ews4800_r3k_state::cpu_map);
	m_cpu->set_fpu(mips1_device_base::MIPS_R3010);

	// 8 SIMM slots
	RAM(config, m_ram);
	m_ram->set_default_size("16M");
	//m_ram->set_extra_options("80M,144M"); // TODO: ER doesn't go this high?
	m_ram->set_default_value(0);

	// RAM(config, m_debug_ram);
	// m_debug_ram->set_default_size("16K");
	// m_debug_ram->set_default_value(0);

	// RAM(config, m_debug_ram_2);
	// m_debug_ram_2->set_default_size("16K"); // waaaaaaay too big
	// m_debug_ram_2->set_default_value(0);

	// RAM(config, m_debug_ram_3);
	// m_debug_ram_3->set_default_size("16K"); // waaaaaaay too big
	// m_debug_ram_3->set_default_value(0xff);

	// // scsi bus and devices
	// NSCSI_BUS(config, m_scsibus);
	// NSCSI_CONNECTOR(config, "scsi:0", ews4800_scsi_devices, "harddisk");
	// NSCSI_CONNECTOR(config, "scsi:1", ews4800_scsi_devices, nullptr);
	// NSCSI_CONNECTOR(config, "scsi:2", ews4800_scsi_devices, nullptr);
	// NSCSI_CONNECTOR(config, "scsi:3", ews4800_scsi_devices, nullptr);
	// NSCSI_CONNECTOR(config, "scsi:4", ews4800_scsi_devices, nullptr);
	// NSCSI_CONNECTOR(config, "scsi:5", ews4800_scsi_devices, nullptr);
	// NSCSI_CONNECTOR(config, "scsi:6", ews4800_scsi_devices, nullptr);

	// // scsi host adapter (NCR53C96)
	// NSCSI_CONNECTOR(config, "scsi:7").option_set("ncr53c96", NCR53C94).clock(24_MHz_XTAL).machine_config([this](device_t *device)
	// 																									 {
	// 		ncr53c94_device &adapter = downcast<ncr53c94_device &>(*device);

	// 		adapter.set_busmd(ncr53c94_device::busmd_t::BUSMD_1);
	// 		adapter.irq_handler_cb().set(*this, FUNC(ews4800_r3k_state::scsi_irq));
	// 		adapter.drq_handler_cb().set(*this, FUNC(ews4800_r3k_state::scsi_drq_w)); 
	// 		});

	// // ethernet
	AM7990(config, m_net);
	// m_net->intr_out().set(FUNC(ews4800_r3k_state::lance_irq));
	// m_net->dma_in().set(FUNC(ews4800_r3k_state::lance_r));
	// m_net->dma_out().set(FUNC(ews4800_r3k_state::lance_w));

	// // mouse on channel A, keyboard on channel B?
	SCC85C30(config, m_scc[0], 4.915200_MHz_XTAL); // TODO: clock unconfirmed
	SCC85C30(config, m_scc[1], 4.915200_MHz_XTAL); // TODO: clock unconfirmed

	RS232_PORT(config, m_serial[0], default_rs232_devices, nullptr);
	m_serial[0]->cts_handler().set(m_scc[0], FUNC(scc85230_device::ctsa_w));
	m_serial[0]->dcd_handler().set(m_scc[0], FUNC(scc85230_device::dcda_w));
	m_serial[0]->rxd_handler().set(m_scc[0], FUNC(scc85230_device::rxa_w));
	m_scc[0]->out_rtsa_callback().set(m_serial[0], FUNC(rs232_port_device::write_rts));
	m_scc[0]->out_txda_callback().set(m_serial[0], FUNC(rs232_port_device::write_txd));
	m_scc[0]->out_dtra_callback().set(m_serial[0], FUNC(rs232_port_device::write_dtr));

	RS232_PORT(config, m_serial[1], default_rs232_devices, nullptr);
	m_serial[1]->cts_handler().set(m_scc[0], FUNC(z80scc_device::ctsb_w));
	m_serial[1]->dcd_handler().set(m_scc[0], FUNC(z80scc_device::dcdb_w));
	m_serial[1]->rxd_handler().set(m_scc[0], FUNC(z80scc_device::rxb_w));
	m_scc[0]->out_rtsb_callback().set(m_serial[1], FUNC(rs232_port_device::write_rts));
	m_scc[0]->out_txdb_callback().set(m_serial[1], FUNC(rs232_port_device::write_txd));
	m_scc[0]->out_dtrb_callback().set(m_serial[1], FUNC(rs232_port_device::write_dtr));

	// RTC (Dallas DS1387, which is an AT RTC + 4K NVRAM + embedded battery + embedded crystal)
	MC146818(config, m_rtc, 32.768_kHz_XTAL);

	// MK48T08(config, m_rtc);
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	// 1280 x 1024
	// TODO: current timings estimated based on MIPS Magnum (which uses the same resolution but has a slightly different pixclock)
	//       and the default position of the NEC monitor ROM cursor compared to real hardware.
	//       Need to figure out how the math here actually is determined
	m_screen->set_raw(107'500'000, 1688, 350, 1280 + 350, 1100, 52, 1024 + 52);
	m_screen->set_screen_update(FUNC(ews4800_r3k_state::screen_update));
	BT459(config, m_bt459, 107'500'000);
	RAM(config, m_vram[0], 0).set_default_size("2M");
	RAM(config, m_vram[1], 0).set_default_size("2M");
}

ROM_START(ews4800_210)
	ROM_REGION32_BE(0x40000, "eprom", 0)
	ROM_SYSTEM_BIOS(0, "ews4800_210", "ews4800_210")
	ROMX_LOAD("210.bin", 0x00000, 0x40000, CRC(270a53dd) SHA1(1f4edc48229e109dbcb0aac26cc9cba0e942a649), ROM_BIOS(0))
ROM_END

} // anonymous namespace

/*   YEAR   NAME         PARENT  COMPAT  MACHINE      INPUT  CLASS          INIT  COMPANY  FULLNAME       FLAGS */
COMP(1993,  ews4800_210, 0,      0,      ews4800_210, 0,     ews4800_r3k_state, init, "NEC",   "EWS4800/210", MACHINE_IS_SKELETON)
