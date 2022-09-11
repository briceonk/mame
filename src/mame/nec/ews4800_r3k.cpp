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

#define LOG_GENERAL (1U << 0)

#define VERBOSE LOG_GENERAL
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
		, m_vram(*this, "vram")
		// , m_scsibus(*this, "scsi")
		// , m_scsi(*this, "scsi:7:ncr53c96")
		// , m_net(*this, "net")
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

	// void int_check();

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
	required_device<ram_device> m_vram;
	
	// required_device<nscsi_bus_device> m_scsibus;
	// required_device<ncr53c94_device> m_scsi;
	// required_device<am7990_device> m_net;

	// // internal state
	// uint32_t intc_status = 0;
	// uint32_t intc_mask = 0;
	// uint32_t asob_int_mask = 0;
	// uint32_t asob_int_status = 0;
	// uint32_t asob_dma_int = 0;
	// uint32_t asob_dma_status = 0; // Not 100% sure what this is called or what it does
	// uint32_t led_state = 0;
	// uint32_t error_code = 0;
	// uint32_t timer_register = 0; // ???
	// uint32_t clock_register = 0;
	// uint32_t vmechk = 0x0;

	// // SCSI DMA
	// uint32_t scsi_dma_addr = 0;
	// uint32_t scsi_dma_tcount = 0;
	// uint32_t scsi_dma_cmd = 0; // not 100% sure about this one
	// emu_timer *m_dma_check;
	// TIMER_CALLBACK_MEMBER(dma_check);
	// bool scsi_drq;

	// uint32_t unknown_register_0 = 0x0;
	// uint32_t unknown_register_01 = 0x0;
	// uint32_t unknown_register_02 = 0x0;
	// uint32_t unknown_register_1 = 0x707;
	// uint32_t unknown_register_3 = 0x1000;
	// uint32_t unknown_register_4 = 0x1;
	// uint32_t unknown_register_5 = 0x1;
	// uint32_t unknown_register_6 = 0x1;
	// uint32_t unknown_register_7 = 0x1;
	// uint32_t unknown_register_8 = 0x1;
	// uint32_t unknown_register_9 = 0x1;
	// uint32_t unknown_register_10 = 0x1;
	// uint32_t unknown_register_11 = 0x1;
	// uint32_t unknown_register_12 = 0x80000000;
	// uint32_t unknown_register_13 = 0;
	// uint32_t unknown_register_14 = 0;
	// uint32_t unknown_register_15 = 0;

	// // These masks are based on what the monitor ROM expects the resulting CPU interrupt to be set.
	// static constexpr uint32_t INTC_INT5 = 0xf8000000;
	// static constexpr uint32_t INTC_INT4 = 0x07c00000;
	// static constexpr uint32_t INTC_INT3 = 0x003e0000;
	// static constexpr uint32_t INTC_INT2 = 0x0001f000;
	// static constexpr uint32_t INTC_INT1 = 0x00000fc0;
	// static constexpr uint32_t INTC_INT0 = 0x0000003f;
	// uint32_t interrupt_masks[6] = {INTC_INT0, INTC_INT1, INTC_INT2, INTC_INT3, INTC_INT4, INTC_INT5};
	// bool m_int_state[6] = {false, false, false, false, false, false};
	// const int interrupt_map[6] = {0, 1, 2, 3, 4, 5};

	// // INTC interrupt controller
	// // The EWS4800/310 appears to use the same interrupt controller as the TRA2 board (EWS4800/360, another APbus model)
	// // https://github.com/NetBSD/src/blob/05082e19134c05f2f4b6eca73223cdc6b5ab09bf/sys/arch/ews4800mips/include/sbd_tr2a.h
	// // https://github.com/NetBSD/src/blob/05082e19134c05f2f4b6eca73223cdc6b5ab09bf/sys/arch/ews4800mips/ews4800mips/tr2a_intr.c
	// void intc_clear_w(offs_t offset, uint32_t data);
	// uint32_t intc_status_r(offs_t offset);
	// void intc_mask_w(offs_t offset, uint32_t data);
	// uint32_t intc_mask_r(offs_t offset);

	// // Hardware timer TIMER0 (10ms period)
	// // TODO: Is this programmable somehow? 100Hz is what the NetBSD kernel expects.
	// emu_timer *m_timer0_timer;
	// TIMER_CALLBACK_MEMBER(timer0_clock);
	// void timer0_w(offs_t offset, uint32_t data);

	// // Debug
	// void patch_rom(address_map &map);
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

// void ews4800_r3k_state::patch_rom(address_map &map)
// {
// 	// Bypass checksum
// 	map(0x1fc0390c, 0x1fc0390f).lr32(NAME([]() { return 0x11400009; })); // beq $t2,$0,$bfc03934

// 	// Bypass part of the CPU logical check - it does some seemingly invalid stuff with the ADDU instruction that fails with the current emulation
// 	// Specifically, it tries to run ADDU on improperly sign-extended 32-bit values by manipulating them in 64 bit mode first
// 	// map(0x1fc05ee4, 0x1fc05ee7).lr32(NAME([]() { return 0x14430005; })); // bne $v0,$v1,$bfc05efc
// 	// map(0x1fc05f04, 0x1fc05f07).lr32(NAME([]() { return 0x14430005; })); // bne $v0,$v1,$bfc05f1c
//     // map(0x1fc05f2c, 0x1fc05f2f).lr32(NAME([]() { return 0x14430006; })); // bne $v0,$v1,$bfc05f48

// 	// Bypass obnoxious primary cache test - the r4000.cpp driver doesn't have a primary cache yet
// 	map(0x1fc06940, 0x1fc06943).lr32(NAME([]() { return 0xbf01e27; })); // j $bfc0789c
// 	map(0x1fc06944, 0x1fc06947).lr32(NAME([]() { return 0x0; })); // nop for branch delay slot
// }

void ews4800_r3k_state::cpu_map(address_map &map)
{
	map.unmap_value_low();
	map(0x1fc00000, 0x1fc3ffff).rom().region("eprom", 0);

	map(0x1b012000, 0x1b0120ff).rw(m_rtc, FUNC(mc146818_device::read_direct), FUNC(mc146818_device::write_direct)).umask32(0xff000000); // extends past this for rest of NVRAM??
	map(0x10000000, 0x101fffff).lrw8(NAME([this](offs_t offset){ return m_vram->read(offset); }), NAME([this](offs_t offset, uint8_t data) { m_vram->write(offset, data); })); // guess

  //map(0x10400000, 0x1057ffff).lrw8(NAME([this](offs_t offset)
	map(0x10400000, 0x105fffff).lrw8(NAME([this](offs_t offset)
		{ 
			return m_vram->read(offset); 
		}), 
		NAME([this](offs_t offset, uint8_t data) 
		{
			//LOG("0x104%5x: 0x%8x\n", offset, data);
			m_vram->write(offset, data); 
		}));
	
/*
	map(0x10400000, 0x1043ffff).lrw8(
		NAME([this](offs_t offset)
		{ 
			return m_vram->read(offset * 8); 
		}), 
		NAME([this](offs_t offset, uint8_t data) 
		{
			// LOG("0x104%5x: 0x%8x\n", offset, data);
			m_vram->write(offset * 8, data); 
		})); // guess
	
	map(0x10440000, 0x1047ffff).lrw8(
	NAME([this](offs_t offset)
	{ 
		return m_vram->read((offset + 1) * 8 ); 
	}), 
	NAME([this](offs_t offset, uint8_t data) 
	{
		// LOG("0x104%5x: 0x%8x\n", offset, data);
		m_vram->write((offset + 1) * 8, data); 
	})); // guess

	map(0x10480000, 0x104bffff).lrw8(
		NAME([this](offs_t offset)
		{ 
			return m_vram->read((offset + 2) * 8); 
		}), 
		NAME([this](offs_t offset, uint8_t data) 
		{
			// LOG("0x104%5x: 0x%8x\n", offset + 0x80000, data);
			m_vram->write((offset + 2) * 8, data); 
		})); // guess

	map(0x104c0000, 0x104fffff).lrw8(
		NAME([this](offs_t offset)
		{ 
			return m_vram->read((offset + 3) * 8); 
		}), 
		NAME([this](offs_t offset, uint8_t data) 
		{
			// LOG("0x104%5x: 0x%8x\n", offset + 0x80000, data);
			m_vram->write((offset + 3) * 8, data); 
		})); // guess
	
	map(0x10500000, 0x1053ffff).lrw8(
		NAME([this](offs_t offset)
		{ 
			return m_vram->read((offset + 4) * 8);
		}),
		NAME([this](offs_t offset, uint8_t data) 
		{
			// LOG("0x105%5x: 0x%8x\n", offset, data); 
			m_vram->write((offset + 4) * 8, data);
		})); // guess	
	
	map(0x10540000, 0x1057ffff).lrw8(
		NAME([this](offs_t offset)
		{ 
			return m_vram->read((offset + 5) * 8);
		}),
		NAME([this](offs_t offset, uint8_t data) 
		{
			// LOG("0x105%5x: 0x%8x\n", offset, data); 
			m_vram->write((offset + 5) * 8, data);
		})); // guess
	
	map(0x10580000, 0x105bffff).lrw8(
		NAME([this](offs_t offset)
		{
			return m_vram->read((offset + 6) * 8); 
		}),
		NAME([this](offs_t offset, uint8_t data) 
		{
			// LOG("0x105%5x: 0x%8x\n", offset + 0x80000, data); 
			m_vram->write((offset + 6) * 8, data); 
		})); // guess
	
	map(0x105c0000, 0x105fffff).lrw8(
		NAME([this](offs_t offset)
		{
			return m_vram->read((offset + 7) * 8); 
		}),
		NAME([this](offs_t offset, uint8_t data) 
		{
			// LOG("0x105%5x: 0x%8x\n", offset + 0x80000, data); 
			m_vram->write((offset + 7) * 8, data); 
		})); // guess
	*/

	// map(0x10c00000, 0x10c7ffff).lrw8(NAME([this](offs_t offset){ return m_vram->read(offset); }), NAME([this](offs_t offset, uint8_t data) { m_vram->write(offset, 0); })); // guess, block write

	
	map(0x15f00e00, 0x15f00e03).lr32(NAME([]() { return 0x10;})); // fb present?
	map(0x15f00c50, 0x15f00c5f).m(m_bt459, FUNC(bt459_device::map)).umask32(0xff);
	//map(0x)

	// map(0x1fbfffe0, 0x1fbfffff).ram();
	// map(0xfffff0, 0xffffff).ram();
	// patch_rom(map);

	// // Interrupt controller
	// map(0x1e000000, 0x1e000003).w(FUNC(ews4800_r3k_state::intc_clear_w));
	// map(0x1e000004, 0x1e000007).r(FUNC(ews4800_r3k_state::intc_status_r));
	// map(0x1e000008, 0x1e00000b).rw(FUNC(ews4800_r3k_state::intc_mask_r), FUNC(ews4800_r3k_state::intc_mask_w));

	// map(0x1e000040, 0x1e000043).lrw32(NAME([this]()
	// 									{
	// 									// LOG("read vmechk 0x%x (%s)\n", vmechk, machine().describe_context());
	// 									return 0x0; }),
	// 								NAME([this](offs_t offset, uint32_t val)
	// 									{
	// 									vmechk = val;
	// 									LOG("write vmechk 0x%x (%s)\n", vmechk, machine().describe_context());
	// 									}));

	// map(0x1e4a0040, 0x1e4a0043).lrw32(NAME([this]()
	// 									   {
	// 										LOG("read led status 0x%x (%s)\n", led_state, machine().describe_context());
	// 										return led_state; }),
	// 								  NAME([this](offs_t offset, uint32_t val)
	// 									   {
	// 										led_state = val;
	// 										LOG("write led status 0x%x (%s)\n", led_state, machine().describe_context());
	// 										}));
	// map(0x1e4a0044, 0x1e4a0047).lrw32(NAME([this]()
	// 									{
	// 									LOG("read error code 0x%x (%s)\n", error_code, machine().describe_context());
	// 									return error_code; }),
	// 								NAME([this](offs_t offset, uint32_t val)
	// 									{
	// 									error_code = val;
	// 									LOG("write error_code 0x%x (%s)\n", error_code, machine().describe_context());
	// 									}));

	// // Timekeeper NVRAM and RTC
	// map(0x1e490000, 0x1e497fff).rw(m_rtc, FUNC(mk48t08_device::read), FUNC(mk48t08_device::write)).umask32(0xff000000);
	// // Force zs console - something in the ROM monitor resets this to graphics, which isn't emulated.
	// map(0x1e4932a0, 0x1e4932a3).lr32(NAME([]() { return 0x01000000; }));
	// map(0x1e493030, 0x1e493033).lr32(NAME([]() { return 0x02000000; })); // boot device control
	// map(0x1e493024, 0x1e493027).lr32(NAME([]() { return 0x01000000; })); // no keyboard (probably something needs to change so it detects no keyboard)

	// map(0x1e400000, 0x1e400007).rw(m_net, FUNC(am7990_device::regs_r), FUNC(am7990_device::regs_w));
	map(0x1b010000, 0x1b01000f)
		.rw(m_scc[0], FUNC(z80scc_device::ab_dc_r), FUNC(z80scc_device::ab_dc_w))
		.umask32(0xff000000);
	map(0x1b011000, 0x1b01100f).rw(m_scc[1], FUNC(z80scc_device::ab_dc_r), FUNC(z80scc_device::ab_dc_w)).umask32(0xff000000);
	// map(0x1e410000, 0x1e41000f).m(m_scsi, FUNC(ncr53c94_device::map));

	// // Debug below this line
	// /*
	// map(0x1e000010, 0x1e000fff).lrw8(NAME([this](offs_t offset)
	// 									  {
	// 										uint8_t val = m_debug_ram->read(offset);
	// 										// LOG("read 0x%x, returning 0x%x (%s)\n", offset,val, machine().describe_context());
	// 										return val; }),
	// 								 NAME([this](offs_t offset, uint8_t val)
	// 									  {
	// 												// LOG("write 0x%x = 0x%x (%s)\n", offset, val, machine().describe_context());
	// 												m_debug_ram->write(offset, val); }));
	// */

	// map(0x1e000050, 0x1e00006f)
	// 	.lr32(NAME([]()
	// 			   { return 0xff; }));

	// map(0x1e000020, 0x1e000023).lrw32(NAME([this](offs_t offset)
	// 									   { return timer_register; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   {
	// 											timer_register = data;
	// 											if (data == 0xc07c) // TODO: what is this value actually signaling?
	// 											{
	// 												LOG("Enabling timer! Value = 0x%x (%s)\n", data, machine().describe_context());
	// 												m_timer0_timer->adjust(attotime::zero, 0, attotime::from_hz(100));
	// 											}
	// 											else
	// 											{
	// 												LOG("Disabling timer! Value = 0x%x (%s)\n", data, machine().describe_context());
	// 												m_timer0_timer->adjust(attotime::never);
	// 											} }));
	// map(0x1e000024, 0x1e000027).lrw32(NAME([this]()
	// 									   { 
	// 										LOG("read ur0 0x%x (%s)\n", unknown_register_0, machine().describe_context()); 
	// 										return unknown_register_0; }),
	// 								  NAME([this](offs_t offset, uint32_t val)
	// 									   {
	// 										unknown_register_0 = val;
	// 										LOG("write ur0 0x%x (%s)\n", unknown_register_0, machine().describe_context());
	// 										}));
	// map(0x1e000028, 0x1e00002b).lrw32(NAME([this]()
	// 									   { 
	// 										LOG("read ur02 0x%x (%s)\n", unknown_register_02, machine().describe_context()); 
	// 										return unknown_register_02; }),
	// 								  NAME([this](offs_t offset, uint32_t val)
	// 									   {
	// 										unknown_register_02 = val;
	// 										LOG("write ur02 0x%x (%s)\n", unknown_register_02, machine().describe_context());
	// 										}));

	// map(0x1e00002c, 0x1e00002f).lrw32(NAME([this]()
	// 									   { 
	// 										LOG("read 2c (%s)\n", machine().describe_context()); 
	// 										return 0x30d40; }),
	// 								  NAME([this](offs_t offset, uint32_t val)
	// 									   {
	// 										LOG("write 2c only 0x%x (%s)\n", val, machine().describe_context());
	// 										}));  
	// //(NAME([]() { return 0x30d40; })); // TODO: 0x30d40 is written to 0x24-0x27 right before this - maybe this should mirror what is there?
	
	// map(0x1e000074, 0x1e000077).lrw32(NAME([this]()
	// 									   { 
	// 										LOG("read ur01 0x%x (%s)\n", unknown_register_01, machine().describe_context()); 
	// 										return unknown_register_01; }),
	// 								  NAME([this](offs_t offset, uint32_t val)
	// 									   {
	// 										unknown_register_01 = val;
	// 										LOG("write ur01 0x%x (%s)\n", unknown_register_01, machine().describe_context());
	// 										}));
	
	


	// map(0x1e0000c0, 0x1e0000c3).lr32(NAME([]()
	// 									  { return 0x101E101E; }));
	// map(0x1e0000c4, 0x1e0000c7).lr32(NAME([]()
	// 									  { return 0xaa0f0000; }));
	// map(0x1e0000e0, 0x1e0000e3).lr32(NAME([]()
	// 									  { return 0x8A78001F; }));
	// map(0x1e0000e4, 0x1e0000e7).lr32(NAME([]()
	// 									  { return 0x1F04028; }));
	// map(0x1e0000e8, 0x1e0000eb).lr32(NAME([]()
	// 									  { return 0x1; }));


	// // some ASObus thing? Not sure what ASObus is, but it is mentioned in the NetBSD source code and seems to handle ZS, keyboard+mouse, SCSI, and Ethernet.
	
	// map(0x1e409000, 0x1e4090ff).ram();
	// map(0x1e409004, 0x1e409007).lrw32(NAME([this]()
	// 									   { return unknown_register_1; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { unknown_register_1 = data; }));

	// map(0x1e40a000, 0x1e40a0ff).ram(); // contains interrupt stuff for ASObus

	// map(0x1e40a008, 0x1e40a00b).lrw32(NAME([this]() // TODO: actually apply DMA mask
	// 									   { /*LOG("get asobus dma status = 0x%x (%s)\n", asob_dma_status, machine().describe_context());*/ return asob_dma_status; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { LOG("set asobus dma status = 0x%x\n", data); asob_dma_status = data; }));
	// map(0x1e40a00c, 0x1e40a00f).lrw32(NAME([this]()
	// 									   {
	// 										LOG("get asobus int mask = 0x%x\n", asob_int_mask);
	// 										return asob_int_mask; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   {
	// 										 LOG("set asobus int mask = 0x%x\n", data); 
	// 									   	 asob_int_mask = data; }));
	// map(0x1e40a010, 0x1e40a013).lrw32(NAME([this]()
	// 									   {
	// 										LOG("get asobus int status = 0x%x (%s)\n", asob_int_status, machine().describe_context());
	// 										return asob_int_status; 
	// 									   }), NAME([this](offs_t offset, uint32_t data) {
	// 										 LOG("set asobus int status = 0x%x\n", data); 
	// 									   	 asob_int_status = data; 
	// 									   }));
	// map(0x1e408000, 0x1e408003).lrw32(NAME([this]()
	// 									   {
	// 											LOG("read asobus dma int 0x%x (%s)\n", asob_dma_int, machine().describe_context()); 
	// 											return asob_dma_int; }),
	// 								  NAME([this](offs_t offset, uint8_t val)
	// 									   {
	// 											LOG("write asobus dma int 0x%x (%s)\n", val, machine().describe_context());
	// 											asob_dma_int = val; })); // contains DMA interrupt stuff
	
	// // ASObus device windows
	// map(0x1e408010, 0x1e4080ff).ram();
	// map(0x1e418000, 0x1e4180ff).ram();
	// map(0x1e418008, 0x1e41800b).lrw32(NAME([this]()
	// 									   {
	// 	LOG("read scsi dma cmd 0x%x (%s)\n", scsi_dma_cmd, machine().describe_context());
	// 	return scsi_dma_cmd; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   {
	// 	LOG("write scsi dma cmd 0x%x (%s)\n", data, machine().describe_context());
	// 	scsi_dma_cmd = data; 
	// 	if((scsi_dma_cmd & 0xff) == 0x7) // TODO: actual bit definitions
	// 	{
	// 		m_dma_check->adjust(attotime::zero);
	// 	}
	// 	}));
	// map(0x1e418010, 0x1e418013).lrw32(NAME([this]()
	// 									   {
	// 	LOG("read scsi dma addr 0x%x (%s)\n", scsi_dma_addr, machine().describe_context());
	// 	return scsi_dma_addr; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   {
	// 	LOG("write scsi dma addr 0x%x (%s)\n", data, machine().describe_context());
	// 	scsi_dma_addr = data; }));

	// map(0x1e418014, 0x1e418017).lrw32(NAME([this]()
	// 									{
	// LOG("read scsi dma tcount 0x%x (%s)\n", scsi_dma_tcount, machine().describe_context());
	// return scsi_dma_tcount; }),
	// 								NAME([this](offs_t offset, uint32_t data)
	// 									{
	// LOG("write scsi dma tcount 0x%x (%s)\n", data, machine().describe_context());
	// scsi_dma_tcount = data; }));

	// map(0x1e428000, 0x1e4280ff).ram();
	// map(0x1e438000, 0x1e4380ff).ram();
	// map(0x1e448000, 0x1e4480ff).ram();
	// map(0x1e458000, 0x1e4580ff).ram();
	// map(0x1e468000, 0x1e4680ff).ram();
	// map(0x1e478000, 0x1e4780ff).ram();
	// map(0x1e488000, 0x1e4880ff).ram();
	// map(0x1e498000, 0x1e4980ff).ram();
	// map(0x1e4a8000, 0x1e4a80ff).ram();
	// map(0x1e4b8000, 0x1e4b80ff).ram();
	// map(0x1e4c8000, 0x1e4c80ff).ram();
	// map(0x1e4d8000, 0x1e4d80ff).ram();
	// map(0x1e4e8000, 0x1e4e80ff).ram();
	// map(0x1e4f8000, 0x1e4f80ff).ram();

	// // APbus?
	// map(0x1e807020, 0x1e807023).lr32(NAME([this]()
	// 									  { 
	// 										LOG("returning 0x43700000 (%s)\n", machine().describe_context());
	// 										return 0x43700000; }));

	// /*
	// map(0x1e804008, 0x1e80400b).lr32(NAME([]()
	// 									  { return 0x00; }));
	// map(0x1e804018, 0x1e80401b).lr32(NAME([]()
	// 									{ return 0x00; }));
	// map(0x1e804028, 0x1e80402b).lr32(NAME([]()
	// 									{ return 0x00; }));
	// map(0x1e807020, 0x1e807023)
	// 	.lr32(NAME([]()
	// 			   { return 0x43700000; }));
	// 			   */
	// map(0x1e805020, 0x1e805023).lrw32(NAME([this]()
	// 									   { return unknown_register_3; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { unknown_register_3 = data; }));
	// map(0x1e805000, 0x1e805003).lrw32(NAME([this]()
	// 									   { 
	// 										LOG("read ur4, returning 0x%x (%s)\n", unknown_register_4, machine().describe_context()); 
	// 										return unknown_register_4; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { 
	// 										LOG("write ur4 = 0x%x (%s)\n", data, machine().describe_context());
	// 										unknown_register_4 = data; }));
	// map(0x1e805004, 0x1e805007).lrw32(NAME([this]()
	// 									   { 
	// 										LOG("read ur5, returning 0x%x (%s)\n", unknown_register_5, machine().describe_context()); 
	// 										return unknown_register_5; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { 
	// 										LOG("write ur5 = 0x%x (%s)\n", data, machine().describe_context());
	// 										unknown_register_5 = data; }));
	// map(0x1e805008, 0x1e80500b).lrw32(NAME([this]()
	// 									   { 
	// 										LOG("read ur6, returning 0x%x (%s)\n", unknown_register_6, machine().describe_context()); 
	// 										return unknown_register_6; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { 
	// 										LOG("write ur6 = 0x%x (%s)\n", data, machine().describe_context());
	// 										unknown_register_6 = data; }));
	// map(0x1e80500c, 0x1e80500f).lrw32(NAME([this]()
	// 									   { 
	// 										LOG("read ur7, returning 0x%x (%s)\n", unknown_register_7, machine().describe_context()); 
	// 										return unknown_register_7; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { 
	// 										LOG("write ur7 = 0x%x (%s)\n", data, machine().describe_context());
	// 										unknown_register_7 = data; }));
	// map(0x1e805010, 0x1e805013).lrw32(NAME([this]()
	// 									   { return unknown_register_8; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { unknown_register_8 = data; }));
	// map(0x1e805014, 0x1e805017).lrw32(NAME([this]()
	// 									   { return unknown_register_9; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { unknown_register_9 = data; }));
	// map(0x1e805018, 0x1e80501b).lrw32(NAME([this]()
	// 									   { return unknown_register_10; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { unknown_register_10 = data; }));
	// map(0x1e80501c, 0x1e80501f).lrw32(NAME([this]()
	// 									   { return unknown_register_11; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { unknown_register_11 = data; }));
	// map(0x1e80502c, 0x1e80502f).lrw32(NAME([this]()
	// 									   { 
	// 										LOG("read ur12, returning 0x%x (%s)\n", unknown_register_12, machine().describe_context()); 
	// 										return unknown_register_12; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   { 
	// 										LOG("write ur12 = 0x%x (%s)\n", data, machine().describe_context());
	// 										unknown_register_12 = data; }));
	// map(0x1e806000, 0x1e80600f).ram();
	// map(0x1e80600c, 0x1e80600f).lrw32(NAME([this]() // wtf is this - bp bfc02280 to get past whatever this is
	// 									   { 
	// 											// machine().debug_break();
	// 											LOG("read ur13, returning 0x%x (%s)\n", unknown_register_13, machine().describe_context());
	// 											return unknown_register_13;
	// 										}),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 										{
	// 											LOG("write ur13 = 0x%x (%s)\n", data, machine().describe_context());
	// 											unknown_register_13 = data;
	// 										}));

	// //map(0x1e807004, 0x1e807007).lr32(NAME([]() { return 0x00; }));
	// // map(0x1e807008, 0x1e80700b).lr32(NAME([]() { return 0x00; }));

	// map(0x1e807500, 0x1e8075ff)
	// 	.ram(); // used to go to 777f

	// map(0x1e805028, 0x1e80502b).lrw32(NAME([this](offs_t offset)
	// 									   {
	// 	LOG("attempt to read other 0x%x (%s)\n", offset, machine().describe_context());
	// 	return 0x0; }),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 									   {
	// 											LOG("write other 0x%x = 0x%x (%s)\n", offset, data, machine().describe_context());
	// 											m_debug_ram_2->write(data * 4, 0); // yeah, this is weird af. Self test? Queue? who tf knows
	// 											m_debug_ram_2->write(data * 4 + 1, 0); // TODO: maybe should be the other bytes of data, but I'll deal with it later
	// 											m_debug_ram_2->write(data * 4 + 2, 0);
	// 											m_debug_ram_2->write(data * 4 + 3, data); }));
	// map(0x1e807600, 0x1e80767f).lrw8(NAME([this](offs_t offset)
	// 										{ 
	// 											//machine().debug_break();
	// 											uint8_t val = m_debug_ram_2->read(offset);
	// 											LOG("read offset 0x%x, returning 0x%x (%s)\n", offset, val, machine().describe_context());
	// 											return val;
	// 										}),
	// 								 NAME([this](offs_t offset, uint8_t data)
	// 										{
	// 											LOG("write nonsense = 0x%x (%s)\n", data, machine().describe_context());
	// 											m_debug_ram_2->write(offset, data); 
	// 										})); // Reads this and expects to get back out 0x1e805028 written value back (with MSB unset), but acts as RAM before that loc is written.. If that passes, break at bfc02438 and add RAM to 1e807010
	// map(0x1e807680, 0x1e80777f).ram();
	// map(0x1e807010, 0x1e80701f).ram();

	// map(0x1e4a0008, 0x1e4a000b).lrw32(NAME([this]()
	// 										{
	// 											// machine().debug_break();
	// 											LOG("read clock register, returning 0x%x (%s)\n", clock_register, machine().describe_context());
	// 											return clock_register;
	// 										}),
	// 								  NAME([this](offs_t offset, uint32_t data)
	// 										{
	// 											LOG("write clock_register = 0x%x (%s)\n", data, machine().describe_context());
	// 												// NetBSD sets this to 0x80 when setting up - what frequency does this run at? How is this different from the other timer?
	// 												// This feels like a complete hack, either that or I am missing something
	// 											clock_register = data;
	// 											if (data == 0x80) // TODO: what is this value actually signaling?
	// 											{
	// 												LOG("Enabling timer! Value = 0x%x\n", data);
	// 												m_timer0_timer->adjust(attotime::zero, 0, attotime::from_hz(100));
	// 											}
	// 											else
	// 											{
	// 												LOG("Disabling timer! Value = 0x%x\n", data);
	// 												m_timer0_timer->adjust(attotime::never);
	// 											}
	// 										}));

	// // framebuffer
	// //map(0xf0f00e00, 0xf0f00e03).lr32(NAME([]() { return 0x10; }));

	// map(0xf0200000, 0xf02fffff).ram();
	// map(0xf0500000, 0xf05fffff).ram();
	// map(0xf0700000, 0xf07fffff).ram();
	// map(0xf0f00e00, 0xf0f00e03).lr32(NAME([]()
	// 									  { return 0x10; }));
	
	// // audio
	// map(0x1e460000, 0x1e460003).lr32(NAME([]()
	// 									  { return 0x80000000; }));
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
		m_bt459->screen_update(screen, bitmap, cliprect, m_vram->pointer());

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
	// AM7990(config, m_net);
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
	// m_screen->set_raw(107'500'000, 1504, (296 + 20), (1184 + 296 + 20), 920, 34, (884 + 34)); // just copy_pasted everything but the clock from gt.cpp for now
	// 1280 x 1024
	//m_screen->set_raw(107'500'000, 1504, 20, 1504 - 204, 1124, 34, 1124 - 34);
	//m_screen->set_raw(107'500'000, 1280 + 29, 296, 1280 + 296, 1124, 34, 1024 + 34);
	m_screen->set_raw(107'500'000, 2048, 0, 2048, 1024, 0, 1024);
	
	m_screen->set_screen_update(FUNC(ews4800_r3k_state::screen_update));
	BT459(config, m_bt459, 107'500'000);
	RAM(config, m_vram, 0).set_default_size("2M");
}

ROM_START(ews4800_210)
	ROM_REGION32_BE(0x40000, "eprom", 0)
	ROM_SYSTEM_BIOS(0, "ews4800_210", "ews4800_210")
	ROMX_LOAD("210.bin", 0x00000, 0x40000, CRC(270a53dd) SHA1(1f4edc48229e109dbcb0aac26cc9cba0e942a649), ROM_BIOS(0))
ROM_END

} // anonymous namespace

/*   YEAR   NAME         PARENT  COMPAT  MACHINE      INPUT  CLASS          INIT  COMPANY  FULLNAME       FLAGS */
COMP(1993,  ews4800_210, 0,      0,      ews4800_210, 0,     ews4800_r3k_state, init, "NEC",   "EWS4800/210", MACHINE_IS_SKELETON)
