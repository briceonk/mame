// license:BSD-3-Clause
// copyright-holders:Brice Onken

/*
 * 68k-based NEC EWS4800 workstations
 *
 * TODO:
 *  - everything
 */

#include "emu.h"
#include "cpu/m68000/m68040.h"
#include "machine/ram.h"

#define LOG_INTERRUPT (1U << 1)
#define LOG_ALL_INTERRUPT (1U << 2)

#define VERBOSE (LOG_GENERAL|LOG_INTERRUPT|LOG_ALL_INTERRUPT)
#include "logmacro.h"

namespace {

class ews4800_68k_state : public driver_device
{
public:
	ews4800_68k_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, "cpu")
		, m_ram(*this, "ram")
        , m_eprom(*this, "eprom")
	{
	}

	void ews4800_35(machine_config &config);
	void init();

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;
	void cpu_map(address_map &map);

private:
	required_device<m68040_device> m_cpu;
    // TODO: 030 IOP
	required_device<ram_device> m_ram;
	required_region_ptr<u32> m_eprom;
};

void ews4800_68k_state::machine_start()
{
}

void ews4800_68k_state::machine_reset()
{
    m_cpu->space(0).install_rom(0x00000000, 0x0001ffff, m_eprom); // 68k systems start from 0
}

void ews4800_68k_state::init()
{
}

void ews4800_68k_state::cpu_map(address_map &map)
{
	map.unmap_value_low();
    map(0xb8000000, 0xb801ffff).rom().region("eprom", 0); // guess based on where it starts
   
}

void ews4800_68k_state::ews4800_35(machine_config &config)
{

    M68040(config, m_cpu, 100_MHz_XTAL / 2); // guess, 100MHz crystal near to the chip
	m_cpu->set_addrmap(AS_PROGRAM, &ews4800_68k_state::cpu_map);

	RAM(config, m_ram);
	m_ram->set_default_size("16M"); // TODO: actual RAM size
	m_ram->set_default_value(0);
}

ROM_START(ews4800_35)
	ROM_REGION32_BE(0x20000, "eprom", 0)
	ROM_SYSTEM_BIOS(0, "ews4800_35", "ews4800_35")
	ROMX_LOAD("ews4800_35_m5m27c101kdip32.bin", 0x00000, 0x20000, CRC(dd9174b9) SHA1(7528f2eb097a331711cafb9f67d3ee3f2d17cdd3), ROM_BIOS(0))
ROM_END

} // anonymous namespace

/*   YEAR   NAME         PARENT  COMPAT  MACHINE    INPUT  CLASS              INIT  COMPANY  FULLNAME       FLAGS */
COMP(1993,  ews4800_35, 0,      0,      ews4800_35, 0,     ews4800_68k_state, init, "NEC",   "EWS4800/35", MACHINE_IS_SKELETON) // TODO: correct year
