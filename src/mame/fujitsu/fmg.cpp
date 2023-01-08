// license:BSD-3-Clause
// copyright-holders:Brice Onken

/*
 * Fujitsu (FACOM) G-100 Series Workstations
 *
 * Sources:
 *   - https://museum.ipsj.or.jp/en/computer/work/0006.html
 *   - http://radioc.web.fc2.com/weblib/y2k/fujitsu/tmp2C0.html
 *
 * TODO:
 *   - Everything
 */

#include "emu.h"

#include "cpu/m68000/m68020.h"
#include "machine/ram.h"

// #include "debugger.h"

#define VERBOSE 0
#include "logmacro.h"

class facom_g_100_state : public driver_device
{
public:
	facom_g_100_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, "cpu")
		, m_ram(*this, "ram")
	{
	}

	void fmg150(machine_config &config);
	void init_common();

protected:
	void cpu_map(address_map &map);

	void common(machine_config &config);

private:
	// devices
	required_device<m68020_device> m_cpu;
	required_device<ram_device> m_ram;
};

void facom_g_100_state::init_common()
{
}

void facom_g_100_state::cpu_map(address_map &map)
{
	map(0x00000000, 0x0001ffff).rom().region("rom", 0);
	map(0x2f4a0000, 0x2f4bffff).rom().region("rom", 0);
	map(0xfffd0000, 0xfffeffff).rom().region("rom", 0);
}

void facom_g_100_state::common(machine_config &config)
{
	// Guess based on avaliable info and board pic
	M68020(config, m_cpu, 32_MHz_XTAL / 2);
	m_cpu->set_addrmap(AS_PROGRAM, &facom_g_100_state::cpu_map);

	RAM(config, m_ram);
	m_ram->set_default_size("4M");
	m_ram->set_extra_options("5M,6M,7M,8M");
	m_ram->set_default_value(0);
}

void facom_g_100_state::fmg150(machine_config &config)
{
	common(config);
}

ROM_START(fmg150)
	ROM_REGION32_BE(0x20000, "rom", 0)
	ROM_SYSTEM_BIOS(0, "fmg150", "VERSION 13/03 RELEASE 1990/03/26")
	ROMX_LOAD("fmg150.bin", 0x00000, 0x20000, CRC(c2020430) SHA1(acc3847841675fbc2272742b6c1b0afc615a1bd1), ROM_BIOS(0))
ROM_END

/*   YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT    CLASS              INIT         COMPANY    FULLNAME       FLAGS */
COMP(1987, fmg150,  0,      0,      fmg150,  0,       facom_g_100_state, init_common, "Fujitsu", "FACOM G-150", MACHINE_NOT_WORKING|MACHINE_NO_SOUND)
