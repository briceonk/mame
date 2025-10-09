// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
   Sony NEWS M68K systems.

   Sources:
     - http://wiki.netbsd.org/ports/news68k/

   TODO:
     - mouse/keyboard
     - graphics/slots

   The Sony NEWS Portable Workstation NWS-1250 is a "laptop" (weight of more than 8 Kg) with a black-and-white LCD (1120×780),
   a keyboard, a 3.5″ floppy disk drive, a (SCSI) harddisk, and interfaces for mouse, audio (phones, line in, mic in), SCSI,
   Ethernet (AUI), serial (DB9), and parallel (proprietary).

   This is its main PCB layout:
   _____________________________________________________________________________________________________________________________________________
  |               __________   _____________________   _______  _______               ____________________________    _____                    |
  |              |MB834200A|  | EPROM AM27C1024    |  |HC257_| |HC257_|              |||||||||||||||||||||||||||||   |·····|                   |
  |              |         |  |                    |   _______  _______            __________       _________       _________                  |
  |              |_________|  |____________________|  |HC257_| |HC257_|           |         | Xtal | Sony   | Xtal | Sony   |  ___             |
  |                            _______                                            |HD64646FS|  19  |WSC-AIF2|  741 |CDX1123 | |  |<-74HC244A   |__
  |               __________  |74HC32A  __________   _______                      |_________|      |        |      |        | |__|              __|_
  |              |MB834200A|   _______ |Intel    |  ACT11004                     ___ ___           |________|      |________|                  |____|
__|              |         |  |ALS05A| |N82077   |   __             6 x 74F00J->|  | |  |   _________________________________________          __ |
|   ________     |_________|   _______ |         |  |-|                         |__| |__|   |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|         | | |
|  |74ACT139                  |74LS14| |_________|  |-| __________  __________   ___ ___    _________________________________________ DIPSx8->| | |
|   ________                                        |-||HM62256LFP |HM62256LFP  |  | |  |   |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|         |_| |
|  74ACT11244               ____________   ___      |-||_________| |_________|  |__| |__|   _________________________________________        ___  |
|   ________               | Sony      |  |  |      |-|                          ___ ___    |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|       T7705 |
|  74ACT11244              |WSC-MEMPAK |  |  |      |-|    ____________         |  | |  |   _________________________________________         __  |
|   ________               |9030EK712  |  |__|      |_|   | Sony      |         |__| |__|   |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_| Switch->__| |
|  74ACT11244       Xtal   |           | SG51KH           |WSC-LCMC   |               __    _________________________________________         __  |
|                 4915.2   |___________| 50 MHz           |9025EK442  |              |..|   |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_| Switch->__| |
|   ________                    ________                  |           |              |..|   _________________________________________         __  |
|  |74ACT139                   |ACT11244   ________       |___________|              |..|   |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_| Switch->__| |
|   ________                              |ACT11245                    :|            |..|   _________________________________________        ___  |
|  |ACT1124             ________________   ________                 :| :|            |..|   |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|      74AS00 |
|   ________           | MK48T02B-25   |  |ACT11245                 :| :|            |..|   _________________________________________             |
|  |AM27S21PC                                                                        |__|   |_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|             |
|  _____________________________________       ________  ________  ________            ________   ________   ________   ________   ______________ |
|_|                                    |____  |ACT11353 |ACT11353 |ACT11020           |AC11004|  |AC11004|  |AC11004|  |AC11004|  |o o o o o o o ||
  |____________________________________|    |  ________  ________                ___    ___    ___    ___    ___    _________     ________        |
                                            | |ACT11353 |ACT11353               |  |   |  |   |  |   |  |   |  |   |Sony    |    |AC11004|        |
                                            |  ________  ________  ________    BCT245 BCT245 BCT245 BCT245 BCT245  |WSC-LANCE   __________        |
                                            | |ACT11353 |ACT11353 |ACT11027     |__|   |__|   |__|   |__|   |__|   |________| HM6264ALFP-12T      |
                                            |                                                                                   __________    ___ |
                                            | _____________       _____________      _____________                            HM6264ALFP-12T |__<-SG51KH 32 MHz
                                            || Motorola   |      | Motorola   |     | Sony       |                  __________________________    |
                                            ||MC68882FN25A|      |XC68030FE25B|     |L7A0266     |                 | AMD                     |    |
                                            ||            |      |            |     |WSC-ICKDMAC |                 | AM7990PC                |    |
                                            ||            |      |            |     |9019        |                 |_________________________|    |
                                            ||____________|      |____________|     |____________|          ___________    ___________            |
                                            |  ___________   ________   ________                           |Sony      |   |AM7992BDC_|            |
                                            | | Zilog    |  |74HC374|  74HCT244A       ________   ________ |CXD1185Q  |    _________              |
                                            | |Z85C3008VSC                ________    |74ACT139  |ACT11002 |__________|   SG51K 20 MHz            |
                                            |                            |MC1489A|                                                                |
                                            |             ________        _____________________________________                                   |
                                            |            |MC145406       |::::::::::::::::::::::::::::::::::::|                                   |
                                            |  __________   __________   _______________________                 __________________               |
                                            |_|         |__|         |__|                      |________________|                 |_______________|
                                              |_________|  |_________|  |______________________|                |_________________|

 On the other side of the PCB there are a few components too:
  - 3 x HM62256LFP-12T
  - 1 x DS1000S-50

 NWS-1250 came bundled with a Sony mouse based on a Fujitsu MB88201H MCU (undumped mask ROM 512 x 8 bits).

 */

#include "emu.h"

#include "cpu/m68000/m68030.h"

// memory
#include "machine/ram.h"

// various hardware
#include "machine/timekpr.h"
#include "machine/z80scc.h"
#include "machine/am79c90.h"
#include "machine/upd765.h"
#include "dmac_0266.h"
#include "news_hid.h"
#include "machine/ncr5380.h"
#include "machine/cxd1185.h"

// video
#include "screen.h"
#include "video/bt45x.h"

// busses and connectors
#include "machine/nscsi_bus.h"
#include "bus/nscsi/cd.h"
#include "bus/nscsi/hd.h"
#include "bus/rs232/rs232.h"

#include "machine/input_merger.h"
#include "imagedev/floppy.h"

#define VERBOSE LOG_GENERAL
#include "logmacro.h"

#define DESKTOP_GRAPHICS 0


namespace {

class news_68k_base_state : public driver_device
{
public:
	news_68k_base_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, "cpu")
		, m_ram(*this, "ram")
		, m_dma(*this, "dma")
		, m_rtc(*this, "rtc")
		, m_scc(*this, "scc")
		, m_net(*this, "net")
		, m_fdc(*this, "fdc")
		, m_hid(*this, "hid")
		, m_serial(*this, "serial%u", 0U)
		, m_irq5(*this, "irq5")
		, m_irq7(*this, "irq7")
		, m_sw1(*this, "SW1")
		, m_eprom(*this, "eprom")
		, m_led(*this, "led%u", 0U)
	{
	}

	void init_common();

protected:
	// driver_device overrides
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	// address maps
	void cpu_map(address_map &map) ATTR_COLD;
	void cpu_autovector_map(address_map &map) ATTR_COLD;

	// machine config
	void common(machine_config &config);
	void config_scc(machine_config &config, const char *default_device_name);

	enum irq_number : unsigned
	{
		IPIRQ1     = 0,
		IPIRQ3     = 1,
		LANCE      = 2,
		VME4_AUDIO = 3, // Desktop = VME4, Laptop = Audio
		VME2       = 4,
		FDC        = 5,
		PRINTER    = 6,
		SCSI       = 7,
	};
	template <irq_number Number> void irq_w(int state);
	void int_check();

	void timer(s32 param);
	void timer_w(u8 data);

	u32 bus_error_r();

	void poweron_w(u8 data);

	// devices
	required_device<m68030_device> m_cpu;
	required_device<ram_device> m_ram;
	required_device<dmac_0266_device> m_dma; // TODO: move dma config to subcls b/c it refers to different scctrl?
	required_device<m48t02_device> m_rtc;
	required_device<z80scc_device> m_scc;
	required_device<am7990_device> m_net;
	required_device<upd765_family_device> m_fdc;
	required_device<news_hid_hle_device> m_hid;

	required_device_array<rs232_port_device, 2> m_serial;

	required_device<input_merger_device> m_irq5;
	required_device<input_merger_device> m_irq7;
	required_ioport m_sw1;

	required_region_ptr<u32> m_eprom;
	std::unique_ptr<u16[]> m_net_ram;
	output_finder<2> m_led;

	emu_timer *m_timer;

	u8 m_intst;
	u8 m_parity_vector;
	bool m_int_state[2];
	bool m_scc_irq_state;
	bool m_parity_irq_state;
};

class news_68k_desktop_state : public news_68k_base_state
{
public:
	news_68k_desktop_state(machine_config const &mconfig, device_type type, char const *tag)
		: news_68k_base_state(mconfig, type, tag)
		, m_scsi(*this, "scsi:7:cxd1180")
#if DESKTOP_GRAPHICS
		, m_screen(*this, "screen")
		, m_ramdac(*this, "ramdac")
		, m_vram(*this, "vram")
#endif
	{
	}

	void nws1580(machine_config &config) ATTR_COLD;

protected:
	void desktop_cpu_map(address_map &map);

#if DESKTOP_GRAPHICS
	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect) { return 0; }
#endif

	required_device<cxd1180_device> m_scsi;

#if DESKTOP_GRAPHICS
	// TODO: some (but not all) of this can be consolidated with the laptop state later
	required_device<screen_device> m_screen;
	required_device<bt458_device> m_ramdac;
	required_device<ram_device> m_vram;
#endif
};

class news_68k_laptop_state : public news_68k_base_state
{
public:
	news_68k_laptop_state(machine_config const &mconfig, device_type type, char const *tag)
	: news_68k_base_state(mconfig, type, tag)
	, m_scsi(*this, "scsi:7:cxd1185")
	, m_lcd(*this, "lcd")
	, m_vram(*this, "vram")
	{
	}

	void nws1250(machine_config &config) ATTR_COLD;
	virtual void machine_start() override ATTR_COLD;

protected:
	void laptop_cpu_map(address_map &map);
	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect);

	required_device<cxd1185_device> m_scsi;
	required_device<screen_device> m_lcd;
	required_shared_ptr<u32> m_vram;

	bool m_lcd_enable = false;
	bool m_lcd_dim = false;
};

void news_68k_base_state::machine_start()
{
	m_led.resolve();

	m_net_ram = std::make_unique<u16[]>(8192);

	m_timer = timer_alloc(FUNC(news_68k_base_state::timer), this);

	m_intst = 0;
	for (bool &int_state : m_int_state)
		int_state = false;

	m_scc_irq_state = false;
	m_parity_irq_state = false;
}

void news_68k_laptop_state::machine_start()
{
	news_68k_base_state::machine_start();

	save_item(NAME(m_lcd_enable));
	save_item(NAME(m_lcd_dim));
	m_lcd_enable = false;
	m_lcd_dim = false;
}

void news_68k_base_state::machine_reset()
{
	// eprom is mapped at 0 after reset
	m_cpu->space(0).install_rom(0x00000000, 0x0001ffff, m_eprom);
}

void news_68k_base_state::init_common()
{
	// HACK: hardwire the rate
	m_fdc->set_rate(500000);
}

void news_68k_base_state::cpu_map(address_map &map)
{

}

void news_68k_desktop_state::desktop_cpu_map(address_map &map)
{
	cpu_map(map);
	map(0xe0cc0000, 0xe0cc0007).m(m_scsi, FUNC(ncr5380_device::map));

		// Below this line is original
	map(0xe0000000, 0xe000ffff).rom().region("eprom", 0);

	// 0xe0c40000 // centronics
	map(0xe0c80000, 0xe0c80003).m(m_fdc, FUNC(upd72067_device::map));
	map(0xe0c80100, 0xe0c80100).rw(m_fdc, FUNC(upd72067_device::dma_r), FUNC(upd72067_device::dma_w));

	map(0xe0d00000, 0xe0d00007).m(m_hid, FUNC(news_hid_hle_device::map_68k));
	map(0xe0d40000, 0xe0d40003).rw(m_scc, FUNC(z80scc_device::ab_dc_r), FUNC(z80scc_device::ab_dc_w));
	map(0xe0d80000, 0xe0d807ff).rw(m_rtc, FUNC(m48t02_device::read), FUNC(m48t02_device::write));
	map(0xe0dc0000, 0xe0dc0000).lw8([this](u8 data) { m_led[0] = BIT(data, 0); m_led[1] = BIT(data, 1); }, "led_w");

	map(0xe0e00000, 0xe0e03fff).lrw16(
		[this](offs_t offset) { return m_net_ram[offset]; }, "net_ram_r",
		[this](offs_t offset, u16 data, u16 mem_mask) { COMBINE_DATA(&m_net_ram[offset]); }, "net_ram_w");
	map(0xe0e80000, 0xe0e80017).m(m_dma, FUNC(dmac_0266_device::map));
	// e0ec0000 // sound board
	map(0xe0f00000, 0xe0f00003).rw(m_net, FUNC(am7990_device::regs_r), FUNC(am7990_device::regs_w));
	// e0f40000
	//map(0xe0f40000, 0xe0f40000).lr8([]() { return 0xfb; }, "scc_ridsr_r");

	map(0xe1000000, 0xe1000000).w(FUNC(news_68k_desktop_state::timer_w));
	map(0xe1080000, 0xe1080000).lw8([this](u8 data) { LOG("parity check enable 0x%02x\n", data); }, "parity_check_enable_w");
	map(0xe1180000, 0xe1180000).lw8([this](u8 data) { m_cpu->set_input_line(INPUT_LINE_IRQ2, bool(data)); }, "irq2_w");
	map(0xe1200000, 0xe1200000).lw8([this](u8 data) { m_cpu->space(0).install_ram(0, m_ram->mask(), 0xc0000000, m_ram->pointer()); }, "ram_enable");
	map(0xe1280000, 0xe1280000).lw8([this](u8 data) { m_cpu->set_input_line(INPUT_LINE_IRQ1, bool(data)); }, "ast_w");
	map(0xe1300000, 0xe1300000).lw8([this](u8 data) { LOG("cache enable 0x%02x (%s)\n", data, machine().describe_context()); }, "cache_enable_w");
	// 0xe1380000 // power on/off
	map(0xe1900000, 0xe1900000).lw8([this](u8 data) { LOG("cache clear 0x%02x\n", data); }, "cache_clear_w");
	map(0xe1a00000, 0xe1a00000).lw8([this](u8 data) { LOG("parity interrupt clear 0x%02x\n", data); }, "parity_interrupt_clear_w");
	// 0xe1b00000 // fdc vfo external/internal
	map(0xe1c00000, 0xe1c000ff).rom().region("idrom", 0);
	map(0xe1c00100, 0xe1c00103).lr8([this]() { LOG("Read sw1 = 0x%x\n", m_sw1->read()); return u8(m_sw1->read()); }, "sw1_r");
	// HACK: disable fdc irq for NetBSD
	map(0xe1c00200, 0xe1c00200).lrw8([this]() { return m_intst; }, "intst_r", [this](u8 data) { irq_w<FDC>(0); m_parity_vector = data; }, "parity_vector_w");

	// external I/O
	map(0xf0000000, 0xffffffff).r(FUNC(news_68k_desktop_state::bus_error_r));
#if DESKTOP_GRAPHICS
	// POPC
	//map(0xf0fc0000, 0xf0fc0003).unmaprw();
	// f0fc0000 & 0x40 == 0x00 -> popm
	// f0fc0000 & 0xc0 == 0xc0 -> popc
	map(0xf0fc0000, 0xf0fc0001).lr16([]() {return 0x00c0; }, "popc_probe"); // lower 2 bits give busy state
	map(0xf0fc4000, 0xf0fc4007).m(m_ramdac, FUNC(bt458_device::map)).umask32(0x00ff00ff);

	//map(0xf0fc0000, 0xf10bffff).rom().region("krom", 0);
#endif

	// 0xf0c30000 expansion lance #1
	// 0xf0c20000   lance #1 memory
	// 0xf0c38000   lance #1 etherid
	// 0xf0c70000 expansion lance #2
	// 0xf0c60000   lance #2 memory
	// 0xf0c78000   lance #2 etherid

	// 0xf0d04000 isdn?

	// 0xf0f00000 nwb512_base
	// 0xf0fc0000 nwb512krom_base
	// 0xf0700000 nwb225_base
	// 0xf0600000 nwb225krom_base
}

void news_68k_laptop_state::laptop_cpu_map(address_map &map)
{
	cpu_map(map);

	map(0xe0000000, 0xe001ffff).rom().region("eprom", 0);

	map(0xe1000000, 0xe1000000).w(FUNC(news_68k_laptop_state::poweron_w));

	map(0xe1240000, 0xe1240007).m(m_hid, FUNC(news_hid_hle_device::map_nws12xx_keyboard));
	map(0xe1280000, 0xe1280007).m(m_hid, FUNC(news_hid_hle_device::map_nws12xx_mouse));
	map(0xe1400000, 0xe14000ff).rom().region("idrom", 0);
	map(0xe1420000, 0xe14207ff).rw(m_rtc, FUNC(m48t02_device::read), FUNC(m48t02_device::write));
	map(0xe1680000, 0xe1680000).lr8([this] { return u8(m_sw1->read()); }, "sw1_r");
	map(0xe1780000, 0xe1780003).rw(m_scc, FUNC(z80scc_device::ab_dc_r), FUNC(z80scc_device::ab_dc_w));

	// // above this line is 100% legit
	map(0xe1040000, 0xe1040000).lw8([this](u8 data) { m_cpu->space(0).install_ram(0, m_ram->mask(), m_ram->pointer()); }, "ram_enable"); // guess
	map(0xe1080000, 0xe1080000); // TODO: random theory to investigate later: is this the memory controller? If I add more than 8MB of memory, does the written value the second time change?

	map(0xe1200000, 0xe1200000).lr8([this] { return m_intst; }, "intst_r"); // TODO: make sure this is accurate by commenting it out and trying to use something it has status for

	map(0xe2000000, 0xe20fffff).rom().region("krom", 0);
	map(0xe4000000, 0xe401ffff).ram().share("vram");

	map(0xe1580000, 0xe1580007).m(m_fdc, FUNC(n82077aa_device::map));
	map(0xe15c0100, 0xe15c0100).rw(m_fdc, FUNC(n82077aa_device::dma_r), FUNC(n82077aa_device::dma_w));

	map(0xe1a00000, 0xe1a03fff).lrw16(
	[this](offs_t offset) { return m_net_ram[offset]; }, "net_ram_r",
	[this](offs_t offset, u16 data, u16 mem_mask) { COMBINE_DATA(&m_net_ram[offset]); }, "net_ram_w");
	map(0xe1a40000, 0xe1a40003).rw(m_net, FUNC(am7990_device::regs_r), FUNC(am7990_device::regs_w));

	map(0xe1c00000, 0xe1c00017).m(m_dma, FUNC(dmac_0266_device::map));
	map(0xe1900000, 0xe190000f).m(m_scsi, FUNC(cxd1185_device::map));

	map(0xe11c0000, 0xe11c000f); // TODO: abortctl?

	// above this line is 50% legit

	// below this line is wrong or unverified

	// e1500001 = LED control?
	map(0xe1500000, 0xe1500000).lw8([this] (u32 data) { LOG("(%s) Write unknown 1 = 0x%x\n", machine().describe_context(), data); }, "unknown_1_w");
	map(0xe1500001, 0xe1500001).lw8([this] (u32 data) { LOG("(%s) Write unknown 2 = 0x%x\n", machine().describe_context(), data); }, "unknown_2_w");
	map(0xe1500002, 0xe1500002).lw8([this] (u32 data) { m_lcd_enable = bool(data); LOG("(%s) %s LCD\n", machine().describe_context(), m_lcd_enable ? "Enabled" : "Disabled"); }, "lcd_enable_w");
	// WRONG: map(0xe1a00000, 0xe1a00003).lw32([this] (u32 data) { m_lcd_dim = BIT(data, 0); }, "lcd_dim_w");
	map(0xe1480000, 0xe148001b).lr8([this] (offs_t offset) { LOG("%s crtc read offset %x\n", machine().describe_context(), offset); return 0xff; }, "lfbm_crtc_r");
	map(0xe1480000, 0xe148001b).lw8([this] (offs_t offset, u8 data) { LOG("crtc offset %x 0x%02x\n", offset, data); }, "lfbm_crtc_w");

	// 0xe0c40000 // centronics

	// map(0xe0dc0000, 0xe0dc0000).lw8([this](u8 data) { m_led[0] = BIT(data, 0); m_led[1] = BIT(data, 1); }, "led_w");
	//
	// //map(0xe0f40000, 0xe0f40000).lr8([]() { return 0xfb; }, "scc_ridsr_r");
	//
	// map(0xe1000000, 0xe1000000).w(FUNC(news_68k_laptop_state::timer_w));
	// map(0xe1080000, 0xe1080000).lw8([this](u8 data) { LOG("parity check enable 0x%02x\n", data); }, "parity_check_enable_w");
	// map(0xe1180000, 0xe1180000).lw8([this](u8 data) { m_cpu->set_input_line(INPUT_LINE_IRQ2, bool(data)); }, "irq2_w");
	// map(0xe1200000, 0xe1200000).lw8([this](u8 data) { m_cpu->space(0).install_ram(0, m_ram->mask(), 0xc0000000, m_ram->pointer()); }, "ram_enable");
	// map(0xe1280000, 0xe1280000).lw8([this](u8 data) { m_cpu->set_input_line(INPUT_LINE_IRQ1, bool(data)); }, "ast_w");
	// map(0xe1300000, 0xe1300000).lw8([this](u8 data) { LOG("cache enable 0x%02x (%s)\n", data, machine().describe_context()); }, "cache_enable_w");
	// // 0xe1380000 // power on/off
	// map(0xe1900000, 0xe1900000).lw8([this](u8 data) { LOG("cache clear 0x%02x\n", data); }, "cache_clear_w");
	// map(0xe1a00000, 0xe1a00000).lw8([this](u8 data) { LOG("parity interrupt clear 0x%02x\n", data); }, "parity_interrupt_clear_w");
	// // 0xe1b00000 // fdc vfo external/internal
	//
	// // external I/O
	map(0xf0000000, 0xffffffff).r(FUNC(news_68k_laptop_state::bus_error_r));
}

void news_68k_base_state::cpu_autovector_map(address_map &map)
{
	map(0xfffffff3, 0xfffffff3).lr8(NAME([]() { return m68000_base_device::autovector(1); }));
	map(0xfffffff5, 0xfffffff5).lr8(NAME([]() { return m68000_base_device::autovector(2); }));
	map(0xfffffff7, 0xfffffff7).lr8(NAME([]() { return m68000_base_device::autovector(3); }));
	map(0xfffffff9, 0xfffffff9).lr8(NAME([]() { return m68000_base_device::autovector(4); }));
	map(0xfffffffb, 0xfffffffb).lr8(NAME([this]() { return m_scc_irq_state ? m_scc->m1_r() : m68000_base_device::autovector(5); }));
	map(0xfffffffd, 0xfffffffd).lr8(NAME([]() { return m68000_base_device::autovector(6); }));
	map(0xffffffff, 0xffffffff).lr8(NAME([this]() { return m_parity_irq_state ? m_parity_vector : m68000_base_device::autovector(7); }));
}

template <news_68k_base_state::irq_number Number> void news_68k_base_state::irq_w(int state)
{
	LOG("irq number %d state %d\n", Number, state);

	if (state)
		m_intst |= 1U << Number;
	else
		m_intst &= ~(1U << Number);

	int_check();
}

void news_68k_base_state::int_check()
{
	// TODO: assume 43334443, masking?
	static int const int_line[] = { INPUT_LINE_IRQ3, INPUT_LINE_IRQ4 };
	static u8 const int_mask[] = { 0x71, 0x8e };

	for (unsigned i = 0; i < std::size(m_int_state); i++)
	{
		bool const int_state = m_intst & int_mask[i];

		if (m_int_state[i] != int_state)
		{
			m_int_state[i] = int_state;
			m_cpu->set_input_line(int_line[i], int_state);
		}
	}
}

void news_68k_base_state::timer_w(u8 data)
{
	LOG("timer_w 0x%02x\n", data);

	if (data)
		m_timer->adjust(attotime::from_hz(100));
	else
		m_cpu->set_input_line(INPUT_LINE_IRQ6, CLEAR_LINE);
}

void news_68k_base_state::timer(s32 param)
{
	m_cpu->set_input_line(INPUT_LINE_IRQ6, ASSERT_LINE);
}

u32 news_68k_base_state::bus_error_r()
{
	if (!machine().side_effects_disabled())
		m_cpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);

	return 0;
}

void news_68k_base_state::poweron_w(u8 data)
{
	LOG("(%s) Write POWERON = 0x%x\n", machine().describe_context(), data);

	if (!machine().side_effects_disabled() && !data)
	{
		machine().schedule_exit();
	}
}

u32 news_68k_laptop_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect)
{
	if (!m_lcd_enable)
		return 0;

	rgb_t const black = rgb_t::black();
	rgb_t const white = m_lcd_dim ? rgb_t(191, 191, 191) : rgb_t::white();

	u32 const *pixel_pointer = m_vram;

	for (int y = screen.visible_area().min_y; y <= screen.visible_area().max_y; y++)
	{
		for (int x = screen.visible_area().min_x; x <= screen.visible_area().max_x; x += 32)
		{
			u32 const pixel_data = *pixel_pointer++;

			for (unsigned i = 0; i < 32; i++)
				bitmap.pix(y, x + i) = BIT(pixel_data, 31 - i) ? black : white;
		}
	}

	return 0;
}

static void news_scsi_devices(device_slot_interface &device)
{
	device.option_add("harddisk", NSCSI_HARDDISK);
	device.option_add("cdrom", NSCSI_CDROM);
}

void news_68k_base_state::common(machine_config &config)
{
	M48T02(config, m_rtc);

	DMAC_0266(config, m_dma, 0);
	m_dma->set_bus(m_cpu, 0);

	INPUT_MERGER_ANY_HIGH(config, m_irq5);
	m_irq5->output_handler().set_inputline(m_cpu, INPUT_LINE_IRQ5);

	AM7990(config, m_net);
	m_net->intr_out().set(FUNC(news_68k_base_state::irq_w<LANCE>)).invert();
	m_net->dma_in().set([this](offs_t offset) { return m_net_ram[(offset >> 1) & 0x1fff]; });
	m_net->dma_out().set([this](offs_t offset, u16 data, u16 mem_mask) { COMBINE_DATA(&m_net_ram[(offset >> 1) & 0x1fff]); });

	// scsi bus and devices
	NSCSI_BUS(config, "scsi");

	/*
	 * NWS-1580:
	 * CDC WREN V HH 94221-5 (5.25" half-height SCSI-1 single-ended)
	 * 1544 cylinders, 5 heads, 52 sectors/cylinder, ~170MiB formatted
	 *
	 * Vendor   Product          Rev. Vendor-specific
	 * CDC      94221-5          5457 00018715
	 */
	NSCSI_CONNECTOR(config, "scsi:0", news_scsi_devices, "harddisk");
	NSCSI_CONNECTOR(config, "scsi:1", news_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:2", news_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:3", news_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:4", news_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:5", news_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:6", news_scsi_devices, nullptr);

	NEWS_HID_HLE(config, m_hid);

	SOFTWARE_LIST(config, "software_list").set_original("sony_news").set_filter("CISC");
}

void news_68k_base_state::config_scc(machine_config &config, const char *default_device_name)
{
	SCC85C30(config, m_scc, 4'915'200); // 3993600); TODO: configure clock as well
	m_scc->out_int_callback().set(
		[this](int state)
		{
			m_scc_irq_state = bool(state);
			m_irq5->in_w<2>(state);
		});

	// scc channel A
	RS232_PORT(config, m_serial[0], default_rs232_devices, default_device_name);
	m_serial[0]->cts_handler().set(m_scc, FUNC(z80scc_device::ctsa_w));
	m_serial[0]->dcd_handler().set(m_scc, FUNC(z80scc_device::dcda_w));
	m_serial[0]->rxd_handler().set(m_scc, FUNC(z80scc_device::rxa_w));
	m_scc->out_rtsa_callback().set(m_serial[0], FUNC(rs232_port_device::write_rts));
	m_scc->out_txda_callback().set(m_serial[0], FUNC(rs232_port_device::write_txd));

	// scc channel B
	RS232_PORT(config, m_serial[1], default_rs232_devices, nullptr);
	m_serial[1]->cts_handler().set(m_scc, FUNC(z80scc_device::ctsb_w));
	m_serial[1]->dcd_handler().set(m_scc, FUNC(z80scc_device::dcdb_w));
	m_serial[1]->rxd_handler().set(m_scc, FUNC(z80scc_device::rxb_w));
	m_scc->out_rtsb_callback().set(m_serial[1], FUNC(rs232_port_device::write_rts));
	m_scc->out_txdb_callback().set(m_serial[1], FUNC(rs232_port_device::write_txd));
}

void news_68k_desktop_state::nws1580(machine_config &config)
{
	M68030(config, m_cpu, 50_MHz_XTAL / 2);
	m_cpu->set_addrmap(AS_PROGRAM, &news_68k_desktop_state::desktop_cpu_map);
	m_cpu->set_addrmap(m68000_base_device::AS_CPU_SPACE, &news_68k_desktop_state::cpu_autovector_map);

	// 16 SIMM slots for RAM arranged as two groups of 8 slots, with each bank
	// corresponding to a pair of slots in each group; first bank soldered in
	RAM(config, m_ram);
	m_ram->set_default_size("8M");
	// TODO: assume only 1M modules are supported
	m_ram->set_extra_options("4M,12M,16M");
	m_ram->set_default_value(0);

	common(config);
	config_scc(config, "terminal");

	INPUT_MERGER_ANY_HIGH(config, m_irq7);
	m_irq7->output_handler().set_inputline(m_cpu, INPUT_LINE_IRQ7);

	UPD72067(config, m_fdc, 16_MHz_XTAL);
	m_fdc->intrq_wr_callback().set(FUNC(news_68k_desktop_state::irq_w<FDC>));
	m_fdc->drq_wr_callback().set(m_irq7, FUNC(input_merger_device::in_w<0>));
	FLOPPY_CONNECTOR(config, "fdc:0", "35hd", FLOPPY_35_HD, true, floppy_image_device::default_pc_floppy_formats).enable_sound(false);

	// scsi host adapter
	NSCSI_CONNECTOR(config, "scsi:7").option_set("cxd1180", CXD1180).machine_config(
		[this](device_t *device)
		{
			auto &adapter = downcast<cxd1180_device &>(*device);

			adapter.irq_handler().set(*this, FUNC(news_68k_desktop_state::irq_w<SCSI>));
			adapter.irq_handler().append(m_dma, FUNC(dmac_0266_device::eop_w));
			adapter.drq_handler().set(m_dma, FUNC(dmac_0266_device::req_w));

			subdevice<dmac_0266_device>(":dma")->dma_r_cb().set(adapter, FUNC(cxd1180_device::dma_r));
			subdevice<dmac_0266_device>(":dma")->dma_w_cb().set(adapter, FUNC(cxd1180_device::dma_w));
		});

#if DESKTOP_GRAPHICS
	m_hid->irq_out<news_hid_hle_device::KEYBOARD>().set(m_irq5, FUNC(input_merger_device::in_w<0>));
	m_hid->irq_out<news_hid_hle_device::MOUSE>().set(m_irq5, FUNC(input_merger_device::in_w<1>));

	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_raw(64.0_MHz_XTAL, 1024, 0, 1024, 768, 0, 768);
	m_screen->set_screen_update(FUNC(news_68k_base_state::screen_update));

	// AM81C458-80JC
	BT458(config, m_ramdac, 64.0_MHz_XTAL);

	// 32 x MB81461-12 (256Kbit ZIP VRAM)
	RAM(config, m_vram);
	m_vram->set_default_size("1MiB");
	m_vram->set_default_value(0);
#endif
}

void news_68k_laptop_state::nws1250(machine_config &config)
{
	M68030(config, m_cpu, 50_MHz_XTAL / 2);
	m_cpu->set_addrmap(AS_PROGRAM, &news_68k_laptop_state::laptop_cpu_map);
	m_cpu->set_addrmap(m68000_base_device::AS_CPU_SPACE, &news_68k_laptop_state::cpu_autovector_map);

	RAM(config, m_ram);
	m_ram->set_default_size("8M");
	m_ram->set_extra_options("4M,12M");
	m_ram->set_default_value(0);

	common(config);
	config_scc(config, nullptr);

	INPUT_MERGER_ANY_HIGH(config, m_irq7);
	m_irq7->output_handler().set_inputline(m_cpu, INPUT_LINE_IRQ7);

	N82077AA(config, m_fdc, 16_MHz_XTAL);
	m_fdc->intrq_wr_callback().set(FUNC(news_68k_laptop_state::irq_w<FDC>));
	m_fdc->drq_wr_callback().set(m_irq7, FUNC(input_merger_device::in_w<0>));
	FLOPPY_CONNECTOR(config, "fdc:0", "35hd", FLOPPY_35_HD, true, floppy_image_device::default_pc_floppy_formats).enable_sound(false);

	// scsi host adapter
	NSCSI_CONNECTOR(config, "scsi:7").option_set("cxd1185", CXD1185).machine_config(
		[this](device_t *device)
		{
			auto &adapter = downcast<cxd1185_device &>(*device);

			adapter.irq_out_cb().set(*this, FUNC(news_68k_laptop_state::irq_w<SCSI>));
			adapter.irq_out_cb().append(m_dma, FUNC(dmac_0266_device::eop_w));
			adapter.drq_out_cb().set(m_dma, FUNC(dmac_0266_device::req_w));

			subdevice<dmac_0266_device>(":dma")->dma_r_cb().set(adapter, FUNC(cxd1185_device::dma_r));
			subdevice<dmac_0266_device>(":dma")->dma_w_cb().set(adapter, FUNC(cxd1185_device::dma_w));
		});

	// Integrated LCD panel
	SCREEN(config, m_lcd, SCREEN_TYPE_LCD);
	m_lcd->set_raw(52416000, 1120, 0, 1120, 780, 0, 780);
	m_lcd->set_screen_update(FUNC(news_68k_laptop_state::screen_update));
}

static INPUT_PORTS_START(nws15x0)
	PORT_START("SW1")
	PORT_DIPNAME(0x07, 0x07, "Display") PORT_DIPLOCATION("SW1:1,2,3")
	PORT_DIPSETTING(0x07, "Console")
	PORT_DIPSETTING(0x06, "NWB-512")
	PORT_DIPSETTING(0x03, "NWB-225A")
	PORT_DIPSETTING(0x00, "Autoselect")

	PORT_DIPNAME(0x08, 0x08, "Boot Device") PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(0x08, "SCSI")
	PORT_DIPSETTING(0x00, "Floppy")

	PORT_DIPNAME(0x10, 0x10, "Automatic Boot") PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(0x10, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))

	PORT_DIPNAME(0x20, 0x20, "Diagnostic Mode") PORT_DIPLOCATION("SW1:6")
	PORT_DIPSETTING(0x20, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))

	PORT_DIPUNUSED_DIPLOC(0xc0, 0xc0, "SW1:7,8")
INPUT_PORTS_END

static INPUT_PORTS_START(nws12x0)
	PORT_START("SW1")
	PORT_DIPNAME(0x07, 0x02, "Display") PORT_DIPLOCATION("SW1:1,2,3")
	PORT_DIPSETTING(0x07, "Console")
	PORT_DIPSETTING(0x02, "LCD")
	PORT_DIPSETTING(0x00, "Autoselect")

	PORT_DIPNAME(0x08, 0x08, "Boot Device") PORT_DIPLOCATION("SW1:4")
	PORT_DIPSETTING(0x08, "SCSI")
	PORT_DIPSETTING(0x00, "Floppy")

	PORT_DIPNAME(0x10, 0x10, "Automatic Boot") PORT_DIPLOCATION("SW1:5")
	PORT_DIPSETTING(0x10, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))

	PORT_DIPNAME(0x20, 0x20, "Diagnostic Mode") PORT_DIPLOCATION("SW1:6")
	PORT_DIPSETTING(0x20, DEF_STR(Off))
	PORT_DIPSETTING(0x00, DEF_STR(On))

	PORT_DIPUNUSED_DIPLOC(0xc0, 0xc0, "SW1:7,8")
INPUT_PORTS_END

ROM_START(nws1250)
	ROM_REGION32_BE(0x20000, "eprom", 0)
	ROM_SYSTEM_BIOS(0, "nws1250-20", "SONY NET WORK STATION MC68030 Monitor Release 2.0")
	ROMX_LOAD("nws1200_9006_am27c1024.bin", 0x00000, 0x20000, CRC(0b836746) SHA1(0dd7ed246c203646747eb99a72e3b91bb702796c), ROM_BIOS(0) | ROM_GROUPWORD | ROM_REVERSE)
	ROM_SYSTEM_BIOS(1, "nws1250-20a", "SONY NET WORK STATION MC68030 Monitor Release 2.0A")
	ROMX_LOAD("nws-1200_ver_2.0a_9010.ic2", 0x00000, 0x20000, CRC(87eca9d2) SHA1(235585a55bc2b3206cfec532852526a638eccad2), ROM_BIOS(1) | ROM_GROUPWORD | ROM_REVERSE)

	// AM27S21PC PROM
	ROM_REGION32_BE(0x100, "idrom", 0)
	//ROM_LOAD("n1250_50292_am27s21pc.ic36", 0x000, 0x100, NO_DUMP)
	ROM_LOAD("idrom.bin", 0x000, 0x100, CRC(8cf47e35) SHA1(3eef8168ffb8f7879bcbac9e8fee2115a191ae83) BAD_DUMP)

	// 2 x MB834200A (mask ROM)
	ROM_REGION32_BE(0x100000, "krom", ROMREGION_ERASEFF)
	// ROM_LOAD64_BYTE("mb834200a-20_051_aa_9020_g07.ic1",  0x00000, 0x20000, NO_DUMP)
	// ROM_LOAD64_BYTE("mb834200a-20_052_aa_9002_g02.ic13", 0x00001, 0x20000, NO_DUMP)
	ROM_LOAD32_DWORD("mb834200b_u44.bin", 0x00000, 0x80000, CRC(6a50162a) SHA1(92383c3ad7aaa7b2f9c8cf781c6dcddffe7b9af8))
	ROM_LOAD32_DWORD("mb834200b_u45.bin", 0x80000, 0x80000, CRC(f2886c9b) SHA1(76363bb7ef884bcf51c50ac56963d513fe776c2e))
ROM_END

ROM_START(nws1580)
	ROM_REGION32_BE(0x10000, "eprom", 0)
	ROM_SYSTEM_BIOS(0, "nws1580", "SONY NET WORK STATION MC68030 Monitor Release 1.3")
	ROMX_LOAD("pws-1500__ver_1.3__8906.bin", 0x00000, 0x10000, CRC(76395ad9) SHA1(c2ae00218c23cef6519a4d7c74ac2c552790dfd4), ROM_BIOS(0))

	// MB7114 256x4 TTL PROM
	ROM_REGION32_BE(0x100, "idrom", 0)
	ROM_LOAD("n1580_50093.ic63", 0x000, 0x100, CRC(a7f293d6) SHA1(21deffed69e07af515ffc5511bdbf73a2a4c14fb))

	// 2 x HN62321BP (128K x 8-bit mask ROM)
	ROM_REGION32_BE(0x100000, "krom", ROMREGION_ERASEFF)
	ROM_LOAD64_BYTE("aa1.ic14", 0x00000, 0x20000, CRC(db274954) SHA1(4bc9b8a862ce9bdbf43c70f84921253876e21e58))
	ROM_LOAD64_BYTE("aa2.ic15", 0x00001, 0x20000, CRC(0d7686c7) SHA1(b0be18166b4690518e6a11ea194cc1c7a1ea6347))
ROM_END

} // anonymous namespace


//   YEAR  NAME     PARENT  COMPAT  MACHINE  INPUT    CLASS                   INIT         COMPANY  FULLNAME    FLAGS
COMP(1988, nws1580, 0,      0,      nws1580, nws15x0, news_68k_desktop_state, init_common, "Sony",  "NWS-1580", MACHINE_NOT_WORKING)
COMP(1990, nws1250, 0,      0,      nws1250, nws12x0, news_68k_laptop_state,  init_common, "Sony",  "NWS-1250", MACHINE_NOT_WORKING)
