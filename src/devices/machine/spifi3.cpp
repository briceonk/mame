// license:BSD-3-Clause
// copyright-holders:Brice Onken,Olivier Galibert

/*
 * HP 1TV3-0302 SPIFI3-SE SCSI controller
 *
 * References:
 * - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/spifireg.h
 * - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/spifi.c
 *
 * TODO: Everything. This is a placeholder that only logs read/writes.
 */

#include "emu.h"
#include "spifi3.h"

#define VERBOSE 1
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SPIFI3, spifi3_device, "spifi3", "HP 1TV3-0302 SPIFI3 SCSI-2 Protocol Controller")

spifi3_device::spifi3_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: nscsi_device(mconfig, SPIFI3, tag, owner, clock)
	, nscsi_slot_card_interface(mconfig, *this, DEVICE_SELF)
{
}

void spifi3_device::map(address_map &map)
{
	// Ugly address map
	map(0x00, 0x03).lrw32(NAME([this]() { LOG("read spifi_reg.spstat = 0x%x\n", spifi_reg.spstat); return spifi_reg.spstat; }), NAME([this](uint32_t data) { LOG("write spifi_reg.spstat = 0x%x\n", data); spifi_reg.spstat = data; }));
	map(0x04, 0x07).lrw32(NAME([this]() { LOG("read spifi_reg.cmlen = 0x%x\n", spifi_reg.cmlen); return spifi_reg.cmlen; }), NAME([this](uint32_t data) { LOG("write spifi_reg.cmlen = 0x%x\n", data); spifi_reg.cmlen = data; }));
	map(0x08, 0x0b).lrw32(NAME([this]() { LOG("read spifi_reg.cmdpage = 0x%x\n", spifi_reg.cmdpage); return spifi_reg.cmdpage; }), NAME([this](uint32_t data) { LOG("write spifi_reg.cmdpage = 0x%x\n", data); spifi_reg.cmdpage = data; }));
	map(0x0c, 0x0f).lrw32(NAME([this]() { LOG("read spifi_reg.count_hi = 0x%x\n", spifi_reg.count_hi); return spifi_reg.count_hi; }), NAME([this](uint32_t data) { LOG("write spifi_reg.count_hi = 0x%x\n", data); spifi_reg.count_hi = data; }));
	map(0x10, 0x13).lrw32(NAME([this]() { LOG("read spifi_reg.count_mid = 0x%x\n", spifi_reg.count_mid); return spifi_reg.count_mid; }), NAME([this](uint32_t data) { LOG("write spifi_reg.count_mid = 0x%x\n", data); spifi_reg.count_mid = data; }));
	map(0x14, 0x17).lrw32(NAME([this]() { LOG("read spifi_reg.count_low = 0x%x\n", spifi_reg.count_low); return spifi_reg.count_low; }), NAME([this](uint32_t data) { LOG("write spifi_reg.count_low = 0x%x\n", data); spifi_reg.count_low = data; }));
	map(0x18, 0x1b).lrw32(NAME([this]() { LOG("read spifi_reg.svptr_hi = 0x%x\n", spifi_reg.svptr_hi); return spifi_reg.svptr_hi; }), NAME([this](uint32_t data) { LOG("write spifi_reg.svptr_hi = 0x%x\n", data); spifi_reg.svptr_hi = data; }));
	map(0x1c, 0x1f).lrw32(NAME([this]() { LOG("read spifi_reg.svptr_mid = 0x%x\n", spifi_reg.svptr_mid); return spifi_reg.svptr_mid; }), NAME([this](uint32_t data) { LOG("write spifi_reg.svptr_mid = 0x%x\n", data); spifi_reg.svptr_mid = data; }));
	map(0x20, 0x23).lrw32(NAME([this]() { LOG("read spifi_reg.svptr_low = 0x%x\n", spifi_reg.svptr_low); return spifi_reg.svptr_low; }), NAME([this](uint32_t data) { LOG("write spifi_reg.svptr_low = 0x%x\n", data); spifi_reg.svptr_low = data; }));
	map(0x24, 0x27).lrw32(NAME([this]() { LOG("read spifi_reg.intr = 0x%x (%s)\n", spifi_reg.intr, machine().describe_context()); return spifi_reg.intr; }), NAME([this](uint32_t data) { LOG("write spifi_reg.intr = 0x%x\n", data); spifi_reg.intr = data; }));
	map(0x28, 0x2b).lrw32(NAME([this]() { LOG("read spifi_reg.imask = 0x%x\n", spifi_reg.imask); return spifi_reg.imask; }), NAME([this](uint32_t data) { LOG("write spifi_reg.imask = 0x%x\n", data); spifi_reg.imask = data; }));
	map(0x2c, 0x2f).lrw32(NAME([this]() { LOG("read spifi_reg.prctrl = 0x%x\n", spifi_reg.prctrl); return spifi_reg.prctrl; }), NAME([this](uint32_t data) { LOG("write spifi_reg.prctrl = 0x%x\n", data); spifi_reg.prctrl = data; }));
	map(0x30, 0x33).lrw32(NAME([this]() { LOG("read spifi_reg.prstat = 0x%x\n", spifi_reg.prstat); return spifi_reg.prstat; }), NAME([this](uint32_t data) { LOG("write spifi_reg.prstat = 0x%x\n", data); spifi_reg.prstat = data; }));
	map(0x34, 0x37).lrw32(NAME([this]() { LOG("read spifi_reg.init_status = 0x%x\n", spifi_reg.init_status); return spifi_reg.init_status; }), NAME([this](uint32_t data) { LOG("write spifi_reg.init_status = 0x%x\n", data); spifi_reg.init_status = data; }));
	map(0x3c, 0x3f).lrw32(NAME([this]() { LOG("read spifi_reg.fifodata = 0x%x\n", spifi_reg.fifodata); return spifi_reg.fifodata; }), NAME([this](uint32_t data) { LOG("write spifi_reg.fifodata = 0x%x\n", data); spifi_reg.fifodata = data; }));
	map(0x44, 0x47).lrw32(NAME([this]() { LOG("read spifi_reg.data_xfer = 0x%x\n", spifi_reg.data_xfer); return spifi_reg.data_xfer; }), NAME([this](uint32_t data) { LOG("write spifi_reg.data_xfer = 0x%x\n", data); spifi_reg.data_xfer = data; }));
	map(0x48, 0x4b).lrw32(NAME([this]() { LOG("read spifi_reg.autocmd = 0x%x\n", spifi_reg.autocmd); return spifi_reg.autocmd; }), NAME([this](uint32_t data) { LOG("write spifi_reg.autocmd = 0x%x\n", data); spifi_reg.autocmd = data; }));
	map(0x4c, 0x4f).lrw32(NAME([this]() { LOG("read spifi_reg.autostat = 0x%x\n", spifi_reg.autostat); return spifi_reg.autostat; }), NAME([this](uint32_t data) { LOG("write spifi_reg.autostat = 0x%x\n", data); spifi_reg.autostat = data; }));
	map(0x50, 0x53).lrw32(NAME([this]() { LOG("read spifi_reg.resel = 0x%x\n", spifi_reg.resel); return spifi_reg.resel; }), NAME([this](uint32_t data) { LOG("write spifi_reg.resel = 0x%x\n", data); spifi_reg.resel = data; }));
	map(0x58, 0x5b).lrw32(NAME([this]() { LOG("read spifi_reg.prcmd = 0x%x\n", spifi_reg.prcmd); return spifi_reg.prcmd; }), NAME([this](uint32_t data) { LOG("write spifi_reg.prcmd = 0x%x\n", data); spifi_reg.prcmd = data; }));
	map(0x64, 0x67).lrw32(NAME([this]() { LOG("read spifi_reg.loopctrl = 0x%x\n", spifi_reg.loopctrl); return spifi_reg.loopctrl; }), NAME([this](uint32_t data) { LOG("write spifi_reg.loopctrl = 0x%x\n", data); spifi_reg.loopctrl = data; }));
	map(0x68, 0x6b).lrw32(NAME([this]() { LOG("read spifi_reg.loopdata = 0x%x\n", spifi_reg.loopdata); return spifi_reg.loopdata; }), NAME([this](uint32_t data) { LOG("write spifi_reg.loopdata = 0x%x\n", data); spifi_reg.loopdata = data; }));
	map(0x6c, 0x6f).lrw32(NAME([this]() { LOG("read spifi_reg.identify = 0x%x\n", spifi_reg.identify); return spifi_reg.identify; }), NAME([this](uint32_t data) { LOG("write spifi_reg.identify = 0x%x\n", data); spifi_reg.identify = data; }));
	map(0x70, 0x73).lrw32(NAME([this]() { LOG("read spifi_reg.complete = 0x%x\n", spifi_reg.complete); return spifi_reg.complete; }), NAME([this](uint32_t data) { LOG("write spifi_reg.complete = 0x%x\n", data); spifi_reg.complete = data; }));
	map(0x74, 0x77).lrw32(NAME([this]() { LOG("read spifi_reg.scsi_status = 0x%x\n", spifi_reg.scsi_status); return spifi_reg.scsi_status; }), NAME([this](uint32_t data) { LOG("write spifi_reg.scsi_status = 0x%x\n", data); spifi_reg.scsi_status = data; }));
	map(0x78, 0x7b).lrw32(NAME([this]() { LOG("read spifi_reg.data = 0x%x\n", spifi_reg.data); return spifi_reg.data; }), NAME([this](uint32_t data) { LOG("write spifi_reg.data = 0x%x\n", data); spifi_reg.data = data; }));
	map(0x7c, 0x7f).lrw32(NAME([this]() { LOG("read spifi_reg.icond = 0x%x\n", spifi_reg.icond); return spifi_reg.icond; }), NAME([this](uint32_t data) { LOG("write spifi_reg.icond = 0x%x\n", data); spifi_reg.icond = data; }));
	map(0x80, 0x83).lrw32(NAME([this]() { LOG("read spifi_reg.fastwide = 0x%x\n", spifi_reg.fastwide); return spifi_reg.fastwide; }), NAME([this](uint32_t data) { LOG("write spifi_reg.fastwide = 0x%x\n", data); spifi_reg.fastwide = data; }));
	map(0x84, 0x87).lrw32(NAME([this]() { LOG("read spifi_reg.exctrl = 0x%x\n", spifi_reg.exctrl); return spifi_reg.exctrl; }), NAME([this](uint32_t data) { LOG("write spifi_reg.exctrl = 0x%x\n", data); spifi_reg.exctrl = data; }));
	map(0x88, 0x8b).lrw32(NAME([this]() { LOG("read spifi_reg.exstat = 0x%x\n", spifi_reg.exstat); return spifi_reg.exstat; }), NAME([this](uint32_t data) { LOG("write spifi_reg.exstat = 0x%x\n", data); spifi_reg.exstat = data; }));
	map(0x8c, 0x8f).lrw32(NAME([this]() { LOG("read spifi_reg.test = 0x%x\n", spifi_reg.test); return spifi_reg.test; }), NAME([this](uint32_t data) { LOG("write spifi_reg.test = 0x%x\n", data); spifi_reg.test = data; }));
	map(0x90, 0x93).lrw32(NAME([this]() { LOG("read spifi_reg.quematch = 0x%x\n", spifi_reg.quematch); return spifi_reg.quematch; }), NAME([this](uint32_t data) { LOG("write spifi_reg.quematch = 0x%x\n", data); spifi_reg.quematch = data; }));
	map(0x94, 0x97).lrw32(NAME([this]() { LOG("read spifi_reg.quecode = 0x%x\n", spifi_reg.quecode); return spifi_reg.quecode; }), NAME([this](uint32_t data) { LOG("write spifi_reg.quecode = 0x%x\n", data); spifi_reg.quecode = data; }));
	map(0x98, 0x9b).lrw32(NAME([this]() { LOG("read spifi_reg.quetag = 0x%x\n", spifi_reg.quetag); return spifi_reg.quetag; }), NAME([this](uint32_t data) { LOG("write spifi_reg.quetag = 0x%x\n", data); spifi_reg.quetag = data; }));
	map(0x9c, 0x9f).lrw32(NAME([this]() { LOG("read spifi_reg.quepage = 0x%x\n", spifi_reg.quepage); return spifi_reg.quepage; }), NAME([this](uint32_t data) { LOG("write spifi_reg.quepage = 0x%x\n", data); spifi_reg.quepage = data; }));
	// mirror of above values goes here
	map(0x200, 0x3ff).rw(FUNC(spifi3_device::cmd_buf_r), FUNC(spifi3_device::cmd_buf_w)).umask32(0xff);

	// Below this line probably won't need to change
	map(0x38, 0x3b).rw(FUNC(spifi3_device::fifoctrl_r), FUNC(spifi3_device::fifoctrl_w));
	map(0x40, 0x43).lrw32(NAME([this]() { LOG("read spifi_reg.config = 0x%x\n", spifi_reg.config); return spifi_reg.config; }), NAME([this](uint32_t data) { LOG("write spifi_reg.config = 0x%x\n", data); spifi_reg.config = data; }));
	map(0x54, 0x57).w(FUNC(spifi3_device::select_w));
	map(0x54, 0x57).lr32(NAME([this]() { LOG("read spifi_reg.select = 0x%x\n", spifi_reg.select); return spifi_reg.select; }));
	map(0x5c, 0x5f).rw(FUNC(spifi3_device::auxctrl_r), FUNC(spifi3_device::auxctrl_w));
	map(0x60, 0x63).w(FUNC(spifi3_device::autodata_w));
	map(0x60, 0x63).lr32(NAME([this]() { LOG("read spifi_reg.autodata = 0x%x\n", spifi_reg.autodata); return spifi_reg.autodata; }));
}

uint8_t spifi3_device::cmd_buf_r(offs_t offset)
{
	// find which cmd entry
	// 8 slots in the buffer, 16 bytes each
	// so, divide the offset by 16 (truncated) to get the cmd entry
	int cmd_entry = offset / 16;

	// now, return the right item
	// this is ugly, I need to improve this
	uint8_t result = 0;
	int register_offset = offset % 16;
	if (register_offset < 12)
	{
		result = spifi_reg.cmbuf[cmd_entry].cdb[register_offset];
	} 
	else if (register_offset == 12)
	{
		result = spifi_reg.cmbuf[cmd_entry].quecode;
	}
	else if (register_offset == 13)
	{
		result = spifi_reg.cmbuf[cmd_entry].quetag;
	}
	else if (register_offset == 14)
	{
		result = spifi_reg.cmbuf[cmd_entry].idmsg;
	}
	else if (register_offset == 15)
	{
		result = spifi_reg.cmbuf[cmd_entry].status;
	}

	LOG("SPIFI3: cmd_buf_r(0x%x) -> 0x%x\n", offset, result);

	return result;
}

void spifi3_device::cmd_buf_w(offs_t offset, uint8_t data)
{
	LOG("SPIFI3: cmd_buf_w(0x%x, 0x%x)\n", offset, data);
	// find which cmd entry
	// 8 slots in the buffer, 16 bytes each
	// so, divide the offset by 16 (truncated) to get the cmd entry
	int cmd_entry = offset / 16;

	// now, write the appropriate item
	// this is ugly, I need to improve this
	int register_offset = offset % 16;
	if (register_offset < 12)
	{
		spifi_reg.cmbuf[cmd_entry].cdb[register_offset] = data;
	}
	else if (register_offset == 12)
	{
		spifi_reg.cmbuf[cmd_entry].quecode = data;
	}
	else if (register_offset == 13)
	{
		spifi_reg.cmbuf[cmd_entry].quetag = data;
	}
	else if (register_offset == 14)
	{
		spifi_reg.cmbuf[cmd_entry].idmsg = data;
	}
	else if (register_offset == 15)
	{
		spifi_reg.cmbuf[cmd_entry].status = data;
	}
}

uint32_t spifi3_device::auxctrl_r()
{
	LOG("read spifi_reg.auxctrl = 0x%x\n", spifi_reg.auxctrl);
	return spifi_reg.auxctrl;
}

void spifi3_device::auxctrl_w(uint32_t data)
{
	LOG("write spifi_reg.auxctrl = 0x%x\n", data);
	spifi_reg.auxctrl = data;
	if(spifi_reg.auxctrl & AUXCTRL_SRST)
	{
		// reset of some kind
		LOG("SRST asserted\n");
	}
	if(spifi_reg.auxctrl & AUXCTRL_CRST)
	{
		// chip reset?
		LOG("CRST asserted\n");
	}
	if(spifi_reg.auxctrl & AUXCTRL_SETRST)
	{
		// bus reset?
		LOG("SETRST asserted\n");
	}
	if(spifi_reg.auxctrl & AUXCTRL_DMAEDGE)
	{
		// do we need to take action here? might be what enables DMA mode/DRQ?
		LOG("DMAEDGE asserted\n");
	}
}

uint32_t spifi3_device::fifoctrl_r()
{
	LOG("read spifi_reg.fifoctrl = 0x%x\n", spifi_reg.fifoctrl);

	auto evenCount = 8 - m_even_fifo.size(); // How does the count actually work? need to test
	spifi_reg.fifoctrl &= ~FIFOC_FSLOT;
	spifi_reg.fifoctrl |= evenCount & FIFOC_FSLOT;

	return spifi_reg.fifoctrl;
}

void spifi3_device::fifoctrl_w(uint32_t data)
{
	LOG("write spifi_reg.fifoctrl = 0x%x\n", data);
	//spifi_reg.fifoctrl = data; // TODO: this might not be persisted - read/write might be different. TBD.
	if(spifi_reg.fifoctrl & FIFOC_SSTKACT) { LOG("fifoctrl.SSTKACT: w unimplemented"); } // likely RO guess: NetBSD uses this to know when synchronous data should be loaded into the FIFO?
	if(spifi_reg.fifoctrl & FIFOC_RQOVRN) { LOG("fifoctrl.RQOVRN: w unimplemented"); } // likely RO - Whatever this is, it would cause NetBSD to panic
	if(spifi_reg.fifoctrl & FIFOC_CLREVEN)
	{
		LOG("Clearing even FIFO of %d items", m_even_fifo.size());
		while (!m_even_fifo.empty())
		{
			m_even_fifo.pop();
		}
	}
	if(spifi_reg.fifoctrl & FIFOC_CLRODD)
	{
		LOG("Clearing odd FIFO of %d items", m_odd_fifo.size());
		while (!m_odd_fifo.empty())
		{
			m_odd_fifo.pop();
		}
	}
	if(spifi_reg.fifoctrl & FIFOC_FLUSH) { LOG("fifoctrl.FLUSH: unimplemented"); } // flush FIFO
	if(spifi_reg.fifoctrl & FIFOC_LOAD) { LOG("fifoctrl.LOAD: unimplemented"); } // Load FIFO synchronously
}

void spifi3_device::select_w(uint32_t data)
{
	LOG("write spifi_reg.select = 0x%x\n", data);
	spifi_reg.select = data;

	if(spifi_reg.select & SEL_ISTART)
	{
		LOG("SPIFI command start requested, but is unimplemented!\n");
		// TODO: start command here
	}
}

void spifi3_device::autodata_w(uint32_t data)
{
	LOG("write spifi_reg.autodata = 0x%x\n", data);
	spifi_reg.autodata = data;

	if(spifi_reg.autodata & ADATA_EN)
	{
		LOG("autodata enabled! target %d direction %s\n", spifi_reg.autodata & ADATA_TARGET_ID, spifi_reg.autodata & ADATA_IN ? "in" : "out");
	}
}

void spifi3_device::check_irq()
{
	/* TODO: SPIFI3 equiv
	bool oldirq = irq;
	irq = istatus != 0;

	if(irq != oldirq)
	{
		m_irq_handler(irq);
	}
	*/
}

void spifi3_device::reset_disconnect()
{
	scsi_bus->ctrl_w(scsi_refid, 0, ~S_RST);

	// command_pos = 0; TODO: spifi3 equiv
	// command_length = 0; TODO: spifi3 equiv
	// memset(command, 0, sizeof(command)); TODO: spifi3 equiv
	mode = MODE_D;
}

void spifi3_device::step(bool timeout)
{
	uint32_t ctrl = scsi_bus->ctrl_r();
	uint32_t data = scsi_bus->data_r();
	//uint8_t c     = command[0] & 0x7f; TODO: spifi3 equiv
	// SPIFI3 has an 8-slot buffer for commands. NetBSD source code seems to show that the initiator commands
	// are written into cmbuf[id] (7), with the ability to pull info about other IDs from other cmbuf slots.

	//LOGMASKED(LOG_STATE, "state=%d.%d %s\n", state & STATE_MASK, (state & SUB_MASK) >> SUB_SHIFT, timeout ? "timeout" : "change");

	if(mode == MODE_I && !(ctrl & S_BSY)) // Idle mode or bus isn't doing anything
	{
		state = IDLE; // Force idle state (if BSY is now deasserted and we're in idle mode)
		// istatus |= I_DISCONNECT; // Set disconnected flag TODO: spifi3 equiv
		reset_disconnect();
		check_irq();
	}
	switch(state & SUB_MASK ? state & SUB_MASK : state & STATE_MASK)
	{
		case IDLE:
		{
			// Don't need to do anything
			break;
		}

		case BUSRESET_WAIT_INT:
		{
			// Bus was reset by a command, go to idle state and clear reset signal
			state = IDLE;
			scsi_bus->ctrl_w(scsi_refid, 0, S_RST);
			reset_disconnect();

			// TODO: spifi3 equiv of the below
			/*if (!(config & 0x40)) {
				istatus |= I_SCSI_RESET;
				check_irq();
			}*/
			break;
		}

		case ARB_COMPLETE << SUB_SHIFT: 
		{
			if(!timeout) // Synchronize state to clock
			{
				break;
			}

			// Scan to see if we won arbitration
			int arbitrationWinner;
			for(arbitrationWinner = 7; arbitrationWinner >= 0 && !(data & (1<<arbitrationWinner)); arbitrationWinner--) {};
			if(arbitrationWinner != scsi_id) {
				scsi_bus->data_w(scsi_refid, 0);
				scsi_bus->ctrl_w(scsi_refid, 0, S_ALL);
				fatalerror("spifi3_device::step need to wait for bus free (lost arbitration)\n");
			}

			// Now that we won arbitration, we need to assert SEL and wait for the bus to settle.
			state = (state & STATE_MASK) | (ARB_ASSERT_SEL << SUB_SHIFT);
			scsi_bus->ctrl_w(scsi_refid, S_SEL, S_SEL);
			//delay(6); TODO: spifi3 equiv
			break;
		}

	case ARB_ASSERT_SEL << SUB_SHIFT:
	{
		if(!timeout) // Synchronize state to clock
		{
			break;
		}

		// scsi_bus->data_w(scsi_refid, (1<<scsi_id) | (1<<bus_id)); TODO: spifi3 equiv
		state = (state & STATE_MASK) | (ARB_SET_DEST << SUB_SHIFT);
		//delay_cycles(4); TODO: spifi3 equiv
		break;
	}

	case ARB_SET_DEST << SUB_SHIFT:
	{
		if(!timeout) // Synchronize state to clock
		{
			break;
		}

		state = (state & STATE_MASK) | (ARB_RELEASE_BUSY << SUB_SHIFT);
		// scsi_bus->ctrl_w(scsi_refid, c == CD_SELECT_ATN || c == CD_SELECT_ATN_STOP ? S_ATN : 0, S_ATN|S_BSY); TODO: spifi3 equiv
		// delay(2); TODO: spifi3 equiv
		break;
	}

	case ARB_RELEASE_BUSY << SUB_SHIFT:
	{
		if(!timeout) // Synchronize state to clock
		{
			break;
		}

		if(ctrl & S_BSY) // Check if target responded
		{
			state = (state & STATE_MASK) | (ARB_DESKEW_WAIT << SUB_SHIFT);
			/* TODO: spifi3 equiv
			if(c == CD_RESELECT)
			{
				scsi_bus->ctrl_w(scsi_refid, S_BSY, S_BSY);
			}
			delay_cycles(2);
			*/
		} 
		else // If not, we ran out of time - wait until the next timeout and check again
		{
			state = (state & STATE_MASK) | (ARB_TIMEOUT_BUSY << SUB_SHIFT);
			/* TODO: spifi3 equiv?
#ifdef DELAY_HACK
			delay(1);
#else
			delay(8192*select_timeout);
#endif
*/
		}
		break;
	}

	case ARB_DESKEW_WAIT << SUB_SHIFT:
	{
		if(!timeout)
		{
			break;
		}

		scsi_bus->data_w(scsi_refid, 0);
		scsi_bus->ctrl_w(scsi_refid, 0, S_SEL); // Clear SEL - target may now assert REQ

		/* TODO: spifi3 equiv
		if(c == CD_RESELECT) 
		{
			LOG("mode switch to Target\n");
			mode = MODE_T;
		} 
		else 
		{
			LOG("mode switch to Initiator\n");
			mode = MODE_I;
		}
		*/

		state &= STATE_MASK; // Clear sub step?
		step(true); // Process next step
		break;
	}

	case ARB_TIMEOUT_BUSY << SUB_SHIFT:
	{
		if(timeout) // No response from target
		{
			scsi_bus->data_w(scsi_refid, 0);
			LOG("select timeout\n");
			state = (state & STATE_MASK) | (ARB_TIMEOUT_ABORT << SUB_SHIFT); // handle timeout
			// delay(1000); TODO: spifi3 equiv
		}
		else if(ctrl & S_BSY) // Got response from target, wait before allowing transaction
		{
			state = (state & STATE_MASK) | (ARB_DESKEW_WAIT << SUB_SHIFT);
			/*if(c == CD_RESELECT) TODO: spifi3 equiv
			{
				scsi_bus->ctrl_w(scsi_refid, S_BSY, S_BSY);
			}*/
			// delay_cycles(2); TODO: spifi3 equiv
		}
		break;
	}

	case ARB_TIMEOUT_ABORT << SUB_SHIFT: // Need to reset bus unless the target responded
	{
		if(!timeout)
		{
			break;
		}

		if(ctrl & S_BSY) // Last chance for target to respond
		{
			state = (state & STATE_MASK) | (ARB_DESKEW_WAIT << SUB_SHIFT);
			/* TODO: spifi3 equiv
			if(c == CD_RESELECT)
			{
				scsi_bus->ctrl_w(scsi_refid, S_BSY, S_BSY);
			}
			delay_cycles(2);
			*/
		} 
		else // If not, force bus free
		{
			scsi_bus->ctrl_w(scsi_refid, 0, S_ALL);
			state = IDLE;
			// istatus |= I_DISCONNECT; TODO: spifi3 equiv
			reset_disconnect();
			check_irq();
		}
		break;
	}

/*
	case SEND_WAIT_SETTLE << SUB_SHIFT:
		if(!timeout)
			break;

		state = (state & STATE_MASK) | (SEND_WAIT_REQ_0 << SUB_SHIFT);
		step(false);
		break;

	case SEND_WAIT_REQ_0 << SUB_SHIFT:
		if(ctrl & S_REQ)
			break;
		state = state & STATE_MASK;
		scsi_bus->data_w(scsi_refid, 0);
		scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
		step(false);
		break;

	case RECV_WAIT_REQ_1 << SUB_SHIFT:
		if(!(ctrl & S_REQ))
			break;

		state = (state & STATE_MASK) | (RECV_WAIT_SETTLE << SUB_SHIFT);
		delay_cycles(sync_period);
		break;

	case RECV_WAIT_SETTLE << SUB_SHIFT:
		if(!timeout)
			break;

		if((state & STATE_MASK) != INIT_XFR_RECV_PAD)
			fifo_push(scsi_bus->data_r());
		scsi_bus->ctrl_w(scsi_refid, S_ACK, S_ACK);
		state = (state & STATE_MASK) | (RECV_WAIT_REQ_0 << SUB_SHIFT);
		step(false);
		break;

	case RECV_WAIT_REQ_0 << SUB_SHIFT:
		if(ctrl & S_REQ)
			break;
		state = state & STATE_MASK;
		step(false);
		break;

	case DISC_SEL_ARBITRATION_INIT:
		// wait until a command is in the fifo
		if (!fifo_pos) {
			// dma starts after bus arbitration/selection is complete
			check_drq();
			break;
		}

		command_length = fifo_pos + tcounter;
		state = DISC_SEL_ARBITRATION;
		step(false);
		break;

	case DISC_SEL_ARBITRATION:
		if(c == CD_SELECT) {
			state = DISC_SEL_WAIT_REQ;
		} else
			state = DISC_SEL_ATN_WAIT_REQ;

		scsi_bus->ctrl_wait(scsi_refid, S_REQ, S_REQ);
		if(ctrl & S_REQ)
			step(false);
		break;

	case DISC_SEL_ATN_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;
		if((ctrl & S_PHASE_MASK) != S_PHASE_MSG_OUT) {
			function_complete();
			break;
		}
		if(c == CD_SELECT_ATN)
			scsi_bus->ctrl_w(scsi_refid, 0, S_ATN);
		state = DISC_SEL_ATN_SEND_BYTE;
		send_byte();
		break;

	case DISC_SEL_ATN_SEND_BYTE:
		command_length--;
		if(c == CD_SELECT_ATN_STOP) {
			seq = 1;
			function_bus_complete();
		} else {
			state = DISC_SEL_WAIT_REQ;
		}
		break;

	case DISC_SEL_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;
		if((ctrl & S_PHASE_MASK) != S_PHASE_COMMAND) {
			if(!command_length)
				seq = 4;
			else
				seq = 2;
			scsi_bus->ctrl_wait(scsi_refid, 0, S_REQ);
			function_bus_complete();
			break;
		}
		if(seq < 3)
			seq = 3;
		state = DISC_SEL_SEND_BYTE;
		send_byte();
		break;

	case DISC_SEL_SEND_BYTE:
		if(command_length) {
			command_length--;
			if(!command_length)
				seq = 4;
		}

		state = DISC_SEL_WAIT_REQ;
		break;

	case INIT_CPT_RECV_BYTE_ACK:
		state = INIT_CPT_RECV_WAIT_REQ;
		scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
		break;

	case INIT_CPT_RECV_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;

		if((ctrl & S_PHASE_MASK) != S_PHASE_MSG_IN) {
			command_pos = 0;
			bus_complete();
		} else {
			state = INIT_CPT_RECV_BYTE_NACK;
			recv_byte();
		}
		break;

	case INIT_CPT_RECV_BYTE_NACK:
		function_complete();
		break;

	case INIT_MSG_WAIT_REQ:
		if((ctrl & (S_REQ|S_BSY)) == S_BSY)
			break;
		bus_complete();
		break;

	case INIT_XFR:
		switch(xfr_phase) {
		case S_PHASE_DATA_OUT:
		case S_PHASE_COMMAND:
		case S_PHASE_MSG_OUT:
			state = INIT_XFR_SEND_BYTE;

			// can't send if the fifo is empty
			if (fifo_pos == 0)
				break;

			// if it's the last message byte, deassert ATN before sending
			if (xfr_phase == S_PHASE_MSG_OUT && ((!dma_command && fifo_pos == 1) || (dma_command && tcounter == 1)))
				scsi_bus->ctrl_w(scsi_refid, 0, S_ATN);

			send_byte();
			break;

		case S_PHASE_DATA_IN:
		case S_PHASE_STATUS:
		case S_PHASE_MSG_IN:
			// can't receive if the fifo is full
			if (fifo_pos == 16)
				break;

			// if it's the last message byte, ACK remains asserted, terminate with function_complete()
			state = (xfr_phase == S_PHASE_MSG_IN && (!dma_command || tcounter == 1)) ? INIT_XFR_RECV_BYTE_NACK : INIT_XFR_RECV_BYTE_ACK;

			recv_byte();
			break;

		default:
			LOG("xfer on phase %d\n", scsi_bus->ctrl_r() & S_PHASE_MASK);
			function_complete();
			break;
		}
		break;

	case INIT_XFR_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;

		// check for command complete
		if ((dma_command && (status & S_TC0) && (dma_dir == DMA_IN || fifo_pos == 0)) // dma in/out: transfer count == 0
		|| (!dma_command && (xfr_phase & S_INP) == 0 && fifo_pos == 0)      // non-dma out: fifo empty
		|| (!dma_command && (xfr_phase & S_INP) == S_INP && fifo_pos == 1)) // non-dma in: every byte
			state = INIT_XFR_BUS_COMPLETE;
		else
			// check for phase change
			if((ctrl & S_PHASE_MASK) != xfr_phase) {
				command_pos = 0;
				state = INIT_XFR_BUS_COMPLETE;
			} else {
				state = INIT_XFR;
			}
		step(false);
		break;

	case INIT_XFR_SEND_BYTE:
		state = INIT_XFR_WAIT_REQ;
		step(false);
		break;

	case INIT_XFR_RECV_BYTE_ACK:
		state = INIT_XFR_WAIT_REQ;
		scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
		break;

	case INIT_XFR_RECV_BYTE_NACK:
		state = INIT_XFR_FUNCTION_COMPLETE;
		step(false);
		break;

	case INIT_XFR_FUNCTION_COMPLETE:
		// wait for dma transfer to complete or fifo to drain
		if (dma_command && !(status & S_TC0) && fifo_pos)
			break;

		function_complete();
		break;

	case INIT_XFR_BUS_COMPLETE:
		// wait for dma transfer to complete or fifo to drain
		if (dma_command && !(status & S_TC0) && fifo_pos)
			break;

		bus_complete();
		break;

	case INIT_XFR_SEND_PAD_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;

		if((ctrl & S_PHASE_MASK) != xfr_phase) {
			command_pos = 0;
			bus_complete();
		} else {
			state = INIT_XFR_SEND_PAD;
			send_byte();
		}
		break;

	case INIT_XFR_SEND_PAD:
		decrement_tcounter();
		if(!(status & S_TC0)) {
			state = INIT_XFR_SEND_PAD_WAIT_REQ;
			step(false);
		} else
			function_complete();
		break;

	case INIT_XFR_RECV_PAD_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;

		if((ctrl & S_PHASE_MASK) != xfr_phase) {
			command_pos = 0;
			bus_complete();
		} else {
			state = INIT_XFR_RECV_PAD;
			recv_byte();
		}
		break;

	case INIT_XFR_RECV_PAD:
		decrement_tcounter();
		if(!(status & S_TC0)) {
			state = INIT_XFR_RECV_PAD_WAIT_REQ;
			scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
			step(false);
		} else
			function_complete();
		break;
	*/
	default:
		LOG("step() unexpected state %d.%d\n", state & STATE_MASK, (state & SUB_MASK) >> SUB_SHIFT);
		exit(0);
	}

}
