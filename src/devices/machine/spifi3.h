// license:BSD-3-Clause
// copyright-holders:Brice Onken,Tsubai Masanari
// thanks-to:Patrick Mackinlay,Olivier Galibert

/*
 * HP 1TV3-0302 SPIFI3-SE SCSI controller
 *
 * Datasheets for this seem to be impossible to find - the only avaliable implementation to reference that I have
 * found is the Sony NEWS APBus NetBSD driver. Hopefully a datasheet will turn up eventually.
 * Based on internet research, it seems some HP PA-RISC systems also used the SPIFI3, including the E55.
 *
 * Register definitions were derived from the NetBSD source code, copyright (c) 2000 Tsubai Masanari.
 *
 * References:
 * - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/spifireg.h
 * - https://github.com/NetBSD/src/blob/trunk/sys/arch/newsmips/apbus/spifi.c
 */

#ifndef MAME_MACHINE_SPIFI3_H
#define MAME_MACHINE_SPIFI3_H

#pragma once

#include "machine/nscsi_bus.h"

class spifi3_device
	: public nscsi_device
	, public nscsi_slot_card_interface
{
public:
	spifi3_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);
	void map(address_map &map);

private:

	enum ScsiMode
	{ 
		MODE_D, 
		MODE_T, 
		MODE_I 
	};

    // State tracking variables
	ScsiMode mode; // Target or initiatior?
	int state; // SCSI controller state

	enum { IDLE };

	enum {
		// Bus initiated sequences
		BUSINIT_SETTLE_DELAY = 1,
		BUSINIT_ASSERT_BUS_SEL,
		BUSINIT_MSG_OUT,
		BUSINIT_RECV_BYTE,
		BUSINIT_ASSERT_BUS_RESEL,
		BUSINIT_WAIT_REQ,
		BUSINIT_RECV_BYTE_NACK,

		// Bus SCSI Reset
		BUSRESET_WAIT_INT,
		BUSRESET_RESET_BOARD,

		// Disconnected state commands
		DISC_SEL_ARBITRATION_INIT,
		DISC_SEL_ARBITRATION,
		DISC_SEL_ATN_WAIT_REQ,
		DISC_SEL_ATN_SEND_BYTE,
		DISC_SEL_WAIT_REQ,
		DISC_SEL_SEND_BYTE,
		DISC_REC_ARBITRATION,
		DISC_REC_MSG_IN,
		DISC_REC_SEND_BYTE,
		DISC_RESET,

		// Command sequence
		CMDSEQ_CMD_PHASE,
		CMDSEQ_RECV_BYTE,

		// Target commands
		TARGET_SEND_BYTE,
		TARGET_CMD_RECV_BYTE,
		TARGET_MSG_RECV_BYTE,
		TARGET_MSG_RECV_PAD,
		TARGET_DISC_SEND_BYTE,
		TARGET_DISC_MSG_IN,
		TARGET_DISC_SEND_BYTE_2,

		// Initiator commands
		INIT_MSG_WAIT_REQ,
		INIT_XFR,
		INIT_XFR_SEND_BYTE,
		INIT_XFR_SEND_PAD_WAIT_REQ,
		INIT_XFR_SEND_PAD,
		INIT_XFR_RECV_PAD_WAIT_REQ,
		INIT_XFR_RECV_PAD,
		INIT_XFR_RECV_BYTE_ACK,
		INIT_XFR_RECV_BYTE_NACK,
		INIT_XFR_FUNCTION_COMPLETE,
		INIT_XFR_BUS_COMPLETE,
		INIT_XFR_WAIT_REQ,
		INIT_CPT_RECV_BYTE_ACK,
		INIT_CPT_RECV_WAIT_REQ,
		INIT_CPT_RECV_BYTE_NACK
	};

	enum {
		// Arbitration
		ARB_WAIT_BUS_FREE = 1,
		ARB_COMPLETE,
		ARB_ASSERT_SEL,
		ARB_SET_DEST,
		ARB_RELEASE_BUSY,
		ARB_TIMEOUT_BUSY,
		ARB_TIMEOUT_ABORT,
		ARB_DESKEW_WAIT,

		// Send/receive byte
		SEND_WAIT_SETTLE,
		SEND_WAIT_REQ_0,
		RECV_WAIT_REQ_1,
		RECV_WAIT_SETTLE,
		RECV_WAIT_REQ_0
	};

	enum {
		STATE_MASK = 0x00ff,
		SUB_SHIFT  = 8,
		SUB_MASK   = 0xff00
	};

	enum { BUS_BUSY, BUS_FREE_WAIT, BUS_FREE };

	enum {
		S_GROSS_ERROR     = 0x40,
		S_PARITY          = 0x20,
		S_TC0             = 0x10,
		S_TCC             = 0x08,

		I_SCSI_RESET      = 0x80,
		I_ILLEGAL         = 0x40,
		I_DISCONNECT      = 0x20,
		I_BUS             = 0x10,
		I_FUNCTION        = 0x08,
		I_RESELECTED      = 0x04,
		I_SELECT_ATN      = 0x02,
		I_SELECTED        = 0x01,

		CM_NOP             = 0x00,
		CM_FLUSH_FIFO      = 0x01,
		CM_RESET           = 0x02,
		CM_RESET_BUS       = 0x03,
		CD_RESELECT        = 0x40,
		CD_SELECT          = 0x41,
		CD_SELECT_ATN      = 0x42,
		CD_SELECT_ATN_STOP = 0x43,
		CD_ENABLE_SEL      = 0x44,
		CD_DISABLE_SEL     = 0x45,
		CD_SELECT_ATN3     = 0x46, // 53c90a
		CT_SEND_MSG        = 0x20,
		CT_SEND_STATUS     = 0x21,
		CT_SEND_DATA       = 0x22,
		CT_DISCONNECT_SEQ  = 0x23,
		CT_TERMINATE       = 0x24,
		CT_COMPLETE        = 0x25,
		CT_DISCONNECT      = 0x27,
		CT_RECV_MSG        = 0x28,
		CT_RECV_CMD        = 0x29,
		CT_RECV_DATA       = 0x2a,
		CT_RECV_CMD_SEQ    = 0x2b,
		CT_ABORT_DMA       = 0x04, // 53c90a
		CI_XFER            = 0x10,
		CI_COMPLETE        = 0x11,
		CI_MSG_ACCEPT      = 0x12,
		CI_PAD             = 0x18,
		CI_SET_ATN         = 0x1a,
		CI_RESET_ATN       = 0x1b, // 53c90a
	};

	enum { DMA_NONE, DMA_IN, DMA_OUT };

	// State-related functions
	void step(bool timeout);
	void check_irq();
	void reset_disconnect();

	// AUXCTRL constants and functions
	const uint32_t AUXCTRL_DMAEDGE = 0x04;
	const uint32_t AUXCTRL_SETRST = 0x20;
	const uint32_t AUXCTRL_CRST = 0x40;
	const uint32_t AUXCTRL_SRST = 0x80;
	uint32_t auxctrl_r();
	void auxctrl_w(uint32_t data);

	// FIFOCTRL constants and functions
	const uint32_t FIFOC_FSLOT = 0x0f; // Free slots in FIFO - max 8. Free slots = 8 - (FIFOCTRL & FIFOC_FSLOT) */
	const uint32_t FIFOC_SSTKACT = 0x10;
	const uint32_t FIFOC_RQOVRN = 0x20;
	const uint32_t FIFOC_CLREVEN = 0x00;
	const uint32_t FIFOC_CLRODD = 0x40;
	const uint32_t FIFOC_FLUSH = 0x80;
	const uint32_t FIFOC_LOAD = 0xc0;
	uint32_t fifoctrl_r();
	void fifoctrl_w(uint32_t data);

	// Command buffer constants and functions
	uint8_t cmd_buf_r(offs_t offset);
	void cmd_buf_w(offs_t offset, uint8_t data);

	struct spifi_cmd_entry
	{
		// NetBSD has these mapped as uint32_t to align the accesses and such
		// in reality, these are all 8-bit values that are mapped, in typical NWS-5000 series
		// fashion, to be 32-bit word aligned.
		// the same probably applies to the register file.
		uint8_t cdb[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		uint8_t quecode = 0;
		uint8_t quetag = 0;
		uint8_t idmsg = 0;
		uint8_t status = 0;
	};

	struct register_file
	{
/*00*/	uint32_t spstat = 0;
		uint32_t cmlen = 0;
		uint32_t cmdpage = 0;
		uint32_t count_hi = 0;

/*10*/	uint32_t count_mid = 0;
		uint32_t count_low = 0;
		uint32_t svptr_hi = 0;
		uint32_t svptr_mid = 0;

/*20*/	uint32_t svptr_low = 0;
		uint32_t intr = 0;
		uint32_t imask = 0;
		uint32_t prctrl = 0;

/*30*/	uint32_t prstat = 0;
		uint32_t init_status = 0;
		uint32_t fifoctrl = 0;
		uint32_t fifodata = 0;

/*40*/	uint32_t config = 0;
		uint32_t data_xfer = 0;
		uint32_t autocmd = 0;
		uint32_t autostat = 0;

/*50*/	uint32_t resel = 0;
		uint32_t select = 0;
		uint32_t prcmd = 0;
		uint32_t auxctrl = 0;

/*60*/  uint32_t autodata = 0;
		uint32_t loopctrl = 0;
		uint32_t loopdata = 0;
		uint32_t identify = 0;

/*70*/  uint32_t complete = 0;
		uint32_t scsi_status = 0x0; //0x1; // MROM reads this to check if the SPIFI is alive at system boot, so the WO description from NetBSD might be wrong.
		uint32_t data = 0;
		uint32_t icond = 0;

/*80*/	uint32_t fastwide = 0;
		uint32_t exctrl = 0;
		uint32_t exstat = 0;
		uint32_t test = 0;

/*90*/	uint32_t quematch = 0;
		uint32_t quecode = 0;
		uint32_t quetag = 0;
		uint32_t quepage = 0;

		// uint32_t image[88]; // mirror of the previous values
		spifi_cmd_entry cmbuf[8];
	} spifi_reg;
};

DECLARE_DEVICE_TYPE(SPIFI3, spifi3_device)

#endif // MAME_MACHINE_SPIFI3_H
