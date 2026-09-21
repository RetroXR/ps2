/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2021  PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with PCSX2.
 *  If not, see <http://www.gnu.org/licenses/>.
 */


#include "R3000A.h"
#include "Common.h"
#include "Config.h"

#include "R5900OpcodeTables.h"
#include "IopBios.h"
#include "IopHw.h"


static bool branch2 = 0;
static u32 branchPC;

/* Debug: LRPS2_IOP_PCHOOK=addr=name[,addr=name...] logs every time the
 * interpreter reaches one of those addresses, with a0-a3. The interpreter
 * only (LRPS2_NO_IOPREC=1): it is for seeing what a driver is asked to do. */
static int s_pchook_n = -1;
static u32 s_pchook_addr[16];
static char s_pchook_name[16][32];

static void pchook_init(void)
{
	const char* s = getenv("LRPS2_IOP_PCHOOK");
	s_pchook_n = 0;
	while (s && *s && s_pchook_n < 16)
	{
		char* end;
		size_t i = 0;
		s_pchook_addr[s_pchook_n] = (u32)strtoul(s, &end, 16);
		if (*end != '=')
			break;
		s = end + 1;
		while (*s && *s != ',' && i < sizeof(s_pchook_name[0]) - 1)
			s_pchook_name[s_pchook_n][i++] = *s++;
		s_pchook_name[s_pchook_n][i] = 0;
		s_pchook_n++;
		if (*s == ',')
			s++;
	}
}

static void pchook_check(void)
{
	for (int i = 0; i < s_pchook_n; i++)
		if (psxRegs.pc == s_pchook_addr[i] && log_cb)
			log_cb(RETRO_LOG_INFO, "[PC] %s a0=%08x a1=%08x a2=%08x a3=%08x ra=%08x\n", s_pchook_name[i],
				psxRegs.GPR.n.a0, psxRegs.GPR.n.a1, psxRegs.GPR.n.a2, psxRegs.GPR.n.a3, psxRegs.GPR.n.ra);
}

static __fi void execI(void)
{
	if (s_pchook_n)
	{
		if (s_pchook_n < 0)
			pchook_init();
		if (s_pchook_n)
			pchook_check();
	}

	// Inject IRX hack
	if (psxRegs.pc == 0x1630 && strlen(EmuConfig.CurrentIRX) > 3)
	{
		// FIXME do I need to increase the module count (0x1F -> 0x20)
		if (iopMemRead32(0x20018) == 0x1F)
			iopMemWrite32(0x20094, 0xbffc0000);
	}

	psxRegs.code = iopMemRead32(psxRegs.pc);

	psxRegs.pc+= 4;
	psxRegs.cycle++;

	//One of the Iop to EE delta clocks to be set in PS1 mode.
	if ((psxHu32(HW_ICFG) & (1 << 3)))
		psxRegs.iopCycleEE -= 9;
	else //default ps2 mode value
		psxRegs.iopCycleEE -= 8;
	psxBSC[psxRegs.code >> 26]();
}

static void doBranch(s32 tar)
{
	branch2 = iopIsDelaySlot = true;
	branchPC = tar;
	execI();
	iopIsDelaySlot = false;
	psxRegs.pc = branchPC;

	iopEventTest();
}


/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, offset                                 *
*********************************************************/

void psxBGEZ()         // Branch if Rs >= 0
{
	if (_i32(_rRs_) >= 0) doBranch(_BranchTarget_);
}

void psxBGEZAL()   // Branch if Rs >= 0 and link
{
	/* The link is written before the condition is read, so with rs == 31
	 * the test sees the return address rather than the old value. That is
	 * deliberate, not the JALR bug in a different place: MIPS leaves r31 as
	 * the source of a linking branch undefined, and all four engines agree
	 * here -- both recompilers set the link before reading rs too, the EE
	 * one in iR5900Branch.cpp and the IOP one in iR3000Atables.cpp.
	 * Reordering it to match JALR would break that agreement rather than
	 * fix anything, and the engines are switchable at run time. */
	_SetLink(31);
	if (_i32(_rRs_) >= 0)
	{
		doBranch(_BranchTarget_);
	}
}

void psxBGTZ()          // Branch if Rs >  0
{
	if (_i32(_rRs_) > 0) doBranch(_BranchTarget_);
}

void psxBLEZ()         // Branch if Rs <= 0
{
	if (_i32(_rRs_) <= 0) doBranch(_BranchTarget_);
}
void psxBLTZ()          // Branch if Rs <  0
{
	if (_i32(_rRs_) < 0) doBranch(_BranchTarget_);
}

void psxBLTZAL()    // Branch if Rs <  0 and link
{
	_SetLink(31);
	if (_i32(_rRs_) < 0)
		{
			doBranch(_BranchTarget_);
		}
}

/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, rt, offset                             *
*********************************************************/

void psxBEQ()   // Branch if Rs == Rt
{
	if (_i32(_rRs_) == _i32(_rRt_)) doBranch(_BranchTarget_);
}

void psxBNE()   // Branch if Rs != Rt
{
	if (_i32(_rRs_) != _i32(_rRt_)) doBranch(_BranchTarget_);
}

/*********************************************************
* Jump to target                                         *
* Format:  OP target                                     *
*********************************************************/
void psxJ(void)
{
	// check for iop module import table magic (C.71: cached resolution)
	u32 delayslot = iopMemRead32(psxRegs.pc);
	if (delayslot >> 16 == 0x2400 && R3000A::irxImportExecCached(psxRegs.pc, delayslot & 0xffff))
		return;

	doBranch(_JumpTarget_);
}

void psxJAL(void)
{
	_SetLink(31);
	doBranch(_JumpTarget_);
}

/*********************************************************
* Register jump                                          *
* Format:  OP rs, rd                                     *
*********************************************************/
void psxJR(void)
{
	doBranch(_u32(_rRs_));
}

void psxJALR(void)
{
	/* Read the target before writing the link. They can be the same
	 * register, and _SetLink writes GPR[rd] directly, so taking the target
	 * from _rRs_ afterwards would jump to the return address instead --
	 * ps2autotests tests/cpu/iop/branchdelay.expected covers exactly that
	 * as "jalr: rs/rd match" and the console branches to the target.
	 *
	 * rpsxJALR in x86/iR3000Atables.cpp has always had this right: it moves
	 * Rs into its writeback register before assigning Rd, and separately
	 * refuses to swap the delay slot when the two match. JALR in
	 * Interpreter.cpp does the same for the EE, taking a temp copy first.
	 * This was the odd one out. */
	const u32 target = _u32(_rRs_);

	if (_Rd_)
	{
		_SetLink(_Rd_);
	}
	doBranch(target);
}

static void intReserve(void) { }
static void intAlloc(void) { }
static void intReset(void) { intAlloc(); }
static void intClear(u32 Addr, u32 Size) { }
static void intShutdown(void) { }

static s32 intExecuteBlock( s32 eeCycles )
{
	psxRegs.iopBreak = 0;
	psxRegs.iopCycleEE = eeCycles;

	while (psxRegs.iopCycleEE > 0)
	{
		if ((psxHu32(HW_ICFG) & 8) && ((psxRegs.pc & 0x1fffffffU) == 0xa0 || (psxRegs.pc & 0x1fffffffU) == 0xb0 || (psxRegs.pc & 0x1fffffffU) == 0xc0))
			psxBiosCall();

		branch2 = 0;
		while (!branch2)
			execI();
	}

	return psxRegs.iopBreak + psxRegs.iopCycleEE;
}

#ifdef ARCH_ARM64
// arm64 recompiler (Phase C.2b) helper: run a single IOP basic block through the
// interpreter -- instructions until the next taken branch (and its delay slot,
// which execI()/doBranch() handle). The arm64 JIT currently emits one per-PC
// block per basic block, each of which calls this; Phase C.2b-2 replaces the
// body with natively translated instructions. Mirrors intExecuteBlock's inner
// loop (incl. the BIOS HLE entry hook) so behaviour is identical.
extern "C" void iopRunBasicBlock_arm64(void)
{
	if ((psxHu32(HW_ICFG) & 8) && ((psxRegs.pc & 0x1fffffffU) == 0xa0 || (psxRegs.pc & 0x1fffffffU) == 0xb0 || (psxRegs.pc & 0x1fffffffU) == 0xc0))
		psxBiosCall();

	branch2 = 0;
	while (!branch2)
		execI();
}
#endif

R3000Acpu psxInt = {
	intReserve,
	intReset,
	intExecuteBlock,
	intClear,
	intShutdown
};
