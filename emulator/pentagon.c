/* pentagon.c -- what makes the Pentagon build of Klive's +3 core a Pentagon
 * 128 with its Beta 128 disk interface (tools/emu/build_emu.py, -DP3_PENTAGON).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The machine: 71,680 T-states a frame (320 lines of 224), no contention,
 * no $1FFD and no +3 floppy controller (the core's timing config and ports
 * are switched by the same macro); ROM 0 the Pentagon's 128 ROM, ROM 1 the
 * 48K BASIC ROM, ROM 2 TR-DOS.
 *
 * The Beta 128's trap, as Fuse's beta.c and z80.c have it: when the CPU
 * fetches an instruction from $3D00-$3DFF with the 48K BASIC ROM in (ROM 1),
 * the TR-DOS ROM replaces it at $0000; when it fetches one from RAM ($4000
 * up), the BASIC ROM is back.  The controller's ports answer only while
 * TR-DOS is in (wd1793.c).  The check runs before every instruction, as
 * Fuse's does before every opcode.
 */

static void p5Trap(void) {
  const uint16_t pc = cpu.pc;
  if (!p5Dos) {
    if ((pc & 0xff00u) == 0x3d00u && spp3eSelectedRom == 1u && !spp3eInSpecialPagingMode) {
      p5Dos = 1u;
      spp3eRebuildMemorySlotMap();
    }
  } else if (pc >= 0x4000u) {
    p5Dos = 0u;
    spp3eRebuildMemorySlotMap();
  }
}

static uint8_t p5BetaPort(uint32_t address) {
  const uint32_t lo = address & 0xffu;
  return p5Dos && (lo == 0x1fu || lo == 0x3fu || lo == 0x5fu || lo == 0x7fu || lo == 0xffu) ? 1u : 0u;
}

void p5Reset(void) {
  p5Dos = 0u;
  p5BetaReset();
}

uint32_t p5DosActive(void) { return p5Dos; }
