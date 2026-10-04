"""Build the emulators the tests and the player page run: a ZX Spectrum +3
and a Pentagon 128 with its Beta 128 disk interface.

    python tools/emu/build_emu.py            # -> build/emu/plus3.wasm (+ plus3-0..3.rom),
                                             #    build/emu/pentagon.wasm (+ pentagon-0..2.rom)
    python tools/emu/build_emu.py --fetch    # clone Klive and Fuse at the pinned commits first

The machine is Klive IDE's ZX Spectrum +3 core (MIT; a freestanding C file
compiled to wasm32: Z80, gate-array contention, $7FFD / $1FFD paging, the
four ROMs, keyboard, sound).  Its floppy controller is a stub (one sector per
command, no timing, no End of Cylinder), so the build splices in
tools/emu/upd765.c -- a uPD765A that behaves as Fuse's does on the +3 and
turns the disk in real time -- by rerouting the core's FDC ports, motor
line, reset and tact rebase to it.  Each patch below must match the pinned
source exactly once; a Klive update that moves them fails the build here
rather than quietly running the stub.  The ROMs are Amstrad's +3 ROMs as
Fuse distributes them (Amstrad allows their distribution; see Fuse's
roms/README.copyright).

The Pentagon is the same core compiled with -DP3_PENTAGON: its timing
(71,680 T-states a frame, no contention), no $1FFD and no uPD765, and
tools/emu/wd1793.c -- the Beta 128's WD1793, after Fuse's wd_fdc.c -- with
tools/emu/pentagon.c, the trap that pages the TR-DOS ROM in and out.  Its
ROMs are the Pentagon's 128 ROM, the 48K BASIC ROM and TR-DOS 5.03, which
Fuse does not ship: they are taken from PENTAGON_ROMS, or JSSpeccy 3's web
bundle (JSSPECCY_WEB/roms), or a Fuse installation's roms folder.
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from paths import EMU_DIR, FUSE_SRC, KLIVE_SRC, clang, pentagon_roms  # noqa: E402

KLIVE_REPO = "https://github.com/Dotneteer/kliveide.git"
KLIVE_COMMIT = "d60036ecbc2b422491f5a766a83933f79a5aca96"
FUSE_REPO = "https://git.code.sf.net/p/fuse-emulator/fuse"
FUSE_COMMIT = "f9a7f62141ed23995c3c832a72f826e78cfb5e19"

HERE = Path(__file__).resolve().parent
CORE = Path("src/emu/machines/zxSpectrumP3e/wasm/spp3e/spp3e.c")
EXPORTS_FROM = Path("scripts/build-spp3e-wasm.cjs")
MEMORY = 8 * 1024 * 1024

# (what, old, new): every `old` must occur exactly once in spp3e.c
PATCHES = [
    ("our FDC's declarations after Klive's FDC state",
     "static uint32_t spp3eFdcDirtyRevision;\n",
     "static uint32_t spp3eFdcDirtyRevision;\n"
     "uint32_t p3FdcReadStatus(void);\nuint32_t p3FdcReadData(void);\n"
     "void p3FdcWriteData(uint32_t value);\nvoid p3FdcSetMotor(uint32_t on);\n"
     "void p3FdcReset(void);\nstatic uint64_t p3TimeBase;\n"
     "#ifdef P3_PENTAGON\n"
     "static uint8_t p5Dos;\nstatic void p5Trap(void);\nstatic uint8_t p5BetaPort(uint32_t address);\n"
     "uint32_t p5BetaRead(uint32_t port);\nvoid p5BetaWrite(uint32_t port, uint32_t value);\nvoid p5Reset(void);\n"
     "#define P3_HAS_UPD 0\n#define P3_ROM0() (p5Dos ? 2u : spp3eSelectedRom)\n"
     "#else\n#define P3_HAS_UPD 1\n#define P3_ROM0() (spp3eSelectedRom)\n#endif\n"),
    # ---- the Pentagon: its frame, no contention, its clock
    ("the Pentagon's frame: 320 lines of 224 T-states, no contention",
     "static const Spp3eScreenConfig spp3eUlaConfig = {\n"
     "  8u, 7u, 48u, 48u, 8u, 192u, 24u, 24u, 128u, 40u, 12u, 2u, 1u,\n"
     "  {0u, 7u, 6u, 5u, 4u, 3u, 2u, 1u}\n"
     "};",
     "#ifdef P3_PENTAGON\n"
     "static const Spp3eScreenConfig spp3eUlaConfig = {\n"
     "  16u, 16u, 48u, 48u, 0u, 192u, 24u, 24u, 128u, 32u, 16u, 2u, 1u,\n"
     "  {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u}\n"
     "};\n"
     "#else\n"
     "static const Spp3eScreenConfig spp3eUlaConfig = {\n"
     "  8u, 7u, 48u, 48u, 8u, 192u, 24u, 24u, 128u, 40u, 12u, 2u, 1u,\n"
     "  {0u, 7u, 6u, 5u, 4u, 3u, 2u, 1u}\n"
     "};\n"
     "#endif"),
    ("the Pentagon's tact tables", "#define SPP3E_TACTS_PER_FRAME 70908u",
     "#ifdef P3_PENTAGON\n#define SPP3E_TACTS_PER_FRAME 71680u\n#else\n#define SPP3E_TACTS_PER_FRAME 70908u\n#endif"),
    ("the Pentagon's clock", "#define SPP3E_BASE_CLOCK_FREQUENCY 3546900u",
     "#ifdef P3_PENTAGON\n#define SPP3E_BASE_CLOCK_FREQUENCY 3500000u\n#else\n#define SPP3E_BASE_CLOCK_FREQUENCY 3546900u\n#endif"),
    # ---- the Pentagon: TR-DOS at $0000 while the trap has it in, and the trap itself
    ("the TR-DOS ROM at $0000 while it is paged in",
     "  spp3eSetRomSlot(0u, spp3eSelectedRom);", "  spp3eSetRomSlot(0u, P3_ROM0());"),
    ("the Beta 128's trap before every instruction",
     "  z80ExecuteCpuCycle();\n  spp3eTacts = z80GetTacts();",
     "#ifdef P3_PENTAGON\n  p5Trap();\n#endif\n  z80ExecuteCpuCycle();\n  spp3eTacts = z80GetTacts();"),
    ("the Beta 128's ports (reads)",
     "uint32_t spp3eReadPort(uint32_t address) {\n",
     "uint32_t spp3eReadPort(uint32_t address) {\n"
     "#ifdef P3_PENTAGON\n  if (p5BetaPort(address)) return p5BetaRead(address);\n#endif\n"),
    ("the Beta 128's ports (writes)",
     "void spp3eWritePort(uint32_t address, uint32_t value) {\n",
     "void spp3eWritePort(uint32_t address, uint32_t value) {\n"
     "#ifdef P3_PENTAGON\n  if (p5BetaPort(address)) { p5BetaWrite(address, value); return; }\n#endif\n"),
    ("no uPD765 on the Pentagon (reads)",
     "  if ((address & 0xf002u) == 0x2000u || (address & 0xf002u) == 0x3000u) {",
     "  if (P3_HAS_UPD && ((address & 0xf002u) == 0x2000u || (address & 0xf002u) == 0x3000u)) {"),
    ("no $1FFD on the Pentagon", "  if ((address & 0xf002u) == 0x1000u) {",
     "  if (P3_HAS_UPD && (address & 0xf002u) == 0x1000u) {"),
    ("no uPD765 on the Pentagon (writes)", "  if ((address & 0xf002u) == 0x3000u) {\n    spp3eFdcWriteDataRegister(value);",
     "  if (P3_HAS_UPD && (address & 0xf002u) == 0x3000u) {\n    spp3eFdcWriteDataRegister(value);"),
    ("the FDC's ports ($2FFD status, $3FFD data) read ours",
     "      ? spp3eFdcReadMainStatusRegister()\n      : spp3eFdcReadDataRegister();",
     "      ? p3FdcReadStatus()\n      : p3FdcReadData();"),
    ("$3FFD writes go to ours",
     "    spp3eFdcWriteDataRegister(value);",
     "    p3FdcWriteData(value);"),
    ("the motor ($1FFD bit 3) drives ours",
     "    spp3eFdcSetMotor(spp3eDiskMotorOn);",
     "    spp3eFdcSetMotor(spp3eDiskMotorOn);\n    p3FdcSetMotor(spp3eDiskMotorOn);"),
    ("a reset resets ours",
     "  spp3eFdcReset();\n  spp3eResetKeyboard();",
     "  spp3eFdcReset();\n  p3FdcReset();\n#ifdef P3_PENTAGON\n  p5Reset();\n#endif\n  spp3eResetKeyboard();"),
    ("our clock survives the core's tact rebase",
     "  spp3eTacts -= by;\n",
     "  spp3eTacts -= by;\n  p3TimeBase += by;\n"),
    # The +2A/+3 gate array contends memory cycles only (Fuse: specplus3.c,
    # contend_delay_no_mreq = none, "no contended ports").  Klive applies the
    # 48K/128K's contention to the cycles that only put an address on the bus
    # (IR during INC rr, ADD HL,rr, EX (SP),HL ... -- $5Cxx with our IM 2
    # table, bank 5) and to I/O, which costs our tick some 13K T-states the
    # real machine does not spend.
    ("no contention on the cycles without MREQ",
     "static void SPP3E_CPU_NOINLINE spp3eCpuDelayAddressBusAccess(uint32_t address) {\n"
     "  if (spp3eIsContendedMemoryAddress(address) != 0u) {\n"
     "    spp3eApplyContentionDelay();\n"
     "  }\n"
     "}",
     "static void SPP3E_CPU_NOINLINE spp3eCpuDelayAddressBusAccess(uint32_t address) {\n"
     "  (void)address;\n"
     "}"),
    ("no contended ports",
     "static void SPP3E_CPU_NOINLINE spp3eDelayPortAccess(uint32_t address) {\n"
     "  const uint8_t lowBit = (address & 0x0001u) != 0u ? 1u : 0u;\n",
     "static void SPP3E_CPU_NOINLINE spp3eDelayPortAccess(uint32_t address) {\n"
     "  (void)address;\n"
     "  spp3eTactPlusN(4u);\n"
     "  return;\n"
     "  const uint8_t lowBit = (address & 0x0001u) != 0u ? 1u : 0u;\n"),
    # the stub's disk buffers and the tape's 5 MB are dead weight in every snapshot
    ("the stub's disk buffers shrink", "#define SPP3E_DISK_DATA_CAPACITY 0x80000u", "#define SPP3E_DISK_DATA_CAPACITY 0x200u"),
    ("the tape buffer shrinks", "#define SPP3E_TAPE_DATA_CAPACITY 0x400000u", "#define SPP3E_TAPE_DATA_CAPACITY 0x10000u"),
    ("the tape save buffer shrinks", "#define SPP3E_TAPE_SAVE_DATA_CAPACITY 0x100000u", "#define SPP3E_TAPE_SAVE_DATA_CAPACITY 0x1000u"),
]

OUR_EXPORTS = [
    "p3DiskImagePtr", "p3DiskImageCapacity", "p3FdcStatsPtr", "p3DriveSetup", "p3DiskInsert",
    "p3DiskEject", "p3DiskWrites", "p3DriveCylinder", "p3DriveMotor", "p3FdcPhase",
    "p3NowLow", "p3NowHigh", "p3FdcReadStatus", "p3FdcReadData", "p3FdcWriteData",
]
PENTAGON_EXPORTS = ["p5DosActive", "p5BetaIntrq", "p5BetaTrack", "p5BetaRead", "p5BetaWrite"]
# machine -> (the wasm, -D flags, extra exports)
MACHINES = {
    "plus3": ("plus3.wasm", [], []),
    "pentagon": ("pentagon.wasm", ["-DP3_PENTAGON"], PENTAGON_EXPORTS),
}


def fetch() -> None:
    for repo, commit, where in ((KLIVE_REPO, KLIVE_COMMIT, KLIVE_SRC), (FUSE_REPO, FUSE_COMMIT, FUSE_SRC)):
        if not where.exists():
            where.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(["git", "clone", "--filter=blob:none", repo, str(where)], check=True)
        subprocess.run(["git", "-C", str(where), "fetch", "--depth", "1", "origin", commit], check=False)
        subprocess.run(["git", "-C", str(where), "checkout", "--quiet", commit], check=True)


def head(where: Path) -> str:
    r = subprocess.run(["git", "-C", str(where), "rev-parse", "HEAD"], capture_output=True, text=True)
    return r.stdout.strip()


def patched_core() -> str:
    source = (KLIVE_SRC / CORE).read_text(encoding="utf-8")
    for what, old, new in PATCHES:
        n = source.count(old)
        if n != 1:
            raise SystemExit(f"build_emu: patch '{what}' matches {n} times in {CORE} "
                             f"(pinned Klive is {KLIVE_COMMIT[:10]}, checkout is {head(KLIVE_SRC)[:10]})")
        source = source.replace(old, new)
    return source + '\n#include "upd765.c"\n#ifdef P3_PENTAGON\n#include "wd1793.c"\n#include "pentagon.c"\n#endif\n'


def exports() -> list[str]:
    text = (KLIVE_SRC / EXPORTS_FROM).read_text(encoding="utf-8")
    block = text[text.index("const productionExports = ["):]
    block = block[:block.index("];")]
    names = [n for n in re.findall(r'"(\w+)"', block) if n != "memory"]
    return names + OUR_EXPORTS


def machine_roms(machine: str) -> list[Path]:
    if machine == "plus3":
        roms = [FUSE_SRC / "roms" / f"plus3-{i}.rom" for i in range(4)]
        for rom in roms:
            if not rom.exists():
                raise SystemExit(f"build_emu: {rom} missing (set FUSE_SRC, or run with --fetch)")
        return roms
    return pentagon_roms()


def build(out: Path, machines=tuple(MACHINES)) -> list[Path]:
    if not (KLIVE_SRC / CORE).exists():
        raise SystemExit(f"build_emu: no Klive checkout at {KLIVE_SRC} (set KLIVE_SRC, or run with --fetch)")
    if head(KLIVE_SRC) != KLIVE_COMMIT:
        print(f"build_emu: warning: Klive checkout is {head(KLIVE_SRC)[:10]}, pinned {KLIVE_COMMIT[:10]}")
    out.mkdir(parents=True, exist_ok=True)
    # the patched core is compiled in place of the original so its relative
    # #includes (the Z80, ULA, keyboard, PSG, beeper and tape) resolve
    core = KLIVE_SRC / CORE
    patched = core.with_name("spp3e-flashback.c")
    patched.write_text(patched_core(), encoding="utf-8", newline="\n")
    cc = clang()
    env = None
    if cc.parent.name == "bin":                     # wasm-ld and clang's DLLs live beside it
        import os
        env = dict(os.environ, PATH=str(cc.parent) + os.pathsep + os.environ.get("PATH", ""))
    built = []
    try:
        for machine in machines:
            name, defines, extra = MACHINES[machine]
            try:
                roms = machine_roms(machine)
            except FileNotFoundError as e:              # the Pentagon's ROMs are optional
                print(f"build_emu: no {machine}: {e}")
                continue
            wasm = out / name
            args = [str(cc), "--target=wasm32", "-std=c11", "-O3", "-ffreestanding", "-fno-builtin", "-nostdlib",
                    f"-I{HERE}", *defines, "-Wl,--no-entry", "-Wl,--export-memory", "-Wl,--strip-all",
                    f"-Wl,--initial-memory={MEMORY}", f"-Wl,--max-memory={MEMORY}"]
            args += [f"-Wl,--export={n}" for n in exports() + extra]
            args += [str(patched), "-o", str(wasm)]
            r = subprocess.run(args, env=env)
            if r.returncode != 0:
                raise SystemExit(f"build_emu: clang failed on the {machine} ({r.returncode})")
            for i, rom in enumerate(roms):
                shutil.copyfile(rom, out / f"{machine}-{i}.rom")
            built.append(wasm)
    finally:
        patched.unlink(missing_ok=True)
    return built


def up_to_date(out: Path = EMU_DIR) -> bool:
    sources = [Path(__file__), HERE / "upd765.c", HERE / "wd1793.c", HERE / "pentagon.c"]
    newest = max(p.stat().st_mtime for p in sources)
    for machine, (name, _, _) in MACHINES.items():
        if machine != "plus3":
            try:
                machine_roms(machine)
            except FileNotFoundError:                   # not buildable here: not out of date either
                continue
        wasm = out / name
        roms = 4 if machine == "plus3" else 3
        if not wasm.exists() or not all((out / f"{machine}-{i}.rom").exists() for i in range(roms)):
            return False
        if wasm.stat().st_mtime < newest:
            return False
    return True


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--fetch", action="store_true", help="clone Klive and Fuse at the pinned commits")
    ap.add_argument("--out", type=Path, default=EMU_DIR)
    a = ap.parse_args()
    if a.fetch:
        fetch()
    for wasm in build(a.out):
        print(f"{wasm} ({wasm.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
