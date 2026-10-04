// A ZX Spectrum +3 for the tests and the player page: Klive IDE's +3 core
// with tools/emu/upd765.c as its floppy controller, compiled to
// build/emu/plus3.wasm by tools/emu/build_emu.py.  No DOM, no node APIs:
// the caller hands over the wasm bytes and the four ROMs.
//
//   const m = await Plus3.create(wasmBytes, [rom0, rom1, rom2, rom3]);
//   m.insertDisk(0, dskBytes);          // drive A: an 80-track double-sided drive by default
//   m.runFrame();                       // one 50 Hz frame (70,908 T-states)
//   m.bank(3)[0x100];                   // RAM bank 3, offset $100
//   m.setKey(7, 0x01, true);            // half-row 7 ($7FFE), bit 0: SPACE

export const BANK_SIZE = 0x4000;
export const FRAME_TSTATES = 70908;

export class Plus3 {
  static async create(wasm, roms) {
    const module = wasm instanceof WebAssembly.Module ? wasm : await WebAssembly.compile(wasm);
    const instance = await WebAssembly.instantiate(module, {});
    return new Plus3(instance, roms);
  }

  constructor(instance, roms) {
    this.x = instance.exports;
    this.mem = this.x.memory;
    this.roms = roms;
    this.powerOn();
  }

  // --- the machine ------------------------------------------------------
  /** power on: RAM cleared, the ROMs in, drive A an 80-track double-sided
   *  drive with no disk, drive B absent */
  powerOn() {
    this.x.spp3eHardReset();
    const rom = new Uint8Array(this.mem.buffer, this.x.spp3eRomPtr(), 4 * BANK_SIZE);
    this.roms.forEach((r, i) => rom.set(r.subarray(0, BANK_SIZE), i * BANK_SIZE));
    this.setDrive(0, 2, 80);
    this.setDrive(1, 0, 0);
    this.x.spp3eReset();
  }
  /** the reset button: RAM kept, disks stay in */
  reset() { this.x.spp3eReset(); }

  runFrame() { this.x.spp3eExecuteFrame(); }
  step() { this.x.spp3eExecuteInstruction(); }
  get frames() { return this.x.spp3eGetFrames(); }
  /** T-states since power-on (exact up to 2^53) */
  get tstates() { return this.x.p3NowHigh() * 0x100000000 + this.x.p3NowLow(); }
  /** T-states into the current frame */
  get frameTstate() { return this.x.spp3eGetCurrentFrameTact(); }

  // --- the CPU ----------------------------------------------------------
  get pc() { return this.x.spp3eGetCpuPc(); }
  set pc(v) { this.x.spp3eSetCpuPc(v); }
  get sp() { return this.x.spp3eGetCpuSp(); }
  set sp(v) { this.x.spp3eSetCpuSp(v); }
  get halted() { return this.x.spp3eGetCpuHalted() !== 0; }
  get iff1() { return this.x.spp3eGetCpuIff1() !== 0; }
  get im() { return this.x.spp3eGetCpuInterruptMode(); }
  get i() { return this.x.spp3eGetCpuIr() >> 8; }

  // --- memory -----------------------------------------------------------
  /** a live view of RAM bank n (the CPU reads and writes the same bytes) */
  bank(n) { return new Uint8Array(this.mem.buffer, this.x.spp3eRamPtr() + n * BANK_SIZE, BANK_SIZE); }
  /** what the CPU sees at addr with the current paging */
  peek(addr) { return this.x.spp3eReadMemory(addr & 0xffff); }
  poke(addr, v) { this.x.spp3eWriteMemory(addr & 0xffff, v & 0xff); }
  /** paging: the bank at $C000, the ROM in $0000, special mode (config) */
  get paging() {
    return {
      bank: this.x.spp3eGetSelectedBank(),
      rom: this.x.spp3eGetSelectedRom(),
      special: this.x.spp3eGetInSpecialPagingMode() ? this.x.spp3eGetSpecialConfigMode() : null,
      shadowScreen: this.x.spp3eGetUseShadowScreen() !== 0,
      motor: this.x.spp3eGetDiskMotorOn() !== 0,
    };
  }
  out(port, value) { this.x.spp3eWritePort(port, value); }
  get border() { return this.x.spp3eGetBorderColor(); }

  // --- keyboard ---------------------------------------------------------
  /** half-row 0..7 ($FEFE .. $7FFE), bit mask 0x01..0x10 */
  setKey(row, mask, down) {
    for (let bit = 0; bit < 5; bit++) if (mask & (1 << bit)) this.x.spp3eSetKeyStatus(row * 5 + bit, down ? 1 : 0);
  }
  releaseKeys() { for (let k = 0; k < 40; k++) this.x.spp3eSetKeyStatus(k, 0); }

  // --- display ----------------------------------------------------------
  get screenWidth() { return this.x.spp3eGetScreenWidth(); }
  get screenHeight() { return this.x.spp3eGetScreenHeight(); }
  /** the rendered frame: one 0xAABBGGRR word a pixel, screenWidth x screenHeight */
  pixels() {
    return new Uint32Array(this.mem.buffer, this.x.spp3ePixelBufferPtr(), this.screenWidth * this.screenHeight);
  }

  // --- sound ------------------------------------------------------------
  setSampleRate(rate) { this.x.spp3eSetAudioSampleRate(rate); }
  /** this frame's samples: interleaved left, right (int16) */
  samples() {
    return new Int16Array(this.mem.buffer, this.x.spp3eAudioSamplesPtr(), this.x.spp3eGetAudioSampleCount() * 2);
  }

  // --- the drives -------------------------------------------------------
  /** heads 1 or 2, cylinders 40 or 80; heads 0 = no drive */
  setDrive(drive, heads, cylinders) { this.x.p3DriveSetup(drive, heads, cylinders); }
  /** a standard or extended DSK image into drive 0 (A) or 1 (B) */
  insertDisk(drive, bytes, { writeProtected = false } = {}) {
    const cap = this.x.p3DiskImageCapacity();
    if (bytes.length > cap) throw new Error(`disk image of ${bytes.length} bytes: the drive takes ${cap}`);
    new Uint8Array(this.mem.buffer, this.x.p3DiskImagePtr(drive), bytes.length).set(bytes);
    const r = this.x.p3DiskInsert(drive, bytes.length, writeProtected ? 1 : 0);
    if (r !== 0) throw new Error(`not a DSK image the drive can read (error ${r})`);
  }
  ejectDisk(drive) { this.x.p3DiskEject(drive); }
  /** the controller's counters and its last command and result bytes */
  fdc() {
    this.x.p5BetaIntrq?.();                    // the Pentagon's WD1793 catches up (the head unloads lazily)
    const v = new DataView(this.mem.buffer, this.x.p3FdcStatsPtr(), 40);
    const bytes = (o, n) => Array.from({ length: n }, (_, i) => v.getUint8(o + i));
    return {
      commands: v.getUint32(0, true), sectorsRead: v.getUint32(4, true), sectorsWritten: v.getUint32(8, true),
      overruns: v.getUint32(12, true), steps: v.getUint32(16, true), errors: v.getUint32(20, true),
      lastCommand: bytes(24, 9), lastResult: bytes(33, 7),
      cylinder: this.x.p3DriveCylinder(0), motor: this.x.p3DriveMotor(0) !== 0, phase: this.x.p3FdcPhase(),
    };
  }

  // --- state ------------------------------------------------------------
  /** the whole machine (the core keeps every bit of state in its memory) */
  snapshot() { return new Uint8Array(this.mem.buffer).slice(); }
  restore(snap) { new Uint8Array(this.mem.buffer).set(snap); }
}
