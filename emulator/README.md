# The emulator's source

`plus3.wasm` is [Klive IDE](https://github.com/Dotneteer/kliveide)'s ZX
Spectrum +3 core (MIT, `LICENSE-klive`) with `upd765.c`, a uPD765A floppy
controller whose behaviour follows [Fuse](https://fuse-emulator.sourceforge.net/)'s
(GPL-2.0-or-later, `COPYING`); the whole is distributed under the GPL,
version 2 or later.

`build_emu.py` is the build: it takes Klive at the commit it names
(`KLIVE_COMMIT`), patches the core to route its floppy ports to
`upd765.c`, and compiles it to wasm32 with clang
(`python build_emu.py --fetch`; it expects to sit in `tools/emu/` of the
port's tree and reads `KLIVE_SRC`, `FUSE_SRC` and `CLANG` from the
environment).  `wd1793.c` and `pentagon.c` are the Pentagon build, which
this page does not use.  `plus3.mjs` is the JavaScript around the core,
inlined into `index.html`.
