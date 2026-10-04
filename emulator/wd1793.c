/* wd1793.c -- the Pentagon's Beta 128 disk interface: a WD1793 (the
 * KR1818VG93 in most Pentagons) behind the TR-DOS ROM, for the Pentagon
 * build of tools/emu/build_emu.py.  It shares upd765.c's drives, disks
 * (TR-DOS .trd and DSK images) and clock.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The behaviour follows Fuse's peripherals/disk/wd_fdc.c and beta.c
 * (Gergely Szasz, Stuart Brady, Philip Kendall; GPL-2.0-or-later):
 *   - ports $1F command / status, $3F track, $5F sector, $7F data, $FF the
 *     interface's system register (write: bits 0-1 drive, bit 2 0 = reset
 *     the controller, bit 3 HLT, bit 4 0 = side 1, bit 5 MFM; read: bit 7
 *     INTRQ, bit 6 DRQ), decoded on the low byte, and only while the
 *     TR-DOS ROM is paged in (the core's trap: an instruction fetched from
 *     $3D00-$3DFF with the 48K ROM in pages it in, one fetched from RAM
 *     pages it out);
 *   - READY is HLD (the head loaded) and the drive's motor follows it: a
 *     type I command with h or V loads the head, a type II or III command
 *     with the head not loaded ends at once, not ready; the head unloads 15
 *     revolutions after a command ends;
 *   - steps at 6, 12, 20 or 30 ms, the verify's settle 15 ms;
 *   - a search gives up after 5 index pulses: RNF (type II / III) or SEEK
 *     ERROR (verify);
 *   - reading the status clears INTRQ, and so does writing a command.
 * As in upd765.c the disk turns in real time: an ID or a byte passes the
 * head at a definite T-state, DRQ rises as each data byte has passed, and
 * one not taken before the next arrives is lost data.  Everything is
 * computed from the T-state of the port access that sees it.
 */

#define WD_BUSY         0x01u
#define WD_INDEX        0x02u           /* type I */
#define WD_DRQ          0x02u           /* type II / III */
#define WD_TRACK0       0x04u           /* type I */
#define WD_LOST         0x04u           /* type II / III */
#define WD_CRC          0x08u
#define WD_SEEK_ERR     0x10u           /* type I */
#define WD_RNF          0x10u           /* type II / III */
#define WD_HEAD         0x20u           /* type I: head loaded */
#define WD_DELETED      0x20u           /* read: a deleted data mark */
#define WD_WP           0x40u
#define WD_NOT_READY    0x80u

#define WD_SEARCH_REVS  5u
#define WD_SETTLE       P3_MS(15)
#define WD_UNLOAD_REVS  15u

enum { WD_IDLE, WD_TYPE1, WD_READ, WD_WRITE, WD_ADDRESS, WD_DONE };

static const uint32_t wdRates[4] = { 6u, 12u, 20u, 30u };

static struct {
  uint8_t command, track, sector, data, status, sys;
  uint8_t drive, side, hld, intrq, statusType;
  uint64_t unloadAt;              /* the head unloads then (~0: never, a command is running) */
  uint8_t state;
  uint64_t doneAt;                /* WD_TYPE1 / WD_DONE: INTRQ at this T-state */
  uint8_t doneStatus;             /* status bits the command ends with */
  /* a sector transfer (read, write) or an ID (read address) */
  P3Sector *sec;
  uint64_t occStart;              /* the index time of its revolution */
  uint16_t len, taken;            /* bytes in it, bytes the CPU took (or gave) */
  uint8_t lost, multi;
  uint8_t idBytes[6];
  uint8_t stepOut;                /* the direction STEP repeats */
} wd;

static P3Drive *wdDrive(void) { return &p3Drives[wd.drive & 1u]; }

static uint8_t wdSpinning(uint64_t t) { return p3Ready(wdDrive(), t); }

static void wdUpdateHead(uint64_t now) {
  if (wd.hld && now >= wd.unloadAt) {
    wd.hld = 0u;
    p3SetDriveMotor(wdDrive(), 0u, wd.unloadAt);
  }
}

/* the head load: the Beta's drive motor follows it */
static void wdLoadHead(uint64_t now) {
  wdUpdateHead(now);
  wd.hld = 1u;
  p3SetDriveMotor(wdDrive(), 1u, now);
  wd.unloadAt = ~(uint64_t)0u;
}

static void wdFinish(uint64_t at, uint8_t status) {
  wd.state = WD_DONE;
  wd.doneAt = at;
  wd.doneStatus = status;
}

/* the next ID passing the head at or after `from` (and after the head has
   settled), before `deadline`; with `match`, the first whose track is TR,
   sector SR (and, for a type II command with C, side S) */
static P3Sector *wdFindId(uint64_t from, uint64_t deadline, uint8_t match, uint64_t *at, uint64_t *rev) {
  P3Drive *d = wdDrive();
  uint8_t side = d->heads > 1u ? wd.side : 0u;
  if (!d->loaded || side >= d->sides || d->cyl >= P3_MAX_CYL) return (P3Sector *)0;
  P3Track *t = &d->track[d->cyl][side];
  uint64_t start = from > d->settleUntil ? from : d->settleUntil;
  for (uint64_t r = p3RevStart(start); r < deadline; r += P3_T_REV) {
    for (uint32_t i = 0u; i < t->count; i++) {
      P3Sector *s = &t->s[i];
      uint64_t when = p3TimeAt(r, s->idEnd);
      if (when < start || !wdSpinning(when)) continue;
      if (when >= deadline) return (P3Sector *)0;
      if (match) {
        if (s->c != wd.track || s->r != wd.sector) continue;
        if ((wd.command & 0x02u) && ((s->h & 1u) != ((wd.command >> 3u) & 1u))) continue;
      }
      *at = when;
      *rev = r;
      return s;
    }
  }
  return (P3Sector *)0;
}

/* data byte k of the sector in hand has passed the head at */
static uint64_t wdByteTime(uint32_t k) {
  return p3TimeAt(wd.occStart, (uint32_t)wd.sec->dataPos + k + 1u);
}

static void wdSearchSector(uint64_t from) {
  uint64_t at, rev;
  uint64_t deadline = from + WD_SEARCH_REVS * P3_T_REV;
  P3Sector *s = wdFindId(from, deadline, 1u, &at, &rev);
  if (s == (P3Sector *)0) {
    wdFinish(deadline, WD_RNF);
    return;
  }
  wd.sec = s;
  wd.occStart = rev;
  wd.len = (uint16_t)(128u << (s->n > 3u ? 3u : s->n));
  wd.taken = 0u;
  wd.lost = 0u;
}

/* the transfer up to `now`: bytes that came and went untaken are lost; the
   sector ends two CRC bytes after its last byte */
static void wdUpdateTransfer(uint64_t now) {
  while (wd.state == WD_READ || wd.state == WD_WRITE) {
    if (wd.sec == (P3Sector *)0) return;
    uint64_t end = wdByteTime((uint32_t)wd.len + 1u);
    if (now < end) {
      /* a byte the CPU has not taken is overwritten when the next one comes */
      while (wd.taken + 1u < wd.len && now >= wdByteTime(wd.taken + 1u)) { wd.taken++; wd.lost = 1u; }
      return;
    }
    if (wd.taken < wd.len) wd.lost = 1u;
    uint8_t status = wd.lost ? WD_LOST : 0u;
    if (wd.state == WD_READ) {
      p3Stats.sectorsRead++;
      if (wd.sec->st2 & P3_ST2_DD) status |= WD_CRC;
      if (wd.sec->st2 & P3_ST2_CM) status |= WD_DELETED;
    } else {
      p3Stats.sectorsWritten++;
      wdDrive()->writes++;
    }
    if (wd.multi && !(status & WD_CRC)) {
      wd.sector++;
      wd.sec = (P3Sector *)0;
      wdSearchSector(end);
      if (wd.state == WD_DONE) { wd.doneStatus |= status; return; }
      continue;
    }
    wdFinish(end, status);
    return;
  }
}

static uint8_t wdBusyAt(uint64_t now) {
  wdUpdateHead(now);
  wdUpdateTransfer(now);
  if ((wd.state == WD_DONE || wd.state == WD_TYPE1 || wd.state == WD_ADDRESS) && now >= wd.doneAt) {
    if (wd.state == WD_ADDRESS) wd.sector = wd.idBytes[0];      /* READ ADDRESS leaves the track number in SR */
    if (wd.doneStatus & (WD_RNF | WD_LOST | WD_CRC)) p3Stats.errors++;
    wd.status = wd.doneStatus;
    wd.state = WD_IDLE;
    wd.intrq = 1u;
    wd.unloadAt = wd.doneAt + WD_UNLOAD_REVS * P3_T_REV;
  }
  return wd.state != WD_IDLE ? 1u : 0u;
}

static uint8_t wdDrq(uint64_t now) {
  if (wd.state == WD_READ && wd.sec != (P3Sector *)0) return now >= wdByteTime(wd.taken) && wd.taken < wd.len;
  if (wd.state == WD_WRITE && wd.sec != (P3Sector *)0) return now + P3_US(32) >= wdByteTime(wd.taken) && wd.taken < wd.len;
  if (wd.state == WD_ADDRESS) {
    return wd.taken < 6u && now >= p3TimeAt(wd.occStart, (uint32_t)wd.sec->idEnd - 5u + wd.taken);
  }
  return 0u;
}

static void wdCommand(uint8_t b, uint64_t now) {
  P3Drive *d = wdDrive();
  wdBusyAt(now);
  wd.intrq = 0u;
  if ((b & 0xf0u) == 0xd0u) {                                   /* type IV: force interrupt */
    wd.state = WD_IDLE;
    wd.statusType = 1u;
    wd.status &= (uint8_t)~(WD_BUSY | WD_WP | WD_CRC | WD_INDEX);
    if (b & 0x08u) wd.intrq = 1u;
    if (wd.hld && wd.unloadAt == ~(uint64_t)0u) wd.unloadAt = now + WD_UNLOAD_REVS * P3_T_REV;
    return;
  }
  if (wd.state != WD_IDLE) return;                             /* busy: ignored */
  p3Stats.commands++;
  p3Stats.lastCommand[0] = b;
  p3Stats.lastCommand[1] = wd.track; p3Stats.lastCommand[2] = wd.sector; p3Stats.lastCommand[3] = wd.data;
  wd.command = b;
  wd.sec = (P3Sector *)0;
  if (!(b & 0x80u)) {                                           /* type I: restore, seek, step */
    wd.statusType = 1u;
    if (b & 0x0cu) wdLoadHead(now);                             /* h or V */
    uint64_t rate = P3_MS(wdRates[b & 0x03u]);
    uint32_t steps = 0u;
    uint8_t cyl = d->cyl;
    if ((b & 0xf0u) == 0x00u) {                                 /* restore: out to track 0 */
      steps = cyl < 255u ? cyl : 255u;
      cyl = (uint8_t)(cyl - steps);
      wd.track = 0u;
      wd.stepOut = 1u;
    } else if ((b & 0xf0u) == 0x10u) {                          /* seek: to the data register's track */
      uint8_t to = wd.data;
      steps = to > wd.track ? to - wd.track : wd.track - to;
      if (to != wd.track) wd.stepOut = to < wd.track ? 1u : 0u;
      if (to > wd.track) cyl = (uint8_t)(cyl + steps < d->cylinders ? cyl + steps : d->cylinders - 1u);
      else cyl = (uint8_t)(cyl > steps ? cyl - steps : 0u);
      wd.track = to;
    } else {                                                    /* step (the last way), step in, step out */
      if ((b & 0x60u) == 0x40u) wd.stepOut = 0u;
      if ((b & 0x60u) == 0x60u) wd.stepOut = 1u;
      steps = 1u;
      if (wd.stepOut) { if (cyl > 0u) cyl--; if (b & 0x10u) wd.track--; }
      else { if (cyl + 1u < d->cylinders) cyl++; if (b & 0x10u) wd.track++; }
    }
    p3Stats.steps += steps;
    d->cyl = cyl;
    uint64_t stepped = now + (uint64_t)steps * rate;
    if (steps) d->settleUntil = stepped + WD_SETTLE;
    uint8_t status = 0u;
    uint64_t done = stepped;
    if (b & 0x04u) {                                            /* verify: an ID of this track */
      uint64_t from = stepped + WD_SETTLE, at, rev;
      uint64_t deadline = from + WD_SEARCH_REVS * P3_T_REV;
      P3Sector *s = (P3Sector *)0;
      for (uint64_t search = from; ;) {
        s = wdFindId(search, deadline, 0u, &at, &rev);
        if (s == (P3Sector *)0 || s->c == wd.track) break;
        search = at + 1u;
      }
      if (s == (P3Sector *)0) { status |= WD_SEEK_ERR; done = deadline; }
      else done = at;
    }
    wd.state = WD_TYPE1;
    wd.doneAt = done;
    wd.doneStatus = status;
    return;
  }
  wd.statusType = 2u;
  if (!wd.hld || now >= wd.unloadAt) {                          /* the Beta's READY is HLD */
    wd.state = WD_TYPE1;
    wd.statusType = 2u;
    wd.doneAt = now;
    wd.doneStatus = WD_NOT_READY;
    return;
  }
  wdLoadHead(now);
  uint64_t start = now + ((b & 0x04u) ? WD_SETTLE : 0u);
  if ((b & 0xc0u) == 0x80u) {                                   /* type II: read / write sector */
    if ((b & 0x20u) && d->wp) {
      wd.state = WD_TYPE1; wd.doneAt = now; wd.doneStatus = WD_WP;
      return;
    }
    wd.state = (b & 0x20u) ? WD_WRITE : WD_READ;
    wd.multi = (b & 0x10u) ? 1u : 0u;
    wdSearchSector(start);
    return;
  }
  if ((b & 0xf0u) == 0xc0u) {                                   /* type III: read address */
    uint64_t at, rev;
    uint64_t deadline = start + WD_SEARCH_REVS * P3_T_REV;
    P3Sector *s = wdFindId(start, deadline, 0u, &at, &rev);
    if (s == (P3Sector *)0) { wdFinish(deadline, WD_RNF); return; }
    wd.sec = s;
    wd.occStart = rev;
    wd.idBytes[0] = s->c; wd.idBytes[1] = s->h; wd.idBytes[2] = s->r; wd.idBytes[3] = s->n;
    wd.idBytes[4] = 0u; wd.idBytes[5] = 0u;
    wd.taken = 0u;
    wd.state = WD_ADDRESS;
    wd.doneAt = p3TimeAt(rev, (uint32_t)s->idEnd + 2u);        /* INTRQ once the CRC is checked */
    wd.doneStatus = 0u;
    return;
  }
  /* read track, write track (format): not emulated -- they end after a revolution, the
     write as write-protected, so TR-DOS's FORMAT fails cleanly */
  wdFinish(start + P3_T_REV, (b & 0x10u) ? WD_WP : 0u);
}

static uint8_t wdStatus(uint64_t now) {
  uint8_t busy = wdBusyAt(now);
  uint8_t s = wd.status & (uint8_t)~(WD_BUSY | WD_DRQ);
  if (busy) s = 0u;
  s |= busy ? WD_BUSY : 0u;
  P3Drive *d = wdDrive();
  if (wd.statusType == 1u) {
    if (d->cyl == 0u) s |= WD_TRACK0; else s &= (uint8_t)~WD_TRACK0;
    if (wd.hld && (wd.sys & 0x08u)) s |= WD_HEAD;
    if (d->loaded && wdSpinning(now) && now % P3_T_REV < P3_T_REV / 20u) s |= WD_INDEX;
    if (!d->loaded || d->wp) s |= WD_WP;
  } else if (wdDrq(now)) {
    s |= WD_DRQ;
  }
  if (!wd.hld) s |= WD_NOT_READY;
  return s;
}

/* ---------------------------------------------------------------- the interface's ports */

uint32_t p5BetaRead(uint32_t port) {
  uint64_t now = p3Now();
  switch (port & 0xffu) {
    case 0x1fu:
      wd.intrq = 0u;
      return wdStatus(now);
    case 0x3fu: return wd.track;
    case 0x5fu: return wd.sector;
    case 0x7fu: {
      wdBusyAt(now);
      if (wd.state == WD_READ && wd.sec != (P3Sector *)0 && wdDrq(now)) {
        uint32_t k = wd.taken++;
        wd.data = k < wd.sec->len ? p3DiskImage[wd.drive & 1u][wd.sec->data + k] : 0xe5u;
      } else if (wd.state == WD_ADDRESS && wdDrq(now)) {
        wd.data = wd.idBytes[wd.taken++];
      }
      return wd.data;
    }
    case 0xffu: {
      wdBusyAt(now);
      uint8_t v = 0u;
      if (wd.intrq) v |= 0x80u;
      if (wdDrq(now)) v |= 0x40u;
      return v;
    }
  }
  return 0xffu;
}

void p5BetaWrite(uint32_t port, uint32_t value) {
  uint64_t now = p3Now();
  uint8_t b = (uint8_t)value;
  switch (port & 0xffu) {
    case 0x1fu: wdCommand(b, now); break;
    case 0x3fu: wd.track = b; break;
    case 0x5fu: wd.sector = b; break;
    case 0x7fu:
      wdBusyAt(now);
      if (wd.state == WD_WRITE && wd.sec != (P3Sector *)0 && wdDrq(now)) {
        uint32_t k = wd.taken++;
        if (k < wd.sec->len) p3DiskImage[wd.drive & 1u][wd.sec->data + k] = b;
      }
      wd.data = b;
      break;
    case 0xffu:
      wdBusyAt(now);
      if (!(b & 0x04u)) {                                       /* reset: the head to track 0, as Fuse does */
        wd.state = WD_IDLE; wd.intrq = 0u; wd.track = 0u; wd.sector = 1u; wd.status = WD_TRACK0;
        wd.statusType = 1u;
        wdDrive()->cyl = 0u;
      }
      wd.sys = b;
      wd.drive = b & 0x01u;                                     /* two drives: B and D are A and C */
      wd.side = (b & 0x10u) ? 0u : 1u;
      break;
  }
}

void p5BetaReset(void) {
  p3Clear(&wd, sizeof(wd));
  wd.sys = 0x3cu;
  wd.status = WD_TRACK0;
  wd.statusType = 1u;
  wd.sector = 1u;
}

uint32_t p5BetaIntrq(void) { wdBusyAt(p3Now()); return wd.intrq; }
uint32_t p5BetaTrack(void) { return wd.track; }
