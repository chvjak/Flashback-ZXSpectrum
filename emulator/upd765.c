/* upd765.c -- the ZX Spectrum +3's floppy controller (NEC uPD765A) and its
 * drives, spliced into Klive IDE's +3 core by tools/emu/build_emu.py.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The command set, the status bits and the +3's wiring follow Fuse's
 * peripherals/disk/upd_fdc.c and fdd.c (Gergely Szasz, Philip Kendall,
 * Stuart Brady, Fredrick Meunier; GPL-2.0-or-later):
 *   - the controller runs at 4 MHz, so SPECIFY's times are doubled;
 *   - TC is not connected: a read that reaches EOT ends with abnormal
 *     termination and End of Cylinder (ST0 $40, ST1 $80) -- the +3's
 *     "success";
 *   - only US0 is wired: units 2 and 3 are drives 0 and 1;
 *   - a drive is ready two revolutions after its motor ($1FFD bit 3) starts,
 *     and stays ready for a revolution and a half after it stops (the disk
 *     still turning: a motor switched back on meanwhile needs no spin-up);
 *   - SENSE INTERRUPT with nothing pending is an invalid command ($80).
 * Unlike Fuse the disk turns in real time.  A track is 6250 bytes a
 * revolution (MFM, 250 kbit/s, 300 rpm: 113.5 T-states a byte) laid out
 * like an IBM System 34 track, so an ID or a data byte passes under the
 * head at a definite T-state: a READ DATA waits for its sector to come
 * round, its bytes arrive one every 32 us (RQM rises for each), a byte not
 * taken before the next one arrives is an overrun (ST1 OR), and after a
 * step the head needs 15 ms to settle before it reads an ID.  Every state
 * change is computed from the T-state of the port access that sees it;
 * nothing runs between accesses.
 *
 * Disk images: standard ("MV - CPC") and extended ("EXTENDED CPC DSK")
 * DSK files, and TR-DOS .trd images (the Pentagon build: its WD1793 is in
 * wd1793.c and shares this file's drives, disks and clock), as the host
 * copies them into p3DiskImage[drive].
 */

#ifdef P3_PENTAGON
#define P3_CLOCK        3500000u                 /* Pentagon CPU clock */
#else
#define P3_CLOCK        3546900u                 /* +3 CPU clock, T-states a second */
#endif
#define P3_T_REV        (P3_CLOCK / 5u)          /* 200 ms: one revolution at 300 rpm */
#define P3_BPT          6250u                    /* MFM bytes a revolution */
#define P3_MS(x)        ((uint64_t)(x) * P3_CLOCK / 1000u)
#define P3_US(x)        ((uint64_t)(x) * P3_CLOCK / 1000000u)
#define P3_SPINUP       P3_MS(400)               /* motor on -> ready: two revolutions (Fuse) */
#define P3_SPINDOWN     P3_MS(300)               /* motor off -> not ready: 1.5 revolutions (Fuse) */
#define P3_SETTLE       P3_MS(15)                /* head settle after the last step */
#define P3_RQM_DELAY    P3_US(12)                /* the FDC digesting a command byte / result byte */

#define P3_MAX_CYL      84u
#define P3_MAX_SEC      29u
#define P3_IMAGE_MAX    0x100000u                /* 1 MB: an 84-cylinder, two-sided extended DSK */

/* MSR */
#define P3_MSR_CB       0x10u
#define P3_MSR_EXM      0x20u
#define P3_MSR_DIO      0x40u
#define P3_MSR_RQM      0x80u
/* ST0..ST3 */
#define P3_ST0_NR       0x08u
#define P3_ST0_EC       0x10u
#define P3_ST0_SE       0x20u
#define P3_ST0_AT       0x40u
#define P3_ST0_IC       0x80u
#define P3_ST0_READY    0xc0u
#define P3_ST1_MA       0x01u
#define P3_ST1_NW       0x02u
#define P3_ST1_ND       0x04u
#define P3_ST1_OR       0x10u
#define P3_ST1_DE       0x20u
#define P3_ST1_EN       0x80u
#define P3_ST2_MD       0x01u
#define P3_ST2_BC       0x02u
#define P3_ST2_WC       0x10u
#define P3_ST2_DD       0x20u
#define P3_ST2_CM       0x40u
#define P3_ST3_T0       0x10u
#define P3_ST3_RY       0x20u
#define P3_ST3_WP       0x40u

enum { P3_CMD_READ_DATA, P3_CMD_WRITE_DATA, P3_CMD_READ_ID, P3_CMD_RECALIBRATE, P3_CMD_SENSE_INT,
       P3_CMD_SPECIFY, P3_CMD_SENSE_DRIVE, P3_CMD_SEEK, P3_CMD_INVALID };
enum { P3_PH_CMD, P3_PH_EXE, P3_PH_RES };
enum { P3_EX_NONE, P3_EX_SEARCH, P3_EX_XFER, P3_EX_DONE };

typedef struct {
  uint8_t id, mask, value, cmdLen, resLen;
} P3Cmd;

/* as Fuse's table: the first match wins */
static const P3Cmd p3Cmds[] = {
  { P3_CMD_READ_DATA,   0x1f, 0x06, 8, 7 },
  { P3_CMD_READ_DATA,   0x1f, 0x0c, 8, 7 },    /* deleted data */
  { P3_CMD_RECALIBRATE, 0xff, 0x07, 1, 0 },
  { P3_CMD_SEEK,        0xff, 0x0f, 2, 0 },
  { P3_CMD_WRITE_DATA,  0x3f, 0x05, 8, 7 },
  { P3_CMD_WRITE_DATA,  0x3f, 0x09, 8, 7 },    /* deleted data */
  { P3_CMD_READ_ID,     0xbf, 0x0a, 1, 7 },
  { P3_CMD_SENSE_INT,   0xff, 0x08, 0, 2 },
  { P3_CMD_SPECIFY,     0xff, 0x03, 2, 0 },
  { P3_CMD_SENSE_DRIVE, 0xff, 0x04, 1, 1 },
  /* READ DIAGNOSTIC, WRITE ID (format), SCAN and VERSION are not needed by
     the game or by +3DOS's reads: they end as invalid commands */
  { P3_CMD_INVALID,     0x00, 0x00, 0, 1 },
};

typedef struct {
  uint8_t c, h, r, n, st1, st2;
  uint16_t len;                   /* bytes stored in the image */
  uint32_t data;                  /* offset of its data in the image */
  uint16_t idEnd;                 /* track byte position just past the ID field's CRC */
  uint16_t dataPos;               /* track byte position of the first data byte */
} P3Sector;

typedef struct {
  uint8_t count;
  P3Sector s[P3_MAX_SEC];
} P3Track;

typedef struct {
  /* the drive */
  uint8_t present, heads, cylinders;
  uint8_t cyl;                    /* where the head is */
  uint8_t motor;
  uint64_t motorOnAt;
  uint64_t motorOffAt;
  uint64_t settleUntil;
  /* the disk in it */
  uint8_t loaded, wp, sides, tracks;
  uint32_t imageLength;
  uint32_t writes;                /* sectors written since the disk went in */
  P3Track track[P3_MAX_CYL][2];
} P3Drive;

typedef struct {
  /* diagnostics the host can read (p3FdcStatsPtr): keep the layout */
  uint32_t commands, sectorsRead, sectorsWritten, overruns, steps, errors;
  uint8_t lastCommand[9];
  uint8_t lastResult[7];
  uint8_t pad[2];
} P3Stats;

static uint8_t p3DiskImage[2][P3_IMAGE_MAX];
static P3Drive p3Drives[2];
static P3Stats p3Stats;
static uint64_t p3TimeBase;       /* moved along when Klive rebases its tact counter */

static struct {
  uint8_t phase;
  uint8_t command, cmdId, cmdLen, resLen;
  uint8_t cmd[9];                 /* the bytes after the command byte */
  uint8_t got;                    /* command bytes received, the command byte included */
  uint8_t res[7];
  uint8_t resPos;
  uint64_t rqmAt;                 /* RQM rises again at this T-state (command / result phases) */
  uint8_t st0, st1, st2;
  /* SPECIFY */
  uint64_t srt, hut, hlt;
  uint8_t nonDma;
  /* the head of the selected drive */
  uint8_t headLoaded;
  uint64_t headUnloadAt;
  /* seeks, per unit */
  uint8_t pcn[4], ncn[4];
  uint8_t seek[4];                /* 0 idle, 1 seek, 2 recalibrate, 4 done, 5 abnormal (EC), 6 not ready */
  uint8_t recalSteps[4];
  uint64_t nextStep[4];
  uint8_t intPending;             /* a READ/WRITE/READ ID result is being read (Fuse's intrq) */
  /* execution */
  uint8_t us, hd, mt, sk, deleted;
  uint8_t ex;
  uint64_t searchFrom, deadline;
  uint64_t occStart;              /* the index time of the revolution the sector is in */
  P3Sector *sec;
  uint16_t xferLen, xferIdx;
  uint8_t xferData;               /* the byte in the data register */
  uint8_t sawId;                  /* the search saw an ID field */
  uint64_t resultAt;
} p3Fdc;

static uint64_t p3Now(void) {
  return p3TimeBase + (uint64_t)cpu.tacts;
}

static void p3Clear(void *target, uint32_t length) {
  uint8_t *t = (uint8_t *)target;
  for (uint32_t i = 0u; i < length; i++) t[i] = 0u;
}

static int p3Match(const uint8_t *a, const char *b, uint32_t n) {
  for (uint32_t i = 0u; i < n; i++) {
    if (a[i] != (uint8_t)b[i]) return 0;
  }
  return 1;
}

/* ---------------------------------------------------------------- time */

/* the T-state at which `pos` bytes of the revolution starting at `rev` have passed */
static uint64_t p3TimeAt(uint64_t rev, uint32_t pos) {
  return rev + ((uint64_t)pos * P3_T_REV + P3_BPT - 1u) / P3_BPT;
}

/* the index pulse at or before t */
static uint64_t p3RevStart(uint64_t t) {
  return t - t % P3_T_REV;
}

/* the second index pulse after t: a search gives up there */
static uint64_t p3SecondIndex(uint64_t t) {
  return p3RevStart(t) + 2u * P3_T_REV;
}

static P3Drive *p3Drive(uint8_t us) {
  return &p3Drives[us & 1u];      /* the +3 wires US0 only */
}

/* up to speed: on for two revolutions, or off for less than 1.5 after being up */
static uint8_t p3Spinning(P3Drive *d, uint64_t now) {
  if (d->motor) return now >= d->motorOnAt + P3_SPINUP ? 1u : 0u;
  return d->motorOffAt >= d->motorOnAt + P3_SPINUP && now < d->motorOffAt + P3_SPINDOWN ? 1u : 0u;
}

static uint8_t p3Ready(P3Drive *d, uint64_t now) {
  return d->present && d->loaded && p3Spinning(d, now) ? 1u : 0u;
}

/* ---------------------------------------------------------------- disk images */

/* where each sector's ID and data are on the track: `fixed` bytes from the
   index to the first sector, then a sector: sync 12, IDAM 4, CHRN 4, CRC 2,
   gap 2, sync 12, DAM 4, data, CRC 2, gap 3 (shrunk while the track would
   not fit in 6250 bytes) */
static void p3LayOutGaps(P3Track *t, uint32_t fixed, uint32_t gap2, uint8_t gap3) {
  uint32_t perSector = 12u + 4u + 4u + 2u + gap2 + 12u + 4u + 2u;
  uint32_t total = fixed;
  for (uint32_t i = 0u; i < t->count; i++) total += perSector + (128u << (t->s[i].n > 6u ? 6u : t->s[i].n));
  uint32_t g = gap3;
  if (t->count > 0u && total + t->count * g > P3_BPT) {
    g = total >= P3_BPT ? 0u : (P3_BPT - total) / t->count;
  }
  uint32_t pos = fixed;
  for (uint32_t i = 0u; i < t->count; i++) {
    P3Sector *s = &t->s[i];
    pos += 12u + 4u + 4u + 2u;
    s->idEnd = (uint16_t)(pos < P3_BPT ? pos : P3_BPT - 1u);
    pos += gap2 + 12u + 4u;
    s->dataPos = (uint16_t)(pos < P3_BPT ? pos : P3_BPT - 1u);
    pos += (128u << (s->n > 6u ? 6u : s->n)) + 2u + g;
  }
}

/* a DSK track as Fuse's IBM34 gaps lay it out: gap 4a 80, sync 12, IAM 4,
   gap 1 50 before the first sector, gap 2 22, the image's gap 3 */
static void p3LayOut(P3Track *t, uint8_t gap3) {
  p3LayOutGaps(t, 80u + 12u + 4u + 50u, 22u, gap3);
}

/* a TR-DOS image: 16 sectors of 256 bytes a track, tracks in the order
   cylinder 0 side 0, cylinder 0 side 1 ..., the geometry in the disk-info
   sector (track 0, sector 9: byte $E3 $16-$19, byte $E7 $10).  Each track
   laid out as Fuse's open_trd does: sector ids 1-16 in the order 1, 9, 2,
   10 ... (interleave 2), TR-DOS gaps (10 bytes after the index, gap 2 22,
   gap 3 60). */
static uint32_t p3ParseTrd(uint32_t drive, uint32_t length) {
  P3Drive *d = &p3Drives[drive];
  const uint8_t *img = p3DiskImage[drive];
  const uint32_t trackBytes = 16u * 256u;
  if (length < 9u * 256u || length % 256u != 0u) return 1u;
  const uint8_t *info = img + 8u * 256u;
  if (info[0xe7] != 0x10u || info[0xe3] < 0x16u || info[0xe3] > 0x19u) return 1u;
  uint8_t sides = (info[0xe3] & 0x08u) ? 1u : 2u;
  uint32_t cylinders = (info[0xe3] & 0x01u) ? 40u : 80u;
  while (cylinders < P3_MAX_CYL - 1u && cylinders * sides * trackBytes < length) cylinders++;
  p3Clear(d->track, sizeof(d->track));
  static const uint8_t order[16] = {1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15, 8, 16};
  for (uint32_t c = 0u; c < cylinders; c++) {
    for (uint32_t h = 0u; h < sides; h++) {
      uint32_t base = (c * sides + h) * trackBytes;
      if (base >= length) break;
      P3Track *t = &d->track[c][h];
      t->count = 16u;
      for (uint32_t j = 0u; j < 16u; j++) {
        P3Sector *s = &t->s[j];
        s->c = (uint8_t)c; s->h = (uint8_t)h; s->r = order[j]; s->n = 1u;
        s->st1 = 0u; s->st2 = 0u;
        s->data = base + (uint32_t)(order[j] - 1u) * 256u;
        s->len = s->data + 256u <= length ? 256u : 0u;
      }
      p3LayOutGaps(t, 10u, 22u, 60u);
    }
  }
  d->tracks = (uint8_t)cylinders;
  d->sides = sides;
  d->imageLength = length;
  return 0u;
}

/* parse the image in p3DiskImage[drive]; 0 = ok */
static uint32_t p3ParseImage(uint32_t drive, uint32_t length) {
  P3Drive *d = &p3Drives[drive];
  const uint8_t *img = p3DiskImage[drive];
  uint8_t extended;
  if (length >= 256u && p3Match(img, "EXTENDED CPC DSK File", 21u)) extended = 1u;
  else if (length >= 256u && p3Match(img, "MV - CPC", 8u)) extended = 0u;
  else return p3ParseTrd(drive, length);
  uint8_t tracks = img[0x30], sides = img[0x31];
  if (sides < 1u || sides > 2u || tracks == 0u || tracks > P3_MAX_CYL) return 2u;
  uint32_t trackSize = (uint32_t)img[0x32] | ((uint32_t)img[0x33] << 8u);
  p3Clear(d->track, sizeof(d->track));
  uint32_t offset = 256u;
  for (uint32_t i = 0u; i < (uint32_t)tracks * sides; i++) {
    uint32_t size = extended ? (uint32_t)img[0x34 + i] * 256u : trackSize;
    if (size == 0u) continue;                              /* unformatted */
    if (offset + 256u > length) break;                     /* the header promised more than the file has */
    const uint8_t *ti = img + offset;
    if (!p3Match(ti, "Track-Info", 10u)) return 3u;
    uint8_t cyl = ti[0x10], head = ti[0x11];
    if (cyl >= P3_MAX_CYL || head > 1u) return 4u;
    P3Track *t = &d->track[cyl][head];
    uint8_t count = ti[0x15];
    if (count > P3_MAX_SEC) count = P3_MAX_SEC;
    t->count = count;
    uint32_t data = offset + 256u;
    for (uint32_t j = 0u; j < count; j++) {
      const uint8_t *si = ti + 0x18 + 8u * j;
      P3Sector *s = &t->s[j];
      s->c = si[0]; s->h = si[1]; s->r = si[2]; s->n = si[3];
      s->st1 = si[4]; s->st2 = si[5];
      uint32_t len = extended ? ((uint32_t)si[6] | ((uint32_t)si[7] << 8u)) : (128u << (ti[0x14] > 6u ? 6u : ti[0x14]));
      if (data + len > length) len = data < length ? length - data : 0u;
      s->len = (uint16_t)len;
      s->data = data;
      data += len;
    }
    p3LayOut(t, ti[0x16]);
    offset += size;
  }
  d->tracks = tracks;
  d->sides = sides;
  d->imageLength = length;
  return 0u;
}

/* ---------------------------------------------------------------- the controller */

static void p3FinishExecution(uint64_t at);

static void p3SetResult(uint8_t length) {
  p3Fdc.resLen = length;
  p3Fdc.resPos = 0u;
  for (uint32_t i = 0u; i < 7u; i++) p3Stats.lastResult[i] = i < length ? p3Fdc.res[i] : 0u;
}

/* the result bytes of a read / write / READ ID: ST0-2, C, H, R, N */
static void p3RwResult(void) {
  p3Fdc.res[0] = p3Fdc.st0;
  p3Fdc.res[1] = p3Fdc.st1;
  p3Fdc.res[2] = p3Fdc.st2;
  p3Fdc.res[3] = p3Fdc.cmd[1];
  p3Fdc.res[4] = p3Fdc.cmd[2];
  p3Fdc.res[5] = p3Fdc.cmd[3];
  p3Fdc.res[6] = p3Fdc.cmd[4];
  p3SetResult(7u);
  if ((p3Fdc.st0 & 0xc0u) != 0u && !((p3Fdc.st0 & P3_ST0_AT) && p3Fdc.st1 == P3_ST1_EN && p3Fdc.st2 == 0u)) {
    p3Stats.errors++;
  }
}

static uint8_t p3SeekBusyBits(uint64_t now);

/* the next sector matching C H R N passing the head at or after `from`, before
   the second index pulse; NULL if none.  Sets WC/BC/ND as the search goes. */
static P3Sector *p3FindSector(uint64_t from, uint64_t *idTime, uint64_t *revOut) {
  P3Drive *d = p3Drive(p3Fdc.us);
  uint64_t deadline = p3SecondIndex(from);
  p3Fdc.deadline = deadline;
  if (!p3Ready(d, from)) return (P3Sector *)0;
  uint8_t side = d->heads > 1u ? p3Fdc.hd : 0u;
  if (side >= d->sides || d->cyl >= P3_MAX_CYL) return (P3Sector *)0;
  P3Track *t = &d->track[d->cyl][side];
  if (t->count == 0u) return (P3Sector *)0;
  uint64_t start = from > d->settleUntil ? from : d->settleUntil;
  for (uint64_t rev = p3RevStart(from); rev < deadline; rev += P3_T_REV) {
    for (uint32_t i = 0u; i < t->count; i++) {
      P3Sector *s = &t->s[i];
      uint64_t at = p3TimeAt(rev, s->idEnd);
      if (at < start) continue;
      if (at >= deadline) return (P3Sector *)0;
      p3Fdc.sawId = 1u;
      if ((s->st1 & P3_ST1_DE) && !(s->st2 & P3_ST2_DD)) continue;   /* CRC error in the ID: not seen */
      p3Fdc.st2 &= (uint8_t)~(P3_ST2_WC | P3_ST2_BC);
      if (s->c != p3Fdc.cmd[1]) {
        p3Fdc.st2 |= P3_ST2_WC;
        if (s->c == 0xffu) p3Fdc.st2 |= P3_ST2_BC;
        continue;
      }
      if (s->r == p3Fdc.cmd[3] && s->h == p3Fdc.cmd[2]) {
        if (s->n != p3Fdc.cmd[4]) continue;
        *idTime = at;
        *revOut = rev;
        return s;
      }
    }
  }
  return (P3Sector *)0;
}

/* look for sector R from `from`; set up the transfer or the failure */
static void p3StartSector(uint64_t from) {
  uint64_t idTime, rev;
  p3Fdc.sawId = 0u;
  P3Sector *s = p3FindSector(from, &idTime, &rev);
  if (s == (P3Sector *)0) {
    p3Fdc.st0 |= P3_ST0_AT;
    p3Fdc.st1 |= P3_ST1_ND;
    if (!p3Fdc.sawId) p3Fdc.st1 |= P3_ST1_MA;
    p3FinishExecution(p3Fdc.deadline);
    return;
  }
  uint8_t deletedMark = (s->st2 & P3_ST2_CM) ? 1u : 0u;
  if (deletedMark != p3Fdc.deleted && p3Fdc.cmdId == P3_CMD_READ_DATA) {
    p3Fdc.st2 |= P3_ST2_CM;
    if (p3Fdc.sk) {                                 /* skip it */
      p3Fdc.cmd[3]++;
      p3StartSector(p3TimeAt(rev, s->dataPos));
      return;
    }
  }
  p3Fdc.sec = s;
  p3Fdc.occStart = rev;
  p3Fdc.xferIdx = 0u;
  uint32_t n = p3Fdc.cmd[4] > 6u ? 6u : p3Fdc.cmd[4];
  uint32_t rlen = 128u << n;
  if (p3Fdc.cmd[4] == 0u && p3Fdc.cmd[7] < 128u) rlen = p3Fdc.cmd[7];
  p3Fdc.xferLen = (uint16_t)rlen;
  p3Fdc.ex = P3_EX_XFER;
}

/* data byte k of the sector in hand is under the head (fully read) at */
static uint64_t p3ByteTime(uint32_t k) {
  return p3TimeAt(p3Fdc.occStart, (uint32_t)p3Fdc.sec->dataPos + k + 1u);
}

static void p3FinishExecution(uint64_t at) {
  p3Fdc.ex = P3_EX_DONE;
  p3Fdc.resultAt = at;
  p3Fdc.headUnloadAt = at + p3Fdc.hut;
  if (p3Fdc.cmdId == P3_CMD_READ_ID) {
    p3Fdc.res[0] = p3Fdc.st0; p3Fdc.res[1] = p3Fdc.st1; p3Fdc.res[2] = p3Fdc.st2;
    p3SetResult(7u);
  } else {
    p3RwResult();
  }
}

/* the sector in hand has been transferred: its CRC, then the next one or the end */
static void p3SectorDone(void) {
  P3Sector *s = p3Fdc.sec;
  uint64_t after = p3ByteTime((uint32_t)p3Fdc.xferLen + 1u);        /* the two CRC bytes */
  if (p3Fdc.cmdId == P3_CMD_READ_DATA) p3Stats.sectorsRead++;
  else p3Stats.sectorsWritten++;
  if (p3Fdc.cmdId == P3_CMD_READ_DATA && (s->st2 & P3_ST2_DD)) {
    p3Fdc.st0 |= P3_ST0_AT;
    p3Fdc.st1 |= P3_ST1_DE;
    p3Fdc.st2 |= P3_ST2_DD;
    p3FinishExecution(after);
    return;
  }
  uint8_t deletedMark = (s->st2 & P3_ST2_CM) ? 1u : 0u;
  if (p3Fdc.cmdId == P3_CMD_READ_DATA && deletedMark != p3Fdc.deleted) {
    if (p3Fdc.cmd[5] > p3Fdc.cmd[3]) p3Fdc.st0 |= P3_ST0_AT;
    p3FinishExecution(after);
    return;
  }
  if (p3Fdc.cmd[3] == p3Fdc.cmd[5]) {                                /* R = EOT */
    if (p3Fdc.mt && p3Fdc.hd == 0u) {
      p3Fdc.hd = 1u;
      p3Fdc.cmd[2] ^= 1u;
      p3Fdc.cmd[3] = 1u;
      p3Fdc.st0 |= 0x04u;
      p3StartSector(after);
      return;
    }
    /* no TC on the +3: the controller carries on past EOT and stops with EN */
    if (p3Fdc.st0 == (uint8_t)((p3Fdc.hd << 2u) | p3Fdc.us) && p3Fdc.st1 == 0u) {
      p3Fdc.st0 |= P3_ST0_AT;
      p3Fdc.st1 |= P3_ST1_EN;
    }
    p3FinishExecution(after);
    return;
  }
  p3Fdc.cmd[3]++;
  p3StartSector(after);
}

static void p3Overrun(uint64_t at) {
  p3Stats.overruns++;
  p3Fdc.st0 |= P3_ST0_AT;
  p3Fdc.st1 |= P3_ST1_OR;
  p3FinishExecution(at);
}

/* bring the execution phase up to `now` */
static void p3Update(uint64_t now) {
  if (p3Fdc.phase != P3_PH_EXE) return;
  if (p3Fdc.ex == P3_EX_SEARCH) {
    if (now < p3Fdc.searchFrom) return;
    p3StartSector(p3Fdc.searchFrom);
  }
  while (p3Fdc.ex == P3_EX_XFER) {
    /* the byte waiting in the data register is lost when the next one arrives */
    uint64_t next = p3ByteTime((uint32_t)p3Fdc.xferIdx + 1u);
    if (now < next) break;
    p3Overrun(next);
  }
  if (p3Fdc.ex == P3_EX_DONE && now >= p3Fdc.resultAt) {
    p3Fdc.phase = P3_PH_RES;
    p3Fdc.intPending = 1u;
    p3Fdc.rqmAt = p3Fdc.resultAt;
  }
}

static uint8_t p3SeekDone(uint32_t i, uint8_t state) {
  p3Fdc.seek[i] = state;
  return state;
}

/* seeks: step each seeking unit up to `now` */
static void p3UpdateSeeks(uint64_t now) {
  for (uint32_t i = 0u; i < 4u; i++) {
    while (p3Fdc.seek[i] == 1u || p3Fdc.seek[i] == 2u) {
      P3Drive *d = p3Drive((uint8_t)i);
      uint64_t t = p3Fdc.nextStep[i];
      if (t > now) break;
      /* the controller steps only a ready drive (datasheet; Fuse's seek_step) */
      if (!p3Ready(d, t)) { p3SeekDone(i, 6u); break; }
      if (p3Fdc.seek[i] == 2u && d->cyl == 0u) {                       /* track 0 found */
        p3Fdc.pcn[i] = 0u;
        p3SeekDone(i, 4u);
        break;
      }
      if (p3Fdc.seek[i] == 2u && p3Fdc.recalSteps[i] == 0u) {         /* 77 steps and no track 0 */
        p3Fdc.pcn[i] = 0u;
        p3SeekDone(i, 5u);
        break;
      }
      if (p3Fdc.seek[i] == 1u && p3Fdc.pcn[i] == p3Fdc.ncn[i]) {
        p3SeekDone(i, 4u);
        break;
      }
      /* a step pulse */
      uint8_t out = p3Fdc.seek[i] == 2u || p3Fdc.pcn[i] > p3Fdc.ncn[i];
      if (out) {
        if (d->cyl > 0u) d->cyl--;
        if (p3Fdc.seek[i] == 2u) p3Fdc.recalSteps[i]--;
        else p3Fdc.pcn[i]--;
      } else {
        if (d->cyl + 1u < d->cylinders) d->cyl++;
        p3Fdc.pcn[i]++;
      }
      p3Stats.steps++;
      d->settleUntil = t + P3_SETTLE;
      p3Fdc.nextStep[i] = t + p3Fdc.srt;
    }
  }
}

static uint8_t p3SeekBusyBits(uint64_t now) {
  p3UpdateSeeks(now);
  uint8_t bits = 0u;
  for (uint32_t i = 0u; i < 4u; i++) {
    if (p3Fdc.seek[i] == 1u || p3Fdc.seek[i] == 2u) bits |= (uint8_t)(1u << i);
  }
  return bits;
}

static void p3Select(uint8_t first) {
  p3Fdc.us = first & 0x03u;
  p3Fdc.hd = (first >> 2u) & 0x01u;
}

/* all command bytes are in: start the command */
static void p3Execute(uint64_t now) {
  p3Stats.commands++;
  p3Stats.lastCommand[0] = p3Fdc.command;
  for (uint32_t i = 0u; i < 8u; i++) p3Stats.lastCommand[i + 1u] = p3Fdc.cmd[i];
  uint8_t id = p3Fdc.cmdId;
  if (id != P3_CMD_SENSE_INT && id != P3_CMD_SPECIFY && id != P3_CMD_INVALID) p3Select(p3Fdc.cmd[0]);
  p3Fdc.rqmAt = now + P3_RQM_DELAY;
  switch (id) {
    case P3_CMD_INVALID:
      p3Fdc.res[0] = 0x80u;
      p3SetResult(1u);
      p3Fdc.phase = P3_PH_RES;
      return;
    case P3_CMD_SPECIFY:
      /* 4 MHz: every time doubled */
      p3Fdc.srt = P3_MS(2u * (16u - (p3Fdc.cmd[0] >> 4u)));
      p3Fdc.hut = P3_MS(2u * ((p3Fdc.cmd[0] & 0x0fu) == 0u ? 128u : (uint32_t)(p3Fdc.cmd[0] & 0x0fu) * 16u));
      p3Fdc.hlt = P3_MS(2u * ((p3Fdc.cmd[1] & 0xfeu) == 0u ? 256u : (uint32_t)(p3Fdc.cmd[1] & 0xfeu)));
      p3Fdc.nonDma = p3Fdc.cmd[1] & 0x01u;
      p3Fdc.phase = P3_PH_CMD;
      return;
    case P3_CMD_SENSE_DRIVE: {
      P3Drive *d = p3Drive(p3Fdc.us);
      uint8_t st3 = (uint8_t)(p3Fdc.us | (p3Fdc.hd << 2u));
      /* no drive: every line low (the ROM's drive B test); an empty drive is write-protected */
      if (d->present && (!d->loaded || d->wp)) st3 |= P3_ST3_WP;
      if (d->present && d->cyl == 0u) st3 |= P3_ST3_T0;
      if (p3Ready(d, now)) st3 |= P3_ST3_RY;
      p3Fdc.res[0] = st3;
      p3SetResult(1u);
      p3Fdc.phase = P3_PH_RES;
      return;
    }
    case P3_CMD_SENSE_INT: {
      p3UpdateSeeks(now);
      for (uint32_t i = 0u; i < 4u; i++) {
        if (p3Fdc.seek[i] >= 4u) {
          uint8_t st0 = (uint8_t)(P3_ST0_SE | i);
          if (p3Fdc.seek[i] == 5u) st0 |= P3_ST0_AT | P3_ST0_EC;
          else if (p3Fdc.seek[i] == 6u) st0 |= P3_ST0_READY | P3_ST0_NR;
          p3Fdc.seek[i] = 0u;
          p3Fdc.res[0] = st0;
          p3Fdc.res[1] = p3Fdc.pcn[i];
          p3SetResult(2u);
          p3Fdc.phase = P3_PH_RES;
          return;
        }
      }
      p3Fdc.res[0] = 0x80u;                        /* nothing pending: invalid */
      p3SetResult(1u);
      p3Fdc.phase = P3_PH_RES;
      return;
    }
    case P3_CMD_SEEK:
    case P3_CMD_RECALIBRATE: {
      uint8_t u = p3Fdc.us;
      p3UpdateSeeks(now);
      p3Fdc.phase = P3_PH_CMD;
      if (p3Fdc.seek[u] == 1u || p3Fdc.seek[u] == 2u) return;        /* one already running */
      if (id == P3_CMD_SEEK) {
        p3Fdc.ncn[u] = p3Fdc.cmd[1];
        p3Fdc.seek[u] = 1u;
      } else {
        p3Fdc.ncn[u] = 0u;
        p3Fdc.recalSteps[u] = 77u;
        p3Fdc.seek[u] = 2u;
      }
      p3Fdc.nextStep[u] = now;
      p3UpdateSeeks(now);
      return;
    }
    case P3_CMD_READ_ID:
    case P3_CMD_READ_DATA:
    case P3_CMD_WRITE_DATA: {
      P3Drive *d = p3Drive(p3Fdc.us);
      p3Fdc.st0 = (uint8_t)((p3Fdc.hd << 2u) | p3Fdc.us);
      p3Fdc.st1 = 0u;
      p3Fdc.st2 = 0u;
      p3Fdc.intPending = 0u;
      p3Fdc.phase = P3_PH_EXE;
      if (!p3Ready(d, now)) {
        p3Fdc.st0 |= P3_ST0_AT | P3_ST0_NR;
        p3FinishExecution(now);
        return;
      }
      if (id == P3_CMD_WRITE_DATA && d->wp) {
        p3Fdc.st0 |= P3_ST0_AT;
        p3Fdc.st1 |= P3_ST1_NW;
        p3FinishExecution(now);
        return;
      }
      uint64_t start = now;
      if (!p3Fdc.headLoaded || now >= p3Fdc.headUnloadAt) start = now + p3Fdc.hlt;
      p3Fdc.headLoaded = 1u;
      p3Fdc.headUnloadAt = ~(uint64_t)0u;
      if (id == P3_CMD_READ_ID) {
        /* the next ID, whatever it is */
        uint8_t side = d->heads > 1u ? p3Fdc.hd : 0u;
        P3Track *t = (side < d->sides && d->cyl < P3_MAX_CYL) ? &d->track[d->cyl][side] : (P3Track *)0;
        uint64_t deadline = p3SecondIndex(start);
        uint64_t from = start > d->settleUntil ? start : d->settleUntil;
        uint64_t best = deadline;
        P3Sector *found = (P3Sector *)0;
        if (t != (P3Track *)0) {
          for (uint64_t rev = p3RevStart(start); rev < deadline && found == (P3Sector *)0; rev += P3_T_REV) {
            for (uint32_t i = 0u; i < t->count; i++) {
              uint64_t at = p3TimeAt(rev, t->s[i].idEnd);
              if (at >= from && at < best) { best = at; found = &t->s[i]; }
            }
          }
        }
        if (found == (P3Sector *)0) {
          p3Fdc.st0 |= P3_ST0_AT;
          p3Fdc.st1 |= P3_ST1_MA | P3_ST1_ND;
          p3Fdc.cmd[1] = 0u; p3Fdc.cmd[2] = 0u; p3Fdc.cmd[3] = 0u; p3Fdc.cmd[4] = 0u;
        } else {
          p3Fdc.cmd[1] = found->c; p3Fdc.cmd[2] = found->h; p3Fdc.cmd[3] = found->r; p3Fdc.cmd[4] = found->n;
        }
        p3Fdc.res[3] = p3Fdc.cmd[1]; p3Fdc.res[4] = p3Fdc.cmd[2];
        p3Fdc.res[5] = p3Fdc.cmd[3]; p3Fdc.res[6] = p3Fdc.cmd[4];
        p3FinishExecution(best);
        return;
      }
      p3Fdc.mt = (p3Fdc.command >> 7u) & 1u;
      p3Fdc.sk = (p3Fdc.command >> 5u) & 1u;
      p3Fdc.deleted = (p3Fdc.command & 0x08u) ? 1u : 0u;
      p3Fdc.searchFrom = start;
      p3Fdc.ex = P3_EX_SEARCH;
      p3Update(now);
      return;
    }
  }
}

/* ---------------------------------------------------------------- ports */

uint32_t p3FdcReadStatus(void) {
  uint64_t now = p3Now();
  p3Update(now);
  uint8_t msr = p3SeekBusyBits(now);
  switch (p3Fdc.phase) {
    case P3_PH_CMD:
      if (p3Fdc.got > 0u) msr |= P3_MSR_CB;
      if (now >= p3Fdc.rqmAt) msr |= P3_MSR_RQM;
      break;
    case P3_PH_EXE:
      msr |= P3_MSR_CB;
      if (p3Fdc.nonDma) msr |= P3_MSR_EXM;
      if (p3Fdc.cmdId == P3_CMD_READ_DATA) msr |= P3_MSR_DIO;
      if (p3Fdc.ex == P3_EX_XFER) {
        uint64_t ready = p3Fdc.cmdId == P3_CMD_READ_DATA ? p3ByteTime(p3Fdc.xferIdx)
                                                          : p3ByteTime(p3Fdc.xferIdx) - P3_US(32);
        if (now >= ready) msr |= P3_MSR_RQM;
      }
      break;
    case P3_PH_RES:
      msr |= P3_MSR_CB | P3_MSR_DIO;
      if (now >= p3Fdc.rqmAt) msr |= P3_MSR_RQM;
      break;
  }
  return msr;
}

uint32_t p3FdcReadData(void) {
  uint64_t now = p3Now();
  p3Update(now);
  if (p3Fdc.phase == P3_PH_EXE) {
    if (p3Fdc.ex != P3_EX_XFER || p3Fdc.cmdId != P3_CMD_READ_DATA) return p3Fdc.xferData;
    if (now < p3ByteTime(p3Fdc.xferIdx)) return p3Fdc.xferData;   /* RQM low: the old byte */
    P3Sector *s = p3Fdc.sec;
    uint32_t k = p3Fdc.xferIdx;
    p3Fdc.xferData = k < s->len ? p3DiskImage[p3Fdc.us & 1u][s->data + k] : 0xe5u;
    p3Fdc.xferIdx++;
    if (p3Fdc.xferIdx >= p3Fdc.xferLen) p3SectorDone();
    return p3Fdc.xferData;
  }
  if (p3Fdc.phase != P3_PH_RES || now < p3Fdc.rqmAt) return 0xffu;
  uint8_t value = p3Fdc.res[p3Fdc.resPos++];
  p3Fdc.rqmAt = now + P3_RQM_DELAY;
  if (p3Fdc.resPos >= p3Fdc.resLen) {
    p3Fdc.phase = P3_PH_CMD;
    p3Fdc.got = 0u;
    p3Fdc.ex = P3_EX_NONE;
    p3Fdc.intPending = 0u;
  }
  return value;
}

void p3FdcWriteData(uint32_t value) {
  uint64_t now = p3Now();
  p3Update(now);
  uint8_t b = (uint8_t)value;
  if (p3Fdc.phase == P3_PH_EXE) {
    if (p3Fdc.ex != P3_EX_XFER || p3Fdc.cmdId != P3_CMD_WRITE_DATA) return;
    if (now < p3ByteTime(p3Fdc.xferIdx) - P3_US(32)) return;       /* RQM low: ignored */
    P3Sector *s = p3Fdc.sec;
    uint32_t k = p3Fdc.xferIdx;
    if (k < s->len) p3DiskImage[p3Fdc.us & 1u][s->data + k] = b;
    p3Fdc.xferIdx++;
    if (p3Fdc.xferIdx >= p3Fdc.xferLen) {
      p3Drive(p3Fdc.us)->writes++;
      p3SectorDone();
    }
    return;
  }
  if (p3Fdc.phase != P3_PH_CMD || now < p3Fdc.rqmAt) return;       /* RQM low: the byte is lost */
  p3Fdc.rqmAt = now + P3_RQM_DELAY;
  if (p3Fdc.got == 0u) {
    const P3Cmd *c = p3Cmds;
    while (c->id != P3_CMD_INVALID && (b & c->mask) != c->value) c++;
    p3Fdc.command = b;
    p3Fdc.cmdId = c->id;
    p3Fdc.cmdLen = c->cmdLen;
    p3Fdc.resLen = c->resLen;
    for (uint32_t i = 0u; i < 9u; i++) p3Fdc.cmd[i] = 0u;
    /* SENSE INTERRUPT with no seek finished is an invalid command */
    if (c->id == P3_CMD_SENSE_INT) {
      p3UpdateSeeks(now);
      uint8_t any = 0u;
      for (uint32_t i = 0u; i < 4u; i++) if (p3Fdc.seek[i] >= 4u) any = 1u;
      if (!any) { p3Fdc.cmdId = P3_CMD_INVALID; p3Fdc.cmdLen = 0u; }
    }
  } else {
    p3Fdc.cmd[p3Fdc.got - 1u] = b;
  }
  p3Fdc.got++;
  if (p3Fdc.got > p3Fdc.cmdLen) {
    p3Fdc.got = 0u;
    p3Execute(now);
  }
}

/* a drive's motor on or off at `now` */
static void p3SetDriveMotor(P3Drive *d, uint8_t on, uint64_t now) {
  if (on && !d->motor) {
    /* still turning from before: no spin-up */
    d->motorOnAt = p3Spinning(d, now) ? (now >= P3_SPINUP ? now - P3_SPINUP : 0u) : now;
  } else if (!on && d->motor) {
    d->motorOffAt = now;
  }
  d->motor = on ? 1u : 0u;
}

void p3FdcSetMotor(uint32_t on) {
  uint64_t now = p3Now();
  for (uint32_t i = 0u; i < 2u; i++) p3SetDriveMotor(&p3Drives[i], on ? 1u : 0u, now);
}

void p3FdcReset(void) {
  uint8_t cyl0 = p3Drives[0].cyl, cyl1 = p3Drives[1].cyl;
  p3Clear(&p3Fdc, sizeof(p3Fdc));
  p3Fdc.phase = P3_PH_CMD;
  p3Fdc.srt = P3_MS(2u * 16u);
  p3Fdc.hut = P3_MS(2u * 240u);
  p3Fdc.hlt = P3_MS(2u * 254u);
  p3Fdc.nonDma = 1u;
  p3Drives[0].motor = 0u;
  p3Drives[1].motor = 0u;
  p3Drives[0].cyl = cyl0;                          /* a reset does not move the heads */
  p3Drives[1].cyl = cyl1;
}

/* ---------------------------------------------------------------- the host's side */

uint8_t *p3DiskImagePtr(uint32_t drive) { return p3DiskImage[drive & 1u]; }
uint32_t p3DiskImageCapacity(void) { return P3_IMAGE_MAX; }
P3Stats *p3FdcStatsPtr(void) { return &p3Stats; }

/* a drive: heads 1 or 2, cylinders 40 or 80; heads 0 = no drive */
void p3DriveSetup(uint32_t drive, uint32_t heads, uint32_t cylinders) {
  P3Drive *d = &p3Drives[drive & 1u];
  d->present = heads > 0u ? 1u : 0u;
  d->heads = (uint8_t)(heads > 2u ? 2u : heads);
  d->cylinders = (uint8_t)(cylinders > P3_MAX_CYL ? P3_MAX_CYL : cylinders);
  if (d->cyl >= d->cylinders && d->cylinders > 0u) d->cyl = (uint8_t)(d->cylinders - 1u);
}

/* the image is in p3DiskImagePtr(drive): parse it; 0 = inserted */
uint32_t p3DiskInsert(uint32_t drive, uint32_t length, uint32_t writeProtected) {
  P3Drive *d = &p3Drives[drive & 1u];
  d->loaded = 0u;
  if (length > P3_IMAGE_MAX) return 9u;
  uint32_t r = p3ParseImage(drive & 1u, length);
  if (r != 0u) return r;
  d->loaded = 1u;
  d->wp = writeProtected ? 1u : 0u;
  d->writes = 0u;
  return 0u;
}

void p3DiskEject(uint32_t drive) {
  p3Drives[drive & 1u].loaded = 0u;
}

uint32_t p3DiskWrites(uint32_t drive) { return p3Drives[drive & 1u].writes; }
uint32_t p3DriveCylinder(uint32_t drive) { return p3Drives[drive & 1u].cyl; }
uint32_t p3DriveMotor(uint32_t drive) { return p3Drives[drive & 1u].motor; }
uint32_t p3FdcPhase(void) { return p3Fdc.phase; }
/* the T-state count since power-on, 64-bit (two halves) */
uint32_t p3NowLow(void) { return (uint32_t)p3Now(); }
uint32_t p3NowHigh(void) { return (uint32_t)(p3Now() >> 32u); }
