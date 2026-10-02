//
// LatSOFAudioDevice.cpp — part of LatSOFAudio, the internal-microphone driver for the
// Dell Latitude 3410 hackintosh: https://github.com/shubhambanekar/LatSOFAudio
//
// Forked from CmlSOFAudio by DexterSLamb (HP Chromebook C1030):
//   https://github.com/DexterSLamb/CmlSOFAudio
// Copyright (c) 2026 DexterSLamb
// Copyright (c) 2026 Shubham Banekar
// SPDX-License-Identifier: BSD-3-Clause — see LICENSE and NOTICE.
//

#include "LatSOFAudioDevice.hpp"
#include "LatSOFKernelAudio.hpp"
#include <IOKit/IOLib.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODMACommand.h>
#include <libkern/libkern.h>

// DMA buffer with bus address (via IODMACommand)
struct DmaBuf {
    IOBufferMemoryDescriptor *md;
    IODMACommand *cmd;
    IOMemoryMap *map;
    UInt64 physAddr;
    void *virtAddr;
    UInt32 size;
};

// ===================== LAB BENCH (patch-44) =====================
// Experiment parameters live in a kernel sysctl string, so a hypothesis can
// be changed WITHOUT rebuilding the kext — no re-approval, no reboot, no
// 3-minute cycle. CTLFLAG_ANYBODY means the developer's own shell can write
// it with no privilege prompt:
//
//     sysctl -w kern.latsof_lab="stage=1 tag=1 fmt=0x4031"
//
// Format: whitespace/comma separated "key=value" tokens. Missing keys fall
// back to the compiled-in stage preset, so an empty value is exactly the old
// behaviour. The retry engine re-reads it on every attempt (~1.5 s), which is
// what makes the loop free.
#include <sys/sysctl.h>

#define LAB_MAX 1024
static char gLabBuf[LAB_MAX];

SYSCTL_DECL(_kern);
SYSCTL_STRING(_kern, OID_AUTO, latsof_lab,
              CTLFLAG_RW | CTLFLAG_ANYBODY,
              gLabBuf, sizeof(gLabBuf), "LatSOF lab parameters");
static bool gLabOidRegistered = false;

// patch-44 LAB BENCH knobs (0 / 0xFFFFFFFF = "unset, use compiled default")
static UInt32 gLabFmt   = 0;           // override SDxFMT (e.g. 0x4031)
static UInt32 gLabChunk = 0;           // BDL chunk bytes (0 = PAGE_SIZE)
static UInt32 gLabAbits = 0;           // IODMACommand numAddressBits override
static UInt32 gLabUnmap = 0;           // 1 = kUnmapped (raw physical DMA)
static UInt32 gLabHold  = 0;           // extra RUN-hold wait in ms
static UInt32 gLabSpib  = 0xFFFFFFFF;  // 0/1 = force SPIB enable off/on
static UInt32 gLabGproc = 0xFFFFFFFF;  // 0/1 = force GPROCEN off/on
static UInt32 gLabNoRun = 0;           // 1 = program the stream but never RUN
// patch-45 LAB knobs. The loader stream used to be hard-wired to
// sIdx = GCAP.ISS, which the HDA spec defines as "number of input streams
// MINUS ONE" — so that picks the LAST INPUT stream, not the first output one
// the comment claims. Making it a knob lets one rebuild sweep the whole 0..15
// stream space from userspace instead of costing an approve+reboot per index.
static UInt32 gLabSIdx   = 0xFFFFFFFF; // loader stream index override (0..15)
static UInt32 gLabNoDefer = 0;         // 1 = never defer to the retry engine
// patch-48: controller-level hygiene, both knob gated so one boot can A/B them.
// The gap this closes: Linux brings the code loader up on a controller IT reset
// itself (hda_dsp_ctrl_init_chip -> hda_dsp_ctrl_link_reset pulses GCTL.CRST
// 0 then 1, and clears every stream's SDnSTS). We live on AppleHDA's
// controller instance and never reset it, so whatever per-stream state it
// accumulated survives into our attempt.
static UInt32 gLabHdaRst = 0;          // 1 = pulse GCTL.CRST before the load
static UInt32 gLabAllSts = 0;          // 1 = clear SDnSTS on all 16 streams

// ===================== patch-53 knobs ======================================
// patch-53 exists because a line-by-line read of the whole CL boot path found
// exactly THREE places where our host state differs from Linux's steady state
// at the moment the code-loader stream is started. All three are knobbed so
// one boot can A/B them, and all three default to the Linux value.
//
// (a) PCI CGCTL (0x48) bit6 = MISCBDCGE.
//     hda_dsp_ctrl_init_chip() brackets the whole controller reset with
//         hda_dsp_ctrl_misc_clock_gating(sdev, false);   // clear bit6
//         ...
//     err:
//         hda_dsp_ctrl_misc_clock_gating(sdev, true);    // SET bit6 back
//     so Linux's steady state (and the BIOS's: measured 0xfd) is bit6 = 1.
//     Our kext cleared it once and never restored it -> measured 0xbd.
//     gLabCg6: 0 = clear (old behaviour), 1 = set (Linux), 2 = leave alone.
static UInt32 gLabCg6  = 1;
//
// (b) DPLBASE (HDA bar 0x70) bit0 = position-buffer ENABLE.
//     hda_dsp_stream_hw_params() ends with
//         if (bus->use_posbuf && bus->posbuf.addr &&
//             !(read(DPLBASE) & SOF_HDA_ADSP_DPLBASE_ENABLE))
//                 write(DPUBASE, upper_32_bits(bus->posbuf.addr));
//                 write(DPLBASE, (u32)bus->posbuf.addr | ENABLE);
//     measured ours: DPLBASE = 0x00200000, bit0 = 0, i.e. position buffer
//     DISABLED while Linux enables it. On a DECOUPLED stream the position
//     buffer is the only host-visible progress channel, and the DSP side is
//     what fills it, so leaving it disabled is a plausible reason for the
//     DSP never to complete a transfer. We allocate our own 128-byte posbuf
//     (Linux: SOF_HDA_DPIB_ENTRY_SIZE * num_total = 8 * 16) rather than
//     re-enabling whatever address AppleHDA left in the register, so we can
//     never point the controller at memory we do not own.
//     gLabDplEn: 0 = leave DPLBASE untouched, 1 = enable (Linux).
static UInt32 gLabDplEn = 1;
//
// (c) BDL entry IOC bits.
//     hda_dsp_stream_setup_bdl() computes
//         ioc = hda->no_ipc_position ? !hstream->no_period_wakeup : 0;
//     and hda_setup_bdle() then sets bdl->ioc = 1 on the FINAL chunk of each
//     period. hda.c decides no_ipc_position with
//         hdev->no_ipc_position = sof_ops(sdev)->pcm_pointer ? 1 : 0;
//     i.e. on any platform that provides a pcm_pointer op (the position-buffer
//     path), EVERY code-loader BDLE carries IOC=1. Our BDL hardcodes ioc = 0.
//     gLabIocAll: 0 = all zero (old), 1 = set IOC on every entry (Linux when
//     no_ipc_position is 1).
static UInt32 gLabIocAll = 1;
// gLabBdlDump: read the BDL table and the FW payload back out of the DMA
// buffers and log them. Never done before - every previous run verified the
// BDL's ADDRESS (bdbarEcho) but never its CONTENTS, so a malformed entry
// (wrong address/length encoding) would have looked identical to "no DMA".
static UInt32 gLabBdlDump = 0;

// gLabCapChain: 1 = locate the PP/SPIB capability structures the way Linux
// does - walk the HDA BAR0 capability LINKED LIST (start at SOF_HDA_LLCH,
// BAR0+0x14; then follow `cap & SOF_HDA_CAP_NEXT_MASK`, at most 10 hops; the
// id is bits 27:16). 0 = keep the historical brute-force scan (first dword in
// 0x500..0x2000 whose bits 27:16 happen to match). A false positive in that
// scan would have sent every PPCTL / SPBFCCTL / SPIB access since patch-20 to
// a register Linux never touches - and the symptom would be exactly what we
// see: a byte-perfect stream descriptor whose DMA never moves.
static UInt32 gLabCapChain = 1;
// gLabMBox: 1 = dump the DSP->host half of the IPC doorbell (HIPCTDR/DA/DD at
// 0x00C0/0x00C4/0x00C8) plus the SRAM mailbox at HDA_DSP_MBOX_UPLINK_OFFSET
// (0x81000). Only the host->DSP half (0x00D0..) has ever been touched, so if
// the ROM has been sending us a ready/error/request message we have been deaf
// to it for 54 patches.
static UInt32 gLabMBox = 0;

// patch-55 — SPIB on a COUPLED stream. This is the single remaining
// host-side deviation from Linux and it was hiding in plain sight behind a
// patch-35 comment that reads "SPIB only makes sense decoupled".
//
// Linux hda-stream.c:1305 — hda_data_stream_prepare(), the non-iccmax branch
// (the CL boot path passes is_iccmax=false, so this IS our branch):
//
//     ret = hda_dsp_stream_hw_params(sdev, hext_stream, dmab, NULL);
//     if (ret < 0) goto out_free;
//     hda_dsp_stream_spib_config(sdev, hext_stream, HDA_DSP_SPIB_ENABLE, size);
//
// No `if (decoupled)` anywhere. The enable is unconditional, and it happens
// AFTER hw_params and BEFORE the trigger, i.e. before RUN — exactly where our
// patch-35 gate sits. And the chip runs COUPLED: cnl_chip_info (cnl.c:450) has
// no `quirks` field at all, so SOF_INTEL_PROCEN_FMT_QUIRK is never set, so
// hw_params never sets PROCEN, so PPCTL stays 0xc0000000 — which is precisely
// what we measure. Linux on this exact silicon therefore boots the code
// loader with COUPLED + SPIB ENABLED, a combination this driver has never
// once executed: the patch-35 gate skips the SPIB writes whenever coupled,
// and every coupled run shows spibctl=0x00000000.
//
// Why SPIB could plausibly matter for a CL DMA that never moves a byte:
// SPBFCCTL is the per-stream "SPIB FIFO channel enable" in the stream-to-
// DSP FIFO bridge. The ROM state we are stuck in is
// FSR_WAIT_FOR_DMA_BUFFER_FULL — the ROM is waiting for the host DMA to
// deliver a buffer it can see. If the host->DSP path on this port is the
// SPIB FIFO bridge rather than plain SDnDMA, then a closed SPIB channel is
// the mechanical reason the ROM waits forever while our descriptor is
// byte-perfect.
//
// Default 1 = Linux behaviour (enable SPIB even when coupled). 0 restores
// the patch-35 gate exactly.
static UInt32 gLabSpibCpl = 1;
// gLabClobber: 1 = snapshot the five descriptor registers straight after we
// program them, then re-check them on every pass of the ROM poll loop and log
// the FIRST divergence. Never done in 54 patches. Every run so far has
// verified the descriptor once, at t=0, and then assumed it stayed put; if
// AppleHDA's stream-quiesce path (or anything else) rewrites SDnCTL / CBL /
// FMT / BDLPL / BDLPU while we wait, the single t=0 snapshot cannot see it —
// and the keeper's RUN re-assert loop would mask a RUN-only clobber while
// leaving the fetch broken.
static UInt32 gLabClobber = 1;
// gLabPie (patch-56): PPCTL BIT(31). Every run in 55 patches has forced it ON
// — see the code that writes `(snap.ppctl & ~pair) | (1U << 31) | GPROCEN`.
// But the two things we are trying to match both leave it OFF:
//   - AppleHDA's working state on this controller reads ppctl=0x40000000,
//     i.e. GPROCEN only, no PIE (measured while the tone played, 2026-10-02);
//   - Linux's hda_dsp_ctrl_ppcap_enable() writes SOF_HDA_PPCTL_GPROCEN and
//     nothing else — SOF never touches PIE.
// PIE has therefore never been tested OFF. Default 1 preserves the old
// behaviour exactly; `pie=0` makes PPCTL byte-equal to AppleHDA's.
static UInt32 gLabPie = 1;

// ===================== patch-46 SAFETY (read this before adding knobs) =====
// Measured 2026-10-01: this kext alone hung the whole machine three times in
// one afternoon. The mechanism is now understood, and it is structural:
//
//   The load path runs on the audio workloop with the command gate held.
//   Anything another thread wants from this device queues behind us, and
//   macOS counts those threads as load even though they burn no CPU — the
//   tell is `top` showing 79% idle next to a load average of 11.8, plus
//   processes in the `stuck` column. The old keeper loop could hold the gate
//   for 3000-8000 x IOSleep(1), i.e. ~3-8 SECONDS per attempt, on a retry
//   tick that fires every ~1.5 s. That is not an experiment, that is a
//   machine-wide stall.
//
// Two structural fixes, both here rather than in the load path:
//   (patch-67 NOTE: auto mode supersedes fix 1 for production — see the
//    PRODUCTIZATION block below. Fix 2's one-shot rule now guards manual
//    runs only; in auto mode the retry engine's bounded budget guards.)
//
//  1. gLabEnable defaults to 0. Unless the user explicitly writes `enable=1`
//     through the lab sysctl, initDSP() returns before it touches ONE
//     register — so a plain boot performs no hardware bring-up at all and
//     cannot hang the machine. The device is still published and the retry
//     engine still ticks, so arming later needs no reboot.
//
//  2. One-shot arming. A real bring-up is allowed only when the lab string
//     has CHANGED since the last run it authorised (`write once -> exactly
//     one attempt, then idle`). Writing a new parameter set is the trigger;
//     nothing else is. No capture-device poking, no reboot, and a mistyped
//     sweep cannot turn into an unattended loop.
//
// The keeper wait is separately hard-capped at LAB_TMAX_CAP so that even an
// armed run cannot reproduce the original 8-second stall.
static UInt32 gLabEnable  = 0;         // 0 = never touch hardware (default)
static char   gLabLast[LAB_MAX];       // lab string at the last armed run
static UInt32 gLabGen     = 0;         // bumped whenever the string changes
static UInt32 gLabGenDone = 0;         // generation whose one run already ran
static bool   gLabHoldOff = false;     // this initDSP() call held off hardware
// patch-48: ceiling raised 1000 -> 3000 ms. Linux hda_cl_copy_fw polls
// FSR_STATE_FW_ENTERED for HDA_DSP_BASEFW_TIMEOUT_US = 3000000, i.e. three
// seconds; capping our window at a third of that means a slow-but-live DMA
// would look exactly like a dead one. 1000 ms of re-asserting RUN was already
// measured to move `rom` by zero, so this is cheap. It stays a hard cap for
// the reason below (and nothing else may hold the audio command gate for
// longer), and the instrumented default remains 200 ms.
#define LAB_TMAX_CAP 3000              // hard ceiling on the keeper wait (ms)

// ===================== patch-67 PRODUCTIZATION (auto mode) ==================
// Three years of debugging ended 2026-10-02 with a fully verified load chain
// (VT-d off + abits=64 -> cl OK rom=0x00000005, IPC round-trip, topology 8/8,
// real DMIC audio). Until now that chain only ran when a human wrote the
// winning knob string through the lab sysctl — the SAFETY gate above treated
// "no sysctl write" as "never touch hardware". That was right while every run
// was an experiment; it is wrong now that the run is the product.
//
// patch-67 keeps every safety mechanism and removes only the human from the
// loop:
//
//  - PRODUCTION PROFILE. When the lab buffer is EMPTY (no sysctl string has
//    been written this boot) and auto mode is on, labReload() synthesizes the
//    exact string that won: the one validated end-to-end on 2026-10-02 18:38
//    (and 18:20 with abits=64 spelled out; 1.1.25+ defaults abits to 64). No
//    knob value is invented — this is byte-for-byte the verified combination.
//    An explicit sysctl string always wins over the profile.
//
//  - AUTO RE-ARM AUTHORITY. In auto mode the generation is NOT consumed by a
//    run. The one-shot rule existed so a mistyped sweep could not loop
//    unattended; in auto mode the authorization is gWakeReinitPending, which
//    the retry engine already bounds (12 genuine attempts, 1.5 s apart, busy
//    checks cost nothing). Wake, device-switch and capture-demand re-arms
//    therefore all work with zero human action, exactly as patch-26/32
//    intended.
//
//  - LOAD PATH STAYS DEFERRED. initHardware never runs the load chain on the
//    matching thread even in auto mode; it defers to the retry engine (~1.5 s
//    later, on the workloop). The DSP is up within seconds of boot with no
//    app involved, and start() still cannot block.
//
//  - FUSE. `sysctl -w kern.latsof_lab="auto=0"` restores yesterday's exact
//    behaviour: buffer non-empty (so no profile synthesis), enable=0 (idle,
//    hardware untouched). Writing "auto=1" (or clearing the string with
//    kern.latsof_lab="") returns to auto mode. Any other explicit string
//    remains a one-shot lab run, identical to pre-patch-67 semantics.
#define LATSOF_PROD_LAB \
    "enable=1 stage=0 trigger=1 coupled=0 sidx=7 tag=1 hdaInit=0 " \
    "prefwrun=1 pie=1 steal=1 strictborrow=0 tmax=3000"
// hdaInit=0 (2026-10-02 20:05 verified): loads the firmware WITHOUT the
// CORB/RIRB hijack — and AppleHDA playback survives the load. hdaInit=3
// (the debugging-era winning value) desyncs AppleHDA's codec command
// channel: 58 in-flight verbs get answered into our RIRB, its state machine
// wedges, its output StartIO blocks forever inside an AppleHDAController
// mutex (spindump: 100/101 samples) — the VoiceMemos machine freeze.
static UInt32 gLabAuto = 1;            // 1 = production auto mode (default)
// patch-68 fix: the boot-arg latsof-auto baseline. labReload() used to stomp
// gLabAuto with the sysctl-string default on every reload — an empty buffer
// has no "auto=" key, so the parse default (1) always won and the boot-arg
// never survived past the first engine tick. The sysctl key now only applies
// when actually present; otherwise the boot-arg baseline holds.
static UInt32 gLabAutoBoot = 1;
// patch-68: initDSP holds the audio command gate for up to LAB_TMAX_CAP ms
// and rewrites global controller state. A second concurrent call — from a
// playback IPC-timeout path, a UserClient probe, anything — used to be
// impossible (the hold-off returned instantly); in auto mode the gate is
// open, so a re-entrant call would deadlock or corrupt a live load. This
// flag makes the invariant explicit.
static bool gInitInProgress = false;

static void labRegister(void) {
    if (gLabOidRegistered) return;
    sysctl_register_oid(&sysctl__kern_latsof_lab);
    gLabOidRegistered = true;
    IOLog("LatSOF: lab sysctl registered (kern.latsof_lab)\n");
}

// patch-47: labReload needs the parser, which is defined below it.
static UInt32 labU32(const char *key, UInt32 dflt);
static const char *labFind(const char *hay, const char *needle);

static void labReload(void) {
    for (int i = 0; gLabBuf[i]; i++) {       // flatten newlines/tabs
        if (gLabBuf[i] == '\n' || gLabBuf[i] == '\r' || gLabBuf[i] == '\t')
            gLabBuf[i] = ' ';
    }
    // patch-67: production profile synthesis. The fuse (`auto=0` written
    // explicitly) makes the buffer non-empty, so synthesis cannot resurrect
    // itself behind the user's back; "auto=1" is the documented way back.
    // This runs BEFORE the generation compare so that the synthesized string
    // is what arms and what the "effective"/ARMED log lines show — the run
    // must be auditable as exactly the verified combination.
    UInt32 autoK = labFind(gLabBuf, "auto=") ? labU32("auto", 1) : gLabAutoBoot;
    if (autoK && (gLabBuf[0] == 0 || strcmp(gLabBuf, "auto=1") == 0)) {
        strlcpy(gLabBuf, LATSOF_PROD_LAB, LAB_MAX);
        gLabAuto = 1;
    } else {
        gLabAuto = autoK;
    }
    // patch-46: every distinct string is one generation, and one generation
    // authorises exactly one real bring-up. This IS the trigger mechanism —
    // see the SAFETY block at the top of the file. "Write new parameters" is
    // the only way to arm a run, which is what makes an unattended loop
    // impossible. strncmp, not strcmp: strncmp is already proven to bind in
    // this kext (used further down for the Status string).
    //
    // patch-67 exception: in auto mode the generation is not consumed (see
    // initDSP below), so the compare here only re-arms after an explicit
    // string edit — the unattended-loop protection now lives in the retry
    // engine's bounded budget instead of in one-shot generations.
    if (strncmp(gLabBuf, gLabLast, LAB_MAX) != 0) {
        strlcpy(gLabLast, gLabBuf, sizeof(gLabLast));
        gLabGen++;
    }
    // patch-47: parse the two knobs the ARMING DECISION depends on right
    // here, not inside initDSP. Measured 2026-10-01: jackPoll's retry engine
    // refused to call initDSP while AppleHDA held the borrow stream
    // programmed, so the knob parse inside initDSP never ran — which meant
    // `sidx` could not be changed to point the busy-check at a free stream,
    // which was the very thing that would have let initDSP run. Circular, and
    // the engine just ticked forever. Anything the *scheduler* must know has
    // to be readable without running the load path.
    gLabEnable  = labU32("enable", 0);
    gLabSIdx    = labU32("sidx", 0xFFFFFFFF);
    gLabNoDefer = labU32("nodefer", 0);
}

// Keep the lab parser free of non-exported kernel symbols: kmutil's binder
// rejected _strnstr ("could not find a kext which exports this symbol"), so
// the substring search is hand-rolled here.
static const char *labFind(const char *hay, const char *needle) {
    if (!hay || !needle || !*needle) return NULL;
    for (const char *h = hay; *h; h++) {
        const char *a = h, *b = needle;
        while (*a && *b && *a == *b) { a++; b++; }
        if (!*b) return h;
    }
    return NULL;
}

// returns default when the key is absent
static UInt32 labU32(const char *key, UInt32 dflt) {
    if (!gLabBuf[0]) return dflt;
    char pat[48];
    snprintf(pat, sizeof(pat), "%s=", key);
    const char *p = labFind(gLabBuf, pat);
    if (!p) return dflt;
    p += strlen(pat);
    char *end = NULL;
    long v = strtol(p, &end, 0);            // base 0: 0x.. works
    if (end == p) return dflt;
    return (UInt32)v;
}

static void labStr(const char *key, char *out, size_t outsz) {
    out[0] = 0;
    if (!gLabBuf[0]) return;
    char pat[48];
    snprintf(pat, sizeof(pat), "%s=", key);
    const char *p = labFind(gLabBuf, pat);
    if (!p) return;
    p += strlen(pat);
    size_t n = 0;
    while (p[n] && p[n] != ' ' && p[n] != ',' && p[n] != '\r' && n < outsz - 1) {
        out[n] = p[n]; n++;
    }
    out[n] = 0;
}

static DmaBuf *allocDma(UInt32 sz, UInt32 align) {
    mach_vm_address_t mask = ~((UInt64)align - 1);
    auto *md = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
        kernel_task, kIOMemoryPhysicallyContiguous | kIOMapInhibitCache, sz, mask);
    if (!md) { IOLog("LatSOF: allocDma %u FAILED: md null\n", sz); return nullptr; }
    IOReturn rc;
    // patch-39: TRUE argument order per IODMACommand.h:
    //   withSpecification(outSegFunc, numAddressBits, maxSegmentSize,
    //                     mappingOptions, maxTransferSize, alignment,
    //                     mapper, refCon)
    // patch-37/38 passed (sz, 1, kMapped, 64, 0) which meant
    // numAddressBits=(UInt8)sz=0, maxSegmentSize=1 (one-byte segments!),
    // maxTransferSize=64 -> prepare() exploded with kIOReturnUnderrun.
    // patch-40: numAddressBits=32, NOT 64. With 64 the AppleVTD mapper placed
    // the 557056-byte firmware buffer at IOVA 0x100000000 (above 4GB); the
    // cAVS HD-A DMA engine is a 32-bit DMA design, so the first descriptor
    // fetch failed and the engine died before touching anything (sts=0x00,
    // lpib stuck, rom frozen at 0x05000001). Constrain the mapping to the
    // 32-bit address space so the IOVA lands below 4GB.
    // patch-44: both the mapping options and the address width are now lab
    // knobs (abits= / unmapped=), because telemetry proved fwBus==fwPhys —
    // AppleVTD is not remapping anything here, so the whole IOMMU angle needs
    // to be re-testable from userspace without a rebuild.
    //
    // patch-66 (2026-10-02): DEFAULT IS NOW 64. THE FIX THAT ENDED THE
    // THREE-YEAR HUNT. The ROM's DSP-side HDAS DMA never fetched a single
    // byte for 65 patches (rom frozen at 0x05000001 = INIT_DONE +
    // FSR_WAIT_FOR_DMA_BUFFER_FULL, DPIB host-RO and pinned at 0) because:
    //   1. AppleVTD (OC DisableIoMapper=false) blocked the DSP master's PCIe
    //      fetches entirely - not even ADSPIS CL_DMA error bits latched.
    //      Fix: OC config Kernel:Quirks:DisableIoMapper = true.
    //   2. With VT-d off, the abits=32 default still forced a <4GB remap of
    //      the BDL (bdlBus=0x31651000 vs bdlPhys=0x17b8c2000) - an IOVA with
    //      NO page table behind it, so the descriptor fetch read garbage.
    //      abits=64 yields bus==phys and the whole pipeline came alive:
    //      DPIB swept 0->557056, rom 0x3->0x5 (FW_ENTERED), IPC ROUND-TRIP
    //      OK, Topology 8 OK 0 FAIL, DMIC records real audio.
    // 64-bit addressing is correct on this controller either way: the HDA
    // BDL address registers are 64-bit (BDPL/BDPU) and Linux routinely uses
    // >4GB DMA addresses on CML.
    UInt32 abits = gLabAbits ? gLabAbits : 64;
    IODMACommand::MappingOptions mopt = gLabUnmap ? IODMACommand::kUnmapped
                                                  : IODMACommand::kMapped;
    auto *cmd = IODMACommand::withSpecification(
        kIODMACommandOutputHost64, abits, 0, mopt, sz, 1);
    if (!cmd) { IOLog("LatSOF: allocDma %u FAILED: withSpecification null\n", sz);
                md->release(); return nullptr; }
    // patch-39: setMemoryDescriptor defaults autoPrepare=true (13.3 SDK), so
    // the prepare() failure was surfacing under this call's name. Be explicit
    // and prepare separately so each error gets its own log line.
    rc = cmd->setMemoryDescriptor(md, false);
    if (rc != kIOReturnSuccess) {
        IOLog("LatSOF: allocDma %u FAILED: setMemoryDescriptor 0x%x\n", sz, rc);
        cmd->release(); md->release(); return nullptr;
    }
    rc = cmd->prepare();
    if (rc != kIOReturnSuccess) {
        IOLog("LatSOF: allocDma %u FAILED: cmd->prepare 0x%x\n", sz, rc);
        cmd->clearMemoryDescriptor(); cmd->release(); md->release(); return nullptr;
    }
    IODMACommand::Segment64 seg; UInt32 numSeg = 1; UInt64 offset = 0;
    rc = cmd->gen64IOVMSegments(&offset, &seg, &numSeg);
    if (rc != kIOReturnSuccess) {
        IOLog("LatSOF: allocDma %u FAILED: gen64IOVMSegments 0x%x numSeg=%u\n", sz, rc, numSeg);
        cmd->complete(); cmd->clearMemoryDescriptor(); cmd->release(); md->release(); return nullptr;
    }
    IOLog("LatSOF: allocDma %u OK bus=0x%llx numSeg=%u\n", sz, seg.fIOVMAddr, numSeg);
    auto *map = md->map(kIOMapInhibitCache);
    if (!map) {
        cmd->complete(); cmd->clearMemoryDescriptor(); cmd->release(); md->release(); return nullptr;
    }
    auto *b = (DmaBuf *)IOMalloc(sizeof(DmaBuf));
    b->md = md; b->cmd = cmd; b->map = map;
    b->physAddr = seg.fIOVMAddr;
    b->virtAddr = (void *)map->getVirtualAddress();
    b->size = sz;
    bzero(b->virtAddr, sz);
    return b;
}

static void freeDma(DmaBuf *b) {
    if (!b) return;
    if (b->map) b->map->release();
    if (b->cmd) { b->cmd->complete(); b->cmd->clearMemoryDescriptor(); b->cmd->release(); }
    if (b->md) { b->md->complete(); b->md->release(); }
    IOFree(b, sizeof(DmaBuf));
}

// Kext module entry
extern "C" kern_return_t LatSOFAudio_start(kmod_info_t *ki, void *data);
extern "C" kern_return_t LatSOFAudio_stop(kmod_info_t *ki, void *data);
__attribute__((visibility("default")))
KMOD_EXPLICIT_DECL(com.hackintosh.LatSOFAudio, "1.1.5", LatSOFAudio_start, LatSOFAudio_stop)
kern_return_t LatSOFAudio_start(kmod_info_t *ki, void *data) { return KERN_SUCCESS; }
kern_return_t LatSOFAudio_stop(kmod_info_t *ki, void *data) { return KERN_SUCCESS; }

// Embedded firmware
extern "C" const unsigned char sof_fw_data[];
extern "C" const unsigned long long sof_fw_size;

// ========== HDA / DSP Register Definitions ==========

// BAR0 (HDA controller)
#define HDA_GCAP            0x00
#define HDA_GCTL            0x08
#define HDA_INTCTL          0x20
#define SD_BASE             0x80
#define SD_SIZE             0x20
#define SD_CTL_SRST         0x01
#define SD_CTL_RUN          0x02
#define SD_CTL_IOCE         0x04
#define SD_REG_STS          0x03
#define SD_REG_CBL          0x08
#define SD_REG_LVI          0x0C
/* PORT CORRECTION (Lenovo port): the "FMT is at 0x14" change was WRONG and
 * is reverted to the upstream value. Authoritative layout, from Linux's
 * include/sound/hda_register.h and sound/soc/sof/intel/hda.h:
 *     0x0E SDxFIFOW   0x10 SDxFIFOS(IZE)   0x12 SDxFMT   0x14 SDxFIFOL
 * So FORMAT is 0x12 (as upstream had) and the FIFO-size register is 0x10.
 * Writing the format to 0x14 stored it in FIFOL and left the real FMT at 0,
 * which is exactly why that build read RUN back as 0x00 (ctl=0x00) while the
 * previous build read 0x02 — the controller drops RUN for a stream whose
 * format field is invalid. Dell was never relying on a stale FMT. */
#define SD_REG_FMT          0x12
#define SD_REG_FIFOS        0x10
#define SD_CTL_INT_MASK     0x1C    /* IOCE|FEIE|DEIE = SD_CTL bits 2,3,4 */
#define SD_REG_BDLPL        0x18
#define SD_REG_BDLPU        0x1C
#define HDA_CL_STREAM_FMT   0x40

// ===== PORT DEBUG (Lenovo): DMA-start hypothesis sweep =====================
// The controller never moves a byte for the code-loader stream on this
// machine: RUN sticks (ctl=0x02), yet DPIB/LPIB stay 0 and the ROM sits in
// INIT_DONE. Both counters can also read 0 when the DMA *is* running but the
// DSP-side link never consumes, so we cannot tell "never started" from
// "started but starved" with today's telemetry.
//
// The retry engine is a free test harness: every start() retry reruns the
// full DSP + code-loader bring-up, so we spend one retry per hypothesis and
// let the telemetry name the winner. Stage 0 is the shipped behaviour, so a
// plain revert of the wrong-FMT build is covered by the same run.
//   0: baseline kext        : CTL=RUN only, PROCEN(sIdx|sIdx+1), no INTCTL
//   1: Linux trigger        : CTL=RUN|0x1C, INTCTL|=1<<sIdx, PROCEN(sIdx)
//   2: as 1 + SD_CTL bit18 kept (AppleHDA's live descriptor carries 0x14001e)
//   3: as 2 + tag 2          (ROM IPC (tag-1)<<9 as well)
//   4: as 2 + tag 3
//   5: as 2 but SPIB left disabled
//   6: as 2 + EM2 L1SEN(bit13) cleared instead of bit14 set, + 20 ms settle
//
// patch-34/35 (00:15 round): FSR decode corrected AGAIN against Linux's
// sound/soc/sof/intel/hda.h — FSR_STATE_MASK is GENMASK(23,0): the state
// code lives in the LOW 24 bits (FSR_STATE_FW_ENTERED = 5), the "0x05"
// nibble we saw at bits 27:24 is FSR_WAIT_STATE 5 = FSR_WAIT_FOR_DMA_
// BUFFER_FULL. So 0x05000001 never meant "firmware entered": it means
// INIT_DONE(1) + ROM WAITING FOR THE DMA TO FILL THE BUFFER. The original
// low-24-bit check was right all along. The DMA genuinely never moves a
// byte — and AppleHDA's own streams run fine on this controller COUPLED
// (BIOS default GPROCEN=0, we force decouple following Linux).
//   0: decoupled baseline  : Linux-style, PROCEN(sIdx), SPIB on   [control]
//   1: COUPLED             : PROCEN bits cleared, SPIB off, fmt 0x40
//   2: COUPLED + FMT 0x4031: coupled, AppleHDA's proven-valid format
//   3: DECOUPLED + 0x4031  : decoupled, SPIB on, valid-encoded format
//   4: COUPLED + keeper    : as 1 + RUN-keeper, 8 s window
//   5: DECOUPLED + keeper  : as 0 + RUN-keeper, 8 s window
// Coupled mode gives a live LPIB, so "engine dead" vs "decouple machinery
// broken" becomes distinguishable in a single boot.
#define FW_STAGE_COUNT      6
static UInt32 gFwStage = 0;

// PP/SPIB/ML capability IDs
#define HDA_CAP_ML_ID       0x02    // Multi-Link capability
#define HDA_CAP_PP_ID       0x03
#define HDA_CAP_SPIB_ID     0x04
#define PP_PPCTL            0x04
#define PP_PPCTL_GPROCEN    (1U << 30)

// Multi-Link registers (from HDA spec, Multi-Link capability)
#define ML_MLCD             0x04    // Multi-Link Count (minus 1)
#define ML_LINK_BASE        0x40    // First link entry offset from mlcap
#define ML_LINK_INTERVAL    0x40    // Size of each link entry
#define ML_LCAP             0x00    // Link Capability
#define ML_LCTL             0x04    // Link Control
#define ML_LOSIDV           0x08    // Link Output Stream ID Validation

// BAR4 (DSP)
#define DSP_ADSPCS          0x04
#define DSP_ADSPIC          0x08
#define ADSPCS_CRST(cm)     ((cm) << 0)
#define ADSPCS_CSTALL(cm)   ((cm) << 8)
#define ADSPCS_SPA(cm)      ((cm) << 16)
#define ADSPCS_CPA(cm)      ((cm) << 24)

// CNL IPC registers (BAR4)
#define IPC_HIPCTDR         0xC0
#define IPC_HIPCTDA         0xC4
#define IPC_HIPCIDR         0xD0
#define IPC_HIPCIDA         0xD4
#define IPC_HIPCCTL         0xE8
#define IPC_BUSY            (1U << 31)
#define IPC_DONE            (1U << 31)

// Vendor Specific Registers
#define HDA_VS_SDXDPIB_XBASE    0x1084  // DPIB register base (playback position)
#define HDA_VS_SDXDPIB_XINTERVAL 0x20   // DPIB register interval per stream
#define HDA_VS_EM2          0x1030  // Extended Mode 2
#define HDA_VS_INTEL_LTRP   0x1048  // Latency Tolerance Reporting; GB mask 0x3F

// PCI Config registers
#define PCI_TCSEL           0x44    // Traffic Class Select

// DesignWare I2C controller registers (BAR0 of PCI 8086:02c5)
#define DW_IC_CON           0x00
#define DW_IC_TAR           0x04
#define DW_IC_DATA_CMD      0x10
#define DW_IC_FS_SCL_HCNT   0x1C
#define DW_IC_FS_SCL_LCNT   0x20
#define DW_IC_INTR_MASK     0x30
#define DW_IC_RAW_INTR_STAT 0x34
#define DW_IC_CLR_INTR      0x40
#define DW_IC_CLR_TX_ABRT   0x54
#define DW_IC_ENABLE        0x6C
#define DW_IC_STATUS        0x70
#define DW_IC_TXFLR         0x74
#define DW_IC_RXFLR         0x78
#define DW_IC_ENABLE_STATUS 0x9C
#define DW_IC_STATUS_TFNF   (1U << 1)  // TX FIFO not full
#define DW_IC_STATUS_RFNE   (1U << 3)  // RX FIFO not empty
#define DW_IC_STATUS_TFE    (1U << 2)  // TX FIFO empty
#define DW_IC_DATA_CMD_READ    0x0100   // read command
#define DW_IC_DATA_CMD_STOP    0x0200   // stop after this byte
#define DW_IC_DATA_CMD_RESTART 0x0400   // restart before this byte

// RT5682 I2C address and key registers
#define RT5682_I2C_ADDR     0x1A
#define RT5682_DEVICE_ID    0x00FF
#define RT5682_RESET        0x0000

// SRAM
#define SRAM_WIN(n)         (0x80000 + (n) * 0x20000)
#define MBOX_UPLINK         0x81000
#define ROM_STATUS          0x80000
// patch-35 (correction of the patch-34 comment): per Linux's
// sound/soc/sof/intel/hda.h, FSR_STATE_MASK is GENMASK(23,0) — the state
// code is the LOW 24 bits and FSR_STATE_FW_ENTERED = 5; the bits 27:24
// field is FSR_WAIT_STATE (5 = FSR_WAIT_FOR_DMA_BUFFER_FULL). So 0x05000001
// = INIT_DONE(1) + waiting-for-DMA — the ROM never got the payload, and
// the original low-24-bit success check was correct all along.
#define FSR_STATE_FW_ENTERED 0x5
#define FSR_INIT_DONE       0x1

// ROM IPC
#define ROM_IPC_CONTROL     0x01000000
#define ROM_IPC_PURGE_FW    0x00004000

// IPC commands
#define SOF_IPC_FW_READY    0x70000000
#define SOF_IPC_EXT_WINDOW  1
#define SOF_IPC_REGION_DOWNBOX  0
#define SOF_IPC_REGION_UPBOX    1

// SOF IPC message structs (matching SOF ABI 3.x, include/sound/sof/stream.h)
struct sof_ipc_host_buffer {
    UInt32 hdr_size;
    UInt32 phy_addr;
    UInt32 pages;
    UInt32 size;
    UInt32 reserved[3];
} __attribute__((packed));

struct sof_ipc_pcm_params {
    UInt32 hdr_size;      // = sizeof(sof_ipc_pcm_params)
    UInt32 hdr_cmd;       // = 0x60010000 (STREAM_PCM_PARAMS)
    UInt32 comp_id;
    UInt32 flags;
    UInt32 reserved[2];
    // stream_params:
    UInt32 params_size;   // = 84
    struct sof_ipc_host_buffer buffer;
    UInt32 direction;
    UInt32 frame_fmt;
    UInt32 buffer_fmt;
    UInt32 rate;
    UInt16 stream_tag;
    UInt16 channels;
    UInt16 sample_valid_bytes;
    UInt16 sample_container_bytes;
    UInt32 host_period_bytes;
    UInt16 no_stream_position;
    UInt8  cont_update_posn;
    UInt8  reserved0;
    SInt16 ext_data_length;
    UInt8  reserved1[2];
    UInt16 chmap[8];
} __attribute__((packed));

// Pipeline component IDs
#define COMP_HOST    1
#define COMP_DAI     2
#define COMP_BUFFER  12
#define FRAME_S16    0
#define FRAME_S24    1
#define FRAME_S32    2
#define DAI_SSP      1
#define DAI_DMIC     2
#define DIR_PLAYBACK 0
#define DIR_CAPTURE  1
#define TIME_TIMER   1

// Pipeline 7: SSP1 playback (MAX98357A speaker)
#define PIPE7_SCHED_ID   70
#define PIPE7_HOST_ID    75
#define PIPE7_BUF0_ID    71
#define PIPE7_DAI_ID     74
#define PIPE7_ID         7

// Pipeline 3: DMIC0 capture (internal microphone)
#define PIPE3_SCHED_ID   30
#define PIPE3_HOST_ID    35
#define PIPE3_BUF0_ID    31
#define PIPE3_DAI_ID     34
#define PIPE3_ID         3

// Pipeline 1: SSP0 playback (RT5682 headphone output)
// Using Linux kprobe comp_ids for Pipeline 1
#define PIPE1_SCHED_ID   5     // Linux: scheduler comp_id=5
#define PIPE1_HOST_ID    0     // Linux: host comp_id=0
#define PIPE1_BUF0_ID    2     // Linux: first buffer comp_id=2
#define PIPE1_DAI_ID     4     // Linux: DAI comp_id=4
#define PIPE1_BUF1_ID    3     // unused (no PGA), kept for compat
#define PIPE1_PGA_ID     1     // unused (no PGA), kept for compat
#define PIPE1_ID         1
#define COMP_VOLUME      5     // SOF_COMP_VOLUME

// Linux topology comp_ids (from kprobe IPC dump - auto-incremented from 0)
#define LINUX_PIPE7_HOST_ID  34   // Pipeline 7 (speaker) host
#define LINUX_PIPE1_HOST_ID  0    // Pipeline 1 (HP playback) host
#define LINUX_PIPE2_HOST_ID  6    // Pipeline 2 (HP capture) host
#define LINUX_PIPE3_HOST_ID  12   // Pipeline 3 (DMIC capture) host

// Pipeline 2: SSP0 capture (RT5682 headset mic via SSP0)
// Full chain: SSP0.IN(dai) → BUF2.0 → PGA2.0 → BUF2.1 → PCM0C(host)
#define PIPE2_SCHED_ID   20
#define PIPE2_HOST_ID    25
#define PIPE2_BUF0_ID    21
#define PIPE2_BUF1_ID    22
#define PIPE2_PGA_ID     23
#define PIPE2_DAI_ID     24
#define PIPE2_ID         2

struct HdaBdlEntry {
    UInt32 addrLow, addrHigh, size, ioc;
} __attribute__((packed));

OSDefineMetaClassAndStructors(LatSOFAudioDevice, IOService)

// ========== Register helpers ==========
static inline UInt32 rd32(volatile UInt8 *b, UInt32 o) { return *(volatile UInt32*)(b+o); }
static inline void   wr32(volatile UInt8 *b, UInt32 o, UInt32 v) { *(volatile UInt32*)(b+o) = v; }
static inline UInt16 rd16(volatile UInt8 *b, UInt32 o) { return *(volatile UInt16*)(b+o); }
static inline void   wr16(volatile UInt8 *b, UInt32 o, UInt16 v) { *(volatile UInt16*)(b+o) = v; }
static inline UInt8  rd8(volatile UInt8 *b, UInt32 o)  { return *(volatile UInt8*)(b+o); }
static inline void   wr8(volatile UInt8 *b, UInt32 o, UInt8 v)  { *(volatile UInt8*)(b+o) = v; }

// LATITUDE FORK patch-22: wake re-init retry state. File-scope statics so
// no header change is needed; every access happens on the workloop
// (jackPoll and the gated PM path), so no locking is required.
static bool gWakeReinitPending = false;
static int  gWakeTries = 0;
static int  gWakeTickDivider = 0;
// review 1 Aug: the wake branch used to infer "this is a real wake" from
// !hwReady, which was only correct because start() always ran initDSP. The
// patch-30 deferred load reaches the initial PM registration with hwReady
// still false, and the wake branch would then release and rebuild BAR maps
// start() just created — harmless when the remap works, but a remap failure
// in that window nulls hdaBase/dspBase and strands the deferral until the
// next physical sleep. Track sleeps explicitly instead of inferring them.
static bool gSleptOnce = false;
// review 1 Aug round 4: sleep-imminent latch for the resume executor. The
// family power-state reads are TOCTOU against its private PM workloop
// (currentPowerState is stored only AFTER the sleep pause; pending is
// better but still cross-thread). kIOMessageSystemWillSleep is
// non-vetoable, arrives gated, and strictly precedes every setPowerState(0)
// in the tree — so this latch, written and read only under the gate, parks
// the executor across the whole WillSleep -> wake span race-free.
static bool gSleepImminent = false;

// LATITUDE FORK patch-24: capture re-arm across sleep.
//
// The HAL plugin has no wake awareness at all — no interest notification, no
// reconnect — and its gDevice_IOIsRunning counter survives sleep. It only
// issues kLatSOF_StartCapture on the 0->1 transition in StartIO, so if an app
// held an input session across the sleep, StartIO never fires again and the
// capture DMA is never restarted: mic dead until something reopens the device.
// If nothing was recording, the next app's StartIO does a fresh StartCapture
// and the mic works — which is exactly why the mic failure looked intermittent.
//
// The kext has to close this itself: the plugin is ad-hoc signed and currently
// rejected by AMFI, so a plugin-side fix cannot ship independently. We latch
// whether capture was live at sleep and reissue it once wake re-init succeeds.
static bool gWasCapturing = false;

// patch-27b: recovery episode budget. Review of patch-27 found an unbounded
// livelock: a firmware that boots but has a dead runtime mailbox made
// initDSP "succeed", the re-arm then timed out, and scheduleDspRecovery
// reset the 12-try budget — forever, blocking the workloop ~8.5 s of every
// ~10. Recovery now gets at most 3 episodes; the budget refills on a
// successful capture start or on a genuine sleep/wake.
static int gRecoveryEpisodes = 0;

// patch-30: how many 1.5 s rounds to wait for AppleHDA to release its output
// descriptor before borrowing it anyway. 20 rounds = ~30 s: long enough for
// an idle engine to be torn down, short enough that a wake never costs the
// microphone for more than half a minute.
#define kProgrammedWaitRounds 20
static int  gProgrammedWaits = 0;
// patch-47: the borrow-anyway warning is an event, not a heartbeat. Before
// this flag the branch that announces it re-ran on every 1.5 s tick forever
// (it never cleared `busy`), so a single boot wrote ~2400 identical lines.
static bool gBorrowWarned = false;
// Escape hatch, per the patch-28 lesson: latsof_strictborrow=0 restores the
// old RUN-bit-only preflight without a rebuild.
static bool gStrictBorrow = true;

// patch-30 fix 1: deferred engine-resume latch (see engineRequestResume).
// Same single-writer discipline as the other latches in this file: set and
// cleared only under the commandGate / on the workloop.
#define kEngineResumeMaxTries 3
static bool gEngineResumePending = false;
static int  gEngineResumeTries   = 0;

// patch-32: HOT vs COLD rebuilds need different borrow policies, and one
// boot-arg cannot express both. Evidence: with latsof_strictborrow=0, three
// cold wakes borrowed a programmed SD7 instantly and worked every time —
// but a rebuild during live session churn (the device-switch wedge) burned
// all 12 tries on "FAILED: ROM IPC timeout", the documented signature of
// borrowing a programmed descriptor whose session is still hot. So: wake
// rebuilds honor the boot-arg (fast), while rebuilds triggered by IPC
// timeouts or capture demands wait for a clean window regardless of it.
static bool gHotRecovery = false;
// And never again lose WHY the retries failed: the give-up status used to
// overwrite the last initDSP failure reason.
static char gLastInitFail[40] = "";

static bool poll32(volatile UInt8 *b, UInt32 o, UInt32 mask, UInt32 val, UInt32 usec) {
    for (UInt32 t = 0; t < usec; t += 500) {
        if ((rd32(b, o) & mask) == val) return true;
        IODelay(500);
    }
    return false;
}

// patch-30 fix 2: one predicate for "is AppleHDA's output descriptor ours to
// borrow", shared by the load path and the wake path so the two can never
// disagree again. Returns -1 controller-not-decoding, 0 quiet, 1 RUN set,
// 2 programmed (CBL/BDL live — RUN clear is not enough, see the jackPoll
// comment: FIFO and DMA position are not register state and cannot be handed
// back to a stream AppleHDA already programmed).
static int outputSdBusyState(volatile UInt8 *hda, UInt16 *gcapOut = nullptr) {
    UInt16 g = rd16(hda, HDA_GCAP);
    if (gcapOut) *gcapOut = g;   // callers log the value the DECISION used
    if (g == 0xFFFF || g == 0x0000) return -1;
    // patch-45: when the lab overrides the loader stream, "busy" must be judged
    // on THAT stream, not on AppleHDA's hard-wired first output — otherwise a
    // swept index would be deferred/approved by the wrong descriptor's state.
    int bIdx = (gLabSIdx != 0xFFFFFFFF && gLabSIdx < 16)
                   ? (int)gLabSIdx : (int)((g >> 8) & 0xF);
    UInt32 o = SD_BASE + (UInt32)bIdx * SD_SIZE;
    if (rd8(hda, o) & SD_CTL_RUN) return 1;
    if (rd32(hda, o + SD_REG_CBL) || rd32(hda, o + SD_REG_BDLPL) ||
        rd32(hda, o + SD_REG_BDLPU)) return 2;
    return 0;
}

// ==================== patch-31: in-kernel AFG wake ====================
//
// Replaces the latsof-afgwake userland daemon (and its Login Items entry).
// MEASURED FAULT (1 Aug): when a jack event lands on an idle engine, AppleHDA
// starts streaming without restoring the codec's Audio Function Group (node
// 0x01) from D3 — audio drives a powered-down path, harsh static, cured only
// by a replug (which forces a full re-init). Writing SET_POWER_STATE D0 to
// the AFG clears it instantly; children cascade.
//
// HOW WE TALK TO THE CODEC. Not CORB/RIRB — that ring is AppleHDA's, and
// contending on it is the same shared-resource bet that cost patches 24-30a.
// The HDA spec provides a second, controller-arbitrated path: the Immediate
// Command Interface (ICOI/IRII/ICIS at 0x60/0x64/0x68). The controller
// injects the command into the link stream itself and routes the response to
// IRII, not RIRB. We claim it only when idle (ICB clear), bound every wait,
// and bail silently on contention or timeout — a lost round costs nothing
// because the next tick retries.
//
// WHEN WE ACT — the discipline that made the daemon safe, plus one insight
// the daemon could not use: SD7's descriptor is visible to us by pure MMIO.
// A clean->programmed/RUN transition on AppleHDA's output descriptor IS the
// engine rebuild that follows a jack insert or playback start — the exact
// moment the fault is born, observable without a single codec command. So:
//   - steady idle:      zero codec traffic (D3 is correct there — leave it)
//   - on SD transition: one GET_POWER_STATE; SET D0 only if not D0
//   - while RUN:        a belt check every ~10 s (one verb), nothing more
// SET_POWER_STATE D0 is idempotent and monotonic (only ever powers UP), so
// even a misjudged write cannot leave two pieces of state disagreeing.
//
// Escape hatch: latsof_afgwake=0 reverts to the userland daemon without a
// rebuild. If ICI itself proves unusable on this controller (never observed,
// but the spec allows quirks), the code counts failures and retires itself,
// publishing AFG-Wake = "ICI unavailable" so the daemon is known to be needed.
#define HDA_ICOI            0x60
#define HDA_IRII            0x64
#define HDA_ICIS            0x68
#define ICIS_ICB            0x1     // immediate command busy
#define ICIS_IRV            0x2     // immediate result valid (W1C)
#define kAfgNid             0x01
// HDA command word: (CAd << 28) | (NID << 20) | (verb12 << 8) | payload8.
// The NID field is bits 27:20 and is easy to leave out — the first cut of
// this patch shipped 0x000F0500/0x00070500, which addresses node 0x00 (the
// ROOT node) instead of the AFG. Root has no power state, so it answered 0,
// the code read that as "already D0", and it silently never corrected
// anything: no AFG-Wake property, no ICI failure, just a headphone hiss the
// daemon had to catch. Build the words from the parts so it cannot recur.
#define kHdaVerb(nid, verb, payload) \
    (((UInt32)(nid) << 20) | ((UInt32)(verb) << 8) | (UInt32)(payload))
#define kVerbGetPower       kHdaVerb(kAfgNid, 0xF05, 0x00)   // 0x001F0500
#define kVerbSetPowerD0     kHdaVerb(kAfgNid, 0x705, 0x00)   // 0x00170500
// patch-31b: jack sense on the headphone pin. SD7 alone is too late —
// AppleHDA does not touch that descriptor until playback starts, which is
// the same instant the fault becomes audible (measured: ~0.5 s of static
// before the SD-triggered correction landed). The CODEC knows sooner: pin
// 0x21's presence bit (31) flips when the connector seats, seconds before
// anyone presses play. Polling it is what lets the kext pre-arm the way the
// userland daemon did via CoreAudio jack notifications.
#define kHpPinNid           0x21
#define kVerbGetPinSense    kHdaVerb(kHpPinNid, 0xF09, 0x00)
#define kPinPresent         0x80000000u
#define kAfgSenseTicks      2       // poll jack sense every 2nd tick (~1 s)
#define kAfgBeltTicks       20      // RUN belt check: every 20th 500 ms tick
#define kAfgIciFailLimit    8       // consecutive ICI failures -> retire

static bool gAfgKextWake  = true;   // boot-arg latsof_afgwake=0 disables
static int  gAfgLastSd    = -2;     // last outputSdBusyState (-2 = unseen)
static UInt32 gAfgLastSig = 0;      // last BDL^CBL^FMT signature of SD7
static int  gAfgSenseTick = 0;      // jack-sense poll divider
static bool gAfgJackWasIn = false;  // last observed jack presence
static int  gAfgBeltTick  = 0;
static int  gAfgIciFails  = 0;
static UInt32 gAfgWakes   = 0;      // corrections issued (telemetry)
static bool   gAfgProbed  = false;  // published the first successful ICI read

// One immediate-interface verb. Returns false on contention or timeout —
// caller treats that as "not this tick", never as an error to act on.
static bool iciVerb(volatile UInt8 *hda, UInt32 verb, UInt32 *resp) {
    for (int t = 0; ; t++) {                     // claim only a quiet ICI
        if (!(rd16(hda, HDA_ICIS) & ICIS_ICB)) break;
        if (t >= 100) return false;              // ~1 ms: someone else owns it
        IODelay(10);
    }
    wr16(hda, HDA_ICIS, ICIS_IRV);               // W1C any stale response
    wr32(hda, HDA_ICOI, verb);
    wr16(hda, HDA_ICIS, ICIS_ICB);
    for (int t = 0; t < 100; t++) {              // ~1 ms response bound
        UInt16 s = rd16(hda, HDA_ICIS);
        if (s & ICIS_IRV) {
            if (resp) *resp = rd32(hda, HDA_IRII);
            wr16(hda, HDA_ICIS, ICIS_IRV);
            return true;
        }
        IODelay(10);
    }
    return false;
}

// patch-49: SD_STS_FIFO_READY (bit 5 = 0x20). Measured 2026-10-01 18:44/18:47
// with the same reader on both streams:
//   AppleHDA's LIVE playback stream  sd7  -> sts=0x20  lpib MOVING
//   every stream WE borrow                -> sts=0x00  lpib stuck at exactly 0
// while ROM_STATUS sits in wait=0x5 (WAIT_FOR_DMA_BUFFER_FULL). Linux's
// include/sound/hda_register.h names that bit:
//     #define SD_STS_FIFO_READY 0x20  /* FIFO ready */
// and the generic HDA core (sound/hda/hdac_stream.c snd_hdac_stream_reset)
// polls it right after the SRST pulse. This helper never did — it deasserts
// SRST and immediately programs the descriptor. A full controller reset
// (GCTL.CRST, patch-48 hdarst=1) does NOT make the bit come back either, so
// this is worth measuring rather than assuming: poll it, remember the result,
// and let the caller log it. If it never asserts we learn that too.
static bool gLastFifoReady = false;    // patch-49: outcome of the last SRST
static void streamReset(volatile UInt8 *hda, UInt32 sd) {
    wr8(hda, sd, 0); wr8(hda, sd + 2, 0); IODelay(100);
    wr8(hda, sd, SD_CTL_SRST);
    for (int t = 0; t < 300; t++) { if (rd8(hda, sd) & SD_CTL_SRST) break; IODelay(10); }
    wr8(hda, sd, 0);
    for (int t = 0; t < 300; t++) { if (!(rd8(hda, sd) & SD_CTL_SRST)) break; IODelay(10); }
    wr8(hda, sd + SD_REG_STS, 0x1C);
    // ~5 ms bound (Linux allows ~900 us); cheap, and this runs once per borrow.
    gLastFifoReady = false;
    for (int t = 0; t < 1000; t++) {
        if (rd8(hda, sd + SD_REG_STS) & 0x20) { gLastFifoReady = true; break; }
        IODelay(5);
    }
}

// LATITUDE FORK patch-24: the borrowed-stream contract.
//
// SD(numISS) — SD7 here — is AppleHDA's first output engine, and the code
// loader has to drive it. Borrowing is therefore unavoidable, so it has to be
// exact: snapshot once, restore once, and never write the descriptor again.
//
// Every sleep/wake failure this driver has had traces back to that contract
// being broken *after* the restore ran, and after the telemetry that claimed
// the restore had succeeded — so the reports read "restored" on wakes where
// the speakers were already dead. Two later blocks re-reset the stream and
// zeroed its BDL/CBL/LVI, and the two early failure paths jumped clean over
// the restore and left the descriptor pointing at our firmware buffer.
//
// Making the restore idempotent and callable from every exit path is what
// turns "we remembered to put it back this time" into an invariant: the last
// write this driver ever makes to the borrowed descriptor is AppleHDA's own
// state, on success and on every failure alike.
struct SdSnapshot {
    UInt32 ppctl, spibEn, spibVal, ctl, cbl, bdpl, bdpu;
    UInt16 lvi, fmt;
    bool   valid;      // snapshot was taken — nothing to give back without it
    bool   restored;   // already handed back — makes repeat calls a no-op
};

static void sdRestore(volatile UInt8 *hda, UInt32 sd, UInt32 sIdx,
                      UInt32 ppCap, UInt32 spibCap, SdSnapshot &s) {
    if (!s.valid || s.restored) return;
    s.restored = true;
    streamReset(hda, sd);
    wr32(hda, sd + SD_REG_BDLPL, s.bdpl);
    wr32(hda, sd + SD_REG_BDLPU, s.bdpu);
    wr32(hda, sd + SD_REG_CBL,   s.cbl);
    wr16(hda, sd + SD_REG_LVI,   s.lvi);
    wr16(hda, sd + SD_REG_FMT,   s.fmt);
    wr16(hda, sd,     (UInt16)(s.ctl & 0xFFFF));
    wr8(hda, sd + 2,  (UInt8)((s.ctl >> 16) & 0xFF));
    if (spibCap) {
        wr32(hda, spibCap + 0x08 + sIdx * 0x08, s.spibVal);
        wr32(hda, spibCap + 0x04, s.spibEn);
    }
    if (ppCap) wr32(hda, ppCap + PP_PPCTL, s.ppctl);
}

// Power states for sleep/wake
static IOPMPowerState sPowerStates[2] = {
    { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 1, kIOPMDeviceUsable | kIOPMPowerOn, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// ========== IOService ==========

bool LatSOFAudioDevice::init(OSDictionary *d) {
    if (!IOService::init(d)) return false;
    pciDevice = nullptr; hdaBarMap = nullptr; dspBarMap = nullptr;
    hdaBase = nullptr; dspBase = nullptr;
    ppCap = 0; spibCap = 0;
    sIdx = 0; sTag = 0; sd = 0;
    capIdx = 0; capTag = 0; capSd = 0;
    outboxOff = 0;
    hwReady = false; isPlaying = false; isCapturing = false;
    lastJackState = false; jackTimer = nullptr;
    commandGate = nullptr; rootDomain = nullptr; powerNotifier = nullptr;
    kernelAudio = nullptr;
    sharedDmaBuf = nullptr; sharedBdlBuf = nullptr;
    capDmaBuf = nullptr; capBdlBuf = nullptr;
    flagsBuf = nullptr;
    i2cPciDevice = nullptr; i2cBarMap = nullptr; i2cBase = nullptr;
    return true;
}

void LatSOFAudioDevice::free() {
    IOService::free();
}

IOService *LatSOFAudioDevice::probe(IOService *provider, SInt32 *score) {
    // LATITUDE FORK: the provider is IOResources, not the PCI device, because
    // AppleHDA owns 8086:02c8 and must keep owning it. The original body cast
    // the provider to IOPCIDevice and bailed out, so start() was never called.
    // Device discovery and validation now happen in start() via a registry walk.
    if (!IOService::probe(provider, score)) return nullptr;
    IOLog("LatSOF: probe() ok, provider=%s\n",
          provider ? provider->getName() : "(null)");
    return this;
}

bool LatSOFAudioDevice::start(IOService *provider) {
    IOLog("LatSOF: start() entered, provider=%s\n",
          provider ? provider->getName() : "(null)");
    if (!IOService::start(provider)) { IOLog("LatSOF: super::start failed\n"); return false; }
    // LATITUDE FORK: we are not this device's driver. AppleHDA owns it and
    // must keep owning it, so we locate the controller in the registry and
    // never call open().
    {
        OSDictionary *m = IOService::serviceMatching("IOPCIDevice");
        OSIterator *it = m ? IOService::getMatchingServices(m) : nullptr;
        if (m) m->release();
        if (it) {
            OSObject *o;
            while ((o = it->getNextObject()) != nullptr) {
                IOPCIDevice *c = OSDynamicCast(IOPCIDevice, o);
                if (!c) continue;
                if (c->configRead16(kIOPCIConfigVendorID) == 0x8086 &&
                    c->configRead16(kIOPCIConfigDeviceID) == 0x02c8) {
                    pciDevice = c; pciDevice->retain(); break;
                }
            }
            it->release();
        }
    }
    if (!pciDevice) { setProperty("Status", "FAILED: cAVS not found"), IOLog("LatSOF: %s\n", "FAILED: cAVS not found"); return false; }
    IOLog("LatSOF: found cAVS: %s\n", pciDevice->getName());
    // NOTE: no pciDevice->open() — AppleHDA holds it.

    pciDevice->setIOEnable(true);
    pciDevice->setBusMasterEnable(true);
    pciDevice->setMemoryEnable(true);
    // LATITUDE FORK: D3->D0 power cycle removed (AppleHDA owns power state)

    // TCSEL: clear TC bits [2:0] to TC0 (like Linux snd_intel_dsp_driver_probe)
    {
        UInt8 tcsel = pciDevice->configRead8(PCI_TCSEL);
        pciDevice->configWrite8(PCI_TCSEL, tcsel & ~0x07);
    }
    // Disable PCI interrupt (CMD bit 10)
    {
        UInt16 cmd = pciDevice->configRead16(0x04);
        pciDevice->configWrite16(0x04, cmd | 0x0400);
    }

    hdaBarMap = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
    dspBarMap = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress4);
    if (!hdaBarMap || !dspBarMap) {
        setProperty("Status", "FAILED: BAR map"), IOLog("LatSOF: %s\n", "FAILED: BAR map");
        goto fail;
    }
    hdaBase = (volatile UInt8 *)hdaBarMap->getVirtualAddress();
    dspBase = (volatile UInt8 *)dspBarMap->getVirtualAddress();

    // ============== patch-54: PCI BAR audit (once per load) =================
    // Linux's HDA_DSP_PP_BAR(1)/SPIB_BAR(2)/DRSM_BAR(3) are NOT necessarily
    // separate PCI BARs on every part - hda_dsp_ctrl_get_caps() assigns them
    // as `bus->remap_addr + <offset inside BAR0>`, i.e. they can live inside
    // the HDA BAR. But we have never once looked at what this HD Audio
    // function actually exposes, and the whole PP/SPIB question depends on it.
    {
        for (int i = 0; i < 6; i++)
            IOLog("LatSOF: BAR%d raw=0x%08x\n", i,
                  pciDevice->configRead32(kIOPCIConfigBaseAddress0 + 4 * i));
        IOLog("LatSOF: BAR0 len=%u BAR4 len=%u\n",
              (unsigned)hdaBarMap->getLength(), (unsigned)dspBarMap->getLength());
        for (int i = 1; i <= 3; i++) {
            IOMemoryMap *m = pciDevice->mapDeviceMemoryWithRegister(
                                 kIOPCIConfigBaseAddress0 + 4 * i);
            if (m) {
                IOLog("LatSOF: BAR%d mapped len=%u\n", i, (unsigned)m->getLength());
                m->release();
            } else {
                IOLog("LatSOF: BAR%d absent\n", i);
            }
        }
    }

    {   // patch-30 escape hatch (see the preflight in jackPoll)
        UInt32 sb = 1;
        if (PE_parse_boot_argn("latsof_strictborrow", &sb, sizeof(sb)))
            gStrictBorrow = (sb != 0);
        // patch-31 escape hatch: latsof_afgwake=0 reverts the AFG keep-alive
        // to the userland daemon without a rebuild.
        UInt32 aw = 1;
        if (PE_parse_boot_argn("latsof_afgwake", &aw, sizeof(aw)))
            gAfgKextWake = (aw != 0);
        // patch-68 escape hatch: latsof-auto=0 boots with auto mode off —
        // the kext never touches hardware until an explicit sysctl arm.
        // This is the A/B switch for isolating AppleHDA coexistence
        // regressions (2026-10-02 machine freeze) without a sysctl race:
        // the boot-time load happens ~2 s after kext start, before any
        // sysctl could be written, so the fuse needed a boot-time form.
        UInt32 au = 1;
        if (PE_parse_boot_argn("latsof_auto", &au, sizeof(au))) {
            gLabAutoBoot = (au != 0);
            gLabAuto = gLabAutoBoot;
            IOLog("LatSOF: boot-arg latsof-auto=%u — auto mode %s\n",
                  (unsigned)au, gLabAuto ? "on" : "OFF (no boot-time load)");
        }
    }

    // patch-30 fix 2: allocate the capture ring here, not in initDSP. This
    // is what lets the load path DEFER initDSP (below) and still publish the
    // kernel audio device: the engine's initHardware needs the ring, nothing
    // in it needs the DSP. Pure memory, no MMIO; initDSP's own !capDmaBuf
    // guards make its copy of this block a no-op.
    {
        if (!capDmaBuf) {
            capDmaBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
                kernel_task, kIOMemoryPhysicallyContiguous | kIODirectionInOut,
                kLatSOF_CapBufferSize, 0xFFFFFFFFFFFFF000ULL);
            if (capDmaBuf) capDmaBuf->prepare();
        }
        // review 1 Aug: BDL only if the ring exists (same rule as initDSP's
        // copy — see the comment there).
        if (!capBdlBuf && capDmaBuf) {
            UInt32 numBdl = (kLatSOF_CapBufferSize + PAGE_SIZE - 1) / PAGE_SIZE;
            UInt32 bdlSz = ((numBdl * 16) + 127) & ~127U;
            capBdlBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
                kIOMemoryPhysicallyContiguous | kIODirectionInOut, bdlSz, 0xFFFFFFFFFFFFFF80ULL);
            if (capBdlBuf && capDmaBuf) {
                capBdlBuf->prepare();
                HdaBdlEntry *cbdl = (HdaBdlEntry *)capBdlBuf->getBytesNoCopy();
                memset(cbdl, 0, bdlSz);
                UInt64 cphys = capDmaBuf->getPhysicalAddress();
                UInt32 crem = kLatSOF_CapBufferSize;
                for (UInt32 i = 0; i < numBdl && crem > 0; i++) {
                    UInt32 chunk = (crem > PAGE_SIZE) ? PAGE_SIZE : crem;
                    cbdl[i].addrLow = (UInt32)(cphys & 0xFFFFFFFF);
                    cbdl[i].addrHigh = (UInt32)(cphys >> 32);
                    cbdl[i].size = chunk; cbdl[i].ioc = 1;
                    crem -= chunk; cphys += chunk;
                }
            }
        }
    }

    // review 1 Aug round 4: the deferred path publishes the mic device from
    // this ring BEFORE any initDSP runs — an allocation failure here would
    // silently forfeit the Siri-visible device for the whole boot (the
    // engine's initHardware fails once and never retries). Fail loudly
    // instead: the §2 gate sees FAILED and the load is retried/rolled back.
    if (!capDmaBuf || !capBdlBuf) {
        setProperty("Status", "FAILED: capture ring allocation");
        IOLog("LatSOF: %s\n", "FAILED: capture ring allocation at load");
        goto fail;
    }

    // patch-30 fix 2: the load-path preflight §6.5 always lacked. Loading
    // while AppleHDA's output stream is RUN (or programmed, under strict
    // borrow) used to borrow it anyway — instant loud static, replug-proof,
    // observed with SD-Borrow ctl=0x14001e. The wake path solved this with
    // deferral to jackPoll's retry engine; hand the load path the exact same
    // machinery instead of a warning. A bounded sleep-wait here was rejected:
    // it blocks the kmutil/matching thread for up to 30 s, and when it
    // expires mid-playback it commits the very static it exists to prevent.
    // st == -1 (controller not decoding) must NOT defer — fall into initDSP
    // so its own GCAP wait and failure reporting still hard-fail start().
    {
        int st = outputSdBusyState(hdaBase);
        // patch-67: in auto mode the load path NEVER runs the chain here, on
        // the matching thread. It defers unconditionally to the retry engine,
        // which picks the job up ~1.5 s later on the workloop — so the DSP
        // comes up seconds after boot with no app involved, while start()
        // still cannot block (the same reason a bounded sleep-wait was
        // rejected in patch-30). The stream state still selects the message.
        bool deferInit = !gLabNoDefer &&
                         (gLabAuto || (st == 1) || (st == 2 && gStrictBorrow));
        if (deferInit) {
            gWakeReinitPending = true;
            gWakeTries = 0;
            gWakeTickDivider = 0;
            gProgrammedWaits = 0;
            gHotRecovery = true;   // patch-32: deferred BECAUSE audio was live
            setProperty("Status", gLabAuto
                ? "auto: DSP init deferred to retry engine (loads ~2s after boot)"
                : "deferred: AppleHDA output busy at load");
            IOLog("LatSOF: %s — deferring DSP init "
                  "to the retry engine (watch Wake-Retry / Status)\n",
                  gLabAuto ? "auto mode: init deferred at load"
                           : (st == 1) ? "AppleHDA output RUNNING at load"
                                       : "AppleHDA output programmed at load");
        } else if (st == 2) {
            IOLog("LatSOF: %s\n", "output programmed at load but "
                  "latsof_strictborrow=0 — borrowing anyway");
        } else if (st == -1) {
            IOLog("LatSOF: %s\n", "controller not decoding at load — "
                  "initDSP will wait/report as before");
        }
        if (!deferInit && !initDSP()) {
            if (gLabHoldOff) {
                // patch-46: the lab bench deliberately kept its hands off the
                // hardware (enable=0, or this generation's one run is already
                // spent). That is NOT a failure. If it were treated as one,
                // start() would fail, the device would never publish, the
                // jack timer would never install, and the only way to arm a
                // run would be a reboot — which is exactly the cycle this
                // patch exists to remove. Park the retry engine instead: it
                // ticks every ~1.5 s, each tick now costs nothing while held
                // off, so writing new lab parameters runs within a second.
                gWakeReinitPending = true;
                gWakeTries = 0;
                gWakeTickDivider = 0;
                gProgrammedWaits = 0;
                gHotRecovery = false;
                IOLog("LatSOF: lab hold-off at load — device published, "
                      "hardware untouched, retry engine parked\n");
            } else if (gWakeReinitPending) {
                // round 4: the borrow-time re-check inside initDSP bailed —
                // output became busy between preflight and borrow. Same
                // deferral as the preflight's own; retry engine takes over.
                gWakeTries = 0;
                gWakeTickDivider = 0;
                gProgrammedWaits = 0;
            } else {
                setProperty("Status", "FAILED: DSP init"), IOLog("LatSOF: %s\n", "FAILED: DSP init");
                goto fail;
            }
        }
    }

    // Create the commandGate BEFORE registering for PM / installing event
    // sources. All hw-touching paths (setPowerState, UserClient methods,
    // WillSleep message) serialize through this gate.
    if (getWorkLoop()) {
        commandGate = IOCommandGate::commandGate(this);
        if (commandGate) {
            getWorkLoop()->addEventSource(commandGate);
        }
    }

    // Power management
    PMinit();
    registerPowerDriver(this, sPowerStates, 2);
    provider->joinPMtree(this);

    // Subscribe to system-wide power events via gIOGeneralInterest. This is
    // the ONLY way to receive kIOMessageSystemWillSleep and
    // kIOPMMessageClamshellStateChange; registerInterestedDriver only
    // delivers PM-state messages like
    // kIOMessageDeviceWillPowerOff, *not* the system-wide ones we need.
    rootDomain = getPMRootDomain();
    if (rootDomain) {
        powerNotifier = rootDomain->registerInterest(
            gIOGeneralInterest, &sPowerInterestHandler, this, nullptr);
    }

    // Jack detection polling timer (every 500ms). Runs on the same workloop
    // as commandGate, so it is naturally serialized with PM and UserClient
    // paths — no extra lock needed.
    jackTimer = IOTimerEventSource::timerEventSource(this,
        OSMemberFunctionCast(IOTimerEventSource::Action, this, &LatSOFAudioDevice::jackPoll));
    if (jackTimer && getWorkLoop()) {
        getWorkLoop()->addEventSource(jackTimer);
        jackTimer->setTimeoutMS(500);
    }

    // Coordination flag page (see kLatSOF_MemFlags). One 4 KiB page, zero
    // initialised. Page-aligned for cheap plugin-side mmap semantics.
    flagsBuf = IOBufferMemoryDescriptor::inTaskWithOptions(
        kernel_task, kIODirectionInOut, PAGE_SIZE, PAGE_SIZE);
    if (flagsBuf) {
        flagsBuf->prepare();
        bzero(flagsBuf->getBytesNoCopy(), PAGE_SIZE);
    }

    // kernel-mic: publish the capture ring as a kernel audio device.
    // Non-fatal on failure — capture still works through the UserClient —
    // but log loudly, because Siri visibility is the entire point.
    kernelAudio = new LatSOFKernelAudioDevice;
    if (kernelAudio) {
        kernelAudio->setOwner(this);
        if (!kernelAudio->init(nullptr)) {
            kernelAudio->release(); kernelAudio = nullptr;
        } else if (!kernelAudio->attach(this)) {
            kernelAudio->release(); kernelAudio = nullptr;
        } else if (!kernelAudio->start(this)) {
            // attach succeeded: detach too, or the dead child stays
            // visible in the IORegistry forever (review minor).
            kernelAudio->detach(this);
            kernelAudio->release(); kernelAudio = nullptr;
        }
        if (!kernelAudio) {
            IOLog("LatSOF: kernel audio device FAILED to publish\n");
        } else {
            IOLog("LatSOF: kernel audio device published\n");
        }
    }

    IOLog("LatSOF: start() completed, hwReady=%d\n", hwReady ? 1 : 0);
    registerService();
    return true;

fail:
    // review 1 Aug: the hoisted capture ring is allocated before initDSP can
    // fail, and stop() never runs after a failed start() — without these two
    // lines every failed load leaks ~128 KiB of wired contiguous memory.
    // Non-null implies prepared here (the pair-atomic guards above), and no
    // DMA references the ring on this path (hwReady never went true).
    if (capBdlBuf) { capBdlBuf->complete(); capBdlBuf->release(); capBdlBuf = nullptr; }
    if (capDmaBuf) { capDmaBuf->complete(); capDmaBuf->release(); capDmaBuf = nullptr; }
    if (hdaBarMap) { hdaBarMap->release(); hdaBarMap = nullptr; }
    if (dspBarMap) { dspBarMap->release(); dspBarMap = nullptr; }
    if (pciDevice) { pciDevice->release(); pciDevice = nullptr; }
    return false;
}

// ==================== I2C + RT5682 ====================

bool LatSOFAudioDevice::initI2C() {
    // LATITUDE FORK: no I2S codecs on this board.
    return true;
#if 0
    // Find Intel LPSS I2C4 controller (PCI 8086:02c5)
    // This is the bus where RT5682 lives (confirmed by Linux: PCI 00:19.0)
    if (!i2cPciDevice) {
        OSDictionary *match = IOService::serviceMatching("IOPCIDevice");
        OSIterator *iter = IOService::getMatchingServices(match);
        if (iter) {
            IOService *svc;
            while ((svc = (IOService *)iter->getNextObject())) {
                IOPCIDevice *pci = OSDynamicCast(IOPCIDevice, svc);
                if (pci && pci->configRead16(kIOPCIConfigVendorID) == 0x8086 &&
                    pci->configRead16(kIOPCIConfigDeviceID) == 0x02c5) {
                    i2cPciDevice = pci;
                    i2cPciDevice->retain();
                    break;
                }
            }
            iter->release();
        }
        if (match) match->release();
    }
    if (!i2cPciDevice) { setProperty("I2C", "PCI 8086:02c5 not found"); return false; }

    // Power up I2C controller: D0 state, bus master, memory enable
    i2cPciDevice->setBusMasterEnable(true);
    i2cPciDevice->setMemoryEnable(true);
    i2cPciDevice->setIOEnable(true);
    // Force D0 power state (PMCSR offset varies, try 0x80 for LPSS)
    { UInt16 pmcsr = i2cPciDevice->configRead16(0x80);
      if (pmcsr & 0x3) { // if not D0
          i2cPciDevice->configWrite16(0x80, pmcsr & ~0x3); // set D0
          IOSleep(10);
      }
    }
    if (!i2cBarMap) i2cBarMap = i2cPciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
    if (!i2cBarMap) { setProperty("I2C", "BAR0 map failed"); return false; }
    i2cBase = (volatile UInt8 *)i2cBarMap->getVirtualAddress();

    // LPSS private registers: reset and unreset (offset 0x200 + 0x04)
    wr32(i2cBase, 0x204, 0);         // Assert reset
    IODelay(1000);
    wr32(i2cBase, 0x204, 3);         // Deassertion: FUNC + IDMA
    IODelay(1000);

    // Disable controller
    wr32(i2cBase, DW_IC_ENABLE, 0);
    for (int t = 0; t < 100; t++) { if (!(rd32(i2cBase, DW_IC_ENABLE_STATUS) & 1)) break; IODelay(100); }

    // Configure: fast mode (400kHz), master, 7-bit addr, restart enable
    wr32(i2cBase, DW_IC_CON, 0x65);
    // Fast mode SCL timing (for ~120MHz LPSS clock)
    wr32(i2cBase, DW_IC_FS_SCL_HCNT, 0x003C);
    wr32(i2cBase, DW_IC_FS_SCL_LCNT, 0x0082);
    // Target address
    wr32(i2cBase, DW_IC_TAR, RT5682_I2C_ADDR);
    // Disable all interrupts
    wr32(i2cBase, DW_IC_INTR_MASK, 0);

    // Enable controller
    wr32(i2cBase, DW_IC_ENABLE, 1);
    for (int t = 0; t < 100; t++) { if (rd32(i2cBase, DW_IC_ENABLE_STATUS) & 1) break; IODelay(100); }

    // Diagnostic: dump I2C controller state
    { char d[128]; snprintf(d, sizeof(d), "CON=0x%x TAR=0x%x STAT=0x%x EN=0x%x",
        rd32(i2cBase, DW_IC_CON), rd32(i2cBase, DW_IC_TAR),
        rd32(i2cBase, DW_IC_STATUS), rd32(i2cBase, DW_IC_ENABLE_STATUS));
      setProperty("I2C-Regs", d); }
    setProperty("I2C", "OK");
    return true;
#endif
}

bool LatSOFAudioDevice::i2cWrite16(UInt16 reg, UInt16 val) {
    if (!i2cBase) return false;
    // Clear any pending errors
    rd32(i2cBase, DW_IC_CLR_TX_ABRT);
    rd32(i2cBase, DW_IC_CLR_INTR);

    // Write 4 bytes: reg_hi, reg_lo, val_hi, val_lo
    UInt8 bytes[4] = { (UInt8)(reg >> 8), (UInt8)(reg & 0xFF),
                       (UInt8)(val >> 8), (UInt8)(val & 0xFF) };
    for (int i = 0; i < 4; i++) {
        UInt32 cmd = bytes[i];
        if (i == 3) cmd |= DW_IC_DATA_CMD_STOP;
        // Wait for TX FIFO not full
        for (int t = 0; t < 1000; t++) { if (rd32(i2cBase, DW_IC_STATUS) & DW_IC_STATUS_TFNF) break; IODelay(10); }
        wr32(i2cBase, DW_IC_DATA_CMD, cmd);
    }
    // Wait for TX complete
    for (int t = 0; t < 1000; t++) { if (rd32(i2cBase, DW_IC_STATUS) & DW_IC_STATUS_TFE) break; IODelay(10); }
    IODelay(100);
    bool ok = (rd32(i2cBase, DW_IC_RAW_INTR_STAT) & (1U << 6)) == 0;
    if (!ok) rd32(i2cBase, DW_IC_CLR_TX_ABRT); // clear abort for next transaction
    return ok;
}

UInt16 LatSOFAudioDevice::i2cRead16(UInt16 reg) {
    if (!i2cBase) return 0;
    rd32(i2cBase, DW_IC_CLR_TX_ABRT);
    rd32(i2cBase, DW_IC_CLR_INTR);

    // Drain any stale RX FIFO data (prevents stale reads after write bursts)
    for (int d = 0; d < 16 && rd32(i2cBase, DW_IC_RXFLR) > 0; d++)
        rd32(i2cBase, DW_IC_DATA_CMD);

    // Wait for bus idle (TX FIFO empty + bus not busy)
    for (int t = 0; t < 1000; t++) { if (rd32(i2cBase, DW_IC_STATUS) & DW_IC_STATUS_TFE) break; IODelay(10); }

    // Write register address (2 bytes), then read 2 bytes
    UInt8 addr[2] = { (UInt8)(reg >> 8), (UInt8)(reg & 0xFF) };
    for (int i = 0; i < 2; i++) {
        for (int t = 0; t < 1000; t++) { if (rd32(i2cBase, DW_IC_STATUS) & DW_IC_STATUS_TFNF) break; IODelay(10); }
        wr32(i2cBase, DW_IC_DATA_CMD, addr[i]);
    }
    // Issue 2 read commands: first with RESTART, last with STOP
    for (int t = 0; t < 1000; t++) { if (rd32(i2cBase, DW_IC_STATUS) & DW_IC_STATUS_TFNF) break; IODelay(10); }
    wr32(i2cBase, DW_IC_DATA_CMD, DW_IC_DATA_CMD_READ | DW_IC_DATA_CMD_RESTART);
    for (int t = 0; t < 1000; t++) { if (rd32(i2cBase, DW_IC_STATUS) & DW_IC_STATUS_TFNF) break; IODelay(10); }
    wr32(i2cBase, DW_IC_DATA_CMD, DW_IC_DATA_CMD_READ | DW_IC_DATA_CMD_STOP);

    // Read 2 bytes from RX FIFO
    UInt8 hi = 0, lo = 0;
    for (int t = 0; t < 1000; t++) { if (rd32(i2cBase, DW_IC_RXFLR) > 0) break; IODelay(10); }
    hi = (UInt8)rd32(i2cBase, DW_IC_DATA_CMD);
    for (int t = 0; t < 1000; t++) { if (rd32(i2cBase, DW_IC_RXFLR) > 0) break; IODelay(10); }
    lo = (UInt8)rd32(i2cBase, DW_IC_DATA_CMD);
    return ((UInt16)hi << 8) | lo;
}

bool LatSOFAudioDevice::initRT5682() {
    // LATITUDE FORK: no I2S codecs on this board.
    return true;
#if 0
    if (!initI2C()) return false;

    // Step 0: Enable I2C mode (RT5682 requires this before any register access)
    // Linux: regmap_write(regmap, RT5682_I2C_MODE, 0x1); usleep_range(10000,15000);
    IOSleep(300); // power-on settle time
    i2cWrite16(0xFFFF, 0x0001); // RT5682_I2C_MODE = 1
    IOSleep(15);

    // Step 1: Verify Device ID (with diagnostic)
    UInt16 devId = i2cRead16(RT5682_DEVICE_ID);
    { UInt32 rawStat = rd32(i2cBase, DW_IC_RAW_INTR_STAT);
      UInt32 rxflr = rd32(i2cBase, DW_IC_RXFLR);
      char d[64]; snprintf(d, sizeof(d), "0x%04x (raw_intr=0x%x rxflr=%u)", devId, rawStat, rxflr);
      setProperty("RT5682-ID", d); }
    if (devId != 0x6530) { setProperty("RT5682", "Wrong device ID"); return false; }

    // Step 2: Calibration (from rt5682_calibrate in Linux)
    // rt5682_reset(): RESET + re-enable I2C_MODE (mandatory after reset!)
    i2cWrite16(0x0000, 0x0000); // RESET
    IOSleep(300);
    i2cWrite16(0xFFFF, 0x0001); // I2C_MODE = 1 (Linux rt5682.c:819)
    IOSleep(15);

    i2cWrite16(0x0008, 0x000F); // I2C_CTRL
    i2cWrite16(0x0063, 0xA2AF); // PWR_ANLG_1
    IOSleep(15);
    i2cWrite16(0x0063, 0xF2AF); // PWR_ANLG_1 + fast VREF
    i2cWrite16(0x0094, 0x0300); // MICBIAS_2
    i2cWrite16(0x0080, 0x8000); // GLB_CLK = RCCLK
    i2cWrite16(0x0061, 0x0100); // PWR_DIG_1: LDO
    i2cWrite16(0x01C1, 0x3800); // HP_IMP_SENS_CTRL_19
    i2cWrite16(0x013A, 0x3000); // CHOP_DAC
    i2cWrite16(0x013C, 0x7005); // CALIB_ADC_CTRL
    i2cWrite16(0x0026, 0x686C); // STO1_ADC_MIXER
    i2cWrite16(0x0044, 0x0D0D); // CAL_REC
    i2cWrite16(0x01DF, 0x0321); // HP_CALIB_CTRL_2
    i2cWrite16(0x01DB, 0x0004); // HP_LOGIC_CTRL_2
    i2cWrite16(0x01DE, 0x7C00); // HP_CALIB_CTRL_1
    i2cWrite16(0x01E0, 0x06A1); // HP_CALIB_CTRL_3
    i2cWrite16(0x002B, 0x0311); // A_DAC1_MUX
    i2cWrite16(0x01DE, 0x7C00); // HP_CALIB_CTRL_1
    i2cWrite16(0x01DE, 0xFC00); // HP_CALIB_CTRL_1: start calibration

    // Poll calibration complete (bit 15 of 0x01EA clears)
    for (int t = 0; t < 60; t++) {
        if (!(i2cRead16(0x01EA) & 0x8000)) break;
        IOSleep(10);
    }

    // Restore defaults after calibration
    i2cWrite16(0x0063, 0x002F); // PWR_ANLG_1
    i2cWrite16(0x0094, 0x0080); // MICBIAS_2
    i2cWrite16(0x0080, 0x0000); // GLB_CLK
    i2cWrite16(0x0061, 0x0000); // PWR_DIG_1
    i2cWrite16(0x013A, 0x2000); // CHOP_DAC
    i2cWrite16(0x013C, 0x2005); // CALIB_ADC_CTRL
    i2cWrite16(0x0026, 0xC0C4); // STO1_ADC_MIXER
    i2cWrite16(0x0044, 0x0C0C); // CAL_REC

    // Step 3: Apply patch list (from rt5682_apply_patch_list)
    i2cWrite16(0x01C1, 0x1000);
    i2cWrite16(0x0100, 0xA020);
    i2cWrite16(0x0008, 0x000F);
    i2cWrite16(0x0156, 0x8266);
    i2cWrite16(0x0210, 0x22B7);
    i2cWrite16(0x0212, 0x0365);
    i2cWrite16(0x0215, 0x0110);
    i2cWrite16(0x0125, 0x0210);
    i2cWrite16(0x01DB, 0x0007);
    i2cWrite16(0x0211, 0xAC00);
    i2cWrite16(0x0016, 0x0104);

    // Step 4: Post-probe config (from rt5682_i2c_probe)
    i2cWrite16(0x008E, 0x0000); // DEPOP_1
    i2cWrite16(0x0063, 0x002C); // PWR_ANLG_1: LDO1_12V + HP_5X
    i2cWrite16(0x0094, 0x0080); // MICBIAS_2
    i2cWrite16(0x00C0, 0x6960); // GPIO_CTRL_1 (matches Linux)
    i2cWrite16(0x0145, 0x0000); // TEST_MODE_CTRL_1
    i2cWrite16(0x0125, 0x0220); // CHARGE_PUMP_1: CP_CLK_HP_300KHz
    i2cWrite16(0x006E, 0x1000); // DMIC_CTRL_1: FIFO_CLK_DIV_2

    // Steps 5+9 (PLL + HP power) DEFERRED to startPlayback()
    // Reason: MCLK only exists when SSP0 is streaming (confirmed on Linux:
    // CLK_DET=0x8001 during playback, 0x0000 after stop).
    // PLL needs MCLK to lock, so must configure after TRIG_START.

    // Step 6: I2S1 format (24-bit I2S, matches Linux 0xA220)
    i2cWrite16(0x0070, 0xA220); // I2S1_SDP: I2S, 24-bit

    // Step 7: TDM/ADDA registers (from Linux regmap - control DAC data routing)
    i2cWrite16(0x0073, 0x1001); // ADDA_CLK_1
    i2cWrite16(0x0075, 0x0002); // I2S1_F_DIV_1
    i2cWrite16(0x0076, 0x0001); // I2S1_F_DIV_2
    i2cWrite16(0x007b, 0x0080); // TDM_ADDA_CTRL_2
    i2cWrite16(0x007c, 0x0100); // TDM_ADDA_CTRL_3
    i2cWrite16(0x007e, 0x0020); // TDM_ADDA_CTRL_5
    i2cWrite16(0x0071, 0xC000); // I2S2_SDP
    i2cWrite16(0x008f, 0x1000); // DEPOP_2
    i2cWrite16(0x008c, 0x0003); // PLL_TRACK_11
    i2cWrite16(0x0083, 0x3100); // PLL_TRACK_1
    i2cWrite16(0x0084, 0x1100); // PLL_TRACK_2
    i2cWrite16(0x0085, 0x1000); // PLL_TRACK_3
    i2cWrite16(0x0086, 0x0005); // PLL_TRACK_4

    // Step 8: DAC volume
    i2cWrite16(0x0019, 0xAFAF); // DAC1_DIG_VOL: 0dB both channels

    // Step 9: Jack detection init (from Linux rt5682_set_jack_detect)
    i2cWrite16(0x009F, 0xD000); // RC_CLK_CTRL: POW_IRQ | POW_JDH | POW_ANA
    i2cWrite16(0x0064, (i2cRead16(0x0064) | 0x0008)); // PWR_ANLG_2: enable PWR_JDH (bit 3)
    i2cWrite16(0x00B7, 0x8000); // IRQ_CTRL_2: JD1_EN=1, JD1_POL=normal
    i2cWrite16(0x00F6, 0x0100); // JD_CTRL_1: enable JD1

    setProperty("RT5682", "Init OK");
    return true;
#endif
}

// ==================== DSP Init (called from start and wake) ====================

bool LatSOFAudioDevice::initDSP() {
    volatile UInt8 *hda = hdaBase;
    volatile UInt8 *dsp = dspBase;
    if (!hda || !dsp) return false;

    // patch-68: re-entrancy guard. See the comment at gInitInProgress.
    if (gInitInProgress) {
        setProperty("Init-Reentry", "bailed");
        IOLog("LatSOF: initDSP re-entered — bailing to the live load\n");
        return false;
    }
    struct InitGuard { bool &f; InitGuard(bool &f_) : f(f_) {} ~InitGuard() { f = false; } } initGuard(gInitInProgress);
    gInitInProgress = true;

    // ===================== patch-46 SAFETY GATE =====================
    // Read the lab bench FIRST, before a single MMIO read or PCI config
    // cycle. The old code called labRegister() ~600 lines further down, so a
    // boot whose early waits failed never exposed the bench at all.
    //
    // No `enable=1` in the string (the default, and every plain boot) means
    // this returns having done nothing whatsoever to the hardware. The user
    // arms a run by writing new parameters; that bumps a generation, and one
    // generation buys exactly one attempt. See the SAFETY block at the top.
    labRegister();
    labReload();
    gLabEnable = labU32("enable", 0);
    // patch-67: in auto mode the generation check is bypassed — the retry
    // engine's 12-attempt budget and its pending flag are the authorization
    // now, so wake / device-switch / capture-demand re-arms need no new
    // sysctl write. With the fuse off (auto=0) this is byte-identical to the
    // old gate.
    if (gLabEnable == 0 || (gLabGen == gLabGenDone && !gLabAuto)) {
        gLabHoldOff = true;
        static int lastIdleMsg = -1;      // 0 = enable=0, 1 = generation spent
        int m = gLabEnable ? 1 : 0;
        if (m != lastIdleMsg) {
            lastIdleMsg = m;
            setProperty("Status", m
                ? "idle: lab generation already spent - write new params to run again"
                : (gLabAuto
                    ? "idle: auto mode off (auto=0) - no hardware touched"
                    : "idle: lab enable=0 - no hardware touched"));
            IOLog("LatSOF: lab idle (%s) - hardware untouched\n",
                  m ? "generation spent" : "enable=0");
        }
        return false;   // callers treat gLabHoldOff as "idle", not "failed"
    }
    gLabHoldOff = false;
    // patch-67: only a manual (fused-off) run consumes its generation. In
    // auto mode the profile string stays armed so the retry engine can
    // rebuild after wake or failure — its own budget is the loop guard.
    if (!gLabAuto) gLabGenDone = gLabGen;    // consume this generation: exactly one run
    IOLog("LatSOF: lab ARMED gen=%u raw=\"%s\"\n", gLabGen,
          gLabBuf[0] ? gLabBuf : "(empty)");
    // ================================================================

    // LATITUDE FORK patch-17: restore D0 before reading anything.
    // Our sleep path lets IOKit PM drop this function to D3 and expects wake
    // to rebuild. But AppleHDA owns the device, so nobody puts it back in D0
    // on our schedule — and a D3 function does not answer memory cycles at
    // all, so every read is 0xFFFF and no amount of waiting helps (patch-15
    // waited its full 3 s and still saw all-ones while AppleHDA's speakers
    // worked fine). setMemoryEnable/setBusMasterEnable are command-register
    // bits and cannot fix this on their own. So: find the PCI Power
    // Management capability and put the function back in D0 ourselves.
    // No-op when already in D0, which is every normal boot.
    if (pciDevice) {
        UInt8 pmCap = 0;
        if (pciDevice->configRead16(0x06) & 0x0010) {   // capability list present
            UInt8 off = pciDevice->configRead8(0x34) & 0xFC;
            for (int g = 0; g < 48 && off >= 0x40; g++) {
                if (pciDevice->configRead8(off) == 0x01) { pmCap = off; break; }
                off = pciDevice->configRead8(off + 1) & 0xFC;
            }
        }
        UInt16 pmcs = pmCap ? pciDevice->configRead16(pmCap + 4) : 0;
        if (pmCap && (pmcs & 0x3) != 0) {
            pciDevice->configWrite16(pmCap + 4, (UInt16)(pmcs & ~0x3));
            IOSleep(10);                                // PCI spec D3hot->D0 recovery
        }
        pciDevice->setMemoryEnable(true);
        pciDevice->setBusMasterEnable(true);
        { char p[72];
          snprintf(p, sizeof(p), "pmcap=0x%02x pmcs=0x%04x after=0x%04x",
                   pmCap, pmcs, pmCap ? pciDevice->configRead16(pmCap + 4) : 0);
          setProperty("PCI-Power", p); }
    }

    // LATITUDE FORK patch-15: never trust registers that read all-ones.
    // At boot the PCI family may not have enabled memory decode yet; at
    // wake, setPowerStateGated's setMemoryEnable is not enough because the
    // function can still be mid D3->D0 restore — that restore belongs to
    // AppleHDA (it owns the PCI device), so we are racing its wake path.
    // With GCAP=0xffff the code loader binds to stream 15 and every
    // "wait for bit set" check passes vacuously; the init then dies at
    // INIT_DONE. Wait, bounded, before deriving ANYTHING.
    {
        int tries = 0;
        UInt16 g = rd16(hda, HDA_GCAP);
        while ((g == 0xFFFF || g == 0x0000) && tries < 50) {   // patch-22: retries replace the long wait
            IOSleep(10);
            g = rd16(hda, HDA_GCAP);
            tries++;
        }
        { char w[48];
          snprintf(w, sizeof(w), "gcap=0x%04x tries=%d ms=%d", g, tries, tries * 10);
          setProperty("GCAP-Wait", w); }
        if (g == 0xFFFF || g == 0x0000) {
            setProperty("Status", "FAILED: controller not decoding (GCAP)"),
                IOLog("LatSOF: %s\n", "FAILED: controller not decoding (GCAP)");
            return false;   // hwReady stays false -> next wake retries
        }
    }

    // LATITUDE FORK patch-18: arrive LAST. After a wake, GCTL.CRST going to
    // 1 is AppleHDA bringing the shared link out of reset; initialising the
    // controller concurrently with its restore is how firmware loads fail.
    // Wait for CRST, then give AppleHDA a further settle window. If CRST
    // never appears (cold boot, load order undefined) degrade to the old
    // behaviour rather than fail — the pre-patch code never checked it.
    {
        int tries = 0;
        while (!(rd32(hda, HDA_GCTL) & 1U) && tries < 100) {   // patch-22: retries replace the long wait
            IOSleep(10);
            tries++;
        }
        bool linkUp = (rd32(hda, HDA_GCTL) & 1U) != 0;
        if (linkUp) IOSleep(750);
        { char h[56];
          snprintf(h, sizeof(h), "crst=%d tries=%d ms=%d",
                   linkUp ? 1 : 0, tries, tries * 10 + (linkUp ? 750 : 0));
          setProperty("HDA-Settle", h); }
    }

    {
        UInt64 dspLen = dspBarMap->getLength();
        UInt16 gcap = rd16(hda, HDA_GCAP);
        int numISS = (gcap >> 8) & 0xF;
        int numOSS = (gcap >> 12) & 0xF;
        { char d[64]; snprintf(d, sizeof(d), "GCAP=0x%04x ISS=%d OSS=%d sIdx=%d",
              gcap, numISS, numOSS, numISS);
          setProperty("HDA-Streams", d);
          IOLog("LatSOF: %s\n", d); }
        // patch-51: GCAP semantics (sound/sound/hda_register.h):
        //   AZX_GCAP_ISS (15<<8)  = number of INPUT stream descriptors
        //   AZX_GCAP_OSS (15<<12) = number of OUTPUT stream descriptors
        // and hda_dsp_stream_init() numbers them
        //   SD0 .. SD(ISS-1)      = CAPTURE, tags 1..ISS
        //   SD(ISS) .. SD(ISS+OSS-1) = PLAYBACK, tags 1..OSS
        // (that is the only way the measured dump makes sense: sd0=tag1 and
        //  sd1=tag2 must be capture, because sd7=tag1 is a live playback
        //  stream - two output streams cannot share tag 1).
        // Measured here: GCAP=0x9701 -> ISS=7, OSS=9, so sd0..sd6 are CAPTURE
        // and sd7..sd15 are PLAYBACK. The code-loader stream MUST be a PLAYBACK
        // stream (Linux: hda_cl_prepare(..., SNDRV_PCM_STREAM_PLAYBACK, ...));
        // every lab run that used sidx=0/1/2 was pointed at a CAPTURE descriptor
        // by accident. The old default sIdx = ISS = 7 is the first playback
        // stream but AppleHDA is actively RUNNING it, so every default attempt
        // was deferred. Do what Linux's hda_dsp_stream_get(PLAYBACK) does: take
        // the first FREE playback stream, with its natural tag (idx-ISS+1).
        int sIdx = numISS; // first playback stream index
        for (int i = numISS; i < numISS + (int)numOSS && i < 16; i++) {
            UInt32 o2 = SD_BASE + (UInt32)i * SD_SIZE;
            if (!(rd8(hda, o2) & SD_CTL_RUN) &&
                rd32(hda, o2 + SD_REG_CBL) == 0 &&
                rd32(hda, o2 + SD_REG_BDLPL) == 0) { sIdx = i; break; }
        }
        UInt32 sTag = (UInt32)(sIdx - numISS + 1);   // natural tag for that SD
        UInt32 sd = SD_BASE + (UInt32)sIdx * SD_SIZE;
        UInt32 ppCap = 0, spibCap = 0;

        // PORT DEBUG (Lenovo): pick this attempt's hypothesis (see the stage
        // table at the top of the file).
        UInt32 stage = gFwStage++ % FW_STAGE_COUNT;
        bool linuxTrigger = (stage >= 1);
        // patch-35: coupled vs decoupled hypothesis. AppleHDA's own streams
        // work coupled on this controller (BIOS ships GPROCEN=0); Linux SOF
        // always decouples the CL stream. If the decouple machinery (SPIB /
        // DPIB handshake) is the reason the DMA never fills the buffer, the
        // coupled stages move LPIB and the ROM follows.
        bool coupled = (stage == 1 || stage == 2 || stage == 4);

        // patch-44 LAB BENCH: re-read the lab sysctl and let it override every
        // knob the stage table encodes. Empty value => identical to the
        // compiled-in behaviour. This is what removes the rebuild/approve/
        // reboot cycle from the debug loop: parameters change from userspace
        // (sysctl -w kern.latsof_lab="...") and the next retry (~1.5 s) picks
        // them up.
        // patch-46: labRegister()/labReload() now run at the very top of
        // initDSP, ahead of any hardware access, and they own the generation
        // counter. Do not call them again here — only parse the knobs.
        {
            UInt32 v;
            char kb[128];
            if ((v = labU32("stage", 0xFFFFFFFF)) != 0xFFFFFFFF && v < FW_STAGE_COUNT * 4) {
                stage = v;
                linuxTrigger = (stage >= 1 || labU32("trigger", 0) != 0);
                coupled = labU32("coupled", (stage == 1 || stage == 2 || stage == 4) ? 1 : 0) != 0;
            }
            if ((v = labU32("tag", 0xFFFFFFFF)) != 0xFFFFFFFF) sTag = v & 0xF;
            gLabFmt    = labU32("fmt", 0);
            gLabChunk  = labU32("chunk", 0);
            gLabAbits  = labU32("abits", 0);
            gLabUnmap  = labU32("unmapped", 0);
            gLabHold   = labU32("hold", 0);
            gLabSpib   = labU32("spib", 0xFFFFFFFF);
            gLabGproc  = labU32("gproc", 0xFFFFFFFF);
            gLabNoRun  = labU32("norun", 0);
            gLabSIdx   = labU32("sidx", 0xFFFFFFFF);   // patch-45
            gLabNoDefer= labU32("nodefer", 0);         // patch-45
            gLabHdaRst = labU32("hdarst", 0);          // patch-48
            gLabAllSts = labU32("allsts", 0);          // patch-48
            // patch-53. Note the defaults are the LINUX values, so a run that
            // says nothing about them now tests the reference configuration.
            gLabCg6     = labU32("cg6", 1);            // 0 clear / 1 set / 2 leave
            gLabDplEn   = labU32("dplen", 1);          // 1 = enable DPLBASE
            gLabIocAll  = labU32("iocall", 1);         // 1 = IOC on every BDLE
            gLabBdlDump = labU32("bdldump", 0);
            gLabCapChain = labU32("capchain", 1);      // patch-54
            gLabMBox     = labU32("mbox", 0);          // patch-54
            gLabSpibCpl  = labU32("spibcpl", 1);       // patch-55
            gLabClobber  = labU32("clobber", 1);       // patch-55
            gLabPie      = labU32("pie", 1);           // patch-56
            labStr("note", kb, sizeof(kb));
            if (kb[0] || gLabBuf[0]) {
                IOLog("LatSOF: lab raw=\"%s\"\n", gLabBuf[0] ? gLabBuf : "(empty)");
            }
        }
        // patch-45: the loader stream index is a lab knob now, so one rebuild
        // sweeps the whole 0..15 space. Re-derive sd because it was computed
        // from the compiled-in sIdx above (line "int sIdx = numISS").
        if (gLabSIdx != 0xFFFFFFFF && gLabSIdx < 16) {
            sIdx = (int)gLabSIdx;
            sd   = SD_BASE + (UInt32)sIdx * SD_SIZE;
        }
        { char s[176]; snprintf(s, sizeof(s),
              "stage=%u tag=%u sidx=%d trig=%u cpl=%u fmt=0x%x chunk=%u abits=%u unmap=%u spib=%d gproc=%d norun=%u nodefer=%u hdarst=%u allsts=%u pie=%u steal=%u",
              stage, sTag, sIdx, linuxTrigger ? 1 : 0, coupled ? 1 : 0, gLabFmt, gLabChunk,
              gLabAbits, gLabUnmap, (int)gLabSpib, (int)gLabGproc, gLabNoRun, gLabNoDefer,
              gLabHdaRst, gLabAllSts, gLabPie, labU32("steal", 0));
          setProperty("FW-Stage", s);
          IOLog("LatSOF: effective %s\n", s); }

        // PORT DEBUG: controller state before we touch anything — global
        // enables plus every non-idle stream descriptor (tag/format/length),
        // so we can see what AppleHDA already owns and whether any stream is
        // actually programmed with our tag.
        UInt32 intctlOrig = rd32(hda, HDA_INTCTL);
        { char g[144];
          snprintf(g, sizeof(g),
                   "GCTL=0x%08x INTCTL=0x%08x EM2=0x%08x DPLBASE=0x%08x DPUBASE=0x%08x PCI48=0x%02x",
                   rd32(hda, HDA_GCTL), intctlOrig, rd32(hda, HDA_VS_EM2),
                   rd32(hda, 0x70), rd32(hda, 0x74),
                   pciDevice ? pciDevice->configRead8(0x48) : 0);
          setProperty("HDA-Global", g); IOLog("LatSOF: %s\n", g); }
        // patch-43: DSP wake / platform-firmware forensics. gctl bit8 is the
        // DSP-wake request on CML; on a machine whose BIOS never runs the
        // SOF firmware the DSP may stay asleep no matter what we do, which
        // would explain a frozen ROM_STATUS and a DMA engine that never
        // fetches. Dump the audit trail once per attempt.
        {
            UInt32 gctlFull = rd32(hda, HDA_GCTL);
            IOLog("LatSOF: dspState gctl=0x%08x dspur=%u adspcs=0x%08x pfwStatus=0x%08x fwStatus=0x%08x c0state=0x%08x",
                  gctlFull, (gctlFull >> 8) & 1U,
                  rd32(dsp, DSP_ADSPCS),
                  rd32(dsp, 0x8100), rd32(dsp, 0x8000),
                  rd32(dsp, 0x0008));
        }
        {
            // patch-49: the old dump capped at 400 chars, so it TRUNCATED at
            // sd9 — the input streams (10..15) were never shown, and neither
            // was SDnSTS (FIFORDY). Both matter now: FIFORDY is the measured
            // difference between AppleHDA's working stream and ours, and the
            // question "is FIFORDY a property of a stream that has been
            // brought up, or only of one that is running?" is answered by
            // reading it on AppleHDA's programmed-but-idle streams before we
            // touch anything. No cap, all 16, with st/fifos/lpib.
            static char buf[1400];
            int n = 0; buf[0] = 0;
            for (int i = 0; i < 16; i++) {
                UInt32 off = SD_BASE + (UInt32)i * SD_SIZE;
                UInt32 c = rd32(hda, off) & 0x00FFFFFF;
                n += snprintf(buf + n, sizeof(buf) - n,
                              "sd%d[t%u c=0x%06x f=0x%04x cbl=%u lvi=%u st=%02x fo=%04x lp=%u] ",
                              i, (c >> 20) & 0xF, c,
                              rd16(hda, off + SD_REG_FMT),
                              rd32(hda, off + SD_REG_CBL),
                              rd16(hda, off + SD_REG_LVI),
                              rd8(hda, off + SD_REG_STS),
                              rd16(hda, off + SD_REG_FIFOS),
                              rd32(hda, off + 0x04));
            }
            setProperty("HDA-StreamDump", buf); IOLog("LatSOF: streams %s\n", buf);
        }
        // patch-48 GROUND TRUTH (before): the dump above says what is
        // PROGRAMMED; this one says what is MOVING. Everything so far has
        // treated `lpib == 0` on our borrowed stream as proof that the DMA
        // engine never fetched a descriptor. That inference has never been
        // checked against a stream that is known to work. AppleHDA's own
        // playback streams are exactly that reference: if their LPIB/DPIB
        // advance across our window while ours stay at 0, the engine is fine
        // and the fault is in our programming; if theirs are frozen too, then
        // a zero position is simply how this controller reports and the whole
        // "engine dead" reading was an artefact. One line, no hardware writes.
        {
            // patch-56: carry the WHOLE descriptor, not just run/lpib. The
            // 16-SD dump already showed that AppleHDA's live playback stream
            // (sd7) differs from every stream we borrow in two fields we never
            // used to sample while it was RUNNING: SDxFMT (0x4031 vs our
            // 0x0040) and SDxFIFOSIZE (0x0160 running / 0x0080 armed vs our
            // 0x0000). Sampling them here — with the tone deliberately looped
            // so a live stream always exists — turns "AppleHDA works and we
            // don't" from an anecdote into a field-by-field reference.
            char buf[1024]; int n = 0; buf[0] = 0;
            for (int i = 0; i < 16 && n < 900; i++) {
                UInt32 off = SD_BASE + (UInt32)i * SD_SIZE;
                UInt32 c = rd32(hda, off) & 0x00FFFFFF;
                UInt32 lp = rd32(hda, off + 0x04);
                if (!(c & SD_CTL_RUN) && !lp) continue;
                UInt32 dpibReg = HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)i;
                n += snprintf(buf + n, sizeof(buf) - n,
                              "sd%d[run=%u c=0x%06x t%u f=0x%04x fo=0x%04x cbl=%u sts=0x%02x lpib=%u dpib=%u] ",
                              i, (c >> 1) & 1U, c, (c >> 20) & 0xF,
                              rd16(hda, off + SD_REG_FMT),
                              rd16(hda, off + SD_REG_FIFOS),
                              rd32(hda, off + SD_REG_CBL),
                              rd8(hda, off + SD_REG_STS),
                              lp, rd32(hda, dpibReg));
            }
            if (n == 0) snprintf(buf, sizeof(buf), "no stream running");
            setProperty("HDA-Moving-Pre", buf);
            IOLog("LatSOF: moving-pre %s\n", buf);
        }
        // LATITUDE FORK patch-20: snapshot of AppleHDA's state on the
        // stream we are about to borrow. Declared here, not at the point of
        // use, because the goto targets below would jump past the
        // initialisation otherwise.
        SdSnapshot snap = {};
        UInt32 fwSize = (UInt32)sof_fw_size;
        UInt32 payloadOffset = 0, payloadSize = 0;
        // patch-36: IOMMU-aware DMA allocation.
        // Decisive finding from the patch-35 boot: AppleVTD is LIVE on this
        // machine (ioreg shows 1 AppleVTD instance; OC DisableIoMapper=false).
        // The old code handed raw physical addresses (getPhysicalAddress /
        // getPhysicalSegment) to the HDA DMA engine. AppleHDA's own streams
        // go through IODMACommand and are IOMMU-mapped; ours were not, so
        // the IOMMU silently dropped every transaction: the engine never
        // fetched the BDL (FIFORDY stayed 0, LPIB/DPIB 0), and the ROM sat
        // in wait=5 (WAIT_FOR_DMA_BUFFER_FULL) for 12+ builds. Every mode
        // switch (coupled/decoupled) or format change changed nothing —
        // exactly what an unmapped-DMA failure looks like.
        // allocDma() uses IODMACommand::gen64IOVMSegments, whose IOVMAddr is
        // a bus address the IOMMU actually maps.
        // patch-53: posDma declared here (not next to its use) because the
        // failure gotos below jump past any later initialiser.
        DmaBuf *fwDma = nullptr, *bdlDma = nullptr, *posDma = nullptr;
        UInt32 numBdl = 0;
        bool fwLoaded = false;
        // patch-52: declared HERE, not next to their use, because the two
        // `goto cleanup` sites above (stream borrow preflight) jump past any
        // later initialisation - C++ forbids jumping into a scope and skipping
        // a variable's initialiser.
        UInt32 imrBoot = 0;      // imr=1  -> IMR (Image Module Recovery) boot
        bool   imrDone = false;  // FW_ENTERED reached through the IMR path
        bool   l1Poke  = false;  // l1poke=1 -> hold EM2.L1SEN clear all run
        UInt32 l1Pokes = 0;

        // EM2 (0x1030). Linux clears L1SEN (bit 13) for streams that are not
        // DMI-L1 compatible (SOF _hda_dsp_stream_get); the reference sets bit
        // 14 instead. Stage 6 tests the Linux variant.
        if (stage == 6)
            wr32(hda, HDA_VS_EM2, rd32(hda, HDA_VS_EM2) & ~0x2000U);
        else
            wr32(hda, HDA_VS_EM2, rd32(hda, HDA_VS_EM2) | 0x4000);
        // patch-49: l1sen knob. EM2 bit 13 is HDA_VS_INTEL_EM2_L1SEN — the
        // PCIe L1 sub-state enable for the HDA link. Linux clears it during
        // controller init (hda_dsp_ctrl_clock_power_gating(..., false)); this
        // driver only ever did so on stage 6, and the default path sets bit 14
        // without touching bit 13. If the link drops into L1 the controller's
        // DMA can stall with LPIB frozen and no error status — which is
        // exactly our signature. A knob so it can be A/B'd without a rebuild.
        // patch-52 `l1poke=1`: a one-shot clear is not what Linux does. Linux
        // clears L1SEN in _hda_dsp_stream_get() and it STAYS clear because SOF
        // owns the controller for the whole boot. Here AppleHDA is alive on the
        // same controller and actively manages L1SEN for its own streams, so a
        // single write at this point can be undone long before the DMA start.
        // l1poke re-clears it on every poll tick for the whole window and
        // counts the re-arms - the count is the proof of who is fighting us.
        l1Poke = labU32("l1poke", 0) != 0;
        if (l1Poke) {
            wr32(hda, HDA_VS_EM2, rd32(hda, HDA_VS_EM2) & ~0x2000U);
            IOLog("LatSOF: lab l1poke=1 — L1SEN cleared + held clear\n");
        }
        { UInt32 l1 = labU32("l1sen", 0xFFFFFFFF);
          if (l1 != 0xFFFFFFFF) {
              UInt32 e = rd32(hda, HDA_VS_EM2);
              e = l1 ? (e | 0x2000U) : (e & ~0x2000U);
              wr32(hda, HDA_VS_EM2, e);
              IOLog("LatSOF: lab l1sen=%u -> EM2=0x%08x\n", l1, e);
          } }
        // patch-61: em2b31 — EM2 bit31. The Ubuntu same-machine dump shows the
        // Linux SOF steady state EM2=0x04806000 while AppleHDA leaves
        // 0x84806000: the ONLY bit difference is bit31 (unnamed in the public
        // headers, set by AppleHDA). Clear it to make EM2 bit-identical to
        // Linux. em2b31=0 clears, 1 sets, absent = leave alone.
        { UInt32 b31 = labU32("em2b31", 0xFFFFFFFF);
          if (b31 != 0xFFFFFFFF) {
              UInt32 e = rd32(hda, HDA_VS_EM2);
              e = b31 ? (e | 0x80000000U) : (e & ~0x80000000U);
              wr32(hda, HDA_VS_EM2, e);
              IOLog("LatSOF: lab em2b31=%u -> EM2=0x%08x\n", b31, e);
          } }
        IODelay(100);

        // ==================== HDA CONTROLLER INIT (match Linux hda_dsp_ctrl_init_chip) =========
        // patch-53: PCI CGCTL bit6 (MISCBDCGE).
        // Linux brackets the controller reset with misc_clock_gating(false)
        // then misc_clock_gating(true), so its STEADY STATE has bit6 = 1, and
        // the BIOS agrees (this register reads 0xfd before we touch it).
        // patch-52 and earlier cleared bit6 and never restored it, leaving the
        // controller in a state that neither Linux nor the BIOS ever produces,
        // at exactly the moment the code-loader DMA has to run. That was
        // simply wrong; the knob still exists so the old value can be A/B'd.
        {
            UInt8 cgctl = pciDevice->configRead8(0x48);
            UInt8 want  = cgctl;
            if      (gLabCg6 == 0) want = (UInt8)(cgctl & ~(UInt8)(1U << 6));
            else if (gLabCg6 == 1) want = (UInt8)(cgctl |  (UInt8)(1U << 6));
            if (want != cgctl) pciDevice->configWrite8(0x48, want);
            IOLog("LatSOF: cg6=%u PCI 0x48 0x%02x->0x%02x (bit6=MISCBDCGE)\n",
                  (unsigned)gLabCg6, cgctl, pciDevice->configRead8(0x48));
        }

        // patch-50/51: PCI CGCTL / PGCTL — Linux's
        // hda_dsp_ctrl_clock_power_gating(), called with enable=false from
        // hda_dsp_pre_fw_run() i.e. immediately BEFORE the CL boot:
        //     val = enable ? PCI_CGCTL_ADSPDCGE : 0;
        //     snd_sof_pci_update_bits(sdev, PCI_CGCTL, PCI_CGCTL_ADSPDCGE, val);
        //     val = enable ? HDA_VS_INTEL_EM2_L1SEN : 0;
        //     if (!enable || !hda->l1_disabled) snd_sof_dsp_update_bits(EM2, L1SEN, val);
        //     val = enable ? 0 : PCI_PGCTL_ADSPPGD;
        //     snd_sof_pci_update_bits(sdev, PCI_PGCTL, PCI_PGCTL_ADSPPGD, val);
        //
        // patch-51 CORRECTION: the PGCTL offset is **0x44**, not 0x4C.
        // sound/soc/sof/intel/hda.h:
        //     #define PCI_TCSEL   0x44
        //     #define PCI_PGCTL   PCI_TCSEL      <- an ALIAS of TCSEL
        //     #define PCI_CGCTL   0x48
        //     #define PCI_PGCTL_ADSPPGD   BIT(2)
        //     #define PCI_CGCTL_ADSPDCGE  BIT(1)
        // so patch-50's `pgctl=` knob wrote a completely unrelated byte at
        // 0x4C and the readback "0x4C=0x00->0x00" never meant anything.
        // 0x44 is the register this driver already clobbers as "TCSEL" with
        // `tcsel & ~0x07` (line ~904) — which CLEARS ADSPPGD, i.e. the exact
        // opposite of Linux. Measured here: 0x48=0xbf (ADSPDCGE=1, clock
        // gating ENABLED vs Linux OFF) and 0x44 = ? (ADSPPGD cleared vs Linux
        // set). Both raw-byte knobs kept, plus `pciGate=1` = Linux's three
        // writes done faithfully.
        {
            UInt8 c0 = pciDevice->configRead8(0x48);
            UInt8 p0 = pciDevice->configRead8(0x44);
            UInt8 c4 = pciDevice->configRead8(0x4C);   // kept only for the log
            UInt32 cgv = labU32("cgctl", 0xFFFFFFFF);
            UInt32 pgv = labU32("pgctl", 0xFFFFFFFF);
            if (cgv != 0xFFFFFFFF) pciDevice->configWrite8(0x48, (UInt8)cgv);
            if (pgv != 0xFFFFFFFF) pciDevice->configWrite8(0x44, (UInt8)pgv);
            if (labU32("pciGate", 0)) {
                UInt8 cg = pciDevice->configRead8(0x48);
                pciDevice->configWrite8(0x48, (UInt8)(cg & ~0x02));   // ADSPDCGE(BIT1)=0
                { UInt32 e2 = rd32(hda, HDA_VS_EM2);
                  wr32(hda, HDA_VS_EM2, e2 & ~0x2000U); }             // L1SEN(BIT13)=0
                UInt8 pg = pciDevice->configRead8(0x44);
                pciDevice->configWrite8(0x44, (UInt8)(pg | 0x04));    // ADSPPGD(BIT2)=1
            }
            IOLog("LatSOF: pci 0x48=0x%02x->0x%02x  0x44=0x%02x->0x%02x  0x4C=0x%02x "
                  "(cgctl=%d pgctl=%d gate=%d)\n",
                  c0, pciDevice->configRead8(0x48),
                  p0, pciDevice->configRead8(0x44), c4,
                  (int)(cgv == 0xFFFFFFFF ? -1 : (int)cgv),
                  (int)(pgv == 0xFFFFFFFF ? -1 : (int)pgv),
                  (int)labU32("pciGate", 0));
        }

        // Clear WAKESTS if controller not in reset
        if (rd32(hda, HDA_GCTL) & 1)
            wr32(hda, 0x0E, 0xFFFF);  // WAKESTS = SOF_HDA_WAKESTS_INT_MASK

        // ========= patch-63: porfirst — DSP POR before the HDA CRST =========
        // The Ubuntu same-machine diff found ONE hard divergence: after OUR
        // full init_chip CRST, STATESTS latches 0x0001 (only CAD0, the
        // ALC257) while Linux reports 0x5 — CAD2, the cAVS DSP's OWN HDA
        // codec, answers the wake on Linux but not for us. On Linux the DSP
        // ROM is already up and waiting when the HDA controller reset runs;
        // here the CRST fired while AppleHDA's firmware still owned the DSP,
        // and the ROM only came up later (prefwrun) — after the wake cycle
        // was over. Hypothesis: the ROM's link DMA attaches to the link via
        // the reset-wake cycle, so the ROM must be WAITING before the CRST.
        // porfirst=1: run the POR now, wait for the ROM to reach
        // FSR_WAIT_FOR_DMA_BUFFER_FULL, then the normal hdaInit CRST below
        // hits a listening codec. The mid-flow POR is skipped when this ran.
        bool porFirstDone = false;
        if (labU32("porfirst", 0) && labU32("prefwrun", 0)) {
            UInt32 hm  = 0xF;                 // host_managed_cores_mask (CNL/CML)
            UInt32 pre = rd32(dsp, DSP_ADSPCS);
            if (pre & ADSPCS_CPA(hm)) {
                UInt32 c = pre;
                c |= ADSPCS_CRST(hm) | ADSPCS_CSTALL(hm);   // stall + reset
                c &= ~ADSPCS_SPA(hm);                       // power down
                wr32(dsp, DSP_ADSPCS, c);
                poll32(dsp, DSP_ADSPCS, ADSPCS_CPA(hm), 0, 50000);
            }
            { UInt32 c = rd32(dsp, DSP_ADSPCS);
              c |= ADSPCS_SPA(hm);
              wr32(dsp, DSP_ADSPCS, c); }
            bool cpaOk = poll32(dsp, DSP_ADSPCS, ADSPCS_CPA(hm), ADSPCS_CPA(hm), 50000);
            { UInt32 c = rd32(dsp, DSP_ADSPCS);
              c &= ~ADSPCS_CRST(hm);                        // out of reset
              wr32(dsp, DSP_ADSPCS, c); }
            poll32(dsp, DSP_ADSPCS, ADSPCS_CRST(hm), 0, 50000);
            IOLog("LatSOF: lab porfirst=1 POR adspcs 0x%08x -> 0x%08x cpaOk=%u\n",
                  pre, rd32(dsp, DSP_ADSPCS), cpaOk ? 1U : 0U);
            /* wait for the ROM to reach its wait state 0x05000001
             * (FSR state=1 INIT_DONE, wait=5 FSR_WAIT_FOR_DMA_BUFFER_FULL) */
            UInt32 romv = rd32(dsp, 0x80000);
            int t;
            for (t = 0; t < 400 && (romv & 0x0F000000U) != 0x05000000U; t++) {
                IODelay(5000);
                romv = rd32(dsp, 0x80000);
            }
            IOLog("LatSOF: lab porfirst=1 ROM wait-state 0x%08x after %d ms\n",
                  romv, t * 5);
            porFirstDone = true;
        }

        // ========= patch-51: faithful hda_dsp_ctrl_init_chip (hdaInit=1) =========
        // The ONE Linux step this driver has deliberately never performed.
        // Since patch-1 the reasoning was "AppleHDA already owns the controller,
        // a CRST pulse would drop its streams" - and hdarst=1 confirmed that it
        // does. But Linux ALWAYS runs this full reset+init before the CL boot, so
        // on every Linux system the DSP FW boot happens on a controller that was
        // freshly reset and re-initialised by the same driver. Ours has always
        // booted on whatever AppleHDA+BIOS left behind, with AppleHDA's streams
        // live. Knob-gated because it costs AppleHDA its streams for this boot
        // (reboot to recover). This SUPERSEDES hdarst=1, which only pulsed CRST
        // and did none of the re-initialisation.
        // Verbatim target: sound/soc/sof/intel/hda-ctrl.c hda_dsp_ctrl_init_chip().
        if (labU32("hdaInit", 0)) {
            UInt32 hdaInitMode = labU32("hdaInit", 0);
            UInt32 g0 = rd32(hda, HDA_GCTL);
            if (hdaInitMode <= 2) {   /* 1/2 = with CRST pulse; 3 = no reset */
            /* hda_codec_set_codec_wakeup(true): WAKEEN = STATESTS */
            wr16(hda, 0x0C, rd16(hda, 0x0E));
            /* clear WAKE_STS if not out of reset */
            if (g0 & 1) wr32(hda, 0x0E, 0xFFFF);
            /* hda_dsp_ctrl_link_reset(true): "0 to enter reset" */
            wr32(hda, HDA_GCTL, g0 & ~1U);
            for (int t = 0; t < 100 && (rd32(hda, HDA_GCTL) & 1); t++) IODelay(500);
            IODelay(1000);                             /* usleep_range(500,1000) */
            /* hda_dsp_ctrl_link_reset(false): "1 to exit reset" */
            wr32(hda, HDA_GCTL, rd32(hda, HDA_GCTL) | 1U);
            for (int t = 0; t < 100 && !(rd32(hda, HDA_GCTL) & 1); t++) IODelay(500);
            IODelay(1200);                             /* usleep_range(1000,1200) */
            /* patch-62: codec detection. STATESTS latches the wake responses
             * DURING reset-exit — read it NOW, before the WAKESTS clear below
             * wipes it. Every previous hdaInit=2 observation (patch-57 single
             * read, patch-58 poll) logged codec_mask=0 because the clear at
             * line "clear WAKESTS / INTSTS" ran first. Linux's init_chip
             * grabs codec_mask before its own clear and reports 0x5 on THIS
             * machine (CAD0 ALC257 + CAD2 the DSP's own HDA codec). */
            {
                UInt16 stsF = 0, stsv = 0; int pms = 0;
                stsF = stsv = (UInt16)(rd16(hda, 0x0E) & 0x7FFFU);
                for (; pms < 1000 && !stsv; pms += 25) {
                    IODelay(25000);
                    stsv = (UInt16)(rd16(hda, 0x0E) & 0x7FFFU);
                }
                IOLog("LatSOF: lab hdaInit=%u STATESTS post-reset first=0x%04x poll=0x%04x after %d ms\n",
                      hdaInitMode, stsF, stsv, pms);
            }
            /* Accept unsolicited responses */
            wr32(hda, HDA_GCTL, rd32(hda, HDA_GCTL) | (1U << 8));
            /* clear stream status for every stream */
            for (int i = 0; i < 16; i++)
                wr8(hda, SD_BASE + (UInt32)i * SD_SIZE + SD_REG_STS, SD_CTL_INT_MASK);
            /* clear WAKESTS / INTSTS */
            wr32(hda, 0x0E, 0xFFFF);
            wr32(hda, 0x24, 0xC00000FFU);
            /* enable CIE and GIE interrupts */
            wr32(hda, HDA_INTCTL, rd32(hda, HDA_INTCTL) | 0xC0000000U);
            IOLog("LatSOF: lab hdaInit=%u gctl 0x%08x->0x%08x intctl=0x%08x intsts=0x%08x\n",
                  hdaInitMode, g0, rd32(hda, HDA_GCTL), rd32(hda, HDA_INTCTL), rd32(hda, 0x24));
            } /* end of CRST-pulse path (modes 1/2); mode 3 skips it entirely */
            // ============ patch-57: hdaInit=2 = CORB/RIRB command DMA ============
            // hdaInit=1 (patch-51) stops where hda_dsp_ctrl_init_chip() also
            // stops... minus one call: hda_codec_init_cmd_io() ->
            // snd_hdac_bus_init_cmd_io() (sound/hda/hdac_controller.c). The
            // 2026-10-02 matrix proved the CL stream is refused in EVERY
            // configuration while AppleHDA's own streams run, and that a CRST
            // pulse makes even the REFUSED-RUN state worse (RUN dropped after
            // hdaInit=1 / cg6=0, and AppleHDA dead for the boot). The one
            // thing a reset controller needs that we have never re-done is the
            // command DMA: after CRST the CORB/RIRB base registers are cleared
            // and per the Intel HD-A spec no codec communication works until
            // software re-inits them. Linux re-inits them here, before the CL
            // boot, on every boot. Verbatim from snd_hdac_bus_init_cmd_io():
            if (hdaInitMode >= 2) {
                /* STATESTS re-read (mode 3: as-is, expect 0 — no wake event;
                 * mode 2: post-WAKESTS-clear, the latched mask was already
                 * reported by the post-reset poll above). */
                UInt16 sts = 0; int pollMs = 0;
                sts = (UInt16)(rd16(hda, 0x0E) & 0x7FFFU);
                UInt16 stsFirst = sts;
                for (; pollMs < 1000 && !sts; pollMs += 25) {
                    IODelay(25000);
                    sts = (UInt16)(rd16(hda, 0x0E) & 0x7FFFU);
                }
                IOLog("LatSOF: lab hdaInit=%u STATESTS first=0x%04x poll=0x%04x after %d ms (codec_mask)\n",
                      hdaInitMode, stsFirst, sts, pollMs);
                static DmaBuf *rb = nullptr;   /* CORB 2048 + RIRB 2048 */
                if (!rb) rb = allocDma(4096, 2048);
                if (rb) {
                    UInt64 base = rb->physAddr;
                    /* CORB set up */
                    wr32(hda, 0x40, (UInt32)(base & 0xFFFFFFFFU));   /* CORBLBASE  */
                    wr32(hda, 0x44, (UInt32)(base >> 32));           /* CORBUBASE  */
                    wr8 (hda, 0x4E, 0x02);                           /* CORBSIZE: 256 entries */
                    wr16(hda, 0x4A, 0);                              /* CORBWP = 0 */
                    wr16(hda, 0x48, 0x8000);                         /* CORBRP reset */
                    for (int t = 0; t < 100 && (rd16(hda, 0x48) & 0x7FFFU); t++) IODelay(10);
                    wr16(hda, 0x48, 0);                              /* clear CORBRP_RST */
                    wr8 (hda, 0x4C, 0x02);                           /* CORBCTL: RUN (DMA on) */
                    /* RIRB set up — second half of the same page */
                    wr32(hda, 0x50, (UInt32)((base + 2048) & 0xFFFFFFFFU));
                    wr32(hda, 0x54, (UInt32)((base + 2048) >> 32));
                    wr8 (hda, 0x5E, 0x02);                           /* RIRBSIZE: 256 entries */
                    wr16(hda, 0x58, 0x8000);                         /* RIRBWP reset */
                    wr16(hda, 0x5A, 1);                              /* RINTCNT = 1 */
                    wr8 (hda, 0x5C, 0x02 | 0x04);                    /* RIRBCTL: DMA_EN|IRQ_EN */
                    wr8 (hda, 0x5D, 0x05);                           /* RIRBSTS clear */
                    IOLog("LatSOF: lab hdaInit=%u CORB@0x%llx RIRB@0x%llx "
                          "corbctl=0x%02x rirbctl=0x%02x corbrp=0x%04x rirbwp=0x%04x\n",
                          hdaInitMode, base, base + 2048, rd8(hda, 0x4C), rd8(hda, 0x5C),
                          rd16(hda, 0x48), rd16(hda, 0x58));
                } else {
                    IOLog("LatSOF: lab hdaInit=%u allocDma(4096) FAILED — no CORB/RIRB\n",
                          hdaInitMode);
                }
                /* hda_codec_set_codec_wakeup(false) — the err: tail of init_chip.
                 * Only meaningful for mode 2 (which set WAKEEN); mode 3 never
                 * touched WAKEEN, so leave it alone (zero AppleHDA disturbance). */
                if (hdaInitMode == 2)
                    wr16(hda, 0x0C, 0);
            }
        }

        // LATITUDE FORK: surgical replacement for the reference's global init.
        // The original cleared status for EVERY stream and blind-wrote INTCTL,
        // which would clobber AppleHDA's stream 0 and its interrupt enables.
        // We touch only our own loader stream and read-modify-write INTCTL.
        {
            const UInt32 kLoaderStream = 1;   // SD0 belongs to AppleHDA
            UInt32 sdOff = SD_BASE + kLoaderStream * SD_SIZE;
            // patch-26: wr8, not wr32 — SDSTS is a byte register at +0x03,
            // and the old 32-bit write was misaligned, spanning into the
            // neighbouring read-only bytes. Worked on this chipset; still
            // wrong. streamReset() has always used wr8 here.
            wr8(hda, sdOff + SD_REG_STS, 0x1C);           // our stream only
            // patch-26: the INTCTL GIE/CIE enable that used to follow is
            // deleted. Since patch-18 removed IOCE/SIE from the loader,
            // nothing on our side services or needs any HDA interrupt —
            // poll-only means poll-only at the controller level too. Setting
            // the global enables was harmless only while AppleHDA also had
            // them set; if a retry fired while AppleHDA was mid-restore with
            // GIE deliberately clear, we would have un-gated its latched
            // interrupts at a moment the owner did not expect.
        }

        // Re-enable misc clock gating (PCI CGCTL bit 6)
        { UInt8 cgctl = pciDevice->configRead8(0x48);
          pciDevice->configWrite8(0x48, cgctl | (1U << 6)); }

        // ==================== FIRMWARE LOADING ====================

        UInt32 extSig = *(UInt32 *)sof_fw_data;
        if (extSig == 0x6e614d58) // 'XMan'
            payloadOffset = *(UInt32 *)(sof_fw_data + 4);
        payloadSize = fwSize - payloadOffset;

        if (payloadOffset >= fwSize || payloadSize == 0) {
            setProperty("Status", "FAILED: bad FW payload"), IOLog("LatSOF: %s\n", "FAILED: bad FW payload");
            goto done;
        }

        // ========= patch-54: authoritative capability discovery ============
        // hda_dsp_ctrl_get_caps() (hda-ctrl.c:58) does NOT scan: it reads
        // SOF_HDA_LLCH (BAR0+0x14), then loops
        //     cap  = read(BAR0 + offset)
        //     id   = (cap & GENMASK(27,16)) >> 16
        //     ...  bus->ppcap/spbcap/drsmcap = remap_addr + offset
        //     offset = cap & SOF_HDA_CAP_NEXT_MASK        // 0xFFFF
        // for at most SOF_HDA_MAX_CAPS (10) hops.
        // We have always brute-forced instead. If the brute force ever landed
        // on a dword that merely *looks* like id 3/4, every PPCTL and SPIB
        // access of the last 50 patches went to the wrong register - which
        // would look exactly like a perfect descriptor and zero DMA.
        if (gLabCapChain) {
            UInt32 llchRaw = rd32(hda, 0x14);
            UInt32 off = llchRaw & 0xFFFF;
            IOLog("LatSOF: caps LLCH raw=0x%08x off=0x%04x bar0Len=%u\n",
                  llchRaw, off, (unsigned)hdaBarMap->getLength());
            for (int n = 0; off && n < 12; n++) {
                UInt32 cap = rd32(hda, off);
                UInt32 id  = (cap >> 16) & 0xFFF;
                IOLog("LatSOF: caps [%d] off=0x%04x raw=0x%08x id=%u next=0x%04x\n",
                      n, off, cap, id, cap & 0xFFFF);
                if (id == HDA_CAP_PP_ID   && !ppCap)   ppCap   = off;
                if (id == HDA_CAP_SPIB_ID && !spibCap) spibCap = off;
                off = cap & 0xFFFF;
            }
            IOLog("LatSOF: caps chain pp=0x%x spib=0x%x\n", ppCap, spibCap);
        }

        // PORT FIX (Lenovo XiaoXin 13 / CML): PPCTL.GPROCEN must be enabled
        // BEFORE the DSP core power-up handshake below. The reference Dell
        // BIOS leaves GPROCEN set at boot, so this driver's late PPCTL write
        // (after the borrow) always worked there; Lenovo clears it, and with
        // the processing-engine domain disabled the SPA->CPA handshake can
        // never complete — the exact "FAILED: core power" seen on every try.
        // Linux SOF enables the processing engine during probe, before any
        // firmware load. Read-modify-write so AppleHDA's bits are preserved.
        {
            UInt32 hdaLenE = (UInt32)hdaBarMap->getLength();
            for (UInt32 o = 0x500; o < hdaLenE && o < 0x2000; o += 0x10) {
                UInt16 capId = (rd32(hda, o) >> 16) & 0xFFF;
                if (capId == HDA_CAP_PP_ID) { if (!ppCap) ppCap = o; break; }
            }
            if (ppCap) {
                UInt32 pp = rd32(hda, ppCap + PP_PPCTL);
                { char e[48]; snprintf(e, sizeof(e), "ppctl=0x%08x gprocen=%d",
                                       pp, (pp & PP_PPCTL_GPROCEN) ? 1 : 0);
                  setProperty("PPCTL-Early", e); }
                if (!(pp & PP_PPCTL_GPROCEN)) {
                    wr32(hda, ppCap + PP_PPCTL, pp | PP_PPCTL_GPROCEN);
                    IOLog("LatSOF: %s\n",
                          "GPROCEN was clear — enabled before core power (port fix)");
                }
                // patch-44 knob: gproc=0/1 forces the bit either way, so the
                // "is the processing-engine domain gating DMA?" theory can be
                // tested without a rebuild.
                if (gLabGproc != 0xFFFFFFFF) {
                    UInt32 cur = rd32(hda, ppCap + PP_PPCTL);
                    UInt32 nv = gLabGproc ? (cur | PP_PPCTL_GPROCEN)
                                          : (cur & ~PP_PPCTL_GPROCEN);
                    if (nv != cur) {
                        wr32(hda, ppCap + PP_PPCTL, nv);
                        IOLog("LatSOF: lab gproc=%u -> ppctl 0x%08x\n", gLabGproc, nv);
                    }
                }
            } else {
                setProperty("PPCTL-Early", "PP capability not found");
            }
        }

        // ============== patch-48: HDA controller hygiene (knob gated) ==========
        // See the gLabHdaRst/gLabAllSts block at the top of the file. Measured
        // 2026-10-01: a stream accepts RUN the FIRST time it is borrowed and
        // then refuses it for the rest of the boot, whatever we reprogram, and
        // the DMA never fetches a byte even while RUN holds. Both are exactly
        // what stale, never-cleared stream/controller state looks like — and
        // the one structural thing Linux does that we have never done is put
        // the controller through a reset before loading.
        if (gLabAllSts) {
            // Linux clears SDnSTS for every stream in hda_dsp_ctrl_init_chip.
            for (int i = 0; i < 16; i++)
                wr8(hda, SD_BASE + (UInt32)i * SD_SIZE + SD_REG_STS, (UInt8)SD_CTL_INT_MASK);
        }
        if (gLabHdaRst) {
            wr32(hda, HDA_GCTL, rd32(hda, HDA_GCTL) & ~1U);     // enter reset
            for (int i = 0; i < 200 && (rd32(hda, HDA_GCTL) & 1U); i++) IOSleep(1);
            IODelay(1000);
            wr32(hda, HDA_GCTL, rd32(hda, HDA_GCTL) | 1U);      // exit reset
            for (int i = 0; i < 200 && !(rd32(hda, HDA_GCTL) & 1U); i++) IOSleep(1);
            IOSleep(2);
            IOLog("LatSOF: lab hdarst=1 - HDA controller reset pulsed, GCTL=0x%08x\n",
                  rd32(hda, HDA_GCTL));
        }
        // =======================================================================

        // DSP reset + power up
        wr32(dsp, DSP_ADSPCS, rd32(dsp, DSP_ADSPCS) | ADSPCS_CRST(0xF) | ADSPCS_CSTALL(0xF));
        IODelay(1000);
        wr32(dsp, DSP_ADSPCS, rd32(dsp, DSP_ADSPCS) | ADSPCS_SPA(0xF));
        if (!poll32(dsp, DSP_ADSPCS, ADSPCS_CPA(0xF), ADSPCS_CPA(0xF), 50000)) {
            setProperty("Status", "FAILED: core power"), IOLog("LatSOF: %s\n", "FAILED: core power");
            goto done;
        }

        // Allocate FW DMA buffers (patch-36: via IODMACommand, see above)
        fwDma = allocDma(payloadSize, 0x1000);
        if (!fwDma) { setProperty("Status", "FAILED: FW alloc"), IOLog("LatSOF: %s\n", "FAILED: FW alloc"); goto done; }
        memcpy(fwDma->virtAddr, sof_fw_data + payloadOffset, payloadSize);

        // patch-48: LVI must describe the entries we ACTUALLY filled. The old
        // code derived numBdl from PAGE_SIZE and then filled entries of
        // chunkSz, so any chunk= run larger than a page wrote fewer entries
        // than LVI-1 claimed: the descriptor ended up pointing past its own
        // table. Every chunk= experiment so far therefore tested a malformed
        // descriptor, not the chunk-size theory. Derive the entry count from
        // the chunk instead — which also makes Linux's code-loader layout
        // expressible: hda_cl_prepare sets period_bytes = bufsize, i.e. one
        // fragment, LVI = 0, a single BDL entry (chunk=<payloadSize>).
        UInt32 chunkSz = gLabChunk ? gLabChunk : PAGE_SIZE;
        if (chunkSz > payloadSize) chunkSz = payloadSize;
        {   // keep the entry count sane and every entry 128-byte aligned
            UInt32 minChunk = (((payloadSize + 255) / 256) + 127) & ~127U;
            if (chunkSz < minChunk) chunkSz = minChunk;
        }
        numBdl = (payloadSize + chunkSz - 1) / chunkSz;
        {
            UInt32 bdlSize = ((numBdl * 16) + 127) & ~127U;
            bdlDma = allocDma(bdlSize, 0x80);
            if (!bdlDma) { freeDma(fwDma); fwDma = nullptr;
                           setProperty("Status", "FAILED: BDL alloc"), IOLog("LatSOF: %s\n", "FAILED: BDL alloc"); goto done; }
            HdaBdlEntry *bdl = (HdaBdlEntry *)bdlDma->virtAddr;
            memset(bdl, 0, bdlSize);
            // fwDma is physically contiguous, so the IOMMU mapping is one
            // linear bus region — chunk it into BDL entries as before.
            UInt64 fwBus = fwDma->physAddr;
            UInt32 rem = payloadSize;
            // patch-53: Linux's IOC placement. hda_dsp_stream_setup_bdl()
            // uses period_bytes = bufsize/2 for a single contiguous buffer
            // (chunk_size == bufsize -> period_bytes /= 2), and
            // hda_setup_bdle() sets ioc on the final chunk of each period.
            // So with Linux's own chunking exactly two entries carry IOC=1:
            // the last entry of period 0 and the last entry overall.
            UInt32 periodBytes = payloadSize / 2;
            UInt32 done = 0;
            for (UInt32 i = 0; i < numBdl && rem > 0; i++) {
                UInt32 chunk = (rem > chunkSz) ? chunkSz : rem;
                bdl[i].addrLow = (UInt32)(fwBus & 0xFFFFFFFF);
                bdl[i].addrHigh = (UInt32)(fwBus >> 32);
                bdl[i].size = chunk;
                done += chunk;
                bdl[i].ioc = (gLabIocAll && periodBytes && (done % periodBytes) == 0) ? 1 : 0;
                rem -= chunk; fwBus += chunk;
            }
            if (gLabBdlDump) {
                HdaBdlEntry *b2 = (HdaBdlEntry *)bdlDma->virtAddr;
                for (UInt32 i = 0; i < numBdl && i < 4; i++)
                    IOLog("LatSOF: bdldump[%u] lo=0x%08x hi=0x%08x size=%u ioc=%u\n",
                          i, b2[i].addrLow, b2[i].addrHigh, b2[i].size, b2[i].ioc);
                const UInt8 *pl = (const UInt8 *)fwDma->virtAddr;
                IOLog("LatSOF: bdldump payload %02x %02x %02x %02x %02x %02x %02x %02x "
                      "%02x %02x %02x %02x %02x %02x %02x %02x  (want offset %u)\n",
                      pl[0], pl[1], pl[2], pl[3], pl[4], pl[5], pl[6], pl[7],
                      pl[8], pl[9], pl[10], pl[11], pl[12], pl[13], pl[14], pl[15],
                      (unsigned)payloadOffset);
                const UInt8 *fw = sof_fw_data + payloadOffset;
                IOLog("LatSOF: bdldump source  %02x %02x %02x %02x %02x %02x %02x %02x "
                      "%02x %02x %02x %02x %02x %02x %02x %02x\n",
                      fw[0], fw[1], fw[2], fw[3], fw[4], fw[5], fw[6], fw[7],
                      fw[8], fw[9], fw[10], fw[11], fw[12], fw[13], fw[14], fw[15]);
            }
        }
        IOLog("LatSOF: bdl chunk=%u entries=%u cbl=%u lvi=%u\n",
              chunkSz, numBdl, payloadSize, numBdl - 1);
        UInt64 bdlBus = bdlDma->physAddr;

        // Find PP, SPIB, and ML capabilities
        UInt32 mlCap = 0;
        {
            UInt32 hdaLen = (UInt32)hdaBarMap->getLength();
            for (UInt32 o = 0x500; o < hdaLen && o < 0x2000; o += 0x10) {
                UInt16 capId = (rd32(hda, o) >> 16) & 0xFFF;
                // patch-54: log what the brute force would have chosen, so it
                // can be compared against the chain walk above. capchain=1 has
                // already filled ppCap/spibCap, so these only fire when it did
                // not find them.
                if (capId == HDA_CAP_ML_ID && !mlCap) { mlCap = o;
                    IOLog("LatSOF: caps brute ml=0x%x raw=0x%08x\n", o, rd32(hda, o)); }
                if (capId == HDA_CAP_PP_ID && !ppCap) { ppCap = o;
                    IOLog("LatSOF: caps brute pp=0x%x raw=0x%08x\n", o, rd32(hda, o)); }
                if (capId == HDA_CAP_SPIB_ID && !spibCap) { spibCap = o;
                    IOLog("LatSOF: caps brute spib=0x%x raw=0x%08x\n", o, rd32(hda, o)); }
            }
        }

        // review 1 Aug round 4: the preflight classified this descriptor
        // >=750 ms ago (settle + firmware waits) — AppleHDA may have begun
        // playback since. Re-classify at the last instant BEFORE the
        // snapshot, shrinking the check-to-use window to microseconds.
        // RUN always bails (the instant-loud-static case); a PROGRAMMED
        // descriptor bails only while borrow patience remains — once the
        // retry engine has decided borrow-anyway (patience exhausted),
        // passing it here is what keeps the mic recoverable at all, since
        // SD7 stays programmed forever after AppleHDA's first playback.
        //
        // Placed before the snapshot deliberately: `goto cleanup` then frees
        // fwBuf/bdlBuf and re-masks the DSP interrupts like every other exit,
        // while sdRestore no-ops on the still-invalid snapshot — so the bail
        // writes NOTHING to a descriptor that is not ours. (Bailing after the
        // snapshot would either leak both firmware buffers or, via cleanup,
        // run streamReset on AppleHDA's live stream: the exact harm this
        // check exists to prevent.)
        // patch-59: steal=1 — free tag 1 before the borrow guard. The cAVS
        // ROM's link input DMA is suspected to be hardwired to stream tag 1:
        // Linux hands the CL stream the FIRST free playback stream on every
        // machine (index/tag rule in hda-stream.c), which at firmware load
        // is always tag 1. If the ROM waits on tag 1, a CL stream on tag 2
        // can never be matched by the DSP's link DMA: the FIFO is never
        // allocated, FIFORDY never sets, the host DMA stalls at LPIB=0 —
        // exactly our frozen 0x05000001 signature with RUN stuck. Stop
        // whatever stream currently holds tag 1 so our sidx/tag can claim
        // it. (AppleHDA's stream loses RUN for the rest of the boot; the
        // descriptor stays programmed and is snapshotted/restored as usual.)
        if (labU32("steal", 0)) {
            for (int i = 0; i < 16; i++) {
                UInt32 off = SD_BASE + (UInt32)i * SD_SIZE;
                UInt32 c = rd32(hda, off);
                if (((c >> 20) & 0xFU) == 1 && (c & 0x02)) {
                    IOLog("LatSOF: lab steal=1 stopping sd%d t1 (c=0x%06x)\n", i, c);
                    wr8(hda, off + 2, (UInt8)(rd8(hda, off + 2) & ~0x02U));
                    int t;
                    for (t = 0; t < 200 && (rd32(hda, off) & 0x02); t++) IODelay(50);
                    IOLog("LatSOF: lab steal=1 sd%d now c=0x%06x (%s)\n",
                          i, rd32(hda, off),
                          (rd32(hda, off) & 0x02) ? "STILL RUNNING" : "stopped");
                }
            }
        }
        {
            // patch-50: strictborrow lab override, applied AT the guard rather
            // than in labReload() so nothing earlier in the call chain can undo
            // it. gStrictBorrow otherwise comes ONLY from the boot-arg
            // latsof_strictborrow (line ~923), i.e. an OpenCore config.plist
            // edit plus a reboot. Two clauses gate this borrow —
            // (gStrictBorrow || gHotRecovery) — and startCaptureGated() always
            // sets gHotRecovery = true, so a lab run could NEVER borrow
            // AppleHDA's *programmed* (CBL/BDL live, RUN clear) stream. That is
            // precisely the experiment that answers "can this controller run a
            // stream WE start at all, given AppleHDA brought it up first?" —
            // and it is the only remaining way to separate "our descriptor
            // programming is wrong" from "the engine refuses host-originated
            // RUN on this controller". gProgrammedWaits is also forced past
            // kProgrammedWaitRounds so the `<=` clause cannot re-defer.
            {
                UInt32 sbv = labU32("strictborrow", 0xFFFFFFFF);
                if (sbv == 0) {
                    gStrictBorrow    = false;
                    gHotRecovery     = false;
                    gProgrammedWaits = kProgrammedWaitRounds + 1;
                    IOLog("LatSOF: lab strictborrow=0 — borrow guard forced open\n");
                }
            }
            int st2 = outputSdBusyState(hda);
            if (st2 == 1 || (st2 == 2 && (gStrictBorrow || gHotRecovery) &&
                             gProgrammedWaits <= kProgrammedWaitRounds)) {
                gWakeReinitPending = true;
                setProperty("Status", "deferred: output became busy before borrow");
                IOLog("LatSOF: output became %s between preflight and borrow "
                      "— deferring\n", (st2 == 1) ? "RUNNING" : "programmed");
                goto cleanup;
            }
        }

        // PPCTL: PIE + GPROCEN + decouple the loader streams.
        // LATITUDE FORK: the reference also set (1U << capIdx) here, relying
        // on the member's constructor value — capIdx isn't assigned until
        // ~270 lines later, so this decoupled AppleHDA's SD0 and never SD1.
        // The capture stream now decouples in startCaptureGated().
        // LATITUDE FORK patch-20: SD(sIdx) is AppleHDA's first output
        // engine. Borrow it rather than seize it — snapshot first.
        snap.ppctl   = ppCap   ? rd32(hda, ppCap + PP_PPCTL) : 0;
        snap.spibEn  = spibCap ? rd32(hda, spibCap + 0x04) : 0;
        snap.spibVal = spibCap ? rd32(hda, spibCap + 0x08 + (UInt32)sIdx * 0x08) : 0;
        snap.ctl     = rd32(hda, sd) & 0x00FFFFFF;   // CTL only, never SDSTS
        snap.cbl     = rd32(hda, sd + SD_REG_CBL);
        snap.lvi     = rd16(hda, sd + SD_REG_LVI);
        snap.fmt     = rd16(hda, sd + SD_REG_FMT);
        snap.bdpl    = rd32(hda, sd + SD_REG_BDLPL);
        snap.bdpu    = rd32(hda, sd + SD_REG_BDLPU);
        snap.valid   = true;
        { char b[96];
          snprintf(b, sizeof(b), "sd%d ctl=0x%06x fmt=0x%04x bdl=0x%08x ppctl=0x%08x pp=%u spib=%u",
                   sIdx, snap.ctl, snap.fmt, snap.bdpl, snap.ppctl,
                   ppCap ? 1U : 0U, spibCap ? 1U : 0U);
          setProperty("SD-Borrow", b);
          IOLog("LatSOF: borrow %s\n", b); }

        // patch-42: full forensics on AppleHDA's live stream. Its playback
        // WORKS on this controller, so its configuration is the ground
        // truth: engine activity signals (STS/LPIB/DPIB/FIFOS), global
        // state (GCTL/STATESTS = link wake), and the actual BDL entries
        // (physical-mapped read of AppleHDA's descriptor table).
        {
            UInt32 dpibReg = HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)sIdx;
            IOLog("LatSOF: aLive sd%u lpib=%u dpib=%u sts=0x%02x fifos=0x%04x gctl=0x%08x statests=0x%04x spibVal=0x%08x",
                  sIdx, rd32(hda, sd + 0x04), rd32(hda, dpibReg),
                  rd8(hda, sd + SD_REG_STS), rd16(hda, sd + SD_REG_FIFOS),
                  rd32(hda, HDA_GCTL), rd16(hda, 0x0E), snap.spibVal);
            if (snap.bdpl | snap.bdpu) {
                UInt64 bdlPa = ((UInt64)snap.bdpu << 32) | snap.bdpl;
                // patch-45 fix: SDnBDPL/BDLPU hold a PHYSICAL address, and
                // withAddressRange() expects a VIRTUAL one — so its prepare()
                // failed on every single attempt ("aBdl prepare FAILED"), which
                // is why AppleHDA's descriptor contents were never captured.
                auto *bdlMd = IOMemoryDescriptor::withPhysicalAddress(
                    (IOPhysicalAddress)(bdlPa & ~(UInt64)4095), 4096,
                    kIODirectionOutIn);
                if (bdlMd) {
                    if (bdlMd->prepare() == kIOReturnSuccess) {
                        auto *m = bdlMd->map(kIOMapInhibitCache);
                        if (m) {
                            volatile UInt32 *e = (volatile UInt32 *)m->getVirtualAddress();
                            UInt32 n = (snap.lvi < 7) ? (snap.lvi + 1) : 8;
                            for (UInt32 i = 0; i < n; i++) {
                                UInt32 lo = e[i * 2], hi = e[i * 2 + 1];
                                IOLog("LatSOF: aBdl[%u] addr=0x%08x len=%u ioc=%u",
                                      i, lo, hi & 0xFFFFF, hi >> 31);
                            }
                            m->release();
                        }
                        bdlMd->complete();
                    } else {
                        IOLog("LatSOF: aBdl prepare FAILED (pa=0x%llx)", bdlPa);
                    }
                    bdlMd->release();
                }
            }
        }

        // PPCTL as read-modify-write: the original assigned the whole
        // register and so erased AppleHDA's decouple bits outright.
        // Linux decouples exactly one bit — BIT(host_stream_index). The
        // reference also decouples sIdx+1 (its "link DMA" pairing); the
        // linuxTrigger stages drop that extra bit to match Linux.
        if (ppCap) {
            UInt32 pair = (1U << sIdx) | (1U << (sIdx + 1));
            UInt32 set  = coupled ? 0 : (linuxTrigger ? (1U << sIdx) : pair);
            // patch-56: pie=0 drops BIT(31) so PPCTL can be made byte-equal to
            // AppleHDA's working value 0x40000000 (GPROCEN only) — and to what
            // Linux's hda_dsp_ctrl_ppcap_enable() actually writes.
            UInt32 pieBit = gLabPie ? (1U << 31) : 0;
            wr32(hda, ppCap + PP_PPCTL,
                 (snap.ppctl & ~pair) | pieBit | PP_PPCTL_GPROCEN | set);
            // patch-55: decode PPCTL so the log cannot be misread. Bits:
            //   BIT(31) = PIE, BIT(30) = GPROCEN, BIT(index) = PROCEN(index)
            //            = "decouple host and link DMA, enable DSP features".
            // Linux hda_dsp_stream_hw_params() sets PROCEN(index) UNCONDITIONALLY
            // at the top of the function (hda-stream.c:592) — the
            // SOF_INTEL_PROCEN_FMT_QUIRK only governs a temporary RE-couple
            // around the SDxFMT write, and CNL does not have that quirk, so
            // Linux's STEADY STATE on this chip is DECOUPLED. A run that
            // leaves procen=0 is NOT Linux-faithful no matter what else is set.
            UInt32 nowPp = rd32(hda, ppCap + PP_PPCTL);
            IOLog("LatSOF: PPCTL coupled=%u linuxTrigger=%u sIdx=%u -> 0x%08x "
                  "PIE=%u GPROCEN=%u PROCEN(%u)=%u procen=0x%08x\n",
                  coupled ? 1U : 0U, linuxTrigger ? 1U : 0U, sIdx, nowPp,
                  (nowPp >> 31) & 1U, (nowPp >> 30) & 1U, sIdx,
                  (nowPp >> sIdx) & 1U, nowPp & 0x3FFFFFFFU);
        }

        // Program code loader stream — Linux order, from SOF hda-stream.c
        // hda_dsp_stream_hw_params(), with the authoritative offsets:
        //   clear SD_CTL{RUN|INT_MASK} -> poll RUN==0 -> clear SD_STS
        //   -> stream reset -> BDL addr = 0 -> repeat clear/poll -> clear STS
        //   -> BDL -> tag -> CBL -> FMT -> LVI -> BDL addr -> set int-mask
        //
        // NOTE on the couple/decouple dance: it is gated on
        // SOF_INTEL_PROCEN_FMT_QUIRK, which only APL sets (apl.c). CML
        // (cnl.c/cml.c) does not, so Linux writes SDxFMT while DECOUPLED
        // here — the plain write the reference already used.
        wr8(hda, sd, (UInt8)(rd8(hda, sd) & ~(UInt8)(SD_CTL_RUN | SD_CTL_INT_MASK)));
        for (int t = 0; t < 100 && (rd8(hda, sd) & SD_CTL_RUN); t++) IODelay(10);
        wr8(hda, sd + SD_REG_STS, SD_CTL_INT_MASK);
        streamReset(hda, sd);
        IOLog("LatSOF: srst sd%u fifordy=%u sts=0x%02x fifos=0x%04x\n",
              sIdx, gLastFifoReady ? 1U : 0U,
              rd8(hda, sd + SD_REG_STS), rd16(hda, sd + SD_REG_FIFOS));
        wr32(hda, sd + SD_REG_BDLPL, 0);
        wr32(hda, sd + SD_REG_BDLPU, 0);
        wr8(hda, sd, (UInt8)(rd8(hda, sd) & ~(UInt8)(SD_CTL_RUN | SD_CTL_INT_MASK)));
        for (int t = 0; t < 100 && (rd8(hda, sd) & SD_CTL_RUN); t++) IODelay(10);
        wr8(hda, sd + SD_REG_STS, SD_CTL_INT_MASK);

        wr32(hda, sd + SD_REG_BDLPL, (UInt32)(bdlBus & 0xFFFFFFFF));
        wr32(hda, sd + SD_REG_BDLPU, (UInt32)(bdlBus >> 32));
        // Stream tag lives in SD_CTL bits 20..23 = byte 2's upper nibble.
        // Stage >= 2 also keeps bit 18 (0x04 in byte 2), which AppleHDA's own
        // live descriptor carries (ctl=0x14001e); a plain byte write wipes it,
        // Linux's 32-bit read-modify-write does not.
        wr8(hda, sd + 2, (UInt8)(((sTag & 0xF) << 4) | ((stage >= 2) ? 0x04 : 0x00)));
        wr32(hda, sd + SD_REG_CBL, payloadSize);
        // patch-35 stages 2/3: 0x4031 = 48 kHz / 16-bit / 2ch — the exact
        // format AppleHDA's live streams use on this controller, so the
        // controller cannot call it invalid. 0x40's MULT/DIV fields encode
        // the reserved value 0; if FIFO_READY only comes with valid fields,
        // these stages unstick FIFO setup.
        wr16(hda, sd + SD_REG_FMT,
             gLabFmt ? (UInt16)gLabFmt
                     : ((stage == 2 || stage == 3) ? (UInt16)0x4031 : (UInt16)HDA_CL_STREAM_FMT));
        wr16(hda, sd + SD_REG_LVI, (UInt16)(numBdl - 1));
        wr32(hda, sd + SD_REG_BDLPL, (UInt32)(bdlBus & 0xFFFFFFFF));
        wr32(hda, sd + SD_REG_BDLPU, (UInt32)(bdlBus >> 32));
        if (linuxTrigger)   // Linux sets the interrupt-enable bits before RUN
            wr8(hda, sd, (UInt8)(rd8(hda, sd) | SD_CTL_INT_MASK));

        // patch-35: SPIB only makes sense decoupled. In coupled stages leave
        // the SPIB machinery alone entirely.
        // patch-55: THAT ASSUMPTION IS WRONG FOR LINUX. hda_data_stream_prepare()
        // (hda-stream.c:1305) enables SPIB unconditionally in the non-iccmax
        // branch, and CNL has no PROCEN quirk so it is always coupled — Linux
        // boots the loader COUPLED + SPIB ON. `spibcpl=1` (default) reproduces
        // that; `spibcpl=0` restores the patch-35 gate.
        // NOTE: declared without an initializer on purpose — a `goto` earlier
        // in this function jumps past this point, and C++ only permits jumping
        // over a declaration that has vacuous initialisation.
        bool spibGo;
        spibGo = spibCap && (!coupled || gLabSpibCpl);
        if (spibGo) {
            // Linux: snd_sof_dsp_update_bits(sdev, SPIB_BAR, SPBFCCTL,
            //                                mask, enable << index)
            // i.e. read-modify-write of the bit, NOT a whole-register write.
            UInt32 spi = rd32(hda, spibCap + 0x04);
            if (gLabSpib == 0) spi &= ~(1U << sIdx);          // patch-44 knob
            else if (gLabSpib == 1) spi |= (1U << sIdx);
            else spi |= (1U << sIdx);
            wr32(hda, spibCap + 0x04, spi);
            // Linux: sof_io_write(sdev, hstream->spib_addr, size) where
            // spib_addr = bar[SPIB] + SOF_HDA_SPIB_BASE(0x08)
            //           + SOF_HDA_SPIB_INTERVAL(0x08) * index + SPIB(0x00)
            if (gLabSpib != 0) wr32(hda, spibCap + 0x08 + (UInt32)sIdx * 0x08, payloadSize);
            IOLog("LatSOF: spibcpl=%u coupled=%u sIdx=%u -> SPBFCCTL=0x%08x spib_addr[%u]=%u\n",
                  gLabSpibCpl, coupled ? 1U : 0U, sIdx,
                  rd32(hda, spibCap + 0x04), sIdx,
                  rd32(hda, spibCap + 0x08 + (UInt32)sIdx * 0x08));
        } else if (spibCap) {
            IOLog("LatSOF: spibcpl=0 coupled=%u — SPIB skipped (patch-35 behaviour)\n",
                  coupled ? 1U : 0U);
        }

        // ===================== patch-53: position buffer (DPLBASE) =============
        // Linux hda_dsp_stream_hw_params() ends with:
        //     if (bus->use_posbuf && bus->posbuf.addr &&
        //         !(read(DPLBASE) & SOF_HDA_ADSP_DPLBASE_ENABLE)) {
        //             write(DPUBASE, upper_32_bits(bus->posbuf.addr));
        //             write(DPLBASE, (u32)bus->posbuf.addr | ENABLE);
        //     }
        // Measured state before patch-53: DPLBASE = 0x00200000, bit0 = 0, i.e.
        // the position buffer is DISABLED, while Linux enables it. On a
        // DECOUPLED stream the position buffer is the only host-visible
        // progress channel and the DSP side is its writer, so a disabled
        // position buffer is a genuine behavioural difference at exactly the
        // moment we ask the DSP to consume the code-loader stream.
        //
        // We allocate our own 128-byte posbuf (Linux allocates
        // SOF_HDA_DPIB_ENTRY_SIZE(8) * num_total(16) = 128) rather than just
        // setting the enable bit on whatever address AppleHDA left in the
        // register, so the controller can never be pointed at memory we do
        // not own. DPUBASE carries the upper 32 bits.
        if (gLabDplEn) {
            if (!posDma) posDma = allocDma(128, 0x80);
            if (posDma) {
                UInt32 oldDpl = rd32(hda, 0x70), oldDpu = rd32(hda, 0x74);
                wr32((volatile UInt8 *)posDma->virtAddr, (UInt32)sIdx * 8, 0);
                wr32(hda, 0x74, (UInt32)(posDma->physAddr >> 32));
                wr32(hda, 0x70, (UInt32)(posDma->physAddr & 0xFFFFFFFF) | 0x01U);
                IOLog("LatSOF: dplen=1 DPLBASE 0x%08x->0x%08x DPUBASE 0x%08x->0x%08x posbuf=0x%llx\n",
                      oldDpl, rd32(hda, 0x70), oldDpu, rd32(hda, 0x74), posDma->physAddr);
            } else {
                IOLog("LatSOF: dplen=1 but posbuf alloc FAILED - DPLBASE untouched (0x%08x)\n",
                      rd32(hda, 0x70));
            }
        } else {
            UInt32 d = rd32(hda, 0x70);
            IOLog("LatSOF: dplen=0 DPLBASE left at 0x%08x (bit0=%u)\n", d, d & 1U);
        }

        // ================= patch-51: faithful hda_dsp_pre_fw_run =================
        // Linux gives the DSP a real POWER-ON RESET before every cold FW boot:
        //
        //   hda_dsp_pre_fw_run()                          [hda.c]
        //     /* Power down DSP if left enabled to ensure a clean boot state. */
        //     if (hda_dsp_core_is_enabled(sdev, chip->host_managed_cores_mask)) {
        //             dev_dbg(sdev->dev, "DSP core enabled, power down DSP first\n");
        //             ret = chip->power_down_dsp(sdev);         // = hda_power_down_dsp
        //     }
        //     return hda_dsp_ctrl_clock_power_gating(sdev, false);
        //
        //   hda_power_down_dsp() = hda_dsp_core_reset_power_down(0xF)   [hda-dsp.c]
        //     hda_dsp_core_stall_reset(core_mask)   -> CSTALL |= mask ; CRST |= mask
        //     hda_dsp_core_power_down(core_mask)    -> SPA  &= ~mask ; poll CPA == 0
        //
        //   cl_dsp_init() step 1                          [hda-loader.c]
        //     hda_dsp_core_power_up(clip->host_managed_cores_mask)     // 0xF
        //       SPA |= 0xF ; poll (adspcs & CPA(0xF)) == CPA(0xF) ; CRST &= ~0xF
        //   cl_dsp_init() step 3
        //     hda_dsp_core_run(init_core_mask)          -> CSTALL &= ~1  (core 0)
        //
        // For CNL/CML: cores_num=4, init_core_mask=1, host_managed_cores_mask=0xF.
        //
        // WHY THIS MATTERS: this driver has never power-cycled the DSP. It only
        // ever toggled CRST/CSTALL on whatever state it inherited, so the ROM
        // state machine has never been restarted from a known POR - and on CML
        // laptops the BIOS itself will happily boot cAVS FW for DMIC/SoundWire
        // use, leaving core 0 enabled with the ROM mid-sequence. Measured on
        // entry to this block: adspcs=0x01010e0e, i.e. CRST(0)=0 / CSTALL(0)=0 /
        // SPA(0)=1 / CPA(0)=1 -> core 0 is ENABLED, so Linux would take the
        // power-down branch. Every one of our 50 patches ran the ROM on top of
        // that un-reset state.
        // Knob `prefwrun=1`. Non-destructive to AppleHDA (ADSPCS is BAR4).
        {
            UInt32 por = labU32("prefwrun", 0);
            UInt32 hm  = 0xF;                     // host_managed_cores_mask (CNL/CML)
            UInt32 pre = rd32(dsp, DSP_ADSPCS);
            if (por && !porFirstDone) {           // patch-63: skipped when porfirst ran it
                // ---- hda_dsp_core_reset_power_down(0xF) ----
                if (pre & ADSPCS_CPA(hm)) {
                    UInt32 c = pre;
                    c |= ADSPCS_CRST(hm) | ADSPCS_CSTALL(hm);   // stall + reset
                    c &= ~ADSPCS_SPA(hm);                       // power down
                    wr32(dsp, DSP_ADSPCS, c);
                    poll32(dsp, DSP_ADSPCS, ADSPCS_CPA(hm), 0, 50000);
                }
                // ---- hda_dsp_core_power_up(0xF) ----
                { UInt32 c = rd32(dsp, DSP_ADSPCS);
                  c |= ADSPCS_SPA(hm);
                  wr32(dsp, DSP_ADSPCS, c); }
                bool cpaOk = poll32(dsp, DSP_ADSPCS, ADSPCS_CPA(hm), ADSPCS_CPA(hm), 50000);
                { UInt32 c = rd32(dsp, DSP_ADSPCS);
                  c &= ~ADSPCS_CRST(hm);                        // out of reset
                  wr32(dsp, DSP_ADSPCS, c); }
                poll32(dsp, DSP_ADSPCS, ADSPCS_CRST(hm), 0, 50000);
                IOLog("LatSOF: lab prefwrun=1 POR adspcs 0x%08x -> 0x%08x cpaOk=%u\n",
                      pre, rd32(dsp, DSP_ADSPCS), cpaOk ? 1U : 0U);
            }
        }

        // Set all SSPs to clock consumer/codec provider (CBP_CFP) mode
        // Linux: hda_ssp_set_cbp_cfp() in hda-loader.c — REQUIRED before FW load!
        // SSP base = BAR4 + 0x10000, each SSP = 0x1000, SSC1 offset = 0x4
        // CBP_CFP = BIT(25) | BIT(24) = 0x03000000
        for (int s = 0; s < 3; s++) {  // CNL_SSP_COUNT = 3
            UInt32 ssc1Off = 0x10000 + s * 0x1000 + 0x4;
            wr32(dsp, ssc1Off, rd32(dsp, ssc1Off) | 0x03000000);
        }

        // patch-65: LTRP (BAR0 vendor 0x1048). The Ubuntu diff captured Linux
        // idle LTRP = 0x280800a9 (GB bits[5:0] = 0x29 = 41us, set by BIOS) and
        // SOF's ICL/TGL iccmax boot path forces GB=95us "per HW recommendation
        // during FW boot" (hda-stream.c HDA_LTRP_GB_VALUE_US) — CML boots via
        // sof_cnl_ops so Linux did NOT write it, but the guardband gates the
        // LTR-based L1 exit of the controller, which is the only path the
        // DSP-side HDAS DMA has for its PCIe fetches. If AppleHDA/macOS left
        // a different (or zero) guardband, the DSP master could be stuck in an
        // un-wakeable low-power link state while the ROM waits for data.
        // Knob `ltrp=`: 0 = read-only; 1..255 = write that BYTE at 0x1048
        // (e.g. ltrp=169 -> 0xa9 == Linux idle; ltrp=95 -> GB 31us); larger
        // value = write the whole DWORD (e.g. 0x280800a9).
        {
            UInt32 lv = labU32("ltrp", 0);
            UInt32 old = rd32(hda, 0x1048);
            if (lv) {
                if (lv <= 0xFF) wr8(hda, 0x1048, (UInt8)lv);
                else wr32(hda, 0x1048, lv);
            }
            IOLog("LatSOF: lab ltrp knob=%u 0x1048 0x%08x -> 0x%08x\n",
                  lv, old, rd32(hda, 0x1048));
        }

        // ROM IPC + core run
        //
        // patch-52 `imr=1` — THE IMR (Image Module Recovery) BOOT PATH.
        // cl_dsp_init(sdev, stream_tag, imr_boot=true) [hda-loader.c:86-91]:
        //     ipc_hdr = chip->ipc_req_mask | HDA_DSP_ROM_IPC_CONTROL;
        //     if (!imr_boot)
        //             ipc_hdr |= HDA_DSP_ROM_IPC_PURGE_FW | ((stream_tag - 1) << 9);
        // So on an IMR boot the ROM is told NOT to purge and NOT about any
        // host-DMA channel, and step 7 waits for FSR_STATE_FW_ENTERED instead
        // of FSR_STATE_INIT_DONE (cnl.c: hda_dsp_boot_imr -> cl_init(sdev, 0,
        // true); rom status target FSR_STATE_FW_ENTERED).
        // WHY THIS IS THE RIGHT EXPERIMENT NOW: after 50 host-side patches the
        // controller is provably Linux-identical (patch-52 run set: clean
        // AppleHDA-free board, full hda_dsp_ctrl_init_chip, POR, PCI gating
        // off, exact PPCTL/SPIB/INTCTL/INT_MASK, tag/CBL/LVI/BDL echoed) and
        // the host stream STILL moves zero bytes, while ADSPIS_CL_DMA never
        // latches. So the host->DSP DMA path itself is what is broken on this
        // port. The IMR path never touches it: the ROM restores the base FW
        // from the hardware-protected IMR/LP-SRAM copy that the CSME/BIOS
        // already validated. Our ROM_STATUS is stuck at
        // FSR_WAIT_FOR_DMA_BUFFER_FULL, i.e. exactly the wait the IMR path
        // does not perform. Non-destructive (BAR4 only).
        imrBoot = labU32("imr", 0);
        wr32(dsp, IPC_HIPCIDR, IPC_BUSY | ROM_IPC_CONTROL |
             (imrBoot ? 0 : (ROM_IPC_PURGE_FW | ((sTag - 1) << 9))));
        if (imrBoot)
            IOLog("LatSOF: lab imr=1 — ROM IPC=0x%08x (no PURGE_FW, no stream tag)\n",
                  rd32(dsp, IPC_HIPCIDR));
        {
            UInt32 a = rd32(dsp, DSP_ADSPCS);
            a &= ~ADSPCS_CRST(1); wr32(dsp, DSP_ADSPCS, a);
            poll32(dsp, DSP_ADSPCS, ADSPCS_CRST(1), 0, 50000);
            a = rd32(dsp, DSP_ADSPCS);
            a &= ~ADSPCS_CSTALL(1); wr32(dsp, DSP_ADSPCS, a);
        }

        if (!poll32(dsp, IPC_HIPCIDA, IPC_DONE, IPC_DONE, 500000)) {
            setProperty("Status", "FAILED: ROM IPC timeout"), IOLog("LatSOF: %s\n", "FAILED: ROM IPC timeout"); goto cleanup;
        }
        wr32(dsp, IPC_HIPCIDA, rd32(dsp, IPC_HIPCIDA) | IPC_DONE);

        // Power down cores 1-3, enable IPC interrupts
        { UInt32 a = rd32(dsp, DSP_ADSPCS);
          a |= ADSPCS_CRST(0xE) | ADSPCS_CSTALL(0xE); a &= ~ADSPCS_SPA(0xE);
          wr32(dsp, DSP_ADSPCS, a); }
        wr32(dsp, IPC_HIPCCTL, 0x03);
        wr32(dsp, DSP_ADSPIC, rd32(dsp, DSP_ADSPIC) | 0x01);
        // patch-51: hda.h has a dedicated CL-DMA interrupt source that this
        // driver has never enabled or even read:
        //     #define HDA_DSP_ADSPIC_CL_DMA  BIT(1)
        //     #define HDA_DSP_ADSPIS_CL_DMA  BIT(1)
        // ADSPIS(0x0C)/ADSPIS2(0x14) are the only place the DSP-side CL DMA
        // engine can report a descriptor/FIFO error, and we have never looked.
        if (labU32("cldma", 0)) {
            wr32(dsp, DSP_ADSPIC, rd32(dsp, DSP_ADSPIC) | 0x02);
            IOLog("LatSOF: lab cldma=1 -> ADSPIC=0x%08x\n", rd32(dsp, DSP_ADSPIC));
        }
        IOLog("LatSOF: dspirq adspic=0x%08x adspis=0x%08x adspic2=0x%08x adspis2=0x%08x\n",
              rd32(dsp, 0x08), rd32(dsp, 0x0C), rd32(dsp, 0x10), rd32(dsp, 0x14));

        // Wait INIT_DONE → Start DMA → Wait FW_ENTERED
        //
        // patch-52: for the IMR path step 7's target is FSR_STATE_FW_ENTERED,
        // NOT FSR_STATE_INIT_DONE (hda-loader.c:143-146). The ROM restores the
        // image by itself and never waits for a host DMA buffer, so there is no
        // "start the DMA" phase at all - if it works, FW_ENTERED appears with
        // the code-loader stream untouched.
        imrDone = false;
        if (imrBoot) {
            UInt32 last = 0;
            for (int t = 0; t < 3000; t++) {
                last = rd32(dsp, ROM_STATUS);
                if ((last & 0xFFFFFF) == FSR_STATE_FW_ENTERED) { imrDone = true; break; }
                if (t == 0 || (t % 500) == 499)
                    IOLog("LatSOF: lab imr t=%d rom=0x%08x romerr=0x%08x\n",
                          t + 1, last, rd32(dsp, ROM_STATUS + 0x4));
                IOSleep(1);
            }
            IOLog("LatSOF: lab imr -> rom=0x%08x romerr=0x%08x %s\n",
                  last, rd32(dsp, ROM_STATUS + 0x4),
                  imrDone ? "FW_ENTERED(OK)" : "TIMEOUT");
            if (imrDone) fwLoaded = true;
        } else {
        { bool ok = false;
          for (int t = 0; t < 300; t++) {
              if ((rd32(dsp, ROM_STATUS) & 0xFFFFFF) == FSR_INIT_DONE) { ok = true; break; }
              IOSleep(1);
          }
          if (!ok) { setProperty("Status", "FAILED: INIT_DONE timeout"), IOLog("LatSOF: %s\n", "FAILED: INIT_DONE timeout"); goto cleanup; }
        }
        }
        if (stage == 6) IOSleep(20);   // settle experiment

        // patch-52: everything from here to the end of the ROM poll loop is the
        // code-loader stream phase, which the IMR path does not use. Skip it
        // entirely so nothing disturbs a FW that booted from IMR.
        if (!imrDone) {

        // ---- patch-64: recrst — a SECOND HDA CRST, now that the ROM waits ----
        // (Moved here from the POR site: by this point the stream is fully
        // programmed but RUN is not yet set, and the ROM has confirmed
        // INIT_DONE above — so the reset destroys nothing we cannot rewrite
        // from the registers themselves, right here.) The Ubuntu same-machine
        // diff showed the ROM's own HDA codec (CAD2) answers the reset wake
        // on Linux (codec_mask 0x5) but never for us (0x1): our only CRST
        // fired while AppleHDA's firmware still owned the DSP, long before
        // the ROM came up. Hypothesis: the wake cycle is what attaches the
        // ROM's link DMA to the link, so it must happen while the ROM sits
        // in FSR_WAIT_FOR_DMA_BUFFER_FULL.
        if (labU32("recrst", 0)) {
            UInt32 romv = rd32(dsp, ROM_STATUS);
            int t;
            for (t = 0; t < 400 && (romv & 0x0F000000U) != 0x05000000U; t++) {
                IODelay(5000);
                romv = rd32(dsp, ROM_STATUS);
            }
            IOLog("LatSOF: lab recrst=1 ROM wait-state 0x%08x after %d ms\n",
                  romv, t * 5);
            /* save everything the CRST may clear — stream descriptor included */
            UInt32 sCtl  = rd32(hda, sd) & ~0x03U;
            UInt32 sCbl  = rd32(hda, sd + SD_REG_CBL);
            UInt32 sFmt  = (UInt32)rd16(hda, sd + SD_REG_FMT);
            UInt32 sBdlL = rd32(hda, sd + SD_REG_BDLPL);
            UInt32 sBdlU = rd32(hda, sd + SD_REG_BDLPU);
            UInt32 sLvi  = (UInt32)rd16(hda, sd + SD_REG_LVI);
            UInt32 ppSave  = ppCap   ? rd32(hda, ppCap + PP_PPCTL) : 0;
            UInt32 spibCtl = spibCap ? rd32(hda, spibCap + 0x04) : 0;
            UInt32 spibVal = spibCap ? rd32(hda, spibCap + 0x08 + (UInt32)sIdx * 0x08) : 0;
            UInt32 dplSave = rd32(hda, 0x70);
            UInt32 dpuSave = rd32(hda, 0x74);
            IOLog("LatSOF: lab recrst pre-reset ctl=0x%08x cbl=%u fmt=0x%04x bdll=0x%08x lvi=%u ppctl=0x%08x\n",
                  sCtl, sCbl, sFmt, sBdlL, sLvi, ppSave);
            UInt32 g0r = rd32(hda, HDA_GCTL);
            wr32(hda, HDA_GCTL, g0r & ~1U);
            for (int u = 0; u < 100 && (rd32(hda, HDA_GCTL) & 1); u++) IODelay(500);
            IODelay(1000);
            wr32(hda, HDA_GCTL, rd32(hda, HDA_GCTL) | 1U);
            for (int u = 0; u < 100 && !(rd32(hda, HDA_GCTL) & 1); u++) IODelay(500);
            IODelay(1200);
            wr32(hda, HDA_GCTL, rd32(hda, HDA_GCTL) | (1U << 8));
            { UInt16 sF = 0, sv = 0; int pm = 0;
              sF = sv = (UInt16)(rd16(hda, 0x0E) & 0x7FFFU);
              for (; pm < 1000 && !sv; pm += 25) {
                  IODelay(25000);
                  sv = (UInt16)(rd16(hda, 0x0E) & 0x7FFFU);
              }
              IOLog("LatSOF: lab recrst STATESTS post-reset first=0x%04x poll=0x%04x after %d ms\n",
                    sF, sv, pm); }
            /* rebuild CORB/RIRB — the CRST cleared the bases */
            { static DmaBuf *rb2 = nullptr;
              if (!rb2) rb2 = allocDma(4096, 2048);
              if (rb2) {
                  UInt64 base = rb2->physAddr;
                  wr32(hda, 0x40, (UInt32)(base & 0xFFFFFFFFU));
                  wr32(hda, 0x44, (UInt32)(base >> 32));
                  wr8 (hda, 0x4E, 0x02);
                  wr16(hda, 0x4A, 0);
                  wr16(hda, 0x48, 0x8000);
                  for (int u = 0; u < 100 && (rd16(hda, 0x48) & 0x7FFFU); u++) IODelay(10);
                  wr16(hda, 0x48, 0);
                  wr8 (hda, 0x4C, 0x02);
                  wr32(hda, 0x50, (UInt32)((base + 2048) & 0xFFFFFFFFU));
                  wr32(hda, 0x54, (UInt32)((base + 2048) >> 32));
                  wr8 (hda, 0x5E, 0x02);
                  wr16(hda, 0x58, 0x8000);
                  wr16(hda, 0x5A, 1);
                  wr8 (hda, 0x5C, 0x02 | 0x04);
                  wr8 (hda, 0x5D, 0x05);
                  IOLog("LatSOF: lab recrst CORB@0x%llx RIRB@0x%llx corbctl=0x%02x rirbctl=0x%02x\n",
                        base, base + 2048, rd8(hda, 0x4C), rd8(hda, 0x5C));
              } }
            /* restore what the CRST cleared */
            if (ppCap)   wr32(hda, ppCap + PP_PPCTL, ppSave);
            if (spibCap) { wr32(hda, spibCap + 0x04, spibCtl);
                           wr32(hda, spibCap + 0x08 + (UInt32)sIdx * 0x08, spibVal); }
            wr32(hda, 0x70, dplSave);
            wr32(hda, 0x74, dpuSave);
            wr32(hda, sd + SD_REG_CBL, sCbl);
            wr16(hda, sd + SD_REG_FMT, (UInt16)sFmt);
            wr32(hda, sd + SD_REG_BDLPL, sBdlL);
            wr32(hda, sd + SD_REG_BDLPU, sBdlU);
            wr16(hda, sd + SD_REG_LVI, (UInt16)sLvi);
            wr32(hda, sd, sCtl);
            /* clear stream status residue on every stream */
            for (int i = 0; i < 16; i++)
                wr8(hda, SD_BASE + (UInt32)i * SD_SIZE + SD_REG_STS, SD_CTL_INT_MASK);
            IOLog("LatSOF: lab recrst done ctl=0x%08x cbl=%u ppctl=0x%08x\n",
                  rd32(hda, sd), rd32(hda, sd + SD_REG_CBL),
                  ppCap ? rd32(hda, ppCap + PP_PPCTL) : 0);
        }

        // DMA start. Stage 0 keeps the reference behaviour (RUN only, poll
        // only — patch-18's interrupt-free loader). Stages >= 1 apply Linux's
        // cl_trigger: enable the stream's INTCTL bit and set RUN together with
        // the interrupt-enable bits. This is deliberately regression-prone
        // here: the sweep exists to find out which combination this machine
        // needs, and INTCTL is restored further down either way.
        wr8(hda, sd + SD_REG_STS, SD_CTL_INT_MASK);
        if (gLabNoRun) {
            // patch-44 knob: program the descriptor fully but never start the
            // engine. Isolates "RUN itself wedges the controller" from
            // "the engine cannot fetch".
            IOLog("LatSOF: lab norun=1 — descriptor programmed, RUN withheld\n");
        } else if (linuxTrigger) {
            wr32(hda, HDA_INTCTL, rd32(hda, HDA_INTCTL) | (1U << sIdx));
            wr8(hda, sd, (UInt8)(rd8(hda, sd) | SD_CTL_RUN | SD_CTL_INT_MASK));
        } else {
            wr8(hda, sd, (UInt8)((rd8(hda, sd) & ~(UInt8)SD_CTL_IOCE) | SD_CTL_RUN));
        }
        IODelay(500);

        // patch-65 dpibw=1: the Ubuntu load-window forensics show the ROM's
        // only exit from wait=5 is DPIB reaching CBL (it left wait=5 the exact
        // sample DPIB hit 0x88000). On Linux nobody in the kernel writes DPIB
        // — the DSP-side HDAS DMA advances it. PROBE: if the register is
        // host-writable and the ROM merely polls it, a single CBL write should
        // let the ROM leave wait=5 immediately. If the write reads back 0 the
        // register is RO and the DMA really is dead upstream.
        if (labU32("dpibw", 0) == 1) {
            UInt32 dpr = HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)sIdx;
            UInt32 cblNow = rd32(hda, sd + SD_REG_CBL);
            wr32(hda, dpr, cblNow);
            IOLog("LatSOF: dpibw=1 wrote DPIB=0x%08x readback=0x%08x rom=0x%08x\n",
                  cblNow, rd32(hda, dpr), rd32(dsp, ROM_STATUS));
        }

        // ============== patch-55: descriptor clobber guard ======================
        // Snapshot the five descriptor registers NOW (immediately after RUN has
        // been asserted) and re-check them on every pass of the ROM poll loop
        // below. 54 patches have verified the descriptor exactly once, at t=0,
        // and then assumed it stayed put for the whole wait. If something
        // rewrites it while we poll — AppleHDA's stream-quiesce path is the
        // standing suspect, since the t=0 dump has repeatedly caught `moving`
        // transitions on OTHER streams — the t=0 snapshot cannot see it. And
        // the RUN-keeper would silently mask a RUN-only clobber while leaving
        // the fetch broken, which is exactly the failure shape we have.
        // Masked with ~0x03 so the keeper's own RUN write (bit 1) and SRST
        // (bit 0) are not reported as external damage.
        UInt32 clCtl0  = rd32(hda, sd) & ~0x03U;
        UInt32 clCbl0  = rd32(hda, sd + SD_REG_CBL);
        UInt32 clFmt0  = (UInt32)rd16(hda, sd + SD_REG_FMT);
        UInt32 clBdlL0 = rd32(hda, sd + SD_REG_BDLPL);
        UInt32 clBdlU0 = rd32(hda, sd + SD_REG_BDLPU);
        UInt32 clLvi0  = (UInt32)rd16(hda, sd + SD_REG_LVI);
        UInt32 clHit   = 0;
        if (gLabClobber)
            IOLog("LatSOF: clobber guard armed ctl=0x%08x cbl=%u fmt=0x%04x bdll=0x%08x bdlu=0x%08x lvi=%u\n",
                  clCtl0, clCbl0, clFmt0, clBdlL0, clBdlU0, clLvi0);

        // PORT DEBUG (Lenovo port): is the code-loader DMA moving bytes at
        // all? Full 24-bit SD_CTL (so the tag field is visible), SDxFMT at
        // 0x12, SDxFIFOSIZE at 0x10, LPIB (link position) and DPIB (the
        // vendor register at 0x1084+0x20*idx, which is what Linux reads for
        // playback positions in decoupled mode). Both counters read 0 both
        // when the DMA never started *and* when it started but the DSP-side
        // link never consumes — the STS/ROM columns disambiguate.
        {
            UInt32 dpibReg = HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)sIdx;
            // patch-37: bus vs physical addresses — if IOMMU mapping is real
            // the two differ; identical values mean gen64IOVMSegments fell
            // back to raw physical (mapping silently not applied).
            UInt64 bdlPhysCmp = bdlDma->md->getPhysicalAddress();
            // patch-41: read back what the CONTROLLER actually has. If
            // BDBAR/STRM/CBL/LVI/FMT do not echo what we wrote, the engine
            // has no descriptor table and RUN is a no-op — that would end
            // the "why doesn't it fetch the BDL" mystery in one log line.
            UInt64 bdbarEcho = (UInt64)rd32(hda, sd + SD_REG_BDLPU) << 32
                             | rd32(hda, sd + SD_REG_BDLPL);
            UInt32 ctlEcho = rd32(hda, sd) & 0x00FFFFFF;
            UInt32 strmEcho = (ctlEcho >> 4) & 0xFFFF;
            // patch-42: firmware buffer real physical address vs bus — with
            // the device inside AppleVTD's translation domain these MUST
            // differ; if they're equal the mapping is identity and any
            // address-constraint theory dies here.
            UInt64 fwPhysCmp = fwDma->md->getPhysicalAddress();
            // patch-43: GCTL before/after RUN plus DMI link status. If the
            // run write also flips a DSP-wake bit we can see it; if GCTL is
            // unchanged the engine is gated upstream of the stream descriptor.
            UInt32 gctlNow = rd32(hda, HDA_GCTL);
            char t[1100];
            snprintf(t, sizeof(t),
                     "stage=%u tag=%u t=0 ctl=0x%06x strm=0x%04x fmt=0x%04x fifos=0x%04x sts=0x%02x "
                     "lpib=%u dpib=%u rom=0x%06x romerr=0x%08x ppctl=0x%08x spibctl=0x%08x spibval=0x%08x intctl=0x%08x "
                     "bdlBus=0x%llx bdlPhys=0x%llx bdbarEcho=0x%llx cbl=%u lvi=%u fwBus=0x%llx fwPhys=0x%llx "
                     "gctl=0x%08x dspur=%u adspcs=0x%08x fifow=0x%04x fifol=0x%04x "
                     "ssync=0x%08x gsts=0x%08x dpl=0x%08x dpu=0x%08x osp=%u isp=%u "
                     "ltrp=0x%08x d0i3c=0x%02x pci44=0x%08x pci48=0x%08x pci4c=0x%08x",
                     stage, sTag, ctlEcho, strmEcho, rd16(hda, sd + SD_REG_FMT),
                     rd16(hda, sd + SD_REG_FIFOS), rd8(hda, sd + SD_REG_STS),
                     rd32(hda, sd + 0x04), rd32(hda, dpibReg),
                     rd32(dsp, ROM_STATUS) & 0xFFFFFF, rd32(dsp, ROM_STATUS + 0x4),
                     ppCap ? rd32(hda, ppCap + PP_PPCTL) : 0,
                     spibCap ? rd32(hda, spibCap + 0x04) : 0,
                     spibCap ? rd32(hda, spibCap + 0x08 + (UInt32)sIdx * 0x08) : 0,
                     rd32(hda, HDA_INTCTL),
                     bdlBus, bdlPhysCmp, bdbarEcho,
                     rd32(hda, sd + SD_REG_CBL), rd16(hda, sd + SD_REG_LVI),
                     fwDma->physAddr, fwPhysCmp,
                     gctlNow, (gctlNow >> 8) & 1U, rd32(dsp, DSP_ADSPCS),
                     rd16(hda, sd + 0x0E), rd16(hda, sd + 0x14),
                     rd32(hda, 0x38), rd32(hda, 0x10),
                     rd32(hda, 0x70), rd32(hda, 0x74),
                     (unsigned)rd16(hda, 0x18), (unsigned)rd16(hda, 0x1A),
                     rd32(hda, 0x1048), rd8(hda, 0x104C),
                     pciDevice ? pciDevice->configRead32(0x44) : 0,
                     pciDevice ? pciDevice->configRead32(0x48) : 0,
                     pciDevice ? pciDevice->configRead32(0x4C) : 0);
            setProperty("FW-Load-Debug", t);
            IOLog("LatSOF: cl %s\n", t);
            // patch-54 (mbox=1): the DSP->host half of the IPC plus the SRAM
            // mailbox. HIPCTDR/HIPCTDA/HIPCTDD are CNL_DSP_IPC_BASE(0xc0) +
            // 0x00/0x04/0x08; HDA_DSP_MBOX_UPLINK_OFFSET is 0x81000 (hda.h:177)
            // - NOT the 0x80000 window we have always dumped, which only holds
            // ROM status/error/trace.
            if (gLabMBox) {
                IOLog("LatSOF: hip t0 ct dr=0x%08x da=0x%08x dd=0x%08x | ci dr=0x%08x da=0x%08x dd=0x%08x ctl=0x%08x\n",
                      rd32(dsp, 0xC0), rd32(dsp, 0xC4), rd32(dsp, 0xC8),
                      rd32(dsp, 0xD0), rd32(dsp, 0xD4), rd32(dsp, 0xD8),
                      rd32(dsp, 0xE8));
                IOLog("LatSOF: intsts=0x%08x intctl=0x%08x gsts=0x%08x\n",
                      rd32(hda, 0x24), rd32(hda, HDA_INTCTL), rd32(hda, 0x10));
                for (UInt32 row = 0; row < 0x40; row += 0x20) {
                    char mb[200];
                    int mn = snprintf(mb, sizeof(mb), "mbox[0x%05x]", 0x81000 + row);
                    for (UInt32 k = 0; k < 0x20 && mn < 180; k += 4)
                        mn += snprintf(mb + mn, sizeof(mb) - mn, " %08x",
                                       rd32(dsp, 0x81000 + row + k));
                    IOLog("LatSOF: %s\n", mb);
                }
            }
        }

        { UInt32 romSt = 0, romRaw = 0, romErr = 0;
          UInt32 keeperHits = 0;
          bool keeper = (stage >= 4 || gLabHold);  // patch-44: hold= knob
          // patch-46 SAFETY: this loop runs on the audio workloop with the
          // command gate held, so every millisecond spent here is a
          // millisecond the entire audio stack queues behind us — and those
          // queued threads are what showed up as a load average of 11.8 on a
          // 79%-idle machine (i.e. stuck in D state, not burning CPU).
          // The old 8000 / 3000 default was a 3-8 SECOND machine-wide stall
          // on a tick that fires every 1.5 s. It also bought nothing:
          // hold=8000 was measured re-asserting RUN 8000 times and moving
          // `rom` by exactly zero. Default 200 ms, `tmax=` may raise it, the
          // hard cap below is not negotiable.
          int tMax = 200;
          if (gLabHold) tMax = (int)gLabHold;
          { UInt32 tv = labU32("tmax", 0xFFFFFFFF);
            if (tv != 0xFFFFFFFF) tMax = (int)tv; }
          if (tMax > LAB_TMAX_CAP) tMax = LAB_TMAX_CAP;
          if (tMax < 1) tMax = 1;
          // patch-65 dpibw=2: progressively advance DPIB ourselves (mimicking
          // the Linux load window where DPIB swept 0 -> CBL at ~67MB/s while
          // rom cycled 05000003<->00000003). If the ROM tracks DPIB directly
          // the state machine will walk 0x1 -> 0x2 -> 0x3 -> 0x5 under our
          // fake positions; if rom never moves, DPIB is a DSP-owned RO reg
          // and the fetch path is dead upstream of the host.
          UInt32 dpibw = labU32("dpibw", 0);
          UInt32 dpibwHits = 0;
          for (int t = 0; t < tMax; t++) {
              romRaw = rd32(dsp, ROM_STATUS);
              romSt = romRaw & 0xFFFFFF;        // patch-35: state code = low 24
              romErr = rd32(dsp, ROM_STATUS + 0x4);
              if (dpibw == 2) {
                  UInt32 dpr2 = HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)sIdx;
                  UInt32 cur = rd32(hda, dpr2);
                  UInt32 cblv = rd32(hda, sd + SD_REG_CBL);
                  if (cur < cblv) {
                      UInt32 nx = cur + 0x4000;
                      if (nx > cblv) nx = cblv;
                      wr32(hda, dpr2, nx);
                      dpibwHits++;
                      if (dpibwHits <= 6)
                          IOLog("LatSOF: dpibw=2 dpiB 0x%08x -> 0x%08x rb=0x%08x rom=0x%08x\n",
                                cur, nx, rd32(hda, dpr2), romRaw);
                  }
              }
              // patch-52 l1poke: keep EM2.L1SEN clear for the WHOLE window.
              // Counting re-arms answers "is somebody else re-enabling DMI L1
              // under us" - the only way this machine differs from a Linux box
              // where SOF owns the controller outright.
              if (l1Poke) {
                  UInt32 e2 = rd32(hda, HDA_VS_EM2);
                  if (e2 & 0x2000U) { wr32(hda, HDA_VS_EM2, e2 & ~0x2000U); l1Pokes++; }
              }
              if (romSt == FSR_STATE_FW_ENTERED) { fwLoaded = true; break; }
              // patch-55 clobber guard: compare against the post-RUN snapshot
              // BEFORE the keeper runs, so what we observe is the raw external
              // state and not our own repair.
              if (gLabClobber) {
                  UInt32 c1 = rd32(hda, sd) & ~0x03U;
                  UInt32 c2 = rd32(hda, sd + SD_REG_CBL);
                  UInt32 c3 = (UInt32)rd16(hda, sd + SD_REG_FMT);
                  UInt32 c4 = rd32(hda, sd + SD_REG_BDLPL);
                  UInt32 c5 = rd32(hda, sd + SD_REG_BDLPU);
                  UInt32 c6 = (UInt32)rd16(hda, sd + SD_REG_LVI);
                  UInt32 bits = 0;
                  if (c1 != clCtl0)  bits |= 0x01;
                  if (c2 != clCbl0)  bits |= 0x02;
                  if (c3 != clFmt0)  bits |= 0x04;
                  if (c4 != clBdlL0) bits |= 0x08;
                  if (c5 != clBdlU0) bits |= 0x10;
                  if (c6 != clLvi0)  bits |= 0x20;
                  if (bits) {
                      clHit++;
                      if (clHit <= 3)
                          IOLog("LatSOF: CLOBBER t=%d bits=0x%02x | ctl 0x%08x->0x%08x cbl %u->%u "
                                "fmt 0x%04x->0x%04x bdll 0x%08x->0x%08x bdlu 0x%08x->0x%08x lvi %u->%u\n",
                                t, bits, clCtl0, c1, clCbl0, c2, clFmt0, c3,
                                clBdlL0, c4, clBdlU0, c5, clLvi0, c6);
                  }
              }
              // patch-33 RUN-keeper: something external (AppleHDA's stream
              // quiesce tick, most likely) clears RUN while we only poll.
              // Re-assert immediately and count the hits — the count and the
              // rom advance together are the smoking gun.
              if (keeper && !(rd8(hda, sd) & SD_CTL_RUN)) {
                  wr8(hda, sd, (UInt8)(rd8(hda, sd) | SD_CTL_RUN | SD_CTL_INT_MASK));
                  keeperHits++;
              }
              if ((t % 500) == 499) {
                  UInt32 dpibReg = HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)sIdx;
                  char t2[256];
                  snprintf(t2, sizeof(t2),
                           "t=%d ctl=0x%06x sts=0x%02x fifos=0x%04x lpib=%u dpib=%u "
                           "cbl=%u lvi=%u rom=0x%08x wait=0x%x romerr=0x%08x keeper=%u "
                           "adspis=0x%08x adspis2=0x%08x em2=0x%08x l1N=%u",
                           t + 1, rd32(hda, sd) & 0x00FFFFFF, rd8(hda, sd + SD_REG_STS),
                           rd16(hda, sd + SD_REG_FIFOS), rd32(hda, sd + 0x04),
                           rd32(hda, dpibReg), rd32(hda, sd + SD_REG_CBL),
                           rd16(hda, sd + SD_REG_LVI),
                           romRaw, (romRaw >> 24) & 0xF, romErr, keeperHits,
                           rd32(dsp, 0x0C), rd32(dsp, 0x14), rd32(hda, HDA_VS_EM2),
                           l1Pokes);
                  setProperty("FW-Load-Debug", t2);
                  IOLog("LatSOF: cl %s\n", t2);
              }
              IOSleep(1);
          }
          if (gLabMBox) {
              IOLog("LatSOF: hip end ct dr=0x%08x da=0x%08x dd=0x%08x | ci dr=0x%08x da=0x%08x dd=0x%08x ctl=0x%08x\n",
                    rd32(dsp, 0xC0), rd32(dsp, 0xC4), rd32(dsp, 0xC8),
                    rd32(dsp, 0xD0), rd32(dsp, 0xD4), rd32(dsp, 0xD8),
                    rd32(dsp, 0xE8));
              IOLog("LatSOF: intsts=0x%08x gsts=0x%08x gctl=0x%08x\n",
                    rd32(hda, 0x24), rd32(hda, 0x10), rd32(hda, HDA_GCTL));
          }
          char f[240];
          snprintf(f, sizeof(f), "%s rom=0x%08x romerr=0x%08x keeper=%u clobber=%u lpib=%u dpibw=%u",
                   fwLoaded ? "OK" : "TIMEOUT", romRaw, romErr, keeperHits, clHit,
                   rd32(hda, sd + 0x04), dpibwHits);
          setProperty("FW-Entered", f);
          IOLog("LatSOF: cl %s\n", f);
        }
        }   // patch-52: end of the code-loader stream phase (skipped on imr)

        // ============ patch-52: DSP SRAM / ROM MBOX forensics (sram=1) ============
        // Everything the host touches is now provably Linux-identical while the
        // ROM sits in FSR_WAIT_FOR_DMA_BUFFER_FULL forever, so the only source
        // of truth left is the ROM's OWN memory. Upstream addresses:
        //     SRAM_WINDOW_OFFSET(x) = 0x80000 + x * 0x20000      [hda.h:162]
        //     HDA_DSP_MBOX_OFFSET   = SRAM_WINDOW_OFFSET(0)      [hda.h:164]
        //     HDA_DSP_SRAM_REG_ROM_STATUS = MBOX + 0x0  (0x80000)  <- we read this
        //     HDA_DSP_SRAM_REG_ROM_ERROR  = MBOX + 0x4  (0x80004)  <- romerr
        //     HDA_DSP_SRAM_REG_FW_STATUS  = MBOX + 0x4
        //     HDA_DSP_SRAM_REG_FW_TRACEP  = MBOX + 0x8
        //     HDA_DSP_SRAM_REG_FW_END     = MBOX + 0xc
        // Dump three windows so a ROM that is stuck can tell us itself what it
        // is waiting for (its variables live in this window, not in a register).
        if (labU32("sram", 0)) {
            static const UInt32 wins[3] = { SRAM_WIN(0), SRAM_WIN(1), SRAM_WIN(2) };
            for (int w = 0; w < 3; w++) {
                for (UInt32 row = 0; row < 0x80; row += 0x20) {
                    char b[200];
                    int n = snprintf(b, sizeof(b), "sram[0x%05x]", wins[w] + row);
                    for (UInt32 k = 0; k < 0x20 && n < 180; k += 4)
                        n += snprintf(b + n, sizeof(b) - n, " %08x",
                                      rd32(dsp, wins[w] + row + k));
                    IOLog("LatSOF: %s\n", b);
                }
            }
            IOLog("LatSOF: sram extra ltrp=0x%08x d0i3c=0x%08x adspcs=0x%08x adspis=0x%08x adspis2=0x%08x em2=0x%08x\n",
                  rd32(hda, 0x1048), rd32(hda, 0x104A), rd32(dsp, DSP_ADSPCS),
                  rd32(dsp, 0x0C), rd32(dsp, 0x14), rd32(hda, HDA_VS_EM2));
        }
        // patch-60 (sramscan=1): sweep the whole 32 KB of SRAM window 0 and
        // log every 32-byte row that contains something other than 0x00000000
        // or 0xFFFFFFFF. The cAVS ROM keeps its state variables — and possibly
        // its own log buffer with ASCII traces — in this window; a stuck ROM
        // that cannot tell the host via registers may still be telling us via
        // memory. Read-only, no side effects.
        if (labU32("sramscan", 0)) {
            int logged = 0;
            for (UInt32 a = 0x80000; a < 0x88000 && logged < 64; a += 0x20) {
                UInt32 w0 = rd32(dsp, a);
                if (w0 == 0 || w0 == 0xFFFFFFFFU) {
                    // cheap reject: check the row's first word only if the
                    // rest also look flat — full check below only on interest
                    UInt32 w1 = rd32(dsp, a + 4), w2 = rd32(dsp, a + 8);
                    if (w1 == w0 && w2 == w0) continue;
                }
                char b[200];
                int n = snprintf(b, sizeof(b), "sramscan[0x%05x]", a);
                for (UInt32 k = 0; k < 0x20 && n < 180; k += 4)
                    n += snprintf(b + n, sizeof(b) - n, " %08x", rd32(dsp, a + k));
                IOLog("LatSOF: %s\n", b);
                logged++;
            }
            IOLog("LatSOF: sramscan done rows=%d\n", logged);
        }

        // patch-48 GROUND TRUTH (after): the same census, taken at the end of
        // our window. Diffing it against HDA-Moving-Pre answers the question
        // the whole `lpib == 0` reading rests on: do ANY streams on this
        // controller advance their position while we are running? If
        // AppleHDA's do and ours does not, the engine is not the problem —
        // the routing to the DSP is.
        {
            // patch-56: same full descriptor as moving-pre (see the note there).
            char buf[1024]; int n = 0; buf[0] = 0;
            for (int i = 0; i < 16 && n < 900; i++) {
                UInt32 off = SD_BASE + (UInt32)i * SD_SIZE;
                UInt32 c = rd32(hda, off) & 0x00FFFFFF;
                UInt32 lp = rd32(hda, off + 0x04);
                if (!(c & SD_CTL_RUN) && !lp) continue;
                UInt32 dpibReg = HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)i;
                n += snprintf(buf + n, sizeof(buf) - n,
                              "sd%d[run=%u c=0x%06x t%u f=0x%04x fo=0x%04x cbl=%u sts=0x%02x lpib=%u dpib=%u] ",
                              i, (c >> 1) & 1U, c, (c >> 20) & 0xF,
                              rd16(hda, off + SD_REG_FMT),
                              rd16(hda, off + SD_REG_FIFOS),
                              rd32(hda, off + SD_REG_CBL),
                              rd8(hda, off + SD_REG_STS),
                              lp, rd32(hda, dpibReg));
            }
            if (n == 0) snprintf(buf, sizeof(buf), "no stream running");
            setProperty("HDA-Moving-Post", buf);
            IOLog("LatSOF: moving-post %s\n", buf);
        }

        // Stop code loader DMA and clear any latched status. This now clears
        // the whole interrupt-mask field because the sweep stages set it, and
        // it puts back the only INTCTL bit we may have touched — that is what
        // patch-21 was protecting: patch-18 never set bit sIdx, so an
        // unconditional INTCTL clear there switched off AppleHDA's own enable
        // and silenced playback after every wake-time firmware load.
        wr8(hda, sd, (UInt8)(rd8(hda, sd) & ~(UInt8)(SD_CTL_RUN | SD_CTL_INT_MASK)));
        wr8(hda, sd + SD_REG_STS, SD_CTL_INT_MASK);
        if (linuxTrigger)
            wr32(hda, HDA_INTCTL, rd32(hda, HDA_INTCTL) & ~(1U << sIdx));

        // LATITUDE FORK patch-20: give the stream back. This runs before
        // the fwLoaded check on purpose — a FAILED load used to leave
        // AppleHDA's playback engine pointing at our firmware buffer with
        // its format and BDL wiped, which is what killed the speakers.
        sdRestore(hda, sd, (UInt32)sIdx, ppCap, spibCap, snap);
        { char b[64];
          snprintf(b, sizeof(b), "restored ctl=0x%06x ppctl=0x%08x",
                   rd32(hda, sd) & 0x00FFFFFF, ppCap ? rd32(hda, ppCap + PP_PPCTL) : 0);
          setProperty("SD-Return", b); }

        if (!fwLoaded) { setProperty("Status", "FAILED: FW load"), IOLog("LatSOF: %s\n", "FAILED: FW load"); goto cleanup; }

        // ==================== FW_READY HANDLING ====================

        { bool ready = false;
          for (int t = 0; t < 1000; t++) {
              if (rd32(dsp, IPC_HIPCTDR) & IPC_BUSY) { ready = true; break; }
              IOSleep(1);
          }
          setProperty("FW-Ready", ready ? "OK" : "TIMEOUT");
        }

        if (dspLen >= MBOX_UPLINK + 256) {
            volatile UInt8 *mb = dsp + MBOX_UPLINK;
            UInt32 hSz = rd32(mb, 0x00);
            UInt32 hCmd = rd32(mb, 0x04);

            if (hCmd == SOF_IPC_FW_READY && hSz >= 60 && hSz <= 200) {
                // Parse version
                char ver[80];
                snprintf(ver, sizeof(ver), "%d.%d.%d-%d ABI:%d.%d.%d",
                         rd16(mb, 0x1C), rd16(mb, 0x1E), rd16(mb, 0x20), rd16(mb, 0x22),
                         (rd32(mb, 0x40) >> 24) & 0xFF, (rd32(mb, 0x40) >> 12) & 0xFFF,
                         rd32(mb, 0x40) & 0xFF);
                setProperty("FW-Version", ver);
                // LATITUDE FORK: the fw_ready struct carries the authoritative
                // mailbox offsets/sizes at 0x08..0x14; the reference ignores
                // them. Report them so we can verify the hardcoded fallbacks.
                {
                    char mbx[128];
                    snprintf(mbx, sizeof(mbx),
                             "dspbox=0x%x hostbox=0x%x dspsz=%u hostsz=%u hdrsz=%u",
                             rd32(mb, 0x08), rd32(mb, 0x0C),
                             rd32(mb, 0x10), rd32(mb, 0x14), hSz);
                    setProperty("FW-Mailbox", mbx);

                    // tag is a 6-byte string at 0x3A; Linux prints this as the
                    // trailing field of its version line (expect "57864")
                    char tag[8];
                    for (int i = 0; i < 6; i++) tag[i] = (char)*(volatile UInt8 *)(mb + 0x3A + i);
                    tag[6] = 0; tag[7] = 0;
                    setProperty("FW-Tag", tag);

                    char raw[48];
                    snprintf(raw, sizeof(raw), "abi_raw=0x%08x build=%u",
                             rd32(mb, 0x40), rd16(mb, 0x22));
                    setProperty("FW-Raw", raw);
                }

                // Parse ext_data for mailbox offsets
                UInt32 inboxOff = 0, outboxOff = 0;
                UInt32 extOff = MBOX_UPLINK + hSz;
                for (int i = 0; i < 20; i++) {
                    if (extOff + 12 > dspLen) break;
                    UInt32 eSize = rd32(dsp, extOff), eCmd = rd32(dsp, extOff + 4);
                    UInt32 eType = rd32(dsp, extOff + 8);
                    if (eCmd != SOF_IPC_FW_READY || eSize < 12 || eSize > 4096) break;
                    if (eType == SOF_IPC_EXT_WINDOW) {
                        UInt32 nw = rd32(dsp, extOff + 12);
                        for (UInt32 w = 0; w < nw && w < 8; w++) {
                            UInt32 wb = extOff + 16 + w * 24;
                            if (wb + 24 > dspLen) break;
                            UInt32 wType = rd32(dsp, wb + 4), wId = rd32(dsp, wb + 8);
                            UInt32 wOff = rd32(dsp, wb + 20);
                            if (wType == SOF_IPC_REGION_UPBOX) inboxOff = SRAM_WIN(wId) + wOff;
                            if (wType == SOF_IPC_REGION_DOWNBOX) outboxOff = SRAM_WIN(wId) + wOff;
                        }
                    }
                    extOff += eSize;
                }
                if (!inboxOff) inboxOff = 0x81000;
                // Default outbox 0x80000 is ROM status area — WRONG!
                // Linux SRAM dump shows IPC messages at 0x82000
                // CML SOF mailbox: inbox=SRAM_WIN0+0x1000, outbox=SRAM_WIN0+0x2000
                if (!outboxOff) outboxOff = 0x82000;
                {
                    char ch[96];
                    snprintf(ch, sizeof(ch), "inbox=0x%x outbox=0x%x (%s)",
                             inboxOff, outboxOff,
                             (inboxOff == 0x81000 && outboxOff == 0x82000)
                                 ? "hardcoded fallback" : "from ext_data");
                    setProperty("FW-MailboxUsed", ch);
                }

                // ACK FW_READY
                wr32(dsp, IPC_HIPCTDR, rd32(dsp, IPC_HIPCTDR) | IPC_BUSY);
                wr32(dsp, IPC_HIPCTDA, IPC_DONE);

                // ==================== IPC HELPER ====================
                auto sendIpc = [&](const void *msg, UInt32 msgSize) -> UInt32 {
                    volatile UInt8 *ob = dsp + outboxOff;
                    UInt8 *src = (UInt8 *)msg;
                    for (UInt32 i = 0; i < msgSize; i += 4)
                        wr32(ob, i, *(UInt32 *)(src + i));
                    wr32(dsp, IPC_HIPCIDR, IPC_BUSY);
                    if (!poll32(dsp, IPC_HIPCIDA, IPC_DONE, IPC_DONE, 500000))
                        return 0xFFFFFFFF;
                    UInt32 replyErr = rd32(ob, 8);
                    wr32(dsp, IPC_HIPCIDA, rd32(dsp, IPC_HIPCIDA) | IPC_DONE);
                    wr32(dsp, IPC_HIPCCTL, rd32(dsp, IPC_HIPCCTL) | 0x02);
                    return (replyErr != 0) ? replyErr : 0;
                };

                // LATITUDE FORK patch-24: loader cleanup DELETED. It ran after
                // sdRestore had already handed AppleHDA's SD7 back and after
                // SD-Return was written, so it reset the stream a second time
                // and zeroed every stream's SPIB enable — which is why the
                // speakers died on wakes where the telemetry read "restored".
                // The loader-stop path plus sdRestore already leave the
                // descriptor and SPIB exactly as AppleHDA had them; per the
                // borrowed-stream contract nothing may write SD7 past here.

                // LATITUDE FORK: diagnose ext_data, then prove the downlink mailbox.
                {
                    UInt32 eo = MBOX_UPLINK + hSz;
                    char dmp[96];
                    snprintf(dmp, sizeof(dmp), "@0x%x: %08x %08x %08x %08x",
                             eo, rd32(dsp, eo), rd32(dsp, eo + 4),
                             rd32(dsp, eo + 8), rd32(dsp, eo + 12));
                    setProperty("FW-ExtData", dmp);

                    // Unknown global command. A well-behaved DSP rejects it,
                    // but still rings HIPCIDA — which is what we are testing.
                    struct { UInt32 size; UInt32 cmd; } probeMsg = { 8, 0xF0000000 };
                    UInt32 r = sendIpc(&probeMsg, 8);
                    char res[96];
                    snprintf(res, sizeof(res), "%s ret=0x%x outbox=0x%x",
                             (r == 0xFFFFFFFF) ? "TIMEOUT (downlink wrong?)"
                                               : "ROUND-TRIP OK",
                             r, outboxOff);
                    setProperty("IPC-Test", res);
                }
                // LATITUDE FORK: PIPELINE 1: SSP0 Headphone removed (no I2S codecs)
                // LATITUDE FORK: PIPELINE 7: SSP1 Speaker removed (no I2S codecs)


                // LATITUDE FORK: IPC counters — declared in the removed PIPELINE 1 block.
                int ipcOk = 0, ipcFail = 0;
                auto countIpc = [&](UInt32 r) { if (r == 0) ipcOk++; else ipcFail++; };

                // ==================== PIPELINE 3: DMIC Capture ====================
                { struct { UInt32 size, cmd, comp_id, pipeline_id, sched_id, core,
                           period, priority, period_mips, frames_per_sched,
                           xrun_limit_usecs, time_domain; } __attribute__((packed)) m = {
                      48, 0x30100000, PIPE3_SCHED_ID, PIPE3_ID, PIPE3_HOST_ID, 0,
                      1000, 0, 5000, 0, 0, TIME_TIMER };
                  countIpc(sendIpc(&m, sizeof(m))); }
                { struct { UInt32 size, cmd, id, type, pipeline_id, core, ext,
                           buf_size, caps, flags, reserved; } __attribute__((packed)) m = {
                      44, 0x30200000, PIPE3_BUF0_ID, COMP_BUFFER, PIPE3_ID, 0, 0,
                      9600, 0x71, 0, 0 };
                  countIpc(sendIpc(&m, sizeof(m))); }
                { UInt32 m[76/4] = {}; m[0]=76; m[1]=0x30010000; m[2]=PIPE3_HOST_ID;
                  m[3]=COMP_HOST; m[4]=PIPE3_ID; m[7]=36; m[9]=2; m[10]=2;
                  m[12]=FRAME_S32; m[16]=DIR_CAPTURE;
                  countIpc(sendIpc(m, 76)); }
                { UInt32 m[80/4] = {}; m[0]=80; m[1]=0x30010000; m[2]=PIPE3_DAI_ID;
                  m[3]=COMP_DAI; m[4]=PIPE3_ID; m[7]=36; m[10]=2;
                  m[12]=FRAME_S32; m[16]=DIR_CAPTURE;
                  m[17]=0; m[18]=DAI_DMIC;
                  countIpc(sendIpc(m, 80)); }
                // DAI_CONFIG for DMIC0 — match Linux kprobe [42] exactly
                { UInt8 m[216] = {};
                  *(UInt32*)(m+0)=216; *(UInt32*)(m+4)=0x80010000;
                  *(UInt32*)(m+8)=DAI_DMIC; *(UInt32*)(m+12)=0;
                  *(UInt16*)(m+16)=6; // SOF_DAI_FMT_PDM
                  UInt8 *d=m+52;
                  // Match Linux: hdr.size=0, pdmclk_min=2400000
                  *(UInt32*)(d+0)=0;         // hdr.size (Linux=0, was 164)
                  *(UInt32*)(d+4)=1;         // driver_ipc_version
                  *(UInt32*)(d+8)=2400000;   // pdmclk_min (Linux=2400000, was 500000)
                  *(UInt32*)(d+12)=4800000;  // pdmclk_max
                  *(UInt32*)(d+16)=48000;    // fifo_fs
                  *(UInt16*)(d+24)=32; *(UInt16*)(d+26)=32; // fifo_bits, fifo_bits_b
                  *(UInt16*)(d+28)=40; *(UInt16*)(d+30)=60; // duty_min, duty_max
                  *(UInt32*)(d+32)=1;        // LATITUDE FORK: 1 PDM controller (2 mics)
                  *(UInt32*)(d+44)=400;      // wake_up_time (Linux data[24]=0x190)
                  // PDM0 at DMIC+72 (DAI_CONFIG offset 124)
                  UInt8 *p0=m+124;
                  *(UInt16*)(p0+0)=0; *(UInt16*)(p0+2)=1; *(UInt16*)(p0+4)=1; // id=0, mic_a=1, mic_b=1
                  // LATITUDE FORK: PDM1 removed — this board has one PDM controller
                  countIpc(sendIpc(m, 216)); }
                { struct { UInt32 s,c,src,dst; } __attribute__((packed)) conn[] = {
                      {16, 0x30030000, PIPE3_DAI_ID, PIPE3_BUF0_ID},
                      {16, 0x30030000, PIPE3_BUF0_ID, PIPE3_HOST_ID} };
                  for (int c = 0; c < 2; c++) countIpc(sendIpc(&conn[c], 16)); }
                { struct { UInt32 s,c,id; } __attribute__((packed)) m = {12, 0x30130000, PIPE3_SCHED_ID};
                  countIpc(sendIpc(&m, 12)); }

                { char r[32]; snprintf(r, sizeof(r), "%d OK, %d FAIL", ipcOk, ipcFail);
                  setProperty("Topology", r); }


                // PM_CTX_RESTORE — Linux sends this between PIPE_COMPLETE and PCM_PARAMS
                // Clears pm_prepare_D3 flag in firmware (required for IPC processing)
                { UInt32 pm[3] = {12, 0x40020000, 0};
                  UInt32 pmr = sendIpc(pm, 12);
                  char pmstr[32]; snprintf(pmstr, sizeof(pmstr), "PM_RESTORE=%u", pmr);
                  setProperty("PM", pmstr);
                }

                // LATITUDE FORK patch-27b: a firmware that reaches FW_READY
                // but acknowledges ZERO pipeline IPCs has a dead runtime
                // mailbox — capture can never work on it. Declaring success
                // here fed an unbounded recovery loop: the rebuild "passed",
                // the re-arm timed out, and the retry budget reset forever.
                // Fail the init instead; hwReady stays false, the retry
                // engine's 12-try budget counts, and give-up stays reachable.
                if (ipcOk == 0) {
                    setProperty("Status", "FAILED: runtime IPC dead"),
                        IOLog("LatSOF: %s\n", "FAILED: runtime IPC dead");
                    goto cleanup;
                }

                // Save hardware state
                hdaBase = hda; dspBase = dsp;
                this->ppCap = ppCap; this->spibCap = spibCap; this->mlCap = mlCap;
                this->sIdx = sIdx; this->sTag = sTag; this->sd = sd;
                // patch-32: capture moved SD1 -> SD6 (last input stream).
                // SD1 was chosen when AppleHDA had at most ONE input engine
                // (SD0). Layout 92 brought back TWO (IMic + Lini), and
                // selecting the second one runs a real DMA stream on the
                // next free input SD even though the codec pin is dead —
                // trampling a capture stream parked on SD1. Field failure
                // 2 Aug 01:20: switch input to Line In -> our capture stop
                // IPC timed out -> 12 failed rebuilds -> mic dead until
                // sleep. GCAP says 7 input streams (SD0-SD6); AppleHDA
                // allocates from the bottom, so SD6 collides only if seven
                // input engines run at once. Tag follows idx+1 convention.
                this->capIdx = 6; this->capTag = 7;
                this->capSd = SD_BASE + 6 * SD_SIZE;
                this->outboxOff = outboxOff;
                hwReady = true; isPlaying = false; isCapturing = false; activePlaybackHost = PIPE1_HOST_ID;

                // Allocate DMA buffers (same as before)
                if (!sharedDmaBuf) {
                    // Audio buffer MUST be below 4GB — compressed page table uses 20-bit PFN
                    sharedDmaBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
                        kernel_task, kIOMemoryPhysicallyContiguous | kIODirectionInOut,
                        kLatSOF_BufferSize, 0x00000000FFFFF000ULL);
                    if (sharedDmaBuf) sharedDmaBuf->prepare();
                }
                if (sharedDmaBuf) bzero(sharedDmaBuf->getBytesNoCopy(), kLatSOF_BufferSize);
                if (!sharedBdlBuf) {
                    UInt32 numBdl = (kLatSOF_BufferSize + PAGE_SIZE - 1) / PAGE_SIZE;
                    UInt32 bdlSz = ((numBdl * 16) + 127) & ~127U;
                    sharedBdlBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
                        kIOMemoryPhysicallyContiguous | kIODirectionInOut, bdlSz, 0xFFFFFFFFFFFFFF80ULL);
                    if (sharedBdlBuf && sharedDmaBuf) {
                        sharedBdlBuf->prepare();
                        HdaBdlEntry *abdl = (HdaBdlEntry *)sharedBdlBuf->getBytesNoCopy();
                        memset(abdl, 0, bdlSz);
                        UInt64 phys = sharedDmaBuf->getPhysicalAddress();
                        UInt32 rem = kLatSOF_BufferSize;
                        for (UInt32 i = 0; i < numBdl && rem > 0; i++) {
                            UInt32 chunk = (rem > PAGE_SIZE) ? PAGE_SIZE : rem;
                            abdl[i].addrLow = (UInt32)(phys & 0xFFFFFFFF);
                            abdl[i].addrHigh = (UInt32)(phys >> 32);
                            abdl[i].size = chunk; abdl[i].ioc = 1;
                            rem -= chunk; phys += chunk;
                        }
                    }
                }
                if (!capDmaBuf) {
                    capDmaBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
                        kernel_task, kIOMemoryPhysicallyContiguous | kIODirectionInOut,
                        kLatSOF_CapBufferSize, 0xFFFFFFFFFFFFF000ULL);
                    if (capDmaBuf) capDmaBuf->prepare();
                }
                if (capDmaBuf) bzero(capDmaBuf->getBytesNoCopy(), kLatSOF_CapBufferSize);
                // review 1 Aug: BDL only if the ring exists — a lone capBdlBuf
                // (ring alloc failed) could never be repaired by a later
                // initDSP (its !capBdlBuf guard skips the fill), and capture
                // would arm against all-zero BDL entries. Keeping the pair
                // atomic lets the next initDSP rebuild both.
                if (!capBdlBuf && capDmaBuf) {
                    UInt32 numBdl = (kLatSOF_CapBufferSize + PAGE_SIZE - 1) / PAGE_SIZE;
                    UInt32 bdlSz = ((numBdl * 16) + 127) & ~127U;
                    capBdlBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
                        kIOMemoryPhysicallyContiguous | kIODirectionInOut, bdlSz, 0xFFFFFFFFFFFFFF80ULL);
                    if (capBdlBuf && capDmaBuf) {
                        capBdlBuf->prepare();
                        HdaBdlEntry *cbdl = (HdaBdlEntry *)capBdlBuf->getBytesNoCopy();
                        memset(cbdl, 0, bdlSz);
                        UInt64 cphys = capDmaBuf->getPhysicalAddress();
                        UInt32 crem = kLatSOF_CapBufferSize;
                        for (UInt32 i = 0; i < numBdl && crem > 0; i++) {
                            UInt32 chunk = (crem > PAGE_SIZE) ? PAGE_SIZE : crem;
                            cbdl[i].addrLow = (UInt32)(cphys & 0xFFFFFFFF);
                            cbdl[i].addrHigh = (UInt32)(cphys >> 32);
                            cbdl[i].size = chunk; cbdl[i].ioc = 1;
                            crem -= chunk; cphys += chunk;
                        }
                    }
                }
                // LATITUDE FORK: MAX98357A GPIO removed (no such amp)


                // Initialize RT5682 AFTER all pipelines (SSP0 DAI_CONFIG enables MCLK)
                initRT5682();

                // PCM_PARAMS will be sent in startPlayback() — NOT here.
                // Sending it twice causes firmware to reject the second one.
setProperty("Status", "Ready for UserClient"), IOLog("LatSOF: %s\n", "Ready for UserClient");

            done_inner: ;
            } else {
                wr32(dsp, IPC_HIPCTDR, rd32(dsp, IPC_HIPCTDR) | IPC_BUSY);
                wr32(dsp, IPC_HIPCTDA, IPC_DONE);
            }
        }

        setProperty("Status", "OK"), IOLog("LatSOF: %s\n", "OK");

    cleanup:
        // LATITUDE FORK patch-26: restore the poll-only doctrine — HERE, not
        // above the label. The loader turned on the DSP->host interrupt for
        // the ROM handshake (HIPCCTL = 0x03, ADSPIC |= 1); patch-24 masked it
        // again, but placed the masks on the success path only, so every
        // "goto cleanup" (ROM IPC, INIT_DONE, FW load timeouts) exited with
        // the interrupt fully armed on the line we share with AppleHDA — and
        // a firmware that finished entering just after our 3 s poll gave up
        // would assert it with no handler anywhere. The same off-by-one-label
        // mistake patch-24 itself fixed for the stream restore. Every IPC in
        // this driver polls HIPCIDA; nothing ever needs these enabled.
        wr32(dsp, DSP_ADSPIC, 0);
        wr32(dsp, IPC_HIPCCTL, 0);
        // LATITUDE FORK patch-24: this label is not only a goto target — the
        // success path falls straight into it. The old body zeroed BDLPL,
        // BDLPU, CBL and LVI on AppleHDA's descriptor, after the restore and
        // after SD-Return had been written, wiping the DMA descriptor of a
        // stream we had just promised to hand back untouched. It also ran on
        // the ROM-IPC and INIT_DONE timeout paths, which jump over the normal
        // hand-back entirely and so left SD7 pointing at our firmware buffer.
        //
        // Now every exit lands on the same idempotent restore: a no-op after a
        // successful init, and the only thing that puts the stream back after
        // a failed one.
        sdRestore(hda, sd, (UInt32)sIdx, ppCap, spibCap, snap);
        freeDma(bdlDma);   // patch-36: IODMACommand-owned buffers
        freeDma(fwDma);
        freeDma(posDma);   // patch-53
        // Proof of the contract, read after the last write this function makes
        // to the borrowed descriptor. SD-Final must match SD-Borrow field for
        // field; if it ever doesn't, something wrote SD7 after the hand-back
        // and that is the bug — no inference needed.
        // Only meaningful when we actually borrowed. The round-4 bail (output
        // went busy between preflight and borrow) reaches this label with an
        // invalid snapshot having written nothing — publishing SD-Final there
        // would invite a field-for-field comparison against a STALE SD-Borrow
        // and read as a contract violation that never happened.
        if (snap.valid) { char f[128];
          snprintf(f, sizeof(f),
                   "sd%d ctl=0x%06x fmt=0x%04x bdl=0x%08x cbl=%u lvi=%u ppctl=0x%08x spiben=0x%08x",
                   sIdx, rd32(hda, sd) & 0x00FFFFFF, rd16(hda, sd + SD_REG_FMT),
                   rd32(hda, sd + SD_REG_BDLPL), rd32(hda, sd + SD_REG_CBL),
                   (unsigned)rd16(hda, sd + SD_REG_LVI),
                   ppCap ? rd32(hda, ppCap + PP_PPCTL) : 0,
                   spibCap ? rd32(hda, spibCap + 0x04) : 0);
          setProperty("SD-Final", f); }
    }

done:
    return hwReady;
}

// ==================== Power Management ====================

// Fast mute: cheap, idempotent. Two layers — codec mute (headphone only)
// and DMA-buffer bzero.
//
// Why we don't hard-stop DMA here:
//   Hard-stopping the stream (`wr8(sd, ~SD_CTL_RUN)`) makes silence air-tight
//   against coreaudiod's WriteMix re-filling the buffer, but has an
//   unacceptable side effect: with DMA stopped, the CoreAudio HAL detects a
//   dead stream and calls StopIO→StartIO, which tears down and rebuilds the
//   SOF pipeline. PCM_PARAMS IPC then times out (the abrupt DMA halt leaves
//   DSP state inconsistent), error recovery tries initDSP(), and DSP core
//   power-up fails too ("FAILED: core power") — leaving the kext unusable
//   until reboot.
//
// Keeping only bzero + codec-mute. Trade-off: coreaudiod may still fill
// the ring once or twice after fastMute() before it's throttled, so a
// very short loop-tone (a few hundred ms) can be audible on short
// clamshell. Acceptable — no deadlocks, no FAILED state, and on actual
// system sleep shutdownForSleepGated() resets state (wake rebuilds the
// DSP from scratch, so nothing more is needed on the way down).
void LatSOFAudioDevice::fastMute() {
    if (i2cBase && isPlaying && activePlaybackHost == PIPE1_HOST_ID) {
        i2cWrite16(0x0002, 0x8080);   // HP_CTRL_1: mute L+R
    }
    if (sharedDmaBuf) {
        bzero(sharedDmaBuf->getBytesNoCopy(), kLatSOF_BufferSize);
    }
}

// Short-clamshell path: lid closes but system never reaches system sleep.
// coreaudiod gets throttled, stops filling the DMA ring, hardware keeps
// looping the tail — this is the "loop-tone" symptom. fastMute stops DMA;
// on open we restart it without re-initialising the DSP pipeline. isPlaying
// is *not* cleared, so CoreAudio's HAL doesn't know we paused and resumes
// seamlessly once DMA runs again.
// The clamshell handler no longer touches audio hardware. It only toggles a
// one-byte flag in the shared flags page. The plugin reads this flag on every
// DoIOOperation(WriteMix) cycle and, if set, bzero's its just-written output
// region — digital silence at the source, with zero risk to DMA/DSP state
// (unlike a hardware SD_CTL.RUN clear, which crashed the DSP via the HAL's
// StopIO/StartIO/PCM_PARAMS storm).
void LatSOFAudioDevice::handleClamshellChangeGated(bool closed) {
    if (flagsBuf) {
        uint8_t *f = (uint8_t *)flagsBuf->getBytesNoCopy();
        f[kLatSOF_FlagOff_ClamshellMuted] = closed ? 1 : 0;
    }
    // RT5682 HP pop-click suppression: mute/unmute codec analog path in
    // addition to the digital silence. MAX98357A (speaker amp) has no
    // I2C mute — it relies on the DMA-source digital silence above plus
    // its own datasheet-documented idle auto-mute on a DC-0 input signal.
    if (isPlaying && i2cBase && activePlaybackHost == PIPE1_HOST_ID) {
        i2cWrite16(0x0002, closed ? 0x8080 : 0x0000);   // HP_CTRL_1
    }
}

IOReturn LatSOFAudioDevice::s_handleClamshellChange(OSObject *o, void *closedArg, void *, void *, void *) {
    bool closed = ((uintptr_t)closedArg & 0x1) != 0;
    static_cast<LatSOFAudioDevice *>(o)->handleClamshellChangeGated(closed);
    return kIOReturnSuccess;
}

void LatSOFAudioDevice::shutdownDSPGated() {
    if (!hdaBase || !dspBase) return;

    // Silence first, teardown second — prevents the DMA-ring-loop artefact.
    fastMute();

    // Stop our own streams if running (already on workloop, call gated forms
    // directly). Capture matters as much as playback here: powering the DSP
    // cores down underneath a running capture DMA leaves the engine reading
    // from a dead pipeline.
    if (isPlaying) stopPlaybackGated();
    if (isCapturing) stopCaptureGated();

    // Disable IPC interrupts
    wr32(dspBase, IPC_HIPCCTL, 0);
    wr32(dspBase, DSP_ADSPIC, 0);

    // Power down DSP cores: stall + reset + remove power
    { UInt32 a = rd32(dspBase, DSP_ADSPCS);
      a |= ADSPCS_CRST(0xF) | ADSPCS_CSTALL(0xF);
      a &= ~ADSPCS_SPA(0xF);
      wr32(dspBase, DSP_ADSPCS, a);
    }
    poll32(dspBase, DSP_ADSPCS, ADSPCS_CPA(0xF), 0, 50000);

    // LATITUDE FORK patch-25: everything above this point is BAR4 — the DSP,
    // which is exclusively ours. What used to follow was three writes to
    // BAR0, which is AppleHDA's:
    //
    //     if (ppCap) wr32(hdaBase, ppCap + PP_PPCTL, 0);   // shared register
    //     wr32(hdaBase, HDA_INTCTL, 0);                    // ALL its int enables
    //     wr32(hdaBase, HDA_GCTL, rd32(...) & ~1U);        // global link reset
    //
    // The last one is the exact thing rule 1 of the coexistence design
    // forbids, and the first two blanket-write registers we are only ever
    // allowed to read-modify-write. Inherited from the reference driver,
    // which owned the device and could legitimately do all three.
    //
    // Nothing here needs them. The stream teardown above already cleared our
    // own SDCTL/SDSTS and re-coupled our PPCTL bit; the DSP is powered down
    // through BAR4. Resetting the shared controller only ever meant taking
    // AppleHDA's streams down with us.
    hwReady = false;
}

// LATITUDE FORK patch-27: self-healing after a capture IPC timeout.
//
// Field failure, observed twice (27-28 Jul): a WebRTC client's session
// setup/teardown (Meet in Chrome/Brave — they open, close and reopen the
// device rapidly while joining) leaves the SOF firmware's PCM state
// desynced. From then on every capture PCM_PARAMS or TRIG_START times out:
// the mic is dead until the DSP is rebuilt, and the plugin cannot retry
// (it only issues StartCapture on its 0->1 client transition). Previously
// the only cures were a sleep/wake or a reboot.
//
// The cure already exists in this driver and survived four audit rounds:
// the wake-retry engine plus the capture demand latch. So on an IPC
// timeout we declare the DSP dead exactly as a wake does — hwReady false,
// retry round scheduled — and jackPoll rebuilds the firmware within
// ~1.5 s (deferring while AppleHDA's output is busy, as always). If the
// timeout was in a *start*, the demand latch is already set and the
// rebuild re-arms capture automatically; a mid-recovery StartCapture is
// refused with the demand latched, exactly like the wake window.
//
// Runs gated (capture paths and jackPoll share the workloop), so touching
// the retry statics here is race-free by the same argument as everywhere
// else in this file.
void LatSOFAudioDevice::scheduleDspRecovery(const char *reason) {
    if (!pciDevice || !hdaBase || !dspBase) return;   // teardown — nothing to recover
    // patch-27b: bounded episodes. Without this cap, a rebuild that
    // "succeeds" against a dead mailbox followed by a failed re-arm would
    // re-enter here and reset the retry budget indefinitely.
    if (++gRecoveryEpisodes > 3) {
        setProperty("DSP-Recovery", "exhausted (3 episodes) — sleep/wake to retry");
        IOLog("LatSOF: DSP recovery exhausted; deferring to next sleep/wake\n");
        return;
    }
    hwReady = false;
    gWakeReinitPending = true;
    gWakeTries = 0;
    gWakeTickDivider = 0;
    gHotRecovery = true;      // patch-32: IPC-timeout context — borrow patiently
    // patch-30a: patience is PER EPISODE, not per boot. Without this reset a
    // recovery that follows an earlier one starts with zero patience, borrows
    // AppleHDA's programmed descriptor immediately, and the firmware load then
    // fails — observed 29 Jul as 12 consecutive "FAILED: ROM IPC timeout" with
    // SD-Borrow showing fmt=0x0031 bdl=0x2be41000, i.e. a stream that was very
    // much in use. Every retry engine here refills its own budget; this one
    // must too.
    gProgrammedWaits = 0;
    setProperty("DSP-Recovery", reason);
    IOLog("LatSOF: DSP recovery scheduled: %s\n", reason);
}

// Fast, MMIO-poll-free sleep teardown. See header for rationale.
void LatSOFAudioDevice::shutdownForSleepGated() {
    // fastMute already guards sharedDmaBuf/i2cBase, safe if not alloc'd.
    fastMute();
    // State reset only. PCI D3 (driven by IOKit PM after we return) powers
    // down DSP + HDA. Wake calls initDSP() to fully rebuild — any IPC or
    // HDA-reset polls we did here would be thrown away on wake anyway.
    // patch-24: remember a live capture session so wake can re-arm it. The
    // plugin will not ask again on its own — see gWasCapturing.
    // patch-26: OR, don't assign. A second sleep arriving before the retry
    // round has re-armed (first attempt is ≥1.5 s after wake) used to
    // overwrite a still-pending true latch with isCapturing == false — and
    // the session the app is holding was lost on the second wake. The latch
    // is now cleared only by a successful re-arm or an explicit StopCapture.
    gWasCapturing = gWasCapturing || isCapturing;
    hwReady = false;
    isPlaying = false;
    isCapturing = false;
}

IOReturn LatSOFAudioDevice::setPowerStateGated(unsigned long powerStateOrdinal) {
    if (powerStateOrdinal == 0) {
        // Sleep — use the fast path to prevent MMIO hang-on-sleep.
        gSleptOnce = true;            // see the latch comment: real wakes only
        gWakeReinitPending = false;   // patch-22: cancel any retry round
        shutdownForSleepGated();
    } else {
        gSleepImminent = false;   // round 4: unconditional — even on wakes
                                  // that skip re-init below
        // Wake — full re-init. gSleptOnce keeps this off the initial PM
        // registration, including the patch-30 deferred-load case where
        // hwReady is legitimately still false at registration time.
        // patch-26: do NOT require hdaBase/dspBase here. A failed remap on a
        // previous wake nulls them, and the only code that can repair them is
        // the remap block inside this very branch — gating on the pointers
        // turned one transient remap failure into a driver that was dead
        // until reboot, while its own comment promised "next wake retries".
        if (!hwReady && pciDevice && gSleptOnce) {
            // PCI power cycle
            pciDevice->setBusMasterEnable(true);
            pciDevice->setMemoryEnable(true);
            UInt8 tcsel = pciDevice->configRead8(PCI_TCSEL);
            pciDevice->configWrite8(PCI_TCSEL, tcsel & ~0x07);

            // LATITUDE FORK patch-21: rebuild the BAR mappings before any
            // register access. IOKit can tear down this device's memory
            // mappings across a power transition, leaving hdaBase/dspBase
            // pointing at nothing — which reads back as all-ones and looks
            // exactly like "the controller is not decoding yet", except no
            // amount of waiting helps (observed: GCAP 0xffff for a full
            // 3 s while AppleHDA drove the same registers happily).
            {
                volatile UInt8 *oldHda = hdaBase;
                if (hdaBarMap) { hdaBarMap->release(); hdaBarMap = nullptr; }
                if (dspBarMap) { dspBarMap->release(); dspBarMap = nullptr; }
                hdaBarMap = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
                dspBarMap = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress4);
                if (!hdaBarMap || !dspBarMap) {
                    setProperty("BAR-Remap", "FAILED"),
                        IOLog("LatSOF: %s\n", "FAILED: BAR remap on wake");
                    hdaBase = nullptr; dspBase = nullptr;
                    return kIOPMAckImplied;   // hwReady false -> next wake retries
                }
                hdaBase = (volatile UInt8 *)hdaBarMap->getVirtualAddress();
                dspBase = (volatile UInt8 *)dspBarMap->getVirtualAddress();
                { char r[48];
                  snprintf(r, sizeof(r), "OK changed=%d", (oldHda != hdaBase) ? 1 : 0);
                  setProperty("BAR-Remap", r); }
            }

            // LATITUDE FORK patch-22: do NOT init here. One synchronous
            // shot at a fixed instant is the design that failed all week.
            // Set the flag; jackPoll's 500 ms tick runs attempts with
            // preflight checks until one verifiably succeeds.
            gWakeReinitPending = true;
            gWakeTries = 0;
            gWakeTickDivider = 0;
            gRecoveryEpisodes = 0;   // patch-27b: each wake grants a fresh budget
            gProgrammedWaits  = 0;   // patch-30a: and a fresh borrow patience
            gHotRecovery      = false; // patch-32: cold wake — boot-arg policy
            // review 1 Aug: like every other retry engine here. A wake cycle
            // that skips resumeAudioEngine (engine parked in Resumed after a
            // failed attempt — family only pauses Running engines) would
            // otherwise run on the previous cycle's leftover tries.
            gEngineResumeTries = 0;
            // review 1 Aug round 2: and an EXHAUSTED latch must be re-armed
            // here, or a still-held session is dead forever — once parked in
            // Resumed, no later sleep/wake generates a resumeAudioEngine
            // call to re-latch it (family pauses only Running engines).
            // Same philosophy as gWasCapturing surviving a failed re-arm.
            // Self-healing if over-eager: completeDeferredResume no-ops on
            // Running or clientless engines and the executor clears the latch.
            if (!gEngineResumePending && kernelAudio) {
                LatSOFKernelAudioEngine *e = kernelAudio->getEngine();
                if (e && e->needsDeferredResume()) {
                    gEngineResumePending = true;
                    // round 4: split the telemetry — Resumed is the genuinely
                    // parked anomaly (no family resume will ever come for
                    // it); Paused here is just a normal held-session wake
                    // latched ahead of the family's own resume.
                    if (e->getState() == kIOAudioEngineResumed) {
                        setProperty("Engine-Resume", "re-latched at wake (parked)");
                        IOLog("LatSOF: %s\n",
                              "parked session found at wake — re-latching resume");
                    } else {
                        setProperty("Engine-Resume", "latched at wake");
                        IOLog("LatSOF: %s\n",
                              "held session at wake — latching ahead of family resume");
                    }
                }
            }
            setProperty("Wake-Retry", "scheduled");
        }
    }
    return kIOPMAckImplied;
}

IOReturn LatSOFAudioDevice::s_setPowerState(OSObject *o, void *a0, void *, void *, void *) {
    return static_cast<LatSOFAudioDevice *>(o)->setPowerStateGated((unsigned long)(uintptr_t)a0);
}

IOReturn LatSOFAudioDevice::setPowerState(unsigned long powerStateOrdinal, IOService *whatDevice) {
    if (!commandGate) return setPowerStateGated(powerStateOrdinal);
    return commandGate->runAction(&s_setPowerState, (void *)(uintptr_t)powerStateOrdinal);
}

// System-wide power messages arrive BEFORE setPowerState — use the WillSleep
// hook to pre-mute while userspace is still alive, minimising the audible
// DMA-loop window.
void LatSOFAudioDevice::handleWillSleepGated() {
    gSleepImminent = true;   // round 4: park the resume executor (see latch)
    fastMute();
}
IOReturn LatSOFAudioDevice::s_handleWillSleep(OSObject *o, void *, void *, void *, void *) {
    static_cast<LatSOFAudioDevice *>(o)->handleWillSleepGated();
    return kIOReturnSuccess;
}

// See sPowerInterestHandler: the sleep-parking latch must not survive a
// sleep that never happened.
IOReturn LatSOFAudioDevice::s_handleWakeNotice(OSObject *o, void *, void *, void *, void *) {
    (void)o;
    gSleepImminent = false;
    return kIOReturnSuccess;
}

// System-wide power event handler, registered via
//   rootDomain->registerInterest(gIOGeneralInterest, ...).
// This path — NOT message()/registerInterestedDriver — is the one that
// delivers kIOMessageSystemWillSleep and kIOPMMessageClamshellStateChange.
// Returning kIOReturnSuccess is required for WillSleep (otherwise PM waits
// its full timeout before proceeding).
IOReturn LatSOFAudioDevice::sPowerInterestHandler(void *target, void *refCon,
    UInt32 messageType, IOService *provider, void *messageArg, vm_size_t argSize)
{
    LatSOFAudioDevice *self = static_cast<LatSOFAudioDevice *>(target);
    if (!self) return kIOReturnSuccess;
    if (messageType == kIOMessageSystemWillSleep) {
        if (self->commandGate) self->commandGate->runAction(&s_handleWillSleep);
    } else if (messageType == kIOMessageSystemWillPowerOn ||
               messageType == kIOMessageSystemHasPoweredOn ||
               messageType == kIOMessageSystemWillNotSleep) {
        // review 1 Aug round 4 (self-review): gSleepImminent is set by the
        // non-vetoable WillSleep and cleared in setPowerStateGated's wake
        // arm — but an ABORTED sleep (a driver failing setPowerState, seen
        // on this machine 29 Jul 10:32) returns the system to running
        // without ever delivering our power-state pair, which would park the
        // resume executor for the rest of the session. These messages arrive
        // on every real wake and on every abort, so clearing here closes it.
        if (self->commandGate) self->commandGate->runAction(&s_handleWakeNotice);
    } else if (messageType == kIOPMMessageClamshellStateChange) {
        // arg bitfield: bit 0 (kClamshellStateBit) = clamshell closed.
        if (self->commandGate) {
            self->commandGate->runAction(&s_handleClamshellChange, messageArg);
        }
    }
    return kIOReturnSuccess;
}

void LatSOFAudioDevice::stop(IOService *provider) {
    if (powerNotifier) {
        powerNotifier->remove();
        powerNotifier = nullptr;
    }
    rootDomain = nullptr;
    if (jackTimer) {
        jackTimer->cancelTimeout();
        if (getWorkLoop()) getWorkLoop()->removeEventSource(jackTimer);
        jackTimer->release(); jackTimer = nullptr;
    }
    // kernel-mic: tear the audio device down first, while the workloop and
    // gated capture paths are fully alive — its engine stop calls
    // engineStopCapture, which must find working machinery.
    // review 1 Aug round 3: publish the null UNDER the gate before
    // terminating. PM can still deliver a gated setPowerState until PMstop
    // (see the patch-26 comment below), and the wake re-latch dereferences
    // kernelAudio->getEngine() from that path — an off-gate teardown here
    // was a use-after-free window. After the gated null, an in-flight gated
    // reader has either finished (object fully alive) or sees nullptr.
    if (kernelAudio) {
        LatSOFKernelAudioDevice *ka = kernelAudio;
        if (commandGate) commandGate->runAction(&s_clearKernelAudio);
        else kernelAudio = nullptr;
        ka->terminate(kIOServiceSynchronous);
        ka->release();
    }

    // patch-26: stop our streams BEFORE PMstop. PMstop synchronously
    // delivers setPowerState(0) → shutdownForSleepGated, which clears
    // isPlaying/isCapturing without stopping DMA — correct for sleep,
    // where PCI D3 follows and stops the engines; wrong for teardown,
    // where nothing does. These calls used to sit after PMstop, where
    // their guards were always already false: a kext unload during
    // capture freed capDmaBuf below while SD1 was still DMA-writing into
    // it. Routed through the gate (still installed at this point) because
    // PM's setPowerState can arrive on the gated path until PMstop —
    // "single-threaded in stop" is only true of what runs under the gate.
    // The gated forms no-op on their own flags, so no guards needed here.
    if (commandGate) {
        commandGate->runAction(&s_stopPlayback);
        commandGate->runAction(&s_stopCapture);
    } else {
        if (isPlaying) stopPlaybackGated();
        if (isCapturing) stopCaptureGated();
    }
    PMstop();
    if (commandGate) {
        if (getWorkLoop()) getWorkLoop()->removeEventSource(commandGate);
        commandGate->release();
        commandGate = nullptr;
    }
    if (sharedDmaBuf) { sharedDmaBuf->complete(); sharedDmaBuf->release(); sharedDmaBuf = nullptr; }
    if (sharedBdlBuf) { sharedBdlBuf->complete(); sharedBdlBuf->release(); sharedBdlBuf = nullptr; }
    if (capDmaBuf) { capDmaBuf->complete(); capDmaBuf->release(); capDmaBuf = nullptr; }
    if (capBdlBuf) { capBdlBuf->complete(); capBdlBuf->release(); capBdlBuf = nullptr; }
    if (flagsBuf) { flagsBuf->complete(); flagsBuf->release(); flagsBuf = nullptr; }
    if (i2cBarMap) { i2cBarMap->release(); i2cBarMap = nullptr; }
    if (i2cPciDevice) { i2cPciDevice->release(); i2cPciDevice = nullptr; }
    if (hdaBarMap) { hdaBarMap->release(); hdaBarMap = nullptr; }
    if (dspBarMap) { dspBarMap->release(); dspBarMap = nullptr; }
    if (pciDevice) { pciDevice->close(this); pciDevice->release(); pciDevice = nullptr; }
    IOService::stop(provider);
}

// ==================== Jack Detection Polling ====================

void LatSOFAudioDevice::jackPoll(IOTimerEventSource *sender) {
    // LATITUDE FORK patch-22: wake re-init retry engine. Lives here because
    // this timer already fires every 500 ms on the workloop regardless of
    // hwReady — exactly the cadence and exclusion context needed.
    if (gWakeReinitPending && !hwReady && pciDevice && hdaBase && dspBase) {
        // patch-47: decide whether there is anything to do BEFORE the engine
        // commits to a tick. Measured 2026-10-01: the code below reached the
        // busy-check first, found the borrow stream programmed by AppleHDA,
        // and held `busy` true for ever — so it never called initDSP and
        // never took the 12-try exit either. The engine ticked for the whole
        // session, one log line per 1.5 s, and `sidx` (parsed only *inside*
        // initDSP) could never be changed to point the busy-check anywhere
        // else. Circular. Two register reads per tick is cheap, but an engine
        // that can never reach a terminal state is not.
        labReload();                       // cheap; catches a fresh sysctl write
        // patch-67: in auto mode the profile string stays permanently armed —
        // reaching this tick at all means gWakeReinitPending was raised by a
        // wake, a capture demand or a load-path defer, and the 12-attempt
        // budget below is the loop guard. Only the fuse (auto=0 -> enable=0)
        // parks the engine now.
        bool labArmed = (gLabEnable != 0 &&
                         (gLabGen != gLabGenDone || gLabAuto));
        if (!labArmed) {
            // Park it. Writing new parameters re-raises gWakeReinitPending
            // via the capture-demand path, so arming later still needs no
            // reboot — this only stops the idle spin.
            gWakeReinitPending = false;
            setProperty("Wake-Retry", gLabAuto
                ? "idle: auto mode off via auto=0"
                : "idle: lab gate closed (write params to arm)");
            IOLog("LatSOF: %s\n", gLabAuto
                ? "wake retry engine parked - auto mode disabled (auto=0)"
                : "wake retry engine parked - lab gate closed");
        }
        else if (++gWakeTickDivider >= 3) {            // one attempt per ~1.5 s
            gWakeTickDivider = 0;
            // patch-30: RUN alone is not enough. Measured 29 Jul: at a
            // wake this borrowed SD7 while it read
            //   ctl=0x140000 fmt=0x0031 bdl=0x2907a000 cbl=393216 lvi=95
            // — RUN CLEAR but fully PROGRAMMED by AppleHDA. The loader
            // resets that descriptor, uses it, and restores the registers
            // byte-for-byte, but a stream's FIFO and DMA position are not
            // register state and cannot be handed back. AppleHDA then
            // plays into a descriptor whose internals we scrubbed, and
            // never reprograms it — persistent output static that even
            // replugging the jack does not clear. Only a reboot did.
            //
            // So a programmed descriptor also means "not ours to take".
            // It cannot mean "wait forever", though: AppleHDA leaves SD7
            // programmed after its first playback, so an unbounded wait
            // would mean no microphone for the rest of the session. Wait
            // a bounded number of rounds for a genuinely clean window,
            // then take it anyway — a mic that works with a risk of
            // static beats a mic that never comes back.
            //
            // Classification comes from outputSdBusyState — the same
            // predicate the load path uses, so the two cannot disagree.
            // Patience and borrow-anyway POLICY stay here; the helper only
            // answers "what state is the descriptor in".
            UInt16 g = 0;
            int st = outputSdBusyState(hdaBase, &g);
            bool decoding = (st != -1);
            bool busy = (st == 1), programmed = false;
            if (!busy && decoding && (gStrictBorrow || gHotRecovery)) {
                if (st == 2) {
                    // patch-47 BUG FIX. This branch used to announce
                    // "borrowing anyway" and then leave busy=true — so the
                    // `if (!busy && decoding && initDSP())` below was dead
                    // code whenever AppleHDA leaves its output descriptor
                    // programmed, which is every machine after its first
                    // playback. The engine could not run the load path and
                    // could not reach its own 12-try exit either: it ticked
                    // for ever. The documented intent ("bounded patience,
                    // then take it anyway") is now what the code does.
                    if (++gProgrammedWaits <= kProgrammedWaitRounds && !labArmed) {
                        programmed = true;
                        busy = true;              // defer this round
                    } else {
                        // An armed lab run is a deliberate, watched action;
                        // sitting out 30 s of patience in front of every
                        // experiment would be pure dead time.
                        if (!gBorrowWarned) {
                            gBorrowWarned = true;
                            setProperty("Borrow-Warning",
                                "SD7 still programmed - borrowing anyway");
                            IOLog("LatSOF: SD7 still programmed after %d rounds; "
                                  "borrowing anyway (playback may glitch)\n",
                                  kProgrammedWaitRounds);
                        }
                        busy = false;             // patch-47: actually take it
                    }
                } else {
                    gProgrammedWaits = 0;         // clean window — reset patience
                    gBorrowWarned = false;
                }
            }
            // patch-26: only a real init attempt consumes the retry budget.
            // gWakeTries++ used to run before the busy check, so music
            // auto-resuming at wake for ~18 s burned all 12 rounds on "busy"
            // and the engine gave up without ever calling initDSP — mic dead
            // for the whole awake session. Waiting our turn is now free: the
            // preflight is two register reads per 1.5 s, and the first quiet
            // moment gets a genuine attempt.
            // patch-46: while the lab bench holds the hardware off, an
            // attempt must not consume the 12-round budget. A sweep writes
            // parameters far more than 12 times and the engine has to still
            // be alive for the last write. Safe because a held-off attempt
            // returns from initDSP before touching anything: this cannot
            // become a hot loop, it is just a timer that keeps ticking.
            if (gLabHoldOff) gWakeTries = 0;
            else if (decoding && !busy) gWakeTries++;
            { char w[80];
              snprintf(w, sizeof(w), "n=%d gcap=0x%04x %s", gWakeTries, g,
                       programmed ? "deferred: SD7 programmed"
                                  : (busy ? "busy" : (decoding ? "try" : "nodecode")));
              setProperty("Wake-Retry", w); }
            if (!busy && decoding && initDSP()) {
                gWakeReinitPending = false;
                setProperty("Wake-Retry-Done", "OK"),
                    IOLog("LatSOF: %s\n", "wake re-init OK");
                // patch-24: the plugin will not re-issue StartCapture on its
                // own, so a session that was live at sleep has to be restarted
                // here. Already on the workloop — call the gated form directly,
                // exactly as the jack-change path below does.
                if (gWasCapturing) {
                    IOReturn cr = startCaptureGated();
                    // patch-26: consume the latch only on success. A failed
                    // re-arm (one PCM_PARAMS timeout) now survives the next
                    // sleep/wake instead of being forgotten.
                    if (cr == kIOReturnSuccess) gWasCapturing = false;
                    setProperty("Wake-Capture-Rearm",
                                (cr == kIOReturnSuccess) ? "OK" : "FAILED");
                    IOLog("LatSOF: wake capture re-arm %s\n",
                          (cr == kIOReturnSuccess) ? "OK" : "FAILED");
                }
            } else if (gWakeTries >= 12) {
                gWakeReinitPending = false;
                // patch-26: the capture latch is deliberately NOT cleared
                // here any more. Giving up means 12 genuine init attempts
                // failed this wake; if the app still holds its session, the
                // next sleep/wake preserves the latch (OR in
                // shutdownForSleepGated) and a successful init then re-arms
                // it. Staleness is handled where intent is unambiguous —
                // stopCaptureGated clears the latch on any explicit stop.
                setProperty("Wake-Retry-Done", "GAVE UP after 12 tries"),
                    IOLog("LatSOF: %s\n", "wake re-init gave up after 12 tries");
                // review 1 Aug: make the terminal state self-describing.
                // Without this the last initDSP attempt's bare "FAILED: …"
                // string is what §2's post-install gate sees, and that gate
                // says "roll back" — wrong advice when the real story is
                // "12 attempts exhausted in a busy/hot-load environment".
                { char gu[96];
                  snprintf(gu, sizeof(gu),
                           "FAILED: DSP init retries exhausted (last: %s)",
                           gLastInitFail[0] ? gLastInitFail : "unknown");
                  setProperty("Status", gu); }
            } else if (decoding && !busy && !gLabHoldOff) {
                // patch-46: `!gLabHoldOff` — a held-off tick is not a failed
                // attempt, so it must not overwrite the "idle: lab ..." Status
                // that tells the developer why nothing is happening. Without
                // this the tick rewrap below would relabel an intentional
                // idle as "deferred: init attempt 0/12 failed".
                // review 1 Aug round 2: an attempt RAN and FAILED with budget
                // remaining (initDSP just wrote a bare "FAILED: …" Status).
                // Left standing, that string tells the §2 gate to roll back a
                // build that is mid-retry. Rewrap it as in-progress; keep the
                // reason when it is a fresh initDSP failure string. Fires only
                // on a ran-and-failed event, never on busy/deferred ticks.
                OSString *s = OSDynamicCast(OSString, getProperty("Status"));
                const char *prev = s ? s->getCStringNoCopy() : nullptr;
                char st2[96] = "";
                if (prev && strncmp(prev, "FAILED", 6) == 0) {
                    snprintf(st2, sizeof(st2), "deferred: retrying after %s (%d/12)",
                             prev, gWakeTries);
                    strlcpy(gLastInitFail, prev, sizeof(gLastInitFail));
                }
                else if (!prev || strncmp(prev, "deferred", 8) != 0)
                    snprintf(st2, sizeof(st2), "deferred: init attempt %d/12 failed — retrying",
                             gWakeTries);
                // already "deferred: …" (e.g. the borrow-time re-check's own
                // string) — leave it, it is self-describing
                if (st2[0]) setProperty("Status", st2);
            }
        }
    }
    // patch-30 fix 1 executor. Placed BEFORE the i2cBase bail-out below —
    // i2cBase is permanently null on this board (initI2C returns early), so
    // nothing after that line ever runs. Preflight mirrors the capture
    // re-arm: only act once the rebuild has verifiably succeeded. A failed
    // start schedules its own DSP recovery (both capture-start timeout paths
    // do), which drops hwReady and parks the latch until the rebuild — the
    // bounded tries only spend on attempts that ran and failed. EXCEPT when
    // gRecoveryEpisodes is exhausted: scheduleDspRecovery then returns
    // without dropping hwReady, no rebuild is coming, and retrying at tick
    // rate would stall the workloop ~1 s per attempt for nothing — treat
    // that as terminal immediately (the next wake refills the episode
    // budget AND re-latches a parked session, so nothing is lost).
    // review 1 Aug rounds 3+4: park the executor across sleep entry.
    // kernelAudio is our PM child, so at sleep entry it pauses the engine
    // BEFORE our own setPowerState(0) drops hwReady — a latch executed in
    // that gap would drain the family's SLEEP pause and start capture DMA
    // microseconds before D3. gSleepImminent is the authoritative signal
    // (gated WillSleep, precedes all of it); the family reads are TOCTOU
    // belts only — pendingPowerState is written before the pause loop,
    // currentPowerState covers the wake-completion window. Parked, the
    // latch survives to wake, where budget-refill + re-latch complete it.
    if (gEngineResumePending && hwReady && !gWakeReinitPending && !gSleepImminent &&
        kernelAudio &&
        kernelAudio->getPowerState() != kIOAudioDeviceSleep &&
        kernelAudio->getPendingPowerState() != kIOAudioDeviceSleep) {
        LatSOFKernelAudioEngine *e = kernelAudio->getEngine();
        if (!e) {
            gEngineResumePending = false;
        } else {
            const char *outcome = "idle";
            IOReturn rr = e->completeDeferredResume(&outcome);
            if (rr == kIOReturnSuccess) {
                gEngineResumePending = false;
                setProperty("Engine-Resume", outcome);
                if (!strcmp(outcome, "restarted"))
                    IOLog("LatSOF: %s\n", "engine restarted after resume");
            } else if (++gEngineResumeTries >= kEngineResumeMaxTries ||
                       gRecoveryEpisodes > 3) {
                gEngineResumePending = false;
                setProperty("Engine-Resume", (gRecoveryEpisodes > 3)
                            ? "gave up (recovery exhausted)" : "gave up");
                IOLog("LatSOF: engine resume gave up after %d tries (last=0x%x%s)\n",
                      gEngineResumeTries, rr,
                      (gRecoveryEpisodes > 3) ? ", recovery exhausted" : "");
            } else {
                IOLog("LatSOF: engine resume attempt %d/%d failed (0x%x) — will retry\n",
                      gEngineResumeTries, kEngineResumeMaxTries, rr);
            }
        }
    }
    // patch-31: AFG keep-alive (see the block comment at iciVerb). MMIO-only
    // detection; codec verbs only on an engine (re)build or the slow RUN
    // belt. Runs even while !hwReady would be wrong — the OUTPUT side is
    // AppleHDA's and lives regardless of our DSP state — needs only hdaBase.
    //
    // REVIEW FIX (2 Aug): the first cut triggered on clean->armed only. But
    // AppleHDA leaves SD7 PROGRAMMED after its first playback (patch-30's
    // own measurement), so the descriptor never reads clean again and every
    // later idle-plug rebuild is programmed->programmed — invisible to a
    // 3-state classifier, leaving only the 10 s belt. The rebuild is still
    // MMIO-visible, though: it rewrites BDL/CBL/FMT. So the trigger is any
    // change in the descriptor SIGNATURE (bdl^cbl^fmt) or state class while
    // armed/RUN — which fires at plug time, before playback starts (the
    // pre-arm), and again at RUN start as the belt-and-braces.
    // patch-68: the AFG ICI verbs and the I2C jack read below are only
    // meaningful when OUR engine has (or just had) a client. With auto mode
    // the kext loads the DSP at boot and then sits idle for hours — and the
    // 2026-10-02 freeze forensics (spindump: AppleHDA output StartIO blocked
    // for good inside AppleHDAController) make any standing codec-side
    // traffic from this driver a liability while AppleHDA owns the link.
    // Idle now means: no MMIO beyond the two busy-state reads, zero ICI, zero
    // I2C. AFG keep-alive resumes the moment capture/playback is demanded.
    bool engineInUse = isCapturing || isPlaying || gWasCapturing;
    if (gAfgKextWake && engineInUse && hdaBase && gAfgIciFails < kAfgIciFailLimit) {
        UInt16 g = 0;
        int sd = outputSdBusyState(hdaBase, &g);
        bool rebuild = false;
        if (sd >= 0) {
            UInt32 outSd = SD_BASE + (UInt32)((g >> 8) & 0xF) * SD_SIZE;
            UInt32 sig = rd32(hdaBase, outSd + SD_REG_BDLPL)
                       ^ rd32(hdaBase, outSd + SD_REG_CBL)
                       ^ ((UInt32)rd16(hdaBase, outSd + SD_REG_FMT) << 1);
            rebuild = (sd > 0) && (sd != gAfgLastSd || sig != gAfgLastSig);
            gAfgLastSd = sd;
            gAfgLastSig = sig;
        }
        bool belt = (sd == 1 && ++gAfgBeltTick >= kAfgBeltTicks);
        if (belt || rebuild) gAfgBeltTick = 0;

        // PRE-ARM on the jack itself. Only while the output is not already
        // RUNning — during playback the descriptor path above covers us and
        // this would be pointless codec traffic. One verb per ~1 s at idle,
        // and iciVerb bails on contention, so AppleHDA is never blocked.
        bool jackIn = false;
        if (sd != 1 && ++gAfgSenseTick >= kAfgSenseTicks) {
            gAfgSenseTick = 0;
            UInt32 sense = 0;
            if (iciVerb(hdaBase, kVerbGetPinSense, &sense)) {
                bool present = (sense & kPinPresent) != 0;
                jackIn = present && !gAfgJackWasIn;   // absent -> present
                gAfgJackWasIn = present;
            }
        }
        if (rebuild || belt || jackIn) {
            UInt32 ps = 0;
            if (iciVerb(hdaBase, kVerbGetPower, &ps)) {
                gAfgIciFails = 0;
                // Publish the FIRST successful read even when no correction
                // is needed. Absence of any AFG-* property must mean "the
                // code never ran", never "it ran and silently did nothing" —
                // that ambiguity is what hid the NID bug in the first cut.
                if (!gAfgProbed) {
                    gAfgProbed = true;
                    char pb[48];
                    snprintf(pb, sizeof(pb), "ICI ok, AFG=0x%08x (D%u)",
                             (unsigned)ps, (unsigned)((ps >> 4) & 0xF));
                    setProperty("AFG-Probe", pb);
                    IOLog("LatSOF: AFG probe via ICI: 0x%08x\n", (unsigned)ps);
                }
                if ((ps & 0xF0) != 0) {          // ACTUAL state (bits 7:4) != D0
                    iciVerb(hdaBase, kVerbSetPowerD0, nullptr);
                    gAfgWakes++;
                    setProperty("AFG-Wake", gAfgWakes, 32);
                    IOLog("LatSOF: AFG was D%u at %s — forced D0 (wake #%u)\n",
                          (unsigned)((ps >> 4) & 0xF),
                          jackIn ? "jack insert (pre-arm)" : (rebuild ? "engine rebuild" : "belt check"),
                          (unsigned)gAfgWakes);
                }
            } else if (++gAfgIciFails >= kAfgIciFailLimit) {
                // ICI unusable on this controller: retire, loudly — the
                // userland daemon is then load-bearing again.
                setProperty("AFG-Wake", "ICI unavailable — keep the daemon");
                IOLog("LatSOF: %s\n",
                      "AFG wake retired: ICI never responded; keep latsof-afgwake");
            }
        }
    }
    if (!hwReady || !i2cBase || !engineInUse) goto reschedule;
    {
        UInt16 ajd1 = i2cRead16(0x00F0);
        bool hpIn = (ajd1 & 0x0010) == 0;

        if (isPlaying && hpIn != lastJackState) {
            // jackPoll runs on the workloop, same as commandGate — so PM and
            // UserClient paths are already mutually excluded. No lock needed.
            lastJackState = hpIn;
            isPlaying = false;
            setProperty("Output", hpIn ? "Headphone" : "Speaker");
            // Inline stop: DMA stop + IPC TRIG_STOP + PCM_FREE
            wr8(hdaBase, sd, rd8(hdaBase, sd) & ~(UInt8)(SD_CTL_RUN | SD_CTL_IOCE | 0x08 | 0x10));
            auto sendIpc = [&](UInt32 cmd) {
                wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
                wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
                IODelay(100);
                struct { UInt32 s,c,id; } __attribute__((packed)) m = {12, cmd, activePlaybackHost};
                volatile UInt8 *ob = dspBase + outboxOff;
                for (UInt32 i = 0; i < 12; i += 4) wr32(ob, i, *(UInt32*)((UInt8*)&m + i));
                wr32(dspBase, IPC_HIPCIDR, IPC_BUSY);
                poll32(dspBase, IPC_HIPCIDA, IPC_DONE, IPC_DONE, 500000);
                wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
                wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
            };
            sendIpc(0x60050000); // TRIG_STOP
            sendIpc(0x60030000); // PCM_FREE
            if (i2cBase && activePlaybackHost == PIPE1_HOST_ID) {
                i2cWrite16(0x0083, 0x0000);
                i2cWrite16(0x013a, 0x2000);
                i2cWrite16(0x0003, 0x0000);
                i2cWrite16(0x0002, 0x8080);
                i2cWrite16(0x0061, 0x0000);
            }
            bzero(sharedDmaBuf->getBytesNoCopy(), kLatSOF_BufferSize);

            startPlaybackGated();
        }
    }
reschedule:
    if (sender) sender->setTimeoutMS(500);
}

// ==================== Playback Control (called by UserClient) ====================

IOReturn LatSOFAudioDevice::startPlaybackGated() {
    if (!hwReady || !sharedDmaBuf || !sharedBdlBuf) return kIOReturnNotReady;
    // Stop first if already playing (allows jack re-detection)
    if (isPlaying) stopPlaybackGated();
    if (isPlaying) return kIOReturnSuccess;

    // Match Linux: BDL entries = buffer_size / host_period_bytes = 65536/16384 = 4
    UInt32 hostPeriodBytes = 16384;
    UInt32 numBdl = kLatSOF_BufferSize / hostPeriodBytes;  // = 4 (matching Linux LVI=3)

    // Clean startPlayback: match Linux exactly (no LOSIDV, no ML link, no HP stream)

    // Step 1: Enable INTCTL for this stream
    wr32(hdaBase, HDA_INTCTL, rd32(hdaBase, HDA_INTCTL) | (1U << 31) | (1U << 30) | (1U << sIdx));

    // Step 1b: PPCTL per-stream decouple (Linux hda-stream.c:521-523)
    // Must be set BEFORE stream setup to ensure DSP gateway is active
    if (ppCap) {
        wr32(hdaBase, ppCap + PP_PPCTL,
             rd32(hdaBase, ppCap + PP_PPCTL) | (1U << sIdx));
    }

    // Step 2: Stream reset (Linux does double reset)
    streamReset(hdaBase, sd);

    // Step 2b: Second reset (Linux hda-stream.c:568-591)
    streamReset(hdaBase, sd);

    // Step 2c: Rebuild BDL with period-sized entries (4 × 16KB, matching Linux)
    { HdaBdlEntry *bdl = (HdaBdlEntry *)sharedBdlBuf->getBytesNoCopy();
      memset(bdl, 0, sharedBdlBuf->getLength());
      UInt64 phys = sharedDmaBuf->getPhysicalAddress();
      for (UInt32 i = 0; i < numBdl; i++) {
          bdl[i].addrLow  = (UInt32)((phys + i * hostPeriodBytes) & 0xFFFFFFFF);
          bdl[i].addrHigh = (UInt32)((phys + i * hostPeriodBytes) >> 32);
          bdl[i].size = hostPeriodBytes;
          bdl[i].ioc  = 1;
      }
    }
    UInt64 bdlBus = sharedBdlBuf->getPhysicalAddress();  // patch-37: convert to IODMACommand

    // Step 2d: Program stream tag FIRST (Linux hda-stream.c:602-605)
    wr8(hdaBase, sd + 2, (UInt8)((sTag & 0xF) << 4));

    // Step 2e: Set CBL
    wr32(hdaBase, sd + SD_REG_CBL, kLatSOF_BufferSize);

    // Step 2f: FMT with PPCTL couple/decouple quirk (Linux hda-stream.c:624-637)
    // CML requires: couple → write FMT → decouple
    if (ppCap) {
        // Temporarily COUPLE (clear per-stream bit) before writing format
        wr32(hdaBase, ppCap + PP_PPCTL,
             rd32(hdaBase, ppCap + PP_PPCTL) & ~(1U << sIdx));
    }
    wr16(hdaBase, sd + SD_REG_FMT, 0x0011); // 48kHz, 16-bit, 2ch
    if (ppCap) {
        // Re-DECOUPLE (set per-stream bit) after writing format
        wr32(hdaBase, ppCap + PP_PPCTL,
             rd32(hdaBase, ppCap + PP_PPCTL) | (1U << sIdx));
    }

    // Step 2g: LVI, BDL address
    wr16(hdaBase, sd + SD_REG_LVI, (UInt16)(numBdl - 1));
    wr32(hdaBase, sd + SD_REG_BDLPL, (UInt32)(bdlBus & 0xFFFFFFFF));
    wr32(hdaBase, sd + SD_REG_BDLPU, (UInt32)(bdlBus >> 32));

    // Step 2h: Position buffer enable (Linux hda-stream.c:654-662)
    // patch-53 BUGFIX: this used to write DPLBASE = 0x00000001, i.e. it set
    // the ENABLE bit and simultaneously clobbered the buffer address to 0 -
    // pointing the controller's position DMA at physical address 0 while the
    // comment claimed to be Linux's "bus->posbuf.addr | ENABLE". Preserve the
    // address the firmware/AppleHDA left in the register and only set bit0.
    { UInt32 dplbase = rd32(hdaBase, 0x70);
      if (!(dplbase & 0x01))
          wr32(hdaBase, 0x70, dplbase | 0x01);
    }

    // Step 2i: Enable stream interrupts
    wr8(hdaBase, sd, rd8(hdaBase, sd) | SD_CTL_IOCE | 0x08 | 0x10);

    // Step 3: Detect headphone jack and select output pipeline
    // RT5682 AJD1_CTRL (0x00F0) bit 4: 0=plugged, 1=unplugged
    bool useHeadphone = false;
    if (i2cBase) {
        UInt16 ajd1 = i2cRead16(0x00F0);
        useHeadphone = (ajd1 & 0x0010) == 0;  // bit 4 LOW = jack inserted
        setProperty("Output", useHeadphone ? "Headphone" : "Speaker");
    }
    UInt32 activeHost = useHeadphone ? PIPE1_HOST_ID : PIPE7_HOST_ID;

    // Step 4: PCM_PARAMS IPC
    { struct sof_ipc_pcm_params pcm = {};
      pcm.hdr_size = sizeof(pcm);
      pcm.hdr_cmd  = 0x60010000;
      pcm.comp_id  = activeHost;
      pcm.params_size = 84;
      pcm.buffer.hdr_size = 0;
      pcm.buffer.phy_addr = (UInt32)(sharedDmaBuf->getPhysicalAddress() & 0xFFFFFFFF);
      pcm.buffer.pages = kLatSOF_BufferSize / PAGE_SIZE;
      pcm.buffer.size = kLatSOF_BufferSize;
      pcm.direction = DIR_PLAYBACK;
      pcm.frame_fmt = FRAME_S16;
      pcm.rate = 48000;
      pcm.stream_tag = (UInt16)sTag;
      pcm.channels = 2;
      pcm.sample_valid_bytes = 2;
      pcm.sample_container_bytes = 2;
      pcm.host_period_bytes = 16384;
      pcm.no_stream_position = 1;
      volatile UInt8 *ob = dspBase + outboxOff;
      for (UInt32 i = 0; i < sizeof(pcm); i += 4) wr32(ob, i, *(UInt32*)((UInt8*)&pcm + i));
      wr32(dspBase, IPC_HIPCIDR, IPC_BUSY);
      if (!poll32(dspBase, IPC_HIPCIDA, IPC_DONE, IPC_DONE, 500000)) {
          setProperty("StartPlayback-Error", "PCM_PARAMS timeout");
          // patch-68: was `shutdownDSPGated(); initDSP();` — a full DSP
          // rebuild (3 s of gated work) inside a StartIO error path. The
          // capture path has deferred to the retry engine since patch-27;
          // playback now does exactly the same (scheduleDspRecovery clears
          // hwReady and latches the demand, engine rebuilds on its tick).
          scheduleDspRecovery("playback PCM_PARAMS timeout");
          return kIOReturnTimeout;
      }
      wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
      wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
    }

    // Step 5: Disable SPIB (match Linux default)
    if (spibCap) {
        wr32(hdaBase, spibCap + 0x04, rd32(hdaBase, spibCap + 0x04) & ~(1U << sIdx));
    }

    // Step 6: Start HDA DMA
    wr8(hdaBase, sd + SD_REG_STS, 0x1C);
    wr8(hdaBase, sd, rd8(hdaBase, sd) | SD_CTL_RUN | SD_CTL_IOCE | 0x08 | 0x10);
    IODelay(500);

    // Step 7: TRIG_START IPC
    wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
    wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
    IODelay(100);
    { struct { UInt32 s,c,id; } __attribute__((packed)) m = {12, 0x60040000, activeHost};
      volatile UInt8 *ob = dspBase + outboxOff;
      for (UInt32 i = 0; i < 12; i += 4) wr32(ob, i, *(UInt32*)((UInt8*)&m + i));
      wr32(dspBase, IPC_HIPCIDR, IPC_BUSY);
      if (!poll32(dspBase, IPC_HIPCIDA, IPC_DONE, IPC_DONE, 500000)) {
          setProperty("StartPlayback-Error", "TRIG timeout");
          // patch-68: defer, same as the PCM_PARAMS path above.
          scheduleDspRecovery("playback TRIG_START timeout");
          return kIOReturnTimeout;
      }
      wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
      wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
    }

    // Step 8: RT5682 PLL + HP power (only needed for headphone output)
    if (i2cBase && useHeadphone) {
        IOSleep(50);
        i2cWrite16(0x0081, 0x1481); i2cWrite16(0x0082, 0xC002);
        i2cWrite16(0x0083, 0x3100); // PLL_TRACK_1 (Linux sets this during playback)
        i2cWrite16(0x0080, 0x2000); IOSleep(15);
        i2cWrite16(0x013a, 0x3000); // BCLK control (Linux: 0x2000→0x3000 during playback)
        i2cWrite16(0x006B, 0x8001); i2cWrite16(0x0066, 0x0030);
        i2cWrite16(0x0065, 0x0240); i2cWrite16(0x0063, 0xF2AF); IOSleep(15);
        i2cWrite16(0x0061, 0x8D01); i2cWrite16(0x0062, 0x0400);
        i2cWrite16(0x0064, 0x0008); i2cWrite16(0x002A, 0xA0A0);
        i2cWrite16(0x002B, 0x0311); i2cWrite16(0x0029, 0x8080);
        i2cWrite16(0x0091, 0x0E26); i2cWrite16(0x0003, 0x0000);
        i2cWrite16(0x01DB, 0x0017); i2cWrite16(0x008E, 0x0069);
        i2cWrite16(0x0100, 0xA0A0); i2cWrite16(0x0003, 0x6000);
        i2cWrite16(0x0002, 0x0000); IOSleep(5);
        i2cWrite16(0x0125, 0x0420);
        // Check if MCLK is present (SSP0 active?)
        UInt16 clkDet = i2cRead16(0x006B);
        { char d[32]; snprintf(d, sizeof(d), "CLK=0x%04x", clkDet);
          setProperty("HP-CLK", d); }
    }


    isPlaying = true;
    activePlaybackHost = activeHost;
    setProperty("Status", "Playing"), IOLog("LatSOF: %s\n", "Playing");
    return kIOReturnSuccess;
}

IOReturn LatSOFAudioDevice::stopPlaybackGated() {
    if (!isPlaying) return kIOReturnSuccess;

    // Stop DMA first
    wr8(hdaBase, sd, rd8(hdaBase, sd) & ~(UInt8)(SD_CTL_RUN | SD_CTL_IOCE | 0x08 | 0x10));

    // Helper lambda for sending IPC
    auto sendSimpleIpc = [&](UInt32 cmd) {
        wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
        wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
        IODelay(100);
        struct { UInt32 s,c,id; } __attribute__((packed)) m = {12, cmd, activePlaybackHost};
        volatile UInt8 *ob = dspBase + outboxOff;
        for (UInt32 i = 0; i < 12; i += 4) wr32(ob, i, *(UInt32*)((UInt8*)&m + i));
        wr32(dspBase, IPC_HIPCIDR, IPC_BUSY);
        poll32(dspBase, IPC_HIPCIDA, IPC_DONE, IPC_DONE, 500000);
        wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
        wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
    };

    sendSimpleIpc(0x60050000); // TRIG_STOP
    sendSimpleIpc(0x60030000); // PCM_FREE
    // NOTE: only send ONCE — double PCM_FREE crashes firmware (no guard like Linux ipc3-pcm.c:28)

    // Clear shared buffer
    // Restore RT5682 HP registers to idle state if headphone was active
    if (i2cBase && activePlaybackHost == PIPE1_HOST_ID) {
        i2cWrite16(0x0083, 0x0000); // PLL_TRACK_1: disable
        i2cWrite16(0x013a, 0x2000); // BCLK control: restore default
        i2cWrite16(0x0003, 0x0000); // HP_CTRL_2: clear DAC routing
        i2cWrite16(0x0002, 0x8080); // HP_CTRL_1: mute L+R
        i2cWrite16(0x0061, 0x0000); // PWR_DIG_1: power down
    }

    bzero(sharedDmaBuf->getBytesNoCopy(), kLatSOF_BufferSize);

    isPlaying = false;
    return kIOReturnSuccess;
}

UInt32 LatSOFAudioDevice::getSamplePosition() {
    if (!hwReady) return 0;
    // Use DPIB vendor-specific register (works in decouple mode, unlike LPIB)
    UInt32 dpib = rd32(hdaBase, HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)sIdx);
    return dpib / kLatSOF_BytesPerFrame;  // 4 bytes/frame (2ch S16)
}

IOReturn LatSOFAudioDevice::updateSPIB(UInt32 byteOffset) {
    // Keep SPIB at full buffer size so DMA loops continuously.
    // WriteMix writes data to the ring buffer; DMA reads it all.
    // Setting SPIB to a smaller value causes DMA to stall when it catches up.
    (void)byteOffset;
    return (hwReady && isPlaying) ? kIOReturnSuccess : kIOReturnNotReady;
}

// ==================== Capture Control ====================

IOReturn LatSOFAudioDevice::startCaptureGated() {
    if (!hwReady || !capDmaBuf || !capBdlBuf) {
        // patch-26: a refused start is a DEMAND, not a no-op. The plugin
        // ignores this return, marks IO as running, and only re-issues
        // StartCapture on its next 0->1 client transition — so a StartIO
        // that lands in the not-ready wake window would otherwise be
        // swallowed forever and the mic would stay silent all session.
        // Latch it; the wake retry engine's re-arm serves it after the
        // next successful init. Consumed only after initDSP() returns true, so
        // this cannot start DMA before the hardware is ready — and never
        // at boot, where the latch is only read by the wake path.
        gWasCapturing = true;
        // patch-32: the latch alone is not enough when the retry engine has
        // already GIVEN UP — it would wait for a rebuild that is never
        // coming, and the mic stayed dead until the next sleep (field
        // failure 2 Aug: device-switch wedge, 12 failed rebuilds, then
        // nothing). A refused start is fresh user intent, so treat it like
        // a wake: re-arm the rebuild with fresh budgets. Runs under the
        // gate; bounded — each demand buys one 12-try round, and a user
        // reselecting the mic is exactly who should be able to buy it.
        if (!gWakeReinitPending && hdaBase && dspBase && pciDevice) {
            gWakeReinitPending = true;
            gWakeTries = 0;
            gWakeTickDivider = 0;
            gProgrammedWaits = 0;
            gRecoveryEpisodes = 0;
            gHotRecovery = true;      // session churn context: borrow patiently
            setProperty("Wake-Retry", "re-armed by capture demand");
            IOLog("LatSOF: %s\n",
                  "capture demanded while DSP down — re-arming rebuild");
        }
        return kIOReturnNotReady;
    }
    if (isCapturing) return kIOReturnSuccess;

    UInt32 numBdl = (kLatSOF_CapBufferSize + PAGE_SIZE - 1) / PAGE_SIZE;
    UInt32 paramsErr = 0, trigErr = 0, posnOff = 0;  // LATITUDE FORK: real IPC replies

    // Enable INTCTL for capture stream
    // LATITUDE FORK: poll-only — no stream IRQs. The original enabled
    // INTCTL GIE|CIE|SIE(capIdx) here, but there is no handler and SDSTS is
    // never serviced, so BCIS latched ~94x/sec on the line AppleHDA shares.
    // The kernel throttled it and AppleHDA lost playback. Clear our SIE bit
    // instead of setting it; leave AppleHDA's own bits untouched.
    wr32(hdaBase, HDA_INTCTL, rd32(hdaBase, HDA_INTCTL) & ~(1U << capIdx));

    // LATITUDE FORK: decouple the capture stream (PPCTL bit = global SD
    // index). The init-time PPCTL write ran while capIdx was still 0, so
    // SD1 stayed COUPLED: an input stream waiting on the HDA link for
    // tag-2 data no codec sends — RUN=1, IPCs ack, DPIB pinned at 0.
    // Mirrors startPlaybackGated Step 1b (Linux hda-stream.c:521-523).
    if (ppCap) wr32(hdaBase, ppCap + PP_PPCTL,
                    rd32(hdaBase, ppCap + PP_PPCTL) | (1U << capIdx));

    // Program HDA capture stream (double reset — Linux hda-stream.c:568-591)
    streamReset(hdaBase, capSd);
    streamReset(hdaBase, capSd);
    UInt64 bdlBus = capBdlBuf->getPhysicalAddress();     // patch-37: convert to IODMACommand
    wr32(hdaBase, capSd + SD_REG_BDLPL, (UInt32)(bdlBus & 0xFFFFFFFF));
    wr32(hdaBase, capSd + SD_REG_BDLPU, (UInt32)(bdlBus >> 32));
    wr32(hdaBase, capSd + SD_REG_CBL, kLatSOF_CapBufferSize);
    wr16(hdaBase, capSd + SD_REG_LVI, (UInt16)(numBdl - 1));
    // LATITUDE FORK: CML couple -> write FMT -> decouple quirk, same as
    // startPlaybackGated Step 2f (Linux hda-stream.c:624-637).
    if (ppCap) wr32(hdaBase, ppCap + PP_PPCTL,
                    rd32(hdaBase, ppCap + PP_PPCTL) & ~(1U << capIdx));
    wr16(hdaBase, capSd + SD_REG_FMT, 0x0041); // 48kHz 32-bit 2ch
    if (ppCap) wr32(hdaBase, ppCap + PP_PPCTL,
                    rd32(hdaBase, ppCap + PP_PPCTL) | (1U << capIdx));
    wr8(hdaBase, capSd + 2, (UInt8)((capTag & 0xF) << 4));
    // LATITUDE FORK: poll-only — no stream IRQs. Mask IOCE|FEIE|DEIE here
    // instead of enabling them.
    wr8(hdaBase, capSd, rd8(hdaBase, capSd) & ~(UInt8)(SD_CTL_IOCE | 0x08 | 0x10));

    // PCM_PARAMS for capture
    { struct sof_ipc_pcm_params pcm = {};
      pcm.hdr_size = sizeof(pcm);
      pcm.hdr_cmd  = 0x60010000;
      pcm.comp_id  = PIPE3_HOST_ID;
      pcm.params_size = 84;
      pcm.buffer.hdr_size = 0;
      pcm.buffer.phy_addr = (UInt32)(capDmaBuf->getPhysicalAddress() & 0xFFFFFFFF);
      pcm.buffer.pages = numBdl;
      pcm.buffer.size = kLatSOF_CapBufferSize;
      pcm.direction = DIR_CAPTURE;
      pcm.frame_fmt = FRAME_S32;
      pcm.rate = 48000;
      pcm.stream_tag = (UInt16)capTag;
      pcm.channels = kLatSOF_CapChannels;  // LATITUDE FORK: 2 (see .hpp)
      pcm.sample_valid_bytes = 4;
      pcm.sample_container_bytes = 4;
      pcm.host_period_bytes = kLatSOF_CapBufferSize / 4;  // 4 periods per buffer
      pcm.no_stream_position = 1;  // LATITUDE FORK: we poll DPIB; don't queue unacked posn IPCs
      volatile UInt8 *ob = dspBase + outboxOff;
      for (UInt32 i = 0; i < sizeof(pcm); i += 4) wr32(ob, i, *(UInt32*)((UInt8*)&pcm + i));
      wr32(dspBase, IPC_HIPCIDR, IPC_BUSY);
      if (!poll32(dspBase, IPC_HIPCIDA, IPC_DONE, IPC_DONE, 500000)) {
          // patch-26: re-couple SD1 before bailing. RUN is not set yet on
          // this path, but the stream was decoupled above; leaving the
          // PPCTL bit set contradicted the zero-steady-state-footprint
          // rule and nothing else would ever clear it.
          if (ppCap) wr32(hdaBase, ppCap + PP_PPCTL,
                          rd32(hdaBase, ppCap + PP_PPCTL) & ~(1U << capIdx));
          // Same contract as the not-ready refusal above: the plugin
          // swallows this return, so a timed-out start is a demand too.
          gWasCapturing = true;
          // patch-27: a timed-out PCM_PARAMS means the firmware's PCM
          // state is desynced; nothing short of a rebuild recovers it.
          scheduleDspRecovery("capture PCM_PARAMS timeout");
          return kIOReturnTimeout;
      }
      paramsErr = rd32(ob, 8);           // LATITUDE FORK: sof_ipc_reply.error
      posnOff   = rd32(ob, 12);          // LATITUDE FORK: reply word 3 is actually the comp_id echoed back (35 = DMIC host component), not a position offset
      wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
      wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
    }

    // Disable SPIB for capture stream (Linux: SPIB not needed for capture)
    if (spibCap) {
        wr32(hdaBase, spibCap + 0x04, rd32(hdaBase, spibCap + 0x04) & ~(1U << capIdx));
    }

    // Start DMA
    wr8(hdaBase, capSd + SD_REG_STS, 0x1C);
    // LATITUDE FORK: poll-only — no stream IRQs. Set RUN and nothing else;
    // the interrupt enables stay masked so no completion IRQ is raised.
    wr8(hdaBase, capSd, (rd8(hdaBase, capSd)
                         & ~(UInt8)(SD_CTL_IOCE | 0x08 | 0x10))
                        | SD_CTL_RUN);
    IODelay(500);

    // TRIG_START
    wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
    wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
    IODelay(100);
    { struct { UInt32 s,c,id; } __attribute__((packed)) m = {12, 0x60040000, PIPE3_HOST_ID};
      volatile UInt8 *ob = dspBase + outboxOff;
      for (UInt32 i = 0; i < 12; i += 4) wr32(ob, i, *(UInt32*)((UInt8*)&m + i));
      wr32(dspBase, IPC_HIPCIDR, IPC_BUSY);
      if (!poll32(dspBase, IPC_HIPCIDA, IPC_DONE, IPC_DONE, 500000)) {
          // patch-26: RUN was set a few lines up; returning with it still
          // set left SD1 DMA looping into capDmaBuf with isCapturing ==
          // false — a state stopCaptureGated's !isCapturing early-return
          // could never converge, and a wake re-arm hitting this timeout
          // produced exactly that. Stop the engine and re-couple.
          wr8(hdaBase, capSd, rd8(hdaBase, capSd) & ~(UInt8)SD_CTL_RUN);
          wr8(hdaBase, capSd + SD_REG_STS, 0x1C);
          if (ppCap) wr32(hdaBase, ppCap + PP_PPCTL,
                          rd32(hdaBase, ppCap + PP_PPCTL) & ~(1U << capIdx));
          // Same contract as the not-ready refusal above: the plugin
          // swallows this return, so a timed-out start is a demand too.
          gWasCapturing = true;
          // patch-27: same rationale as the PCM_PARAMS site above.
          scheduleDspRecovery("capture TRIG_START timeout");
          return kIOReturnTimeout;
      }
      trigErr = rd32(ob, 8);             // LATITUDE FORK: sof_ipc_reply.error
      wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
      wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
    }

    // LATITUDE FORK: one-line state snapshot — read it with
    //   ioreg -rc LatSOFAudioDevice -d 1 -w0
    { char dbg[176];
      snprintf(dbg, sizeof(dbg),
          "ppctl=0x%08x ctl=0x%06x sts=0x%02x lpib=%u dpib=%u params=0x%x trig=0x%x comp_id=0x%x",
          ppCap ? rd32(hdaBase, ppCap + PP_PPCTL) : 0xFFFFFFFFu,
          rd32(hdaBase, capSd) & 0xFFFFFF,
          (unsigned)rd8(hdaBase, capSd + SD_REG_STS),
          rd32(hdaBase, capSd + 0x04 /* LPIB */),
          rd32(hdaBase, HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)capIdx),
          paramsErr, trigErr, posnOff);
      setProperty("Capture-Debug", dbg); }

    isCapturing = true;
    gRecoveryEpisodes = 0;   // patch-27b: a working capture refills the budget
    return kIOReturnSuccess;
}

IOReturn LatSOFAudioDevice::stopCaptureGated() {
    // patch-26: an explicit stop is an unambiguous statement of intent, even
    // while hwReady is false. Clearing the wake latch here closes the case
    // where the recording app quits during the retry window: its StopCapture
    // no-ops on isCapturing below, but without this line the stale latch
    // would later re-arm DMA for a session nobody holds.
    // patch-30: one caller is NOT final intent — the family's pauseAudioEngine
    // (via engineStopCapture) promises a resume. That case is covered by the
    // engine-resume latch (engineRequestResume), not by gWasCapturing, so
    // clearing here stays correct for every caller.
    gWasCapturing = false;
    if (!isCapturing) return kIOReturnSuccess;

    // Stop DMA
    wr8(hdaBase, capSd, rd8(hdaBase, capSd) & ~(UInt8)(SD_CTL_RUN | SD_CTL_IOCE | 0x08 | 0x10));
    // LATITUDE FORK: poll-only — no stream IRQs. Clear any latched stream
    // status (BCIS | FIFOE | DESE are write-1-to-clear) and make sure our
    // SIE bit is off, so nothing is left asserting AppleHDA's shared line.
    wr8(hdaBase, capSd + SD_REG_STS, 0x1C);
    wr32(hdaBase, HDA_INTCTL, rd32(hdaBase, HDA_INTCTL) & ~(1U << capIdx));

    // TRIG_STOP + PCM_FREE
    // patch-27: results are no longer ignored. A stop whose IPCs time out
    // leaves the firmware's PCM open — the exact state that made the NEXT
    // start's PCM_PARAMS time out in the field (the "sent twice" firmware
    // behavior the audits flagged as a residual). If either times out,
    // schedule the DSP rebuild now, while nobody wants the mic, instead of
    // leaving the wreck for the next start to trip over.
    bool stopIpcOk = true;
    auto sendCapIpc = [&](UInt32 cmd) {
        wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
        wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
        IODelay(100);
        struct { UInt32 s,c,id; } __attribute__((packed)) m = {12, cmd, PIPE3_HOST_ID};
        volatile UInt8 *ob = dspBase + outboxOff;
        for (UInt32 i = 0; i < 12; i += 4) wr32(ob, i, *(UInt32*)((UInt8*)&m + i));
        wr32(dspBase, IPC_HIPCIDR, IPC_BUSY);
        if (!poll32(dspBase, IPC_HIPCIDA, IPC_DONE, IPC_DONE, 500000))
            stopIpcOk = false;
        wr32(dspBase, IPC_HIPCIDA, rd32(dspBase, IPC_HIPCIDA) | IPC_DONE);
        wr32(dspBase, IPC_HIPCCTL, rd32(dspBase, IPC_HIPCCTL) | 0x02);
    };
    sendCapIpc(0x60050000); // TRIG_STOP
    sendCapIpc(0x60030000); // PCM_FREE
    if (!stopIpcOk) scheduleDspRecovery("capture stop IPC timeout");

    // LATITUDE FORK: post-run snapshot BEFORE the ring is cleared —
    // proves whether audio landed independently of any position register.
    { volatile UInt32 *ring = (volatile UInt32 *)capDmaBuf->getBytesNoCopy();
      char st[160];
      snprintf(st, sizeof(st),
          "dpib=%u lpib=%u sts=0x%02x ring=%08x %08x %08x %08x",
          rd32(hdaBase, HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)capIdx),
          rd32(hdaBase, capSd + 0x04 /* LPIB */),
          (unsigned)rd8(hdaBase, capSd + SD_REG_STS),
          (unsigned)ring[0], (unsigned)ring[1], (unsigned)ring[2], (unsigned)ring[3]);
      setProperty("Capture-Stop", st); }

    // LATITUDE FORK: re-couple SD1 so the steady-state PPCTL footprint
    // is zero again outside a capture run.
    if (ppCap) wr32(hdaBase, ppCap + PP_PPCTL,
                    rd32(hdaBase, ppCap + PP_PPCTL) & ~(1U << capIdx));

    bzero(capDmaBuf->getBytesNoCopy(), kLatSOF_CapBufferSize);
    isCapturing = false;
    return kIOReturnSuccess;
}

UInt32 LatSOFAudioDevice::getCapturePosition() {
    if (!hwReady) return 0;
    UInt32 dpib = rd32(hdaBase, HDA_VS_SDXDPIB_XBASE + HDA_VS_SDXDPIB_XINTERVAL * (UInt32)capIdx);
    return dpib / kLatSOF_CapBytesPerFrame;  // 8 bytes/frame (2ch S32)
}

// ==================== Public wrappers (serialize via commandGate) ====================
//
// Every UserClient-facing entry point and every internally-callable public
// API funnels through commandGate->runAction. Combined with setPowerState
// also running under the gate, this eliminates the PM-vs-userpath races
// that previously caused clamshell-sleep deadlocks (MMIO busy-wait hangs
// when a second thread wrote DSP/HDA registers mid-teardown).
//
// Position queries (getSamplePosition/getCapturePosition) and updateSPIB
// are intentionally NOT gated — they are high-frequency, side-effect-free
// reads (DPIB register / two-bool check) and don't touch DSP state.

IOReturn LatSOFAudioDevice::s_startPlayback(OSObject *o, void *, void *, void *, void *) {
    return static_cast<LatSOFAudioDevice *>(o)->startPlaybackGated();
}
IOReturn LatSOFAudioDevice::s_stopPlayback (OSObject *o, void *, void *, void *, void *) {
    return static_cast<LatSOFAudioDevice *>(o)->stopPlaybackGated();
}
IOReturn LatSOFAudioDevice::s_startCapture (OSObject *o, void *, void *, void *, void *) {
    return static_cast<LatSOFAudioDevice *>(o)->startCaptureGated();
}
IOReturn LatSOFAudioDevice::s_stopCapture  (OSObject *o, void *, void *, void *, void *) {
    return static_cast<LatSOFAudioDevice *>(o)->stopCaptureGated();
}

// LATITUDE FORK patch-25: playback is not implemented on this board and these
// entry points are refused rather than removed — the HAL plugin dispatches by
// selector number, so the numbering must stay exactly as it is.
//
// This is not merely unused code, it is actively hazardous. This fork has no
// I2S codecs, so initI2C() returns early and i2cBase is permanently NULL. That
// forces useHeadphone = false in startPlaybackGated, which selects
// PIPE7_HOST_ID — a pipeline this fork deletes from the topology. PCM_PARAMS to
// a component the firmware has never heard of times out, and the timeout path
// called shutdownDSPGated(), which until patch-25 asserted the global GCTL
// reset. One call to selector 0 from any process would therefore have taken
// AppleHDA's audio down completely.
//
// The plugin is input-only and never calls these; nothing else does either.
IOReturn LatSOFAudioDevice::startPlayback() {
    return kIOReturnUnsupported;
}
IOReturn LatSOFAudioDevice::stopPlayback() {
    return kIOReturnUnsupported;
}
IOReturn LatSOFAudioDevice::startCapture() {
    if (!commandGate) return kIOReturnNotReady;
    return commandGate->runAction(&s_startCapture);
}

// kernel-mic: engine entry points. IOCommandGate::runAction is re-entrant
// safe when already on the gated context (it detects and calls through),
// and it is LOAD-BEARING here, not paranoia: review traced IOAudioFamily
// 600.2 running device PM on a private workloop, so family callbacks are
// not reliably on ours despite the getWorkLoop overrides.
// The demand latch is a HAL-plugin-era mechanism: that client could never
// re-ask, so a refused start had to be remembered and re-armed by jackPoll.
// The family world is the opposite — coreaudiod re-calls
// performAudioEngineStart on client activity by design — and a latched
// re-arm here would start capture DMA with NO running engine attached,
// which nothing would ever stop (review-verified against family sources:
// performAudioEngineStop is only sent to a Running engine). So an
// engine-originated refusal must not leave the latch armed — and the
// refusal and the un-latch MUST be one atomic gated action: round 2 proved
// that as two runActions, jackPoll's tick can win the gate in the gap,
// consume the latch, and start the exact headless DMA this exists to
// prevent, self-re-arming across every later sleep.
IOReturn LatSOFAudioDevice::s_engineStartCapture(OSObject *o, void *, void *, void *, void *) {
    auto *self = static_cast<LatSOFAudioDevice *>(o);
    IOReturn r = self->startCaptureGated();
    if (r != kIOReturnSuccess)
        gWasCapturing = false;   // same gate closure: nothing can interleave
    return r;
}
IOReturn LatSOFAudioDevice::engineStartCapture() {
    if (!commandGate) return kIOReturnNotReady;
    return commandGate->runAction(&s_engineStartCapture);
}
void LatSOFAudioDevice::engineStopCapture() {
    if (commandGate) commandGate->runAction(&s_stopCapture);
}

// patch-30 fix 1: latch only — never a synchronous restart. resumeAudioEngine
// arrives on the family PM path seconds before jackPoll's retry engine has
// rebuilt the DSP; a start issued here would fail NotReady and burn the one
// restart the session gets. jackPoll executes the latch once hwReady is back
// (same shape as the gWasCapturing re-arm, which covers the UserClient world;
// this covers the family world, whose pause cleared isCapturing before the
// sleep latch-OR could see it).
IOReturn LatSOFAudioDevice::s_engineRequestResume(OSObject *o, void *, void *, void *, void *) {
    auto *self = static_cast<LatSOFAudioDevice *>(o);
    gEngineResumePending = true;
    gEngineResumeTries = 0;
    self->setProperty("Engine-Resume", "latched");
    return kIOReturnSuccess;
}
void LatSOFAudioDevice::engineRequestResume() {
    if (commandGate) commandGate->runAction(&s_engineRequestResume);
}

// review 1 Aug round 3: teardown handshake with the kernel audio child. The
// child's engine pointer is unretained and the family frees the engine in
// IOAudioDevice::stop — these gated actions make every later gated reader
// (wake re-latch, jackPoll executor) see nullptr instead of freed memory.
// s_clearKernelAudio additionally covers the owner's own stop(): publishing
// kernelAudio = nullptr under the gate before terminate closes the window
// where a PM setPowerState (deliverable until PMstop) races the teardown.
IOReturn LatSOFAudioDevice::s_kernelAudioTeardown(OSObject *o, void *a0, void *, void *, void *) {
    (void)o;
    auto *d = (LatSOFKernelAudioDevice *)a0;
    gEngineResumePending = false;    // no engine left to resume
    if (d) d->clearEngine();
    return kIOReturnSuccess;
}
void LatSOFAudioDevice::kernelAudioTearingDown(LatSOFKernelAudioDevice *d) {
    if (commandGate) {
        commandGate->runAction(&s_kernelAudioTeardown, d);
    } else {
        gEngineResumePending = false;
        if (d) d->clearEngine();
    }
}
IOReturn LatSOFAudioDevice::s_clearKernelAudio(OSObject *o, void *, void *, void *, void *) {
    static_cast<LatSOFAudioDevice *>(o)->kernelAudio = nullptr;
    return kIOReturnSuccess;
}
IOReturn LatSOFAudioDevice::stopCapture() {
    if (!commandGate) return kIOReturnNotReady;
    return commandGate->runAction(&s_stopCapture);
}

// ==================== IOUserClient ====================

OSDefineMetaClassAndStructors(LatSOFAudioUserClient, IOUserClient)

const IOExternalMethodDispatch LatSOFAudioUserClient::sMethods[kLatSOF_MethodCount] = {
    [kLatSOF_StartPlayback]  = { (IOExternalMethodAction)sStart,      0, 0, 0, 0 },
    [kLatSOF_StopPlayback]   = { (IOExternalMethodAction)sStop,       0, 0, 0, 0 },
    [kLatSOF_GetPosition]    = { (IOExternalMethodAction)sGetPos,     0, 0, 1, 0 },
    [kLatSOF_UpdateSPIB]     = { (IOExternalMethodAction)sUpdateSPIB, 1, 0, 0, 0 },
    [kLatSOF_StartCapture]   = { (IOExternalMethodAction)sStartCap,   0, 0, 0, 0 },
    [kLatSOF_StopCapture]    = { (IOExternalMethodAction)sStopCap,    0, 0, 0, 0 },
    [kLatSOF_GetCapPosition] = { (IOExternalMethodAction)sGetCapPos,  0, 0, 1, 0 },
};

bool LatSOFAudioUserClient::initWithTask(task_t owningTask, void *securityToken, UInt32 type) {
    if (!IOUserClient::initWithTask(owningTask, securityToken, type)) return false;
    clientTask = owningTask; device = nullptr;
    return true;
}

bool LatSOFAudioUserClient::start(IOService *provider) {
    if (!IOUserClient::start(provider)) return false;
    device = OSDynamicCast(LatSOFAudioDevice, provider);
    return device != nullptr;
}

void LatSOFAudioUserClient::stop(IOService *provider) { IOUserClient::stop(provider); }

IOReturn LatSOFAudioUserClient::clientClose() {
    if (device) {
        // Public API — runs under commandGate, safe from races.
        device->stopPlayback();
        device->stopCapture();
    }
    terminate(); return kIOReturnSuccess;
}

IOReturn LatSOFAudioUserClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) {
    if (!device) return kIOReturnBadArgument;
    IOBufferMemoryDescriptor *buf = nullptr;
    if (type == kLatSOF_MemPlayback)    buf = device->getSharedBuffer();
    else if (type == kLatSOF_MemCapture) buf = device->getCaptureBuffer();
    else if (type == kLatSOF_MemFlags)   buf = device->getFlagsBuffer();
    if (!buf) return kIOReturnBadArgument;
    buf->retain();
    *memory = buf;
    *options = 0;
    return kIOReturnSuccess;
}

IOReturn LatSOFAudioUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *arguments,
    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) {
    if (selector >= kLatSOF_MethodCount) return kIOReturnBadArgument;
    return IOUserClient::externalMethod(selector, arguments,
        (IOExternalMethodDispatch *)&sMethods[selector], this, nullptr);
}

IOReturn LatSOFAudioUserClient::sStart(LatSOFAudioUserClient *t, void *r, IOExternalMethodArguments *a) {
    return t->device ? t->device->startPlayback() : kIOReturnNotReady;
}
IOReturn LatSOFAudioUserClient::sStop(LatSOFAudioUserClient *t, void *r, IOExternalMethodArguments *a) {
    return t->device ? t->device->stopPlayback() : kIOReturnNotReady;
}
IOReturn LatSOFAudioUserClient::sGetPos(LatSOFAudioUserClient *t, void *r, IOExternalMethodArguments *a) {
    if (!t->device) return kIOReturnNotReady;
    a->scalarOutput[0] = t->device->getSamplePosition();
    return kIOReturnSuccess;
}
IOReturn LatSOFAudioUserClient::sUpdateSPIB(LatSOFAudioUserClient *t, void *r, IOExternalMethodArguments *a) {
    return t->device ? t->device->updateSPIB((UInt32)a->scalarInput[0]) : kIOReturnNotReady;
}
IOReturn LatSOFAudioUserClient::sStartCap(LatSOFAudioUserClient *t, void *r, IOExternalMethodArguments *a) {
    return t->device ? t->device->startCapture() : kIOReturnNotReady;
}
IOReturn LatSOFAudioUserClient::sStopCap(LatSOFAudioUserClient *t, void *r, IOExternalMethodArguments *a) {
    return t->device ? t->device->stopCapture() : kIOReturnNotReady;
}
IOReturn LatSOFAudioUserClient::sGetCapPos(LatSOFAudioUserClient *t, void *r, IOExternalMethodArguments *a) {
    if (!t->device) return kIOReturnNotReady;
    a->scalarOutput[0] = t->device->getCapturePosition();
    return kIOReturnSuccess;
}
