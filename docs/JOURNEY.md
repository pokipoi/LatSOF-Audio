# The Debugging Journey — how the DMIC came back from the dead

A sanitized log of the full campaign, kept because the *methodology* is as
portable as the code. Timeline is condensed; "boot" means one reboot cycle.

## Phase 0 — Establishing that the hardware was never the problem

- The DMICs are wired to the on-die cAVS DSP, not to the ALC257 codec.
  No AppleHDA layout can ever see them. This single fact killed 47 earlier
  codec-side patch attempts and redirected everything toward the DSP.
- Ground truth source: the Linux SOF driver (sof/sof-pci-dev, hda codepath)
  and its register traces, used as the reference oracle throughout.

## Phase 1 — Getting *anything* to move

**Observation discipline that made the difference:**

- **One boot ≈ 8 experiment slots.** The DMA engine offers ~7 borrowable
  output streams per boot; each failed experiment can burn one. Experiments
  were pre-planned per boot with a control baseline baked into the t=0
  snapshot (Linux static register value table printed alongside).
- **Persistent forensic logging.** A `log stream` sentinel writing to disk
  survives nothing at freeze time — but whatever it captured before the
  freeze is gold. Every freeze was followed by log slicing, not guessing.
- **Never change two variables between boots.** (And when forced to, run a
  dedicated A/B boot matrix to disentangle them — see Phase 4.)

**The DPIB watershed.** DPIB (DMA Position In Buffer, offset `0x1084+0x20n`)
is *host-read-only* — the DSP-side HDAS DMA pushes it. Writing 0x88000 and
reading back 0 proved host-side initialization was fully exhausted and the
blocker had to be on the DSP master path. That redirected all effort.

**Double-variable trap (the almost-fatal one).** Step 1 disabled VT-d.
That alone made BDL addresses *worse* (the 32-bit remap only made sense
with VT-d present). The first post-fix test was therefore invalid —
concluding "VT-d is irrelevant" there would have buried the case forever.
Both fixes landed together, then everything lit up:

```
bdlBus=0x2e5fd0000 == bdlPhys        (64-bit: identity mapping)
cl OK rom=0x00000005 (FW_ENTERED)
dpib=557056                          (full 557 KB firmware in the DSP)
Status=OK / FW-Version=2.2.0 ABI:3.22.1 / IPC ROUND-TRIP OK / Topology 8 OK 0 FAIL
capture RMS: 0.011 (noise floor) → 0.11 peak=1.0 (speech level)
```

## Phase 2 — Kernel deadlock forensics (the freeze)

First production freeze, triggered by Voice Memos. Evidence chain:

1. Kernel log slices: playback path hung after firmware went resident.
2. `spindump` under load (background `afplay` to reproduce the scene):
   AppleHDA output RT thread (priority 97) blocked 100/101 samples inside
   `AppleHDAController`'s internal mutex; the work-loop thread idle —
   the lock holder was invisible. `kIOReturnCannotLock (0xE00002BC)`
   reproducible at will.
3. Root cause: the kext's CORB/RIRB hijack ("hdaInit=3", a legacy from the
   frozen-ROM era) swallowed **58 in-flight AppleHDA verbs** → codec command
   channel desync → StartIO waits forever on the controller lock.
4. Defensive patch set: idle-gating of all AFG ICI/I2C polling, initDSP
   re-entry guard, playback error paths deferred to the recovery scheduler.
5. A/B boot matrix isolated the poison: with auto-load parked, speakers
   were fine (baseline: kext-touches-hardware = AppleHDA dies). With
   `hdaInit=0`, load succeeded, speakers survived, simultaneous
   playback+capture worked. Production default changed to `hdaInit=0`.
   The hijack was only ever needed because firmware never stayed resident
   in the frozen-ROM era; once it does, it's pure poison.

## Phase 3 — Productization (removing the human)

The lab era required a per-boot `sysctl` incantation. Productization:

- production parameter string synthesized by the retry engine,
- DSP init always deferred off the matching thread,
- auto mode doesn't consume generation counters (authorization = the
  engine's 12-attempt budget + pending flag),
- boot-arg baseline separated from sysctl keys (a real bug where every
  engine tick reset the boot-arg was found and fixed by A/B),
- fuses: `latsof-auto=0` (boot) / `auto=0` (runtime).

Verified: mic ready ~3.5 s after boot, zero user action, speakers untouched.

## Phase 4 — The anticlimax that wasn't

Voice Memos showed level-meter activity but recorded nothing. Engine logs
showed audio flowing and the default device correctly selected — the app
was being blocked by **TCC microphone privacy**, evaluated at app launch.
Granted the entitlement, fully quit and relaunched the app: recordings.
Case closed.

## Lessons that transfer to any project like this

1. Read-only registers are your watershed: prove which side of the fence
   the blocker lives on before writing more code.
2. Beware the double-variable trap; if forced to change two things, plan
   the A/B matrix before rebooting, not after.
3. Budget experiments (hardware gives you N shots per boot) — plan per boot.
4. Print the Linux ground-truth values into your own t=0 snapshot; a
   10-second diff catches what hours of reasoning miss.
5. A freeze with no forensic trail is an anecdote; a freeze with log
   slicing + spindump is a root cause. Always have the sentinel running.
