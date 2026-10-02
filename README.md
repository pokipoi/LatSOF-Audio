# LatSOF-Audio — Sound Open Firmware (SOF) DMIC microphone driver for macOS

**Bring Intel Smart Sound Technology (SST / cAVS) digital microphones (DMIC) to macOS — as a real, native IOAudio input device.**

This is a single macOS kernel extension (kext) that implements, entirely in-kernel:

- an HDA controller companion for the Intel cAVS 1.8 DSP (Comet Lake),
- the **Sound Open Firmware (SOF)** boot pipeline: ROM handshake → CL (clean loader) DMA transfer → firmware bring-up → IPC round-trip,
- SOF topology parsing (8 widgets verified) and DMIC capture via the DSP's HDAS DMA engine,
- a native `IOAudioEngine` (`LatSOFKernelAudioEngine`, device name `imic`) that macOS treats as the **default system microphone** — Voice Memos, QuickTime, browsers, and every Core Audio app just work,
- a UserClient "lab console" (sysctl-driven experiment knobs) plus a fully automatic production boot path.

Verified working: **Lenovo XiaoXin 13 (Comet Lake i7-10710U, Realtek ALC257, cAVS 1.8)** on macOS Sequoia 15, OpenCore. The architecture and the debugging methodology generalize to other cAVS platforms (ICL / TGL) — see [docs/ADAPTING.md](docs/ADAPTING.md).

> Why this exists: on Comet Lake laptops, AppleHDA only drives the Realtek codec's analog path. The on-die DSP that owns the PDM digital microphones is **never initialized by macOS at all** — the DMICs are physically wired to the DSP, not to the codec, so no AppleHDA layout patch or DeviceProperties injection can ever bring them up. Linux solved this years ago with SOF; this project ports that capability into a macOS kext.

## The five-layer onion (what actually blocks DMIC on macOS)

Reproducing this on another machine means peeling all five layers, in roughly this order:

1. **VT-d silently kills DSP-side DMA.** The DSP master's PCIe fetches have no mapping in the macOS VT-d domain and are dropped with *zero* error bits set. Fix: OpenCore `Kernel → Quirks → DisableIoMapper = true` (cost ≈ 0 on hackintosh).
2. **BDL address truncation.** With VT-d off, `IODMACommand` with 32-bit address bits force-remaps >4 GB physical pages into a bogus <4 GB bus address (`bdlBus ≠ bdlPhys`) → the ROM/loader reads garbage descriptors. Fix: use 64-bit BDL addressing (`abits=64`), matching what Linux does on CML.
3. **Never hijack the HDA command ring while AppleHDA is alive.** An early design let the kext answer codec verbs on the CORB/RIRB ring ("hdaInit=3"). Once SOF firmware stays resident, this desyncs AppleHDA's codec command channel (58 in-flight verbs swallowed) and AppleHDA's playback StartIO deadlocks forever on a controller lock — full system freeze when any app plays audio. Production mode uses `hdaInit=0` and touches nothing AppleHDA owns.
4. **Productization.** Boot-time auto-load: the retry engine synthesizes a production parameter string, defers DSP init off the matching thread, and arms itself — microphone is ready ~3.5 s after boot with zero user action. Fuses: boot-arg `latsof-auto=0` (persistent) or `sysctl kern.latsof_lab="auto=0"` (runtime).
5. **TCC.** After all of the above, apps still record silence if macOS privacy (TCC) hasn't granted them microphone access. Engine logs are the giveaway: audio flows, app gets nothing. Grant via System Settings, or note that TCC is evaluated **at app launch** — fully quit (Cmd+Q) and relaunch.

The full forensic debugging story — spindump deadlock evidence, the DPIB read-only watershed, the "one boot = 8 experiments" budget, A/B boot matrices — is in [docs/JOURNEY.md](docs/JOURNEY.md).

## Architecture

```
                     macOS kernel
┌──────────────────────────────────────────────────────────┐
│  LatSOFAudio.kext                                        │
│                                                          │
│  IOAudioEngine (imic) ← LatSOFKernelAudio (default dIn)  │
│        │ capture: cycle-wheel DMIC DMA, jackPoll engine  │
│        ▼                                                 │
│  LatSOFAudioDevice                                       │
│   ├─ HDA controller regs (PCI 0x1f.3, cAVS 1.8)          │
│   ├─ SOF firmware: .incbin → CL loader → ROM handshake   │
│   ├─ IPC mailbox (ROUND-TRIP verified, topology parse)   │
│   ├─ HDAS DMA capture streams (64-bit BDL)               │
│   ├─ Product auto-load (defer → ARMED → engine)          │
│   └─ UserClient lab console (sysctl experiment knobs)    │
└──────────────────────────────────────────────────────────┘
        │ PCI / HDAS                    │ I2C (codec vol)
        ▼                               ▼
   Intel cAVS DSP ── SOF fw (sof-cml.ri, not bundled) ── DMIC PDM ×2
   (AppleHDA keeps the ALC257 analog codec: speakers + headset — untouched)
```

Key design point: **coexistence**. AppleHDA keeps full ownership of the codec (speakers/headset). This kext only drives the DSP/DMIC side, shares the controller carefully (power-state awareness, borrowed-stream guards, idle gating of all AFG/I2C polling), and exposes an emergency kill-switch so a bad load can never take playback down with it.

## Requirements

- Intel platform with cAVS 1.x DSP exposing DMIC (Comet Lake verified; ICL/TGL need firmware/topology swap)
- OpenCore (or Clover) with `DisableIoMapper=true` or VT-d correctly mapped
- macOS with Command Line Tools (kext builds with plain clang, no Xcode project needed)
- Ability to rebuild AuxKC / disable Kernel Extension User Consent (`kmutil create -z`, or boot-args per your setup)
- The SOF firmware blob for your SoC, e.g. `sof-cml.ri` from [sof-bin releases](https://github.com/thesofproject/sof-bin/releases)

## Build

```bash
git clone https://github.com/<you>/LatSOF-Audio.git
cd LatSOF-Audio/kext
# drop your firmware in place:
cp ~/Downloads/sof-cml.ri LatSOFAudio/Firmware/
make                # → LatSOFAudio.kext (x86_64)
```

## Install & use

```bash
# after disabling kext consent for this boot / rebuilding AuxKC:
sudo kmutil install --volume / --path-with-history ./LatSOFAudio.kext
sudo kmutil create -z   # rebuild auxiliary kernel collection without consent prompt
```

Reboot. That's it — no per-boot commands. Check:

```bash
log show --last 2m --predicate 'eventMessage CONTAINS "LatSOF"'
# expect: cl OK rom=0x00000005 (FW_ENTERED), IPC ROUND-TRIP OK, Topology 8 OK
```

The engine registers as the default input; any app can record immediately.

## Safety fuses

| Fuse | Scope | Effect |
|---|---|---|
| boot-arg `latsof-auto=0` | next boot | disable production auto-load entirely (kext stays parked) |
| `sudo sysctl -w kern.latsof_lab="auto=0"` | running system | same, without reboot |
| explicit lab string via `kern.latsof_lab` | one shot | override any production default for a single experiment |

## Documentation

- [docs/JOURNEY.md](docs/JOURNEY.md) — the complete debugging campaign: every wrong turn, the forensics that caught a kernel deadlock, and the winning data chain.
- [docs/ADAPTING.md](docs/ADAPTING.md) — porting guide for other laptops/SoCs/codecs, and which of the five layers apply where.

## License

MIT — see [LICENSE](LICENSE).
SOF firmware is fetched at build time from [sof-bin](https://github.com/thesofproject/sof-bin) (BSD-3-Clause / GPL-2.0 by Intel, NXP et al.) and is not redistributed here.

## Keywords / discovery

`hackintosh` `dmic` `digital microphone` `sound open firmware` `sof` `intel smart sound` `cavs` `cml` `comet lake` `alc257` `realtek` `kext` `applehda` `opencore` `ioaudio` `kernel extension` `macos sequoia` `lenovo xiaoxin 13` `hda` `dsp firmware` `ipc` `bdm` `corb rirb`
