# Adapting LatSOF-Audio to other hardware

**Short answer: yes, this is not XiaoXin-13-specific.** The kext was built on
Comet Lake, but every layer was written against the cAVS/SOF architecture,
not against one machine. Here is what transfers, what must change, and how
to tell which of the "five layers" apply to your laptop.

## The one thing that decides everything

Open Linux on the target machine (live USB is fine) and check whether the
DMIC works there via SOF:

```bash
dmesg | grep -i sof      # "sof-audio-pci 0000:00:1f.3: FW ready" etc.
arecord -l               # look for a DMIC capture device
```

- **If Linux+SOF hears the mics**, macOS can too — the hardware path is
  proven; the port is an engineering exercise.
- If Linux uses `snd_hda_intel` only and there's no SOF device, the mics
  hang off the codec, and AppleHDA/AppleALC is the right (and only) place.

## Layer-by-layer applicability

| Layer | Applies to | Notes |
|---|---|---|
| 1. VT-d (DisableIoMapper) | any hackintosh with a DSP master | near-zero cost; required unless you hand-map DMA domains |
| 2. 64-bit BDL addressing | any >4GB-RAM machine using IODMACommand | check `bdlBus == bdlPhys` in logs; on CML Linux also uses >4GB DMA |
| 3. Do not hijack CORB/RIRB while AppleHDA lives | **universal kext law** | never answer codec verbs behind AppleHDA's back; the 58-verb freeze generalizes to any coexistence design |
| 4. Productization (defer + engine + fuses) | any kext doing hardware init at boot | the defer-off-matching-thread and kill-switch patterns are platform-neutral |
| 5. TCC microphone grant | every macOS microphone, always | pure macOS; nothing to do with hardware |

## What must change per SoC

1. **Firmware file.** `sof-cml.ri` (Comet Lake) → `sof-icl.ri` (Ice Lake),
   `sof-tgl.ri` (Tiger Lake) etc. — pick from
   [sof-bin releases](https://github.com/thesofproject/sof-bin/releases)
   and drop into `kext/LatSOFAudio/Firmware/`. No code change (the blob is
   embedded at link time via `.incbin`).
2. **Topology.** The firmware bundle carries the topology (widgets, pipelines,
   DMIC config). The kext parses what the firmware reports; expect widget
   counts to differ from the CML "Topology 8". If the DMIC pipeline differs,
   `tplg_ipc_data.h` constants need review.
3. **PCI identity.** The HDA/DSP controller is matched inside the kext;
   verify the B0/D/F and device ID in `LatSOFAudioDevice.cpp` initHardware
   (CML: `00:1f.3`, device id `0x02C8`; ICL `0x38C8` / TGL `0xA0C8` family —
   confirm against Linux `lspci`).
4. **cAVS generation.** This driver targets cAVS 1.8 (CML). ICL/TGL are
   cAVS 2.x: ROM handshake and IPC formats differ enough to require real
   porting work (start from Linux `sound/soc/sof/intel/` — hda-loader and
   ipc3 drivers are the reference).
5. **Codec side.** If your machine pairs DMIC with a different codec
   (ALC256, ALC3204, ...), nothing changes here — the codec belongs to
   AppleHDA; this kext never touches it (and must never, see Layer 3).

## Checklist for a new target

1. Linux live USB: SOF + DMIC confirmed working?
2. OC: `DisableIoMapper=true`, boot without `hdarst=1` (destructive to
   in-flight descriptors — do not use).
3. Build with the matching firmware blob; check boot log for
   `cl OK rom=0x00000005` (FW_ENTERED), `IPC ROUND-TRIP OK`.
4. Confirm `bdlBus == bdlPhys` in the first load attempt.
5. Verify speakers survive (AppleHDA coexistence), then simultaneous
   playback+capture, then app-level recording (TCC!).
6. Keep a `log stream` sentinel running while validating; budget your
   experiments per boot.

## Upstreaming spirit

If you port this to ICL/TGL, PRs are welcome — the valuable contribution is
a clean per-SoC abstraction of the ROM handshake and IPC layers, currently
CML-specialized inside `LatSOFAudioDevice.cpp`.
