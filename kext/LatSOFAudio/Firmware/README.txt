Firmware is NOT redistributed in this repository.

Download `sof-cml.ri` (or the firmware file matching your SoC) from the
official Sound Open Firmware binary releases:

    https://github.com/thesofproject/sof-bin/releases

and place it in this directory:

    kext/LatSOFAudio/Firmware/sof-cml.ri

The file is embedded into the kext binary at link time via
`sof_firmware_embed.s` (.incbin), so no runtime file loading is needed.

SOF firmware is dual-licensed BSD-3-Clause / GPL-2.0 by its upstream
authors (Intel / NXP et al.) — see the sof-bin repository for details.
This project embeds it unmodified at build time only.
