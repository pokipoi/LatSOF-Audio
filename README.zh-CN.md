# LatSOF-Audio — 用 Sound Open Firmware (SOF) 把 Intel 数字麦克风（DMIC）带进 macOS

**中文说明** | [English](README.md)

这是一个纯内核态（in-kernel）实现的 macOS 内核扩展（kext），把联想小新 13（Comet Lake / ALC257 / cAVS 1.8 DSP）上**从未在 macOS 里活过的内置数字麦克风**做成真正的系统默认输入设备——语音备忘录、QuickTime、浏览器，所有 Core Audio 应用直接可用，**开机零操作**。

## 背景：为什么 DMIC 在 macOS 上天然是死的

Comet Lake 之后的 Intel 笔记本，内置麦克风（PDM 数字麦）接在 CPU 内部的 **cAVS DSP** 上，而不是 Realtek 编解码器上。AppleHDA 只会驱动 codec 的模拟路径，**从不初始化 DSP**——所以无论怎么改 AppleALC layout、注入 DeviceProperties，DMIC 都不可能出声。Linux 那边的答案是 Sound Open Firmware（SOF）；本项目把这套能力整个搬进了 macOS kext。

## 五层洋葱（ macOS 上 DMIC 的全部障碍）

1. **VT-d 静默吞掉 DSP 侧 DMA**——DSP master 的 PCIe 取数在 macOS VT-d 域中无映射，被丢弃且**连故障位都不置**。修复：OC `Kernel → Quirks → DisableIoMapper = true`。
2. **BDL 地址被截断**——VT-d 关闭后，`IODMACommand` 的 32 位地址位对 >4GB 物理页强做 <4GB 重映射（`bdlBus ≠ bdlPhys`）→ ROM/loader 读到垃圾描述符。修复：改用 64 位 BDL 寻址（Linux 在 CML 上同样如此）。
3. **永远不要在 AppleHDA 活着时劫持 HDA 命令环**——早期设计让 kext 代答 CORB/RIRB codec 命令（hdaInit=3）。SOF 固件常驻后，这会吞掉 AppleHDA 在飞的 verb（实测 58 条）→ codec 命令通道失同步 → AppleHDA 播放 StartIO 永久死锁在控制器锁上 → **任何 App 出声即全机冻结**。生产档 `hdaInit=0`，完全不碰 AppleHDA 的领地。
4. **产品化**——开机自动装载：重试引擎合成生产参数串、DSP 初始化恒 defer 离开 matching 线程、自动 arm，开机约 3.5 秒麦克风就位，用户零操作。保险丝：boot-arg `latsof-auto=0`（持久）/ `sysctl kern.latsof_lab="auto=0"`（运行时）。
5. **TCC 麦克风权限**——以上全部修好后，App 录不进去还可能是 macOS 隐私权限（TCC）没给。引擎日志里音频在流、App 收不到就是它。TCC 在 **App 启动时**评估：完全退出（Cmd+Q）重开即生效。

完整的取证式排障故事（spindump 死锁铁证、DPIB 只读分水岭、"一次开机 ≈ 8 个实验格"预算制、A/B 开机判定矩阵）见 [docs/JOURNEY.md](docs/JOURNEY.md)。

## 架构

```
                     macOS kernel
┌──────────────────────────────────────────────────────────┐
│  LatSOFAudio.kext                                        │
│  IOAudioEngine (imic) ← LatSOFKernelAudio（默认输入）      │
│        │ 捕获：cycle-wheel DMIC DMA + jackPoll 引擎       │
│        ▼                                                 │
│  LatSOFAudioDevice                                       │
│   ├─ HDA 控制器寄存器（PCI 0x1f.3，cAVS 1.8）              │
│   ├─ SOF 固件：.incbin 内嵌 → CL loader → ROM 握手        │
│   ├─ IPC mailbox（ROUND-TRIP 验证 + topology 解析）        │
│   ├─ HDAS DMA 捕获流（64 位 BDL）                          │
│   ├─ 生产自动装载（defer → ARMED → 引擎）                  │
│   └─ UserClient 实验台（sysctl 实验旋钮）                  │
└──────────────────────────────────────────────────────────┘
        │ PCI / HDAS                    │ I2C（codec 音量）
        ▼                               ▼
   Intel cAVS DSP ── SOF fw (sof-cml.ri) ── DMIC PDM ×2
   （AppleHDA 继续独占 ALC257：扬声器+耳机——完全不受影响）
```

核心设计点：**共存**。扬声器/耳机继续由 AppleHDA 全权负责；本 kext 只驱动 DSP/DMIC 侧，精心共享控制器（电源态感知、借流守卫、空闲时 AFG/I2C 轮询全门控），并留有紧急停机保险丝，坏装载永远拖不垮播放。

## 需求

- cAVS 1.x DSP 且 DMIC 接在其上的 Intel 平台（Comet Lake 已验证；ICL/TGL 需换固件与 topology）
- OpenCore/Clover，`DisableIoMapper=true` 或正确映射的 VT-d
- Command Line Tools（纯 clang 构建，不需要 Xcode 工程）
- 可重建 AuxKC / 关闭内核扩展用户同意（`kmutil create -z` 等）
- 对应 SoC 的 SOF 固件（如 `sof-cml.ri`，从 [sof-bin releases](https://github.com/thesofproject/sof-bin/releases) 下载）

## 构建

```bash
git clone https://github.com/pokipoi/LatSOF-Audio.git
cd LatSOF-Audio/kext
cp ~/Downloads/sof-cml.ri LatSOFAudio/Firmware/
make                # → LatSOFAudio.kext (x86_64)
```

## 安装与验证

```bash
sudo kmutil install --volume / --path-with-history ./LatSOFAudio.kext
sudo kmutil create -z
```

重启即用。检查：

```bash
log show --last 2m --predicate 'eventMessage CONTAINS "LatSOF"'
# 期望：cl OK rom=0x00000005 (FW_ENTERED)、IPC ROUND-TRIP OK、Topology 8 OK
```

引擎会注册为系统默认输入，所有 App 立即可录。

## 文档

- [docs/JOURNEY.md](docs/JOURNEY.md) —— 完整攻坚台账：走过的弯路、抓内核死锁的取证、胜利时刻的数据链
- [docs/ADAPTING.md](docs/ADAPTING.md) —— 移植指南：其他机型 / 其他 SoC / 其他 codec 如何套用，五层洋葱各自适用于谁

## 许可

MIT（见 [LICENSE](LICENSE)）。SOF 固件构建时从 [sof-bin](https://github.com/thesofproject/sof-bin) 获取（Intel/NXP 等，BSD-3-Clause / GPL-2.0 双许可），本仓库不分发。
