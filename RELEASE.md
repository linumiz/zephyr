# Infineon Traveo T2G CYT4DN — Zephyr RTOS Support (Release Notes)

**Release Version:** `v1.0.0` (Initial Customer Release)  
**Release Date:** `_YYYY-MM-DD_`  
**Release Cycle:** `1.0`

> This release delivers initial Zephyr RTOS enablement for the Infineon TRAVEO™ T2G **CYT4DN** SoC family on the **KIT T2G C2D6M LITE** board.  
> Source is currently maintained in a **downstream fork** and is planned for **upstream submission** to the Zephyr Project. See **Source Release** for branch/tag details.

---

## Contents

- [Source Release](#source-release)  
- [What’s Included](#whats-included)  
- [Supported Hardware](#supported-hardware)  
- [Feature Support Matrix](#feature-support-matrix)  
- [Known Issues & Limitations](#known-issues--limitations)  
- [Build & Flash (Linux)](#build--flash-linux)  
- [Build & Flash (Windows)](#build--flash-windows)  
- [Sample Applications](#sample-applications)  
- [Release Notes](#release-notes)  
  - [Added / Changed](#added--changed)  
  - [Issues Fixed in This Release](#issues-fixed-in-this-release)  
  - [Open Issues (Tracked)](#open-issues-tracked)  
- [Upstreaming Plan](#upstreaming-plan)  
- [Automotive Notes](#automotive-notes)  
- [Support & Issue Reporting](#support--issue-reporting)  
- [Licensing](#licensing)  
- [Appendix: Quick Command Reference](#appendix-quick-command-reference)

---

## Source Release

- **Repository (downstream fork):** <https://github.com/linumiz/zephyr>  
- **Development branch:** `dev/ifx/cyt4dn`  
- **Release tag (this drop):** `v1.0.0-cyt4dn-initial`  _(to be created)_  
- **Commit (frozen for this release):** `_commit-hash-here_`

> Subsequent releases will be tagged similarly (e.g., `v1.1.0-cyt4dn`, `v1.2.0-cyt4dn`), with this `RELEASE.md` updated under **Release Notes** each time.

---

## What’s Included

- Zephyr **SoC** + **board** enablement for **CYT4DN** on **KIT T2G C2D6M LITE** (Cortex-M7 core target).  
- Initial set of **Infineon CAT1 peripheral drivers** integration for Zephyr (see matrix below).  
- Board DTS, Kconfig, clocking, pinmux, and essential bring-up.  
- Tooling integration with **OpenOCD (Infineon/ModusToolbox OpenOCD)** for flashing via `west flash`.

> New to Zephyr? Set up your environment first:  
> **Getting Started:** <https://docs.zephyrproject.org/latest/develop/getting_started/index.html>

---

## Supported Hardware

**SoC**  
- **Infineon CYT4DN (CYT4DNJBZS)** — TRAVEO™ T2G 32-bit Automotive MCU (Cortex-M7)

**Development Board**  
- **Infineon TRAVEO KIT T2G C2D6M LITE**

**Zephyr Board Target**  
- `kit_t2g_c2d6m_lite/cyt4dnjbzs/m7_0`

---

## Feature Support Matrix

> _Status meanings:_ ✔ Supported / ⚠ Partial / ⏳ In progress / ❌ Not yet

| Peripheral / IP        | Status | Short Description                                  | Notes / Tracking          |
|------------------------|:------:|----------------------------------------------------|---------------------------|
| Pin Controller         |   ✔    | SOC pinmux & alternate functions                   | `_link: issue-pinctrl_`   |
| GPIO                   |   ✔    | Digital input/output                                | `_link: issue-gpio_`      |
| UART                   |   ✔    | Serial console / comms                              | `_link: issue-uart_`      |
| I2C                    |   ✔    | Master mode                                         | `_link: issue-i2c_`       |
| SPI                    |   ✔    | Master mode                                         | `_link: issue-spi_`       |
| CAN (Classic/FD)       |   ⚠    | Basic bring-up                                      | **See CAN/DCACHE note** • `_link: issue-can_` |
| PWM                    |   ✔    | Timer-based PWM                                     | `_link: issue-pwm_`       |
| Counter/Timer          |   ✔    | General-purpose counters/timers                     | `_link: issue-counter_`   |
| Watchdog               |   ✔    | System watchdog                                     | `_link: issue-wdt_`       |
| Clock Control          |   ✔    | SoC clock tree setup                                | `_link: issue-clock_`     |

> Replace the `_link:` placeholders with your GitHub issues/PRs.

---

## Known Issues & Limitations

### ⚠ Critical: CAN + Cortex-M7 Data Cache
- When **CAN** is used with **Cortex-M7 D-cache enabled**, the CPU/SoC can **hang** under certain traffic patterns.
- **Workaround in this release:** **D-cache is disabled** on the M7 core for the board configuration to ensure CAN stability.  
  - **Do not** re-enable data cache (e.g., via `CONFIG_*CACHE*`) for this target until cache-safe CAN buffer handling and maintenance paths are implemented.
  - Future releases will add proper cache management (buffer alignment, cache maintenance operations, and/or DMA-safe paths).

### Other Known Gaps
- Low-power modes: ⏳  
- Secondary cores / extended multiprocessing (if applicable): ⏳

---

## Build & Flash (Linux)

> Follow Zephyr’s official setup first:  
> 📖 **Getting Started (Linux/macOS)**: <https://docs.zephyrproject.org/latest/develop/getting_started/index.html>  
> Ensure you have: Git, Python3, CMake, Ninja, DTC, and the **Zephyr SDK** installed.

### 1) Enter your Zephyr workspace
```bash
cd ~/zephyrproject/zephyr
```

### 2) Add downstream remote & checkout release branch
```bash
git remote add linumiz https://github.com/linumiz/zephyr.git
git fetch linumiz
git checkout dev/ifx/cyt4dn
# (Optionally) lock to the release tag:
# git checkout v1.0.0-cyt4dn-initial
```

### 3) Build a sample (e.g., blinky_pwm)
```bash
rm -rf build/
west build -p auto \
  -b kit_t2g_c2d6m_lite/cyt4dnjbzs/m7_0 \
  -s samples/basic/blinky_pwm/
```

### 4) Flash using Infineon OpenOCD
Download and extract **Infineon/ModusToolbox OpenOCD** (or use an existing installation):

```bash
wget https://github.com/Infineon/openocd/releases/download/release-v5.11.0/openocd-5.11.0.4042-linux.tar.gz
tar -xzf openocd-5.11.0.4042-linux.tar.gz
```

Flash:
```bash
west flash \
  --openocd /path/to/openocd/openocd/bin/openocd \
  --openocd-search /path/to/openocd/openocd/scripts/
```

> Replace `/path/to/openocd/...` with your extraction path.

---

## Build & Flash (Windows)

> 📖 **Getting Started (Windows)**: <https://docs.zephyrproject.org/latest/develop/getting_started/index.html#windows>  
> Use **PowerShell** (recommended) and install prerequisites per Zephyr docs (Python, Git, CMake, Ninja, DTC, Zephyr SDK).  
> Windows users often prefer **WSL**; if you use WSL, follow the Linux steps inside your WSL distro.

### 1) Enter your Zephyr workspace (PowerShell)
```powershell
cd $env:USERPROFILE\zephyrproject\zephyr
```

### 2) Add downstream remote & checkout release branch
```powershell
git remote add linumiz https://github.com/linumiz/zephyr.git
git fetch linumiz
git checkout dev/ifx/cyt4dn
# Optional: git checkout v1.0.0-cyt4dn-initial
```

### 3) Build a sample
```powershell
rd /s /q build
west build -p auto `
  -b kit_t2g_c2d6m_lite/cyt4dnjbzs/m7_0 `
  -s samples/basic/blinky_pwm/
```

### 4) Flash using Infineon OpenOCD (PowerShell)
- Download the Windows OpenOCD package from Infineon’s releases page and extract to e.g.:  
  `C:\infineon\openocd-5.11.0.4042-windows\`

```powershell
west flash `
  --openocd "C:\infineon\openocd-5.11.0.4042-windows\openocd\bin\openocd.exe" `
  --openocd-search "C:\infineon\openocd-5.11.0.4042-windows\openocd\scripts\"
```

> Ensure your USB debug probe drivers are installed and the board is powered.

---

## Sample Applications

- **Blinky PWM**: `samples/basic/blinky_pwm/`  
- **Hello World**: `samples/hello_world/`  
- **Console UART sanity**: `samples/subsys/console/getchar/` (or similar)

> See Zephyr’s **Samples** catalog: <https://docs.zephyrproject.org/latest/samples/index.html>

---

## Release Notes

### Added / Changed
- Initial enablement for **CYT4DN** on **KIT T2G C2D6M LITE** (M7_0).  
- Integrated IFX CAT1 drivers for pinctrl, GPIO, UART, I2C, SPI, Counter, Watchdog, PWM, Clock.  
- **CAN**: basic bring-up with **M7 D-cache disabled** to avoid hangs.  
- Board DTS/Kconfig/clock and OpenOCD flashing flow via `west flash`.

### Issues Fixed in This Release
> Replace placeholders with links to GitHub issues/PRs closed in this tag.

| ID / Link                 | Area      | Summary (1-line)                       |
|--------------------------|-----------|----------------------------------------|
| `_link-to-issue-or-pr_`  | Pinctrl   | _e.g., Fix alternate function mapping_ |
| `_link-to-issue-or-pr_`  | Clock     | _e.g., Stabilize PLL init_             |
| `_link-to-issue-or-pr_`  | UART      | _e.g., TX IRQ glitch fix_              |
| `_link-to-issue-or-pr_`  | SPI/I2C   | _e.g., Timing config corrections_      |

### Open Issues (Tracked)
> Keep this aligned with the **Known Issues** section above.

| ID / Link                 | Area | Severity | Summary (1-line)                                | Workaround / Note                    |
|--------------------------|------|:--------:|-------------------------------------------------|-------------------------------------|
| `_link-issue-can-cache_` | CAN  |  High    | Hang when M7 D-cache enabled during CAN ops     | D-cache disabled in this release     |
| `_link-issue-lpm_`       | PM   |  Med     | Low-power modes not validated                   | Scheduled for v1.1                   |
| `_link-issue-fdext_`     | CAN  |  Med     | Extended FD filters incomplete                  | In progress                          |

---

## Upstreaming Plan

- Code is staged in `dev/ifx/cyt4dn` and will be submitted to **Zephyr upstream** in segments (SoC, board, drivers).  
- Post-merge, new customer releases will track Zephyr `main` (or designated LTS) plus minimal downstream patches.

---

## Automotive Notes

- This software is **open source** and **not certified** for functional safety by default.  
- Automotive users must perform system-level validation, including: timing analysis, resource usage, worst-case scenarios, EMI/EMC interactions, and safety/security reviews (ISO 26262 / ISO 21434 as applicable).  
- For this release, **M7 D-cache is intentionally disabled** to avoid CAN instability; performance characterization should account for this configuration.

---

## Support & Issue Reporting

- File bugs or enhancement requests in the downstream repo:  
  <https://github.com/linumiz/zephyr/issues>  
  Please include board, commit/tag, sample, tool versions, and steps to reproduce.

- Security issues: report privately via your Linumiz contact or security reporting channel.

---

## Licensing

- Zephyr and integrated components are licensed under their respective open-source licenses (primarily **Apache-2.0** for Zephyr).  
- See individual files and upstream project LICENSE files for details.

---

## Appendix: Quick Command Reference

**Linux build:**
```bash
west build -p auto -b kit_t2g_c2d6m_lite/cyt4dnjbzs/m7_0 -s samples/basic/blinky_pwm/
```

**Linux flash:**
```bash
west flash --openocd /path/to/openocd/openocd/bin/openocd \
           --openocd-search /path/to/openocd/openocd/scripts/
```

**Windows build (PowerShell):**
```powershell
west build -p auto `
  -b kit_t2g_c2d6m_lite/cyt4dnjbzs/m7_0 `
  -s samples/basic/blinky_pwm/
```

**Windows flash (PowerShell):**
```powershell
west flash `
  --openocd "C:\infineon\openocd-5.11.0.4042-windows\openocd\bin\openocd.exe" `
  --openocd-search "C:\infineon\openocd-5.11.0.4042-windows\openocd\scripts\"
```
