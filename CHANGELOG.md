# Changelog — Infineon Traveo T2G CYT4DN (Zephyr RTOS Support)

All notable changes to this downstream fork for CYT4DN will be documented in this file.  
This project uses **semantic versioning**: `MAJOR.MINOR.PATCH`.

> **Branches/Tags of interest**
> - Downstream development branch: `dev/ifx/cyt4dn`
> - Release tags: `vX.Y.Z-cyt4dn[-qualifier]` (e.g., `v1.0.0-cyt4dn-initial`)

---

## [v1.0.0-cyt4dn-initial] — _YYYY-MM-DD_
**Status:** Initial Customer Release (downstream)

### Added
- SoC + board enablement for **CYT4DN** on **KIT T2G C2D6M LITE** (`kit_t2g_c2d6m_lite/cyt4dnjbzs/m7_0`).
- Integration of IFX CAT1 drivers: **pinctrl**, **GPIO**, **UART**, **I2C**, **SPI**, **Counter/Timer**, **Watchdog**, **PWM**, **Clock Control**.
- OpenOCD (Infineon/ModusToolbox) flashing support via `west flash`.

### Changed
- N/A (initial release).

### Fixed
- _(add links once issues/PRs are available)_  
  - Pinctrl: _e.g., alternate function mapping correction_ — [#link]  
  - Clock: _e.g., PLL initialization stabilization_ — [#link]

### Known Issues
- **CAN + Cortex-M7 D-cache**: system can hang when D-cache is enabled under specific CAN traffic patterns.  
  **Workaround:** D-cache **disabled** in this release. — [#link-can-cache]

### Notes
- Upstreaming planned in phases (SoC, board, drivers) to Zephyr mainline.
- For all details and build/flash instructions, see `RELEASE.md`.

---

## [Unreleased]
> Placeholder for changes merged to `dev/ifx/cyt4dn` after `v1.0.0-cyt4dn-initial` and before the next tagged release.

### Added
- _TBD_

### Changed
- _TBD_

### Fixed
- _TBD_

### Deprecated / Removed
- _TBD_

### Security
- _TBD_

---

## Historical
> Additional versions will be appended here as they are tagged.
