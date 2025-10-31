# Release Process — Infineon Traveo T2G CYT4DN (Zephyr RTOS Support)

This document standardizes how we cut downstream **customer releases** and prepare upstream submissions for the Zephyr Project.

## Versioning

- Use **Semantic Versioning**: `MAJOR.MINOR.PATCH` + suffix `-cyt4dn[-qualifier]` when tagging the downstream fork.  
  Examples: `v1.0.0-cyt4dn-initial`, `v1.1.0-cyt4dn`, `v1.1.1-cyt4dn-hotfix`.
- Increment:
  - **MAJOR** for incompatible changes (e.g., board/SoC naming, DTS/Kconfig breaks).
  - **MINOR** for new features/peripherals, backward-compatible changes.
  - **PATCH** for bug fixes and minor tweaks.

## Pre-Release Checklist

1. **Branch hygiene**
   - Work in `dev/ifx/cyt4dn`.
   - Ensure CI is green (builds for Windows/Linux toolchains if applicable).

2. **Update documentation**
   - `RELEASE.md`: bump version/date, update **Source Release**, **Feature Matrix**, **Known Issues**, **Added/Changed**, **Issues Fixed**, **Open Issues** tables.
   - `CHANGELOG.md`: add a new section for the release.
   - Cross-check that **CAN/DCACHE** note is accurate for this release.

3. **Freeze sources**
   - Determine the release commit: `git rev-parse --short HEAD` ⇒ put in `RELEASE.md`.
   - Optional: create a short `manifest.txt` of key paths changed (SoC, board, drivers).

4. **Build & test matrix**
   - Toolchains: Zephyr SDK (Linux), Windows (native or WSL).  
   - Boards/targets: `kit_t2g_c2d6m_lite/cyt4dnjbzs/m7_0`.
   - Samples: `samples/basic/blinky_pwm/`, `samples/hello_world/`, console samples.  
   - Flash via **Infineon OpenOCD** and validate UART console, GPIO, PWM, I2C, SPI basic loops.
   - **CAN**: verify that D-cache is **disabled** and sanity-check traffic on bus (no hang).

5. **Security & compliance**
   - Review third-party licenses (Zephyr: Apache-2.0; drivers follow upstream licenses).
   - Perform basic static analysis / formatting checks (e.g., `twister -T`, `codespell`, `checkpatch` if applicable).

## Tagging

```bash
# Ensure you are at the release commit
git checkout dev/ifx/cyt4dn
git pull --rebase

# Create and sign the tag
export V_TAG=v1.0.0-cyt4dn-initial   # adjust per release
git tag -s "$V_TAG" -m "CYT4DN Initial Customer Release"

# Push tag
git push origin "$V_TAG"
```

> For subsequent releases, bump `V_TAG` accordingly and ensure `RELEASE.md`/`CHANGELOG.md` are updated first.

## GitHub Release (optional but recommended)

- Create a GitHub Release from the pushed tag:
  - Title: same as the tag (e.g., `v1.0.0-cyt4dn-initial`).
  - Description: paste the **Release Notes** (from `RELEASE.md`: Added/Changed/Fixed/Known Issues, links).
  - Attach any prebuilt artifacts if produced (optional).

## Post-Release

- Update `CHANGELOG.md` by moving any items from **[Unreleased]** to the new version.
- Start a new **[Unreleased]** section for future work.
- If hotfixes are needed:
  - Cherry-pick into `dev/ifx/cyt4dn`.
  - Cut a `vX.Y.Z+1-cyt4dn-hotfix` tag following the same process.

## Upstreaming Workflow

- Split patches logically:
  1. SoC definitions (DTS, Kconfig, clock, pinctrl base).
  2. Board support.
  3. Peripheral drivers (small, reviewable series).
- Follow Zephyr contribution guidelines and CI:
  - Coding style, `twister` builds, bindings & DT validation.
- Track PRs and reference them in `RELEASE.md` for future customer drops.

## Automotive Notes (Carry-Forward)

- This repository is **not** safety-certified. Automotive consumers must validate at system level (ISO 26262, ISO 21434, timing, EMI/EMC).
- **CAN + D-cache**: keep the warning current in each release until resolved; document cache settings explicitly.

## Useful Commands

```bash
# Clean build
rm -rf build/ && west build -p auto \
  -b kit_t2g_c2d6m_lite/cyt4dnjbzs/m7_0 \
  -s samples/basic/blinky_pwm/

# Flash
west flash --openocd /path/to/openocd/openocd/bin/openocd \
           --openocd-search /path/to/openocd/openocd/scripts/

# Generate short log since last tag (helpful for CHANGELOG drafting)
git fetch --tags
LAST_TAG=$(git describe --tags --abbrev=0 --match 'v*-cyt4dn*')
git log --oneline "${LAST_TAG}..HEAD" -- \
  boards/ soc/ drivers/ subsys/ \
  | sed 's/^/- /'
```

## Support & Security

- File issues: <https://github.com/linumiz/zephyr/issues> (include board, tag, toolchain, steps).  
- Security vulnerabilities: report privately via your Linumiz contact or designated security channel.

---

**Maintainers:** _Add names/emails here_  
**Applies to branch:** `dev/ifx/cyt4dn`
