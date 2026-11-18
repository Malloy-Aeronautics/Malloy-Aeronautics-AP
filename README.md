# MA-Copter

<a href="https://ardupilot.org/discord"><img src="https://img.shields.io/discord/674039678562861068.svg" alt="Discord">

[![Test Copter](https://github.com/ArduPilot/ardupilot/workflows/test%20copter/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_copter.yml) [![Test Plane](https://github.com/ArduPilot/ardupilot/workflows/test%20plane/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_plane.yml) [![Test Rover](https://github.com/ArduPilot/ardupilot/workflows/test%20rover/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_rover.yml) [![Test Sub](https://github.com/ArduPilot/ardupilot/workflows/test%20sub/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_sub.yml) [![Test Tracker](https://github.com/ArduPilot/ardupilot/workflows/test%20tracker/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_tracker.yml)

[![Test AP_Periph](https://github.com/ArduPilot/ardupilot/workflows/test%20ap_periph/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_periph.yml) [![Test Chibios](https://github.com/ArduPilot/ardupilot/workflows/test%20chibios/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_chibios.yml) [![Test Linux SBC](https://github.com/ArduPilot/ardupilot/workflows/test%20Linux%20SBC/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_linux_sbc.yml) [![Test Replay](https://github.com/ArduPilot/ardupilot/workflows/test%20replay/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_replay.yml)

[![Test Unit Tests](https://github.com/ArduPilot/ardupilot/workflows/test%20unit%20tests/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_unit_tests.yml) [![test size](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml)

[![Test Environment Setup](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml)

[![Cygwin Build](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml) [![Macos Build](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml)

[![Coverity Scan Build Status](https://scan.coverity.com/projects/5331/badge.svg)](https://scan.coverity.com/projects/ardupilot-ardupilot)

[![Test Coverage](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml)

[![Autotest Status](https://autotest.ardupilot.org/autotest-badge.svg)](https://autotest.ardupilot.org/)

[![MA CI](https://github.com/Malloy-Aeronautics/Malloy-Aeronautics-AP/actions/workflows/ma_ci.yml/badge.svg)](https://github.com/Malloy-Aeronautics/Malloy-Aeronautics-AP/actions/workflows/ma_ci.yml)

MA-Copter is Malloy Aeronautics' flight controller firmware: a fork of [ArduPilot](https://github.com/ArduPilot/ardupilot) Copter 4.3, with the changes our aircraft need on top. Anything not described in a release branch's [RELEASE.md](RELEASE.md) behaves as in ArduPilot Copter 4.3, and the [Copter documentation](https://ardupilot.org/copter/index.html) applies.

## Contents

- [Releases](#releases)
- [Branches and how changes get in](#branches-and-how-changes-get-in)
- [Building](#building)
- [SITL on the T150 model](#sitl-on-the-t150-model)
- [Testing and CI](#testing-and-ci)
- [ArduPilot upstream](#ardupilot-upstream)

## Releases

Each release lives on its own branch, `release/MA-<version>`, and each release candidate is a tag on it, `MA-Copter-<version>-rc.<n>`. The firmware, the release notes and binaries for every release candidate are on the [Releases](https://github.com/Malloy-Aeronautics/Malloy-Aeronautics-AP/releases) page, and the notes for a branch are in its [RELEASE.md](RELEASE.md).

## Branches and how changes get in

| Branch | Use |
|---|---|
| `MA-4.3.0.X` | Main branch |
| `release/MA-<version>` | One per release. Release candidates are tagged here |
| `feature/<name>` | All work, one feature per branch |

- Nothing goes into a release branch without a reviewed pull request, and `ci-passed` has to pass.
- This README is the same on every branch. Change it on `MA-4.3.0.X` and merge it into the release branches; anything specific to one release goes in that branch's `RELEASE.md`.
- Firmware under `build/` (`build/<board>/bin/arducopter.apj`) is tracked for releases. Don't commit local builds of it.

## Building

Set up the build environment as for ArduPilot: [Linux](https://ardupilot.org/dev/docs/building-setup-linux.html), [WSL](https://ardupilot.org/dev/docs/building-setup-windows10_new.html), then:

```bash
git clone --recurse-submodules https://github.com/Malloy-Aeronautics/Malloy-Aeronautics-AP.git
cd Malloy-Aeronautics-AP

# flight controllers
./waf configure --board CubeOrange && ./waf copter
./waf configure --board mRoControlZeroOEMH7 && ./waf copter
# firmware: build/<board>/bin/arducopter.apj

# SITL
./waf configure --board sitl && ./waf copter
```

## SITL on the T150 model

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild -w -L CMAC \
  --model "octa-quad:@ROMFS/models/T150_sim.json" \
  --add-param-file=Tools/autotest/models/T150_sim.param \
  --console --map
```

To fly an aircraft's own parameter file in SITL, add it as another `--add-param-file` after `T150_sim.param`, followed by a SITL-only file that puts back what SITL needs: the `SIM_*`, IMU and compass IDs, battery sensing and `GPS_AUTO_CONFIG 0` from `T150_sim.param`. Never load that SITL-only file onto an aircraft.

## Testing and CI

**MA CI** (`.github/workflows/ma_ci.yml`) runs on pull requests into release branches. `ci-passed` is the one check to require.

**Locally**, before pushing. The same script runs as a git pre-push hook:

```bash
Tools/scripts/ma_local_ci.sh
```

**Unit tests and autotests** for a release's features are in its [RELEASE.md](RELEASE.md).

## ArduPilot upstream

MA-Copter is built on ArduPilot, which is developed by a large community and licensed under the [GNU General Public License, version 3](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt).

- Copter documentation: <https://ardupilot.org/copter/index.html>
- Developer wiki: <https://ardupilot.org/dev/>
- Upstream repository: <https://github.com/ArduPilot/ardupilot>
- Support forum: <https://discuss.ardupilot.org/>
- Upstream maintainers and contributors: [ArduPilot README](https://github.com/ArduPilot/ardupilot#maintainers), [contributors](https://github.com/ArduPilot/ardupilot/graphs/contributors)
