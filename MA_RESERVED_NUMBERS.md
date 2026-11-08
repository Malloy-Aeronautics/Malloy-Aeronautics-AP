# Numbers MA-Copter has taken from ArduPilot

ArduPilot hands out these numbers itself and keeps adding new ones. If upstream starts using
one we've taken, it won't show up as a merge conflict: the number will just quietly mean two
things. So go through this list every time we merge from upstream ArduPilot, and add to it
whenever we take a new one.

| What | Number | What we use it for | Where | Added in |
| --- | --- | --- | --- | --- |
| Copter parameter, `ParametersG2` index | 60 | `SPD_LIM_` group (ground speed limit) | `ArduCopter/Parameters.cpp` | 4.3.0.17 |
| Parameter prefix | `SPD_LIM_` | ground speed limit | `libraries/AC_SpeedLimit/AC_SpeedLimit.cpp` | 4.3.0.17 |
| RC option, `RCx_OPTION` | 199 | `SPEED_LIMIT_OVERRIDE`, pilot switch for the speed limit | `libraries/RC_Channel/RC_Channel.h` | 4.3.0.17 |
| Mode change reason, `ModeReason` | 51 | `SPEED_LIMIT`, RTL started by the speed limit (shows in the MODE log) | `libraries/AP_Vehicle/ModeReason.h` | 4.3.0.17 |
| Log message | `SPDL` | speed limit, 10 Hz while `SPD_LIM_EN` is set | `ArduCopter/speed_limit.cpp` | 4.3.0.17 |

Inside the `SPD_LIM_` group, index 3 was `SPD_LIM_TIME` in an early version. Don't reuse it.

## If upstream takes one of them

- **Parameter index**: don't just move ours. Every aircraft would lose its saved `SPD_LIM_`
  values. Move it with a parameter conversion, or move upstream's instead.
- **RC option**: the aircraft parameter files set `RCx_OPTION 199`, so a new number means
  updating those files and the release notes as well.
- **Mode reason**: it isn't saved anywhere, only logged, so it can move freely. Tell whoever
  reads the logs.
- **Log message name**: can be renamed, but anything that reads our logs will need to know.
