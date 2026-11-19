# MA-Copter 4.3.0.18

> **Aircraft: T150 only. Do not load this firmware on a T400.**

Release branch `release/MA-4.3.0.18`, made from `MA-4.3.0.X` with 4.3.0.17 in it. Release candidates are tagged `MA-Copter-4.3.0.18-rc.<n>`; the firmware for each is on the [Releases](https://github.com/Malloy-Aeronautics/Malloy-Aeronautics-AP/releases) page.

Latest release candidate: **`MA-Copter-4.3.0.18-rc.4`**.

## What's in it

Everything in 4.3.0.17: `MOT_SPOOL_TIME` up to 20 s (FMSMS-387), the CAN gripper fix, and the CI changes (FMSMS-461, FMSMS-493). On top of that:

### GPS (FMSMS-461, FMSMS-511)

- u-blox moving baseline over a Y-wire (base UART1 to rover UART2) is opt-in through `GPS_DRV_OPTIONS` bit 5, and no longer loops on configuration.
- Leaving Y-wire mode puts the rover's UART2 back to UBX and NMEA input.
- RTCM MSM4 instead of MSM7 for the moving baseline, through `GPS_DRV_OPTIONS` bit 6.
- `GPS_RATE_MS` can be 250 (4 Hz); stock 4.3 doesn't allow slower than 5 Hz. The GPS health checks follow the configured rate.
- The moving baseline base no longer sends RTCM 4072.1, which the rover doesn't use.
- GPS affinity in EKF3 works again: the older accuracy checks in GPS primary selection and in each EKF lane's choice of GPS are removed. This changes behaviour only with `GPS_AUTO_SWITCH` 1 or 4, or with the GPS bit of `EK3_AFFINITY` set.

| `GPS_DRV_OPTIONS` bit | Value | Meaning |
|---|---|---|
| 0 | 1 | Moving baseline over UART2 |
| 5 | 32 | Y-wire: base UART1 to rover UART2 |
| 6 | 64 | MSM4 instead of MSM7 |

Y-wire with MSM4 is bits 5 and 6: `GPS_DRV_OPTIONS` = 96.

### IMU fast fifo reset log flood (FMSMS-520)

After the first IMU fast fifo reset, `IMUx fast fifo reset N` was written to the log on every IMU update for the rest of the flight. It's now written once per reset. Only the logging changes: resets are handled as before and the IMU data is unchanged. Every MA release from 4.3.0.14 had the flood.

## Not in this release

- The ground speed limit (`SPD_LIM_*`) was in rc.2 and taken out again in rc.3. It's in 4.3.0.19.

## Known limitations

- Not evaluated on the T400. The GPS changes above that aren't behind a parameter bit have only been flown on the T150.

## Release candidates

| Release candidate | What changed |
|---|---|
| rc.4 | IMU fast fifo reset log flood fix (FMSMS-520, #75) |
| rc.3 | Rebuilt on `MA-4.3.0.X` with 4.3.0.17 in it, plus the GPS changes. The ground speed limit moved to 4.3.0.19 |
| rc.2 | Ground speed limit (FMSMS-456, #69), taken out again in rc.3 |
| rc.1 | u-blox moving baseline GPS fixes, MSM4 and 4 Hz GPS (FMSMS-461, FMSMS-511, #71) |

To check firmware after loading it: the version reads `MA-Copter-4.3.0.18-rc.<n>`.
