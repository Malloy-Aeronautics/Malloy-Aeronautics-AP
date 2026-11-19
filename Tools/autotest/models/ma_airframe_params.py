#!/usr/bin/env python3
"""
Makes an airframe's SITL parameter file from a parameter export of the real aircraft.

  python3 Tools/autotest/models/ma_airframe_params.py T150 ~/Downloads/T150B_MASTER_PARAMS.param
  python3 Tools/autotest/models/ma_airframe_params.py T400 ~/Downloads/T400_005Params_F319.param

writes Tools/autotest/models/<AIRFRAME>.param: the aircraft's own settings (tuning, frame, failsafes,
modes, speeds, EKF, GPS, ...) with everything that belongs to one particular flight controller or
transmitter left out, because SITL has its own:

  - sensor IDs and calibration (INS, compass, baro), board level trim, declination
  - RC calibration (RCn_MIN/MAX/TRIM)
  - counters and statistics (STAT_*, *_TOTAL, FORMAT_VERSION)

The SITL-only settings go on top of it in <AIRFRAME>_sitl.param. To update an airframe, export its
parameters and run this again rather than editing <AIRFRAME>.param by hand.
"""

import argparse
import datetime
import os
import re
import sys

LEFT_OUT = [
    (r"^INS_(ACC|GYR)\d?(_ID|OFFS_[XYZ]|SCAL_[XYZ]|_CALTEMP)$", "IMU IDs and calibration"),
    (r"^COMPASS_(DEV_ID\d?|PRIO\d_ID|OFS\d?_[XYZ]|DIA\d?_[XYZ]|ODI\d?_[XYZ]|MOT\d?_[XYZ]|MOTCT|SCALE\d?|DEC)$",
     "compass IDs and calibration"),
    (r"^BARO\d?_(DEVID|GND_PRESS)$|^BARO_GND_TEMP$", "baro IDs and ground pressure"),
    (r"^AHRS_TRIM_[XYZ]$", "board level trim"),
    (r"^RC\d+_(MIN|MAX|TRIM)$", "RC calibration"),
    (r"^STAT_|_TOTAL$|^FORMAT_VERSION$", "counters"),
]

HERE = os.path.dirname(os.path.abspath(__file__))


def read_params(path):
    params = {}
    with open(path, encoding="utf-8-sig") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = re.split(r"[,\s]+", line)
            if len(fields) < 2:
                sys.exit("%s: can't read line %r" % (path, line))
            params[fields[0]] = fields[1]
    return params


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("airframe", help="airframe name, e.g. T150 (the output is <airframe>.param)")
    ap.add_argument("export", help="parameter file exported from the aircraft")
    ap.add_argument("--out", help="output file (default: %s/<airframe>.param)" % os.path.relpath(HERE))
    args = ap.parse_args()

    params = read_params(args.export)
    kept, dropped = {}, {}
    for name, value in params.items():
        reason = next((r for pattern, r in LEFT_OUT if re.search(pattern, name)), None)
        if reason:
            dropped.setdefault(reason, []).append(name)
        else:
            kept[name] = value

    out = args.out or os.path.join(HERE, args.airframe + ".param")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("# %s parameters for SITL, from %s (%s)\n" % (
            args.airframe, os.path.basename(args.export), datetime.date.today().isoformat()))
        f.write("# Made by ma_airframe_params.py: export the aircraft's parameters and run it again rather\n")
        f.write("# than editing this file. Left out because SITL has its own: %s.\n" % ", ".join(dropped))
        f.write("# SITL-only settings are in %s_sitl.param.\n" % args.airframe)
        for name in sorted(kept):
            f.write("%s,%s\n" % (name, kept[name]))

    print("wrote %s: %d parameters kept, %d left out" % (out, len(kept), sum(len(v) for v in dropped.values())))
    for reason, names in dropped.items():
        print("  %-30s %d" % (reason, len(names)))


if __name__ == "__main__":
    main()
