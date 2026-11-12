#!/usr/bin/env python3
"""
Runs the speed limiter unit tests and SITL autotests and writes a report of what each one
checked, measured and logged, with a plot of every flight. The autotests fly the T150. Run from the repo root with SITL built:

    Tools/scripts/ma_test_evidence.py                        # everything, at 5 m/s
    Tools/scripts/ma_test_evidence.py --tests SpeedLimitLoiter
    Tools/scripts/ma_test_evidence.py --skip-sitl            # unit tests only
    Tools/scripts/ma_test_evidence.py --limits 5 17 25
    Tools/scripts/ma_test_evidence.py --report-only evidence/<folder>
    Tools/scripts/ma_test_evidence.py --merge evidence/all evidence/run1 evidence/run2

Output goes in evidence/<date>-<commit>/: report.html, report.md, results.json, and the logs
of every test under unit/ and autotest/. Exits non-zero if anything failed.
"""

import argparse
import datetime
import glob
import html
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ARDUCOPTER = "Tools/autotest/arducopter.py"
AUTOTEST_DIR = "Tools/autotest"
UNIT_BINARIES = ("build/sitl/tests/test_speed_limiter", "build/sitl/tests/test_speed_limit")

# which console lines to keep in the report
MEASUREMENT_HINTS = ("true speed", "m/s", "past it", "course change", "heading change", "pitch rate",
                     "tilt", "EKF lost", "overshoot", "RTL after", "speeded up", "Exception caught",
                     "MISSION ", "SWEEP ")
NOISE_HINTS = ("Received wanted", "Sending set", " want=", " is now ", "PARAM_VALUE", "statustext=",
               "Waiting for", "Got: ", " AP: ", "Drained", "Set RC", "mavproxy", "Wait groundspeed",
               "groundspeed ", "SIM_STATE", "set_message_rate", "Arming", "Takeoff", "MAV_CMD", "MISSION_ITEM")
MAX_MEASUREMENTS = 40

COPTER_MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL",
                7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
                17: "BRAKE", 18: "THROW", 21: "SMART_RTL", 23: "FOLLOW", 24: "ZIGZAG", 27: "AUTO_RTL"}

LIMIT_PARAMS = ("ATC_SPD_LIM_VAL", "SPD_LIM_VAL")
ENABLE_PARAMS = ("ATC_SPD_LIM_EN", "SPD_LIM_EN")


def run(cmd, **kwargs):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, **kwargs)


def git(*args):
    out = run(["git"] + list(args))
    return out.stdout.strip() if out.returncode == 0 else ""


# -- what's being tested ----------------------------------------------------------------------

def autotest_names(source, pattern):
    '''SpeedLimit test methods and their docstrings, in file order'''
    tests = []
    for m in re.finditer(r"^    def (%s\w*)\(self\):\n        '''(.*?)'''" % pattern, source, re.M | re.S):
        tests.append((m.group(1), " ".join(m.group(2).split())))
    return tests


def ma_group(source):
    '''names listed in testsMA, or None if the file has no testsMA list'''
    m = re.search(r"    def testsMA\(self\):.*?\n        ret = \(\[\n(.*?)        \]\)", source, re.S)
    if m is None:
        return None
    return re.findall(r"self\.(\w+)", m.group(1))


def readable(name):
    '''NoOvershootAfterLongHover -> "No overshoot after long hover"'''
    words = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", " ", name.replace("DISABLED_", "")).split()
    return " ".join([words[0]] + [w.lower() if not w.isupper() else w for w in words[1:]]) if words else name


def unit_descriptions(binary):
    '''a description for each TEST(), from the comment above it or its name'''
    name = os.path.basename(binary)
    sources = glob.glob("libraries/*/tests/%s.cpp" % name)
    if not sources:
        return {}
    lines = open(sources[0]).read().splitlines()
    found = {}
    for i, line in enumerate(lines):
        m = re.match(r"TEST\((\w+),\s*(\w+)\)", line)
        if m is None:
            continue
        above = []
        j = i - 1
        while j >= 0 and lines[j].strip().startswith("//"):
            above.insert(0, lines[j].strip()[2:].strip())
            j -= 1
        inside = []
        for body in lines[i + 1:]:
            s = body.strip()
            if s in ("{", ""):
                continue
            if s.startswith("//"):
                inside.append(s[2:].strip())
                continue
            break
        if above:
            text = " ".join(above)
        else:
            text = readable(m.group(2)) + (": " + " ".join(inside) if inside else ".")
        found["%s.%s" % (m.group(1), m.group(2))] = text
    return found


def constants(source):
    """the pass bands the SITL tests use, with what they mean. A comment that's only a unit
    ("# s") takes its meaning from the comment line above"""
    found = {}
    lines = source.splitlines()
    for i, line in enumerate(lines):
        m = re.match(r"^    ((?:SPEED|FENCE|STICK)_[A-Z_]+) = ([0-9.]+|None)\s*(?:#\s*(.*))?$", line)
        if m is None:
            continue
        note, unit = (m.group(3) or "").strip(), ""
        if note and len(note.split()) == 1 and len(note) <= 4:
            unit, note = note, ""
        if not note and i > 0 and lines[i - 1].strip().startswith("#"):
            note = lines[i - 1].strip().lstrip("#").strip()
        found[m.group(1)] = {"value": m.group(2), "unit": unit, "note": note}
    return found


# -- running ----------------------------------------------------------------------------------

def run_unit_tests(binaries, out_dir):
    results = []
    os.makedirs(out_dir, exist_ok=True)
    for binary in binaries:
        name = os.path.basename(binary)
        xml_path = os.path.join(out_dir, name + ".xml")
        start = time.time()
        res = run([binary, "--gtest_output=xml:" + xml_path, "--gtest_also_run_disabled_tests=0"])
        open(os.path.join(out_dir, name + ".txt"), "w").write(res.stdout)
        descriptions = unit_descriptions(binary)
        cases = []
        if os.path.exists(xml_path):
            for case in ET.parse(xml_path).getroot().iter("testcase"):
                full = "%s.%s" % (case.get("classname"), case.get("name"))
                failures = [(f.get("message") or "").strip().splitlines()[:1] for f in case.findall("failure")]
                status = case.get("status", "run")
                cases.append({
                    "name": full,
                    "result": "DISABLED" if status == "notrun" else ("FAIL" if failures else "PASS"),
                    "time_s": float(case.get("time", 0)),
                    "description": descriptions.get(full, ""),
                    "failure": failures[0][0] if failures and failures[0] else "",
                })
        results.append({"binary": name, "exit": res.returncode, "seconds": round(time.time() - start, 1),
                        "cases": cases})
        print("unit %-28s %s (%d tests)" % (name, "ok" if res.returncode == 0 else "FAILED", len(cases)))
    return results


def clear_autotest_logs():
    for f in glob.glob(os.path.join(AUTOTEST_DIR, "logs", "*.BIN")) + \
            glob.glob(os.path.join(AUTOTEST_DIR, "logs", "LASTLOG.TXT")) + \
            glob.glob(os.path.join(AUTOTEST_DIR, "autotest-*.tlog")):
        os.remove(f)


def run_autotest(name, out_dir, timeout, buildlogs, extra_env=None):
    os.makedirs(out_dir, exist_ok=True)
    clear_autotest_logs()
    env = dict(os.environ, BUILDLOGS=buildlogs)
    env.update(extra_env or {})
    start = time.time()
    try:
        res = run([sys.executable, "autotest.py", "--no-clean", "test.Copter." + name],
                  cwd=AUTOTEST_DIR, env=env, timeout=timeout)
        output, timed_out = res.stdout, False
    except subprocess.TimeoutExpired as e:
        output, timed_out = (e.stdout or "") if isinstance(e.stdout, str) else "", True
    seconds = round(time.time() - start, 1)
    open(os.path.join(out_dir, "output.txt"), "w").write(output)

    passed = ('PASSED: "%s' % name) in output and ('FAILED: "%s' % name) not in output and not timed_out
    reason = ""
    if not passed:
        m = re.search(r'FAILED: "%s[^"]*": (.*)' % name, output)
        last = [ln.strip() for ln in output.splitlines() if ln.strip()]
        reason = ("timed out after %ds" % timeout if timed_out else
                  m.group(1).strip() if m else
                  "autotest didn't run: %s" % last[-1] if last else "no output, see output.txt")

    # failing runs move their logs to BUILDLOGS
    logs = []
    for f in sorted(glob.glob(os.path.join(AUTOTEST_DIR, "logs", "*.BIN")) +
                    glob.glob(os.path.join(AUTOTEST_DIR, "autotest-*.tlog")) +
                    glob.glob(os.path.join(buildlogs, "*-%s-*" % name)) +
                    glob.glob(os.path.join(buildlogs, "*-%s.txt" % name))):
        dest = os.path.join(out_dir, os.path.basename(f))
        shutil.move(f, dest)
        logs.append(os.path.basename(dest))

    return {"result": "PASS" if passed else "FAIL", "seconds": seconds, "reason": reason,
            "measurements": measurements(output, name), "logs": logs,
            "sweep": read_sweep([os.path.join(out_dir, f) for f in logs if f.endswith("sweep.csv")])}


def read_sweep(paths):
    '''rows of the gain sweep CSV'''
    rows = []
    for path in paths:
        lines = open(path).read().splitlines()
        if not lines:
            continue
        keys = lines[0].split(",")
        for line in lines[1:]:
            rows.append(dict(zip(keys, line.split(","))))
    return rows


def measurements(output, name):
    lines = []
    started = False
    for line in output.splitlines():
        if ("########## %s" % name) in line:
            started = True
            continue
        if not started:
            continue
        if ('PASSED: "%s' % name) in line or ('FAILED: "%s' % name) in line:
            break
        text = re.sub(r"^AT-[0-9.]+:\s*", "", line).strip()
        if text.startswith("AP: "):
            continue                    # autopilot messages, not what the test measured
        if not text or not any(h in text for h in MEASUREMENT_HINTS) or any(h in text for h in NOISE_HINTS):
            continue
        if text not in lines:
            lines.append(text)
    return lines[:MAX_MEASUREMENTS]


# -- plots from the flight log ----------------------------------------------------------------

def flight_series(bin_path):
    """speeds, limit, modes, override switch and the speed limit's own SPDL log from a dataflash
    log"""
    try:
        from pymavlink import DFReader
    except ImportError:
        return None
    try:
        log = DFReader.DFReader_binary(bin_path)
    except Exception:
        return None
    true_speed, ekf_speed, modes, params, override, spdl = [], [], [], {}, [], []
    override_channel = None
    while True:
        m = log.recv_match(type=["SIM2", "XKF1", "MODE", "PARM", "RCIN", "SPDL"])
        if m is None:
            break
        t = m.get_type()
        if t == "SIM2":
            true_speed.append((m.TimeUS * 1e-6, math.hypot(m.VN, m.VE)))
        elif t == "XKF1" and getattr(m, "C", 0) == 0:
            ekf_speed.append((m.TimeUS * 1e-6, math.hypot(m.VN, m.VE)))
        elif t == "MODE":
            modes.append((m.TimeUS * 1e-6, COPTER_MODES.get(m.ModeNum, str(m.ModeNum)), getattr(m, "Rsn", None)))
        elif t == "SPDL":
            spdl.append({"t": m.TimeUS * 1e-6, "spd": m.Spd, "lim": m.Lim,
                         "chk": bool(getattr(m, "Chk", getattr(m, "App", 0))), "on": bool(getattr(m, "On", 1)),
                         "trig": bool(getattr(m, "Trig", 0)), "ntc": int(getattr(m, "Ntc", 0))})
        elif t == "PARM":
            if m.Name in LIMIT_PARAMS + ENABLE_PARAMS:
                params.setdefault(m.Name, []).append((m.TimeUS * 1e-6, m.Value))
            found = re.match(r"RC(\d+)_OPTION$", m.Name)
            if found and int(m.Value) == 199:
                override_channel = int(found.group(1))
        elif t == "RCIN" and override_channel is not None:
            on = getattr(m, "C%d" % override_channel, 0) > 1800
            if not override or override[-1][1] != on:
                override.append((m.TimeUS * 1e-6, on))
    if not true_speed:
        return None
    return {"true": true_speed, "ekf": ekf_speed, "modes": [(t, name) for t, name, _ in modes],
            "mode_reasons": modes, "params": params, "override": override, "spdl": spdl}


SPEED_LIMIT_REASON = 51         # ModeReason::SPEED_LIMIT, in the MODE log message


def key_numbers(series):
    """what the speed limit did in this flight, from its own SPDL log (10 Hz, so times are to the
    nearest 0.1 s)"""
    spdl = series.get("spdl") or []
    key = {"max_true_mps": round(max(v for _, v in series["true"]), 2)}
    if not spdl:
        return key
    requests = [s for s in spdl if s["trig"]]
    key["rtl_requests"] = len(requests)
    key["warnings"] = sum(1 for s in spdl if s["ntc"] == 1)
    key["alerts"] = sum(1 for s in spdl if s["ntc"] == 2 and not s["trig"])
    key["gps_heads_up"] = sum(1 for s in spdl if s["ntc"] == 3)

    # time it was enabled but not live, once it had first gone live (so not the wait for GPS
    # after boot): GPS lost, waiting for it to come back, or the pilot switch
    inactive, live_once = 0.0, False
    for a, b in zip(spdl, spdl[1:]):
        live_once = live_once or a["on"]
        if live_once and not a["on"]:
            inactive += b["t"] - a["t"]
    key["inactive_s"] = round(inactive, 1)

    if requests:
        first = requests[0]
        i = spdl.index(first)
        j = i
        while j > 0 and spdl[j - 1]["chk"] and spdl[j - 1]["on"] and spdl[j - 1]["spd"] > spdl[j - 1]["lim"]:
            j -= 1
        key["limit_mps"] = round(first["lim"], 2)
        key["speed_at_rtl_mps"] = round(first["spd"], 2)
        key["reaction_s"] = round(first["t"] - spdl[j]["t"], 2)
        rtl = next((t for t, name, rsn in series["mode_reasons"]
                    if name == "RTL" and rsn == SPEED_LIMIT_REASON and t >= first["t"] - 0.05), None)
        key["in_rtl_s"] = round(rtl - first["t"], 2) if rtl is not None else None
        after = [(t, v) for t, v in series["true"] if first["t"] <= t <= first["t"] + 30]
        if after:
            peak_t, peak_v = max(after, key=lambda p: p[1])
            key["peak_after_mps"] = round(peak_v, 2)
            key["peak_after_s"] = round(peak_t - first["t"], 1)
            back = next((t for t, v in after if t >= peak_t and v <= first["lim"]), None)
            key["back_under_s"] = round(back - first["t"], 1) if back is not None else None
    else:
        checked = [s for s in spdl if s["chk"] and s["on"]]
        if checked:
            top = max(checked, key=lambda s: s["spd"])
            key["max_checked_mps"] = round(top["spd"], 2)
            key["limit_mps"] = round(top["lim"], 2)
    return key


def reaction_text(seconds):
    """the SPDL log is 10 Hz: a request on the first update over the limit is within 0.1 s"""
    return "under 0.1 s" if seconds < 0.05 else "%.1f s" % seconds


def key_summary(key):
    """a few words for the results table"""
    if key.get("rtl_requests"):
        text = "sent home in %s" % reaction_text(key["reaction_s"])
        if key["rtl_requests"] > 1:
            text += ", %d times" % key["rtl_requests"]
        return text
    if "max_checked_mps" in key:
        return "not sent home, checked up to %.1f of %.1f m/s" % (key["max_checked_mps"], key["limit_mps"])
    if "rtl_requests" in key:
        return "not sent home, flew up to %.1f m/s" % key["max_true_mps"]
    return "limit off, flew up to %.1f m/s" % key["max_true_mps"]


def key_chips(key):
    """label and value pairs for a test's key numbers"""
    chips = []
    if key.get("rtl_requests"):
        chips.append(("Over the limit to RTL", reaction_text(key["reaction_s"])))
        chips.append(("EKF speed when sent home", "%.2f m/s, limit %.2f" % (key["speed_at_rtl_mps"], key["limit_mps"])))
        if key.get("peak_after_mps") is not None:
            peak = "%.2f m/s, %.1f s later" % (key["peak_after_mps"], key["peak_after_s"])
            chips.append(("Peak after the RTL request", peak))
        if key.get("back_under_s") is not None:
            chips.append(("Back under the limit", "%.1f s after the request" % key["back_under_s"]))
        if key["rtl_requests"] > 1:
            chips.append(("RTL requests", "%d" % key["rtl_requests"]))
    elif "rtl_requests" in key:
        chips.append(("RTL requests from the limit", "none"))
        if "max_checked_mps" in key:
            chips.append(("Fastest while checked", "%.2f m/s, limit %.2f" % (key["max_checked_mps"], key["limit_mps"])))
    if "rtl_requests" not in key:
        chips.append(("Speed limit", "not enabled in this flight"))
    chips.append(("Top true speed", "%.2f m/s" % key["max_true_mps"]))
    if key.get("inactive_s"):
        chips.append(("Limit inactive in flight", "%.1f s" % key["inactive_s"]))
    told = []
    if key.get("warnings"):
        told.append("%d warning%s" % (key["warnings"], "" if key["warnings"] == 1 else "s"))
    if key.get("alerts"):
        told.append("%d over-limit alert%s" % (key["alerts"], "" if key["alerts"] == 1 else "s"))
    if key.get("gps_heads_up"):
        told.append("%d GPS heads-up" % key["gps_heads_up"])
    if told:
        chips.append(("Pilot was told", ", ".join(told)))
    return chips


def stability(bin_path, series):
    '''how steadily it flew. Pitch tracking is left out, the logged target is the pilot's before
    the limiter touches it'''
    try:
        from pymavlink import DFReader
        log = DFReader.DFReader_binary(bin_path)
    except Exception:
        return None
    roll_err, yaw_err, tilt, rates, alts = [], [], 0.0, [0.0, 0.0, 0.0], []
    while True:
        m = log.recv_match(type=["ATT", "RATE", "CTUN"])
        if m is None:
            break
        t = m.get_type()
        if t == "ATT":
            roll_err.append(abs(m.DesRoll - m.Roll))
            yaw_err.append(abs((m.DesYaw - m.Yaw + 180) % 360 - 180))
            tilt = max(tilt, abs(m.Roll), abs(m.Pitch))
        elif t == "RATE":
            rates = [max(rates[0], abs(m.R)), max(rates[1], abs(m.P)), max(rates[2], abs(m.Y))]
        elif t == "CTUN":
            alts.append(m.Alt)
    flying = [a for a in alts if a > 5]
    steps = limit_steps(series)
    switch = series.get("override", [])
    # overshoot only counts once it has got down to the limit, after the limit changes or the
    # override is released, and never while the override is on
    over, near = None, []
    lim, armed_check, si, oi, override_on = None, False, 0, 0, False
    for t, v in series["true"]:
        while si < len(steps) and steps[si][0] <= t:
            if steps[si][1] != lim:
                lim, armed_check = steps[si][1], False
            si += 1
        while oi < len(switch) and switch[oi][0] <= t:
            if override_on and not switch[oi][1]:
                armed_check = False
            override_on = switch[oi][1]
            oi += 1
        if lim is None or lim <= 0 or override_on:
            continue
        if v <= lim:
            armed_check = True
        if armed_check:
            over = v - lim if over is None else max(over, v - lim)
        if 0.8 * lim <= v <= 1.2 * lim:
            near.append(v - lim)
    swing = None
    if len(near) > 10:
        mean = sum(near) / len(near)
        swing = math.sqrt(sum((x - mean) ** 2 for x in near) / len(near))
    rms = math.sqrt(sum(x * x for x in roll_err) / len(roll_err)) if roll_err else None
    return {
        "max_tilt_deg": round(tilt, 1),
        "roll_err_max_deg": round(max(roll_err), 1) if roll_err else None,
        "roll_err_rms_deg": round(rms, 2) if rms is not None else None,
        "yaw_err_max_deg": round(max(yaw_err), 1) if yaw_err else None,
        "max_rate_dps": [round(r) for r in rates],
        "alt_band_m": [round(min(flying), 1), round(max(flying), 1)] if flying else None,
        "worst_over_limit_mps": round(over, 2) if over is not None else None,
        "speed_swing_rms_mps": round(swing, 2) if swing is not None else None,
    }


def limit_steps(series):
    '''the speed limit over time, only while it was enabled (None while off)'''
    params = series["params"]
    limit = next((params[p] for p in LIMIT_PARAMS if p in params), [])
    enable = next((params[p] for p in ENABLE_PARAMS if p in params), [])
    events = sorted([(t, "val", v) for t, v in limit] + [(t, "en", v) for t, v in enable])
    steps, val, en = [], None, 1
    for t, kind, v in events:
        if kind == "val":
            val = v
        else:
            en = v
        steps.append((t, val if en else None))
    return steps


MODE_COLOURS = {"ALT_HOLD": "#e6effb", "STABILIZE": "#e9f5e6", "RTL": "#fdebe1", "LOITER": "#efe9fa",
                "POSHOLD": "#e6f4f4", "AUTO": "#f6f0dc", "GUIDED": "#f6f0dc", "LAND": "#eceef2",
                "BRAKE": "#f3e6ef"}
_plot_ids = [0]


def svg_plot(series, width=800, height=270):
    """true and EKF speed against the limit, on the modes it flew in, with where the limit wasn't
    live, the RTL requests and the messages the pilot got"""
    _plot_ids[0] += 1
    pid = "p%d" % _plot_ids[0]
    moving = [t for t, v in series["true"] if v > 0.3]
    if moving:
        t0, t_end = max(series["true"][0][0], moving[0] - 8), min(series["true"][-1][0], moving[-1] + 5)
    else:
        t0, t_end = series["true"][0][0], series["true"][-1][0]
    pts = [(t, v) for t, v in series["true"] if t0 <= t <= t_end]
    ekf = [(t, v) for t, v in series["ekf"] if t0 <= t <= t_end]
    spdl = [s for s in series.get("spdl", []) if t0 <= s["t"] <= t_end]
    span = max(t_end - t0, 1.0)

    # the limit while it was live: from SPDL where there is one, otherwise from the parameters
    if spdl:
        limit_runs, run = [], None
        for s in spdl:
            if s["on"] and (run is None or run[2] != s["lim"]):
                if run:
                    limit_runs.append(run)
                run = [s["t"], s["t"], s["lim"]]
            elif s["on"]:
                run[1] = s["t"]
            elif run:
                limit_runs.append(run)
                run = None
        if run:
            limit_runs.append(run)
    else:
        steps = limit_steps(series)
        limit_runs = []
        for i, (t, v) in enumerate(steps):
            if v is None:
                continue
            t_next = steps[i + 1][0] if i + 1 < len(steps) else t_end
            limit_runs.append([max(t, t0), min(t_next, t_end), v])
    limit_runs = [r for r in limit_runs if r[1] > r[0] or len(limit_runs) == 1]

    vmax = max([v for _, v in pts] + [v for _, v in ekf] + [r[2] for r in limit_runs] + [1.0])
    tick = next(s for s in (0.5, 1, 2, 5, 10, 20) if vmax * 1.12 / s <= 6)
    vmax = tick * math.ceil(vmax * 1.12 / tick)
    left, right, top, bottom = 48, 74, 24, 40
    pw, ph = width - left - right, height - top - bottom

    def x(t):
        return left + (min(max(t, t0), t_end) - t0) / span * pw

    def y(v):
        return top + ph - v / vmax * ph

    def path(points):
        step = max(1, len(points) // 1500)
        return " ".join("%s%.1f,%.1f" % ("M" if i == 0 else "L", x(t), y(v))
                        for i, (t, v) in enumerate(points[::step]))

    out = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="100%%" role="img" '
           'aria-label="ground speed against the speed limit" font-family="inherit">' % (width, height),
           '<defs><pattern id="%s-off" width="6" height="6" patternUnits="userSpaceOnUse" '
           'patternTransform="rotate(45)"><rect width="6" height="6" fill="#f2f2f2"/>'
           '<line x1="0" y1="0" x2="0" y2="6" stroke="#c4c4c4" stroke-width="2"/></pattern></defs>' % pid]

    # mode bands, with the name across the top
    bands = []
    for t, name in series["modes"]:
        if t > t_end:
            break
        if bands and bands[-1][1] == name:
            continue
        bands.append((t, name))
    for i, (t, name) in enumerate(bands):
        t_next = bands[i + 1][0] if i + 1 < len(bands) else t_end
        if t_next <= t0:
            continue
        a, b = x(t), x(t_next)
        out.append('<rect x="%.1f" y="%d" width="%.1f" height="%d" fill="%s"/>' %
                   (a, top, max(b - a, 0), ph, MODE_COLOURS.get(name, "#f0f0f0")))
        if b - a > 34:
            out.append('<text x="%.1f" y="%d" font-size="10" fill="#4a5568" font-weight="600">%s</text>' %
                       (a + 3, top - 6, html.escape(name)))

    # where the limit was enabled but not live (no GPS, waiting for it, pilot switch)
    live_once = False
    for a, b in zip(spdl, spdl[1:]):
        live_once = live_once or a["on"]
        if live_once and not a["on"]:
            out.append('<rect x="%.1f" y="%d" width="%.1f" height="%d" fill="url(#%s-off)" opacity="0.85"/>' %
                       (x(a["t"]), top, max(x(b["t"]) - x(a["t"]), 0.6), ph, pid))

    for i in range(0, int(round(vmax / tick)) + 1):
        v = tick * i
        out.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#000" stroke-opacity="0.07"/>' %
                   (left, left + pw, y(v), y(v)))
        out.append('<text x="%d" y="%.1f" font-size="10" text-anchor="end" fill="#5a6474">%g</text>' %
                   (left - 6, y(v) + 3, round(v, 1)))
    step_s = next(s for s in (1, 2, 5, 10, 20, 30, 60, 120) if span / s <= 8)
    t = 0
    while t <= span + 1e-6:
        out.append('<line x1="%.1f" x2="%.1f" y1="%d" y2="%d" stroke="#9aa3b2"/>' %
                   (x(t0 + t), x(t0 + t), top + ph, top + ph + 4))
        out.append('<text x="%.1f" y="%d" font-size="10" text-anchor="middle" fill="#5a6474">%d</text>' %
                   (x(t0 + t), top + ph + 15, t))
        t += step_s
    out.append('<text x="%d" y="%d" font-size="10" text-anchor="middle" fill="#5a6474">time (s)</text>' %
               (left + pw // 2, height - 4))
    out.append('<text x="12" y="%d" font-size="10" fill="#5a6474" text-anchor="middle" '
               'transform="rotate(-90 12 %d)">ground speed (m/s)</text>' % (top + ph // 2, top + ph // 2))
    out.append('<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#c9ced8"/>' % (left, top, pw, ph))

    for a, b, v in limit_runs:
        out.append('<line x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f" stroke="#c0392b" stroke-width="1.6" '
                   'stroke-dasharray="7,4"/>' % (x(a), x(b), y(v), y(v)))
    if limit_runs:
        last = limit_runs[-1]
        out.append('<text x="%d" y="%.1f" font-size="10" fill="#c0392b">limit %g</text>' %
                   (left + pw + 5, y(last[2]) + 3, round(last[2], 2)))
    if ekf:
        out.append('<path d="%s" fill="none" stroke="#e08a2c" stroke-width="1.3" opacity="0.95"/>' % path(ekf))
    out.append('<path d="%s" fill="none" stroke="#1f5fbf" stroke-width="1.8"/>' % path(pts))

    # what the pilot was told, along the bottom
    colours = {1: "#d39b00", 2: "#c0392b", 3: "#7b4fc9"}
    for s in spdl:
        c = colours.get(s["ntc"])
        if c and not s["trig"]:
            out.append('<line x1="%.1f" x2="%.1f" y1="%d" y2="%d" stroke="%s" stroke-width="2"/>' %
                       (x(s["t"]), x(s["t"]), top + ph - 7, top + ph, c))
    # the RTL requests, on the EKF speed it saw
    for s in spdl:
        if s["trig"]:
            cx, cy = x(s["t"]), y(s["spd"])
            out.append('<path d="M%.1f,%.1f l-5,-9 h10 z" fill="#c0392b" stroke="#fff" stroke-width="1"/>' %
                       (cx, cy - 2))
            out.append('<text x="%.1f" y="%.1f" font-size="10" fill="#c0392b" font-weight="600" '
                       'text-anchor="middle">RTL</text>' % (cx, cy - 14))
    out.append("</svg>")
    return "\n".join(out)


def plot_for(test_dir, logs):
    """the speed plot, the stability figures and the key numbers from the test's flight log"""
    bins = [os.path.join(test_dir, f) for f in logs if f.endswith(".BIN")]
    if not bins:
        return None, None, {}
    flight = max(bins, key=os.path.getsize)      # the first, small one is from boot
    series = flight_series(flight)
    if not series:
        return None, None, {}
    return svg_plot(series), stability(flight, series), key_numbers(series)


def sweep_plot(rows, width=760, height=200):
    '''speed swing against P, one line per limit and mode'''
    lines = {}
    for r in rows:
        if r.get("I") not in ("0.05",):
            continue
        lines.setdefault("%s %s m/s" % (r.get("mode", ""), r["limit"]), []).append(
            (float(r["P"]), float(r["p2p_mps"]) / float(r["limit"]) * 100))
    if not lines:
        return ""
    ps = [p for pts in lines.values() for p, _ in pts]
    ymax = max(10.0, max(v for pts in lines.values() for _, v in pts) * 1.1)
    left, right, top, bottom = 44, 150, 12, 28
    pw, ph = width - left - right, height - top - bottom
    lo, hi = math.log(min(ps)), math.log(max(ps)) if max(ps) > min(ps) else math.log(min(ps)) + 1

    def x(p):
        return left + (math.log(p) - lo) / max(hi - lo, 1e-6) * pw

    def y(v):
        return top + ph - v / ymax * ph

    colours = ["#1f5fbf", "#e08a2c", "#2a9d6f", "#8e44ad", "#c0392b", "#5a6474"]
    out = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="100%%" role="img" '
           'aria-label="speed swing against P">' % (width, height),
           '<rect x="%d" y="%d" width="%d" height="%d" fill="#fff" stroke="#d0d4dc"/>' % (left, top, pw, ph),
           '<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#c0392b" stroke-dasharray="5,4"/>' %
           (left, left + pw, y(4), y(4)),
           '<text x="%d" y="%.1f" font-size="9" fill="#c0392b">4%% = oscillating</text>' % (left + 4, y(4) - 3)]
    for i in range(5):
        v = ymax * i / 4
        out.append('<text x="%d" y="%.1f" font-size="10" text-anchor="end" fill="#5a6474">%.0f%%</text>' %
                   (left - 4, y(v) + 3, v))
    for p in sorted(set(ps)):
        out.append('<text x="%.1f" y="%d" font-size="10" text-anchor="middle" fill="#5a6474">%g</text>' %
                   (x(p), height - 12, p))
    out.append('<text x="%d" y="%d" font-size="10" text-anchor="middle" fill="#5a6474">P (log scale)</text>' %
               (left + pw // 2, height - 1))
    for k, (name, pts) in enumerate(sorted(lines.items())):
        pts = sorted(pts)
        c = colours[k % len(colours)]
        out.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.6"/>' %
                   (" ".join("%s%.1f,%.1f" % ("M" if j == 0 else "L", x(p), y(v)) for j, (p, v) in enumerate(pts)), c))
        out += ['<circle cx="%.1f" cy="%.1f" r="3" fill="%s"/>' % (x(p), y(v), c) for p, v in pts]
        label = html.escape(name)
        out.append('<text x="%d" y="%d" font-size="10" fill="%s">%s</text>' % (left + pw + 8, top + 12 + 14 * k, c, label))
    out.append("</svg>")
    return "\n".join(out)


# -- reports ----------------------------------------------------------------------------------

CSS = """
:root { --ink: #14213d; --muted: #4f5b6d; --line: #dcdfe6; --card: #fff; --bg: #f4f5f7;
        --pass: #1e7b3a; --pass-bg: #e6f4ea; --fail: #b3261e; --fail-bg: #fbe9e7; }
* { box-sizing: border-box; }
body { font-family: -apple-system, "Segoe UI", Roboto, Arial, sans-serif; color: var(--ink); background: var(--bg);
       margin: 0; padding: 28px 16px 48px; line-height: 1.45; }
main { max-width: 1040px; margin: 0 auto; }
h1 { font-size: 26px; margin: 0 0 2px; } h2 { font-size: 19px; margin: 36px 0 10px; }
h3 { font-size: 15px; margin: 22px 0 8px; } h4 { font-size: 15px; margin: 0; }
.sub { color: var(--muted); font-size: 14px; margin-bottom: 14px; }
.verdict { border-radius: 10px; padding: 14px 18px; font-size: 17px; font-weight: 600; margin: 14px 0; }
.verdict.PASS { background: var(--pass-bg); color: var(--pass); border: 1px solid #b7dfc2; }
.verdict.FAIL { background: var(--fail-bg); color: var(--fail); border: 1px solid #f1bdb6; }
.verdict small { display: block; font-weight: 400; color: var(--ink); font-size: 13.5px; margin-top: 4px; }
dl.meta { display: grid; grid-template-columns: max-content 1fr; gap: 3px 14px; font-size: 13px; margin: 0; }
dl.meta dt { color: var(--muted); } dl.meta dd { margin: 0; }
code { background: #e9ebef; padding: 1px 5px; border-radius: 4px; font-size: 12.5px; }
table { border-collapse: collapse; width: 100%; background: var(--card); font-size: 13px; }
th, td { border: 1px solid var(--line); padding: 7px 9px; text-align: left; vertical-align: top; }
th { background: #eef0f4; font-weight: 600; }
td.res { white-space: nowrap; } td.res a { color: inherit; text-decoration: none; }
td.res small { display: block; color: var(--muted); font-weight: 400; white-space: normal; }
.tname { font-weight: 600; } .tcode { color: var(--muted); font-size: 11.5px; font-family: Consolas, monospace; }
.tdesc { color: var(--muted); font-size: 12.5px; margin-top: 2px; }
.PASS { color: var(--pass); font-weight: 600; } .FAIL { color: var(--fail); font-weight: 600; }
.DISABLED { color: #7a6a00; font-weight: 600; }
td.PASS { background: #f3faf5; } td.FAIL { background: #fdf3f2; }
.test { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 16px 18px;
        margin: 14px 0; }
.test.failed { border-color: #f1bdb6; box-shadow: inset 4px 0 0 var(--fail); }
.test .head { display: flex; justify-content: space-between; gap: 12px; align-items: baseline; flex-wrap: wrap; }
.badge { font-size: 12.5px; font-weight: 700; padding: 2px 9px; border-radius: 999px; }
.badge.PASS { background: var(--pass-bg); } .badge.FAIL { background: var(--fail-bg); }
.test p { margin: 6px 0; font-size: 13.5px; }
.reason { color: var(--fail); font-weight: 600; }
.chips { display: grid; grid-template-columns: repeat(auto-fill, minmax(190px, 1fr)); gap: 8px; margin: 12px 0; }
.chip { background: #f6f7f9; border: 1px solid #e3e6eb; border-radius: 8px; padding: 7px 10px; }
.chip span { display: block; font-size: 11.5px; color: var(--muted); } .chip b { font-size: 14px; }
details { margin-top: 8px; font-size: 13px; } summary { cursor: pointer; color: var(--muted); }
details ul { margin: 6px 0 4px 18px; padding: 0; font-family: Consolas, monospace; font-size: 12px; }
.files a { margin-right: 12px; font-size: 12.5px; }
.legend { display: flex; flex-wrap: wrap; gap: 6px 18px; font-size: 12px; color: var(--muted); margin: 6px 0 4px; }
.legend i { display: inline-block; vertical-align: middle; margin-right: 5px; }
.note { font-size: 13px; color: var(--muted); }
@media print { body { background: #fff; } .test { break-inside: avoid; } details { display: none; } }
"""

REQUIREMENTS = [
    ("Over the limit in STABILIZE or ALT_HOLD sends it home",
     r"TriggersRTL|BackToAltHold|EnableInFlight|LowerLimitInFlight|RCFailsafe|RTLTakeover|FenceBreach"),
    ("The other modes fly their own speeds and aren't checked", r"NotChecked"),
    ("No RTL when it shouldn't", r"NoTrigger|Vibration|DisabledSanity"),
    ("Without good GPS the limit is inactive, and comes back safely", r"GPS|Sats|Accuracy"),
    ("Pilot switch and messages", r"PilotOverride|Warns"),
]


def requirement(name):
    for title, pattern in REQUIREMENTS:
        if re.search(pattern, name):
            return title
    return "Other"


MODE_WORDS = (("AltHold", "ALT_HOLD"), ("PosHold", "POSHOLD"), ("Stabilize", "STABILIZE"), ("Loiter", "LOITER"),
              ("Guided", "GUIDED"), ("Auto", "AUTO"))


def test_title(name):
    """SpeedLimitAltHoldTriggersRTL -> "ALT_HOLD triggers RTL", OneGPSLostStaysOn -> "One GPS lost stays on" """
    short = name[len("SpeedLimit"):] if name.startswith("SpeedLimit") and len(name) > 10 else name
    for word, mode in MODE_WORDS:
        short = re.sub(r"%s(?=[A-Z]|$)" % word, mode + "_", short)
    words = re.sub(r"(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|_(?=[A-Z][a-z])|_$", " ", short).split()
    if not words:
        return name
    out = [words[0]] + [w if w.isupper() or "_" in w else w.lower() for w in words[1:]]
    return " ".join(out)


def anchor(t):
    return re.sub(r"[^A-Za-z0-9]+", "-", "%s-%s" % (t["name"], t.get("limit") if t.get("limit") is not None else ""))


def summary_counts(data):
    unit_cases = [c for u in data["unit"] for c in u["cases"] if c["result"] != "DISABLED"]
    sitl = data["autotests"]
    failed = [c for c in unit_cases if c["result"] == "FAIL"] + [t for t in sitl if t["result"] == "FAIL"]
    return unit_cases, sitl, failed


def limits_of(data):
    return sorted({t["limit"] for t in data["autotests"] if t.get("limit") is not None})


def write_html(path, data):
    e = html.escape
    meta = data["meta"]
    unit_cases, sitl, failed = summary_counts(data)
    limits = limits_of(data)
    frame = meta.get("frame") or "T150"

    parts = ["<!doctype html><html lang='en'><head><meta charset='utf-8'>",
             "<meta name='viewport' content='width=device-width, initial-scale=1'>",
             "<title>Speed limit test evidence</title><style>%s</style></head><body><main>" % CSS,
             "<h1>Speed limit test evidence</h1>",
             "<div class='sub'>SPD_LIM on the %s in SITL%s</div>" % (
                 e(frame), ", limits " + ", ".join("%g" % v for v in limits) + " m/s" if limits else "")]

    what = []
    if sitl:
        what.append("%d of %d SITL flights" % (sum(1 for t in sitl if t["result"] == "PASS"), len(sitl)))
    if unit_cases:
        what.append("%d of %d unit tests" % (sum(1 for c in unit_cases if c["result"] == "PASS"), len(unit_cases)))
    verdict = "FAIL" if failed else "PASS"
    headline = ("%d failed" % len(failed)) if failed else "Everything passed"
    detail = " and ".join(what) + " passed." if what else "Nothing was run."
    if failed:
        names = [test_title(t["name"]) + (" at %g m/s" % t["limit"] if t.get("limit") is not None else "")
                 for t in sitl if t["result"] == "FAIL"]
        names += [c["name"] for c in unit_cases if c["result"] == "FAIL"]
        detail += " Failed: " + ", ".join(e(n) for n in names[:8]) + (", ..." if len(names) > 8 else "") + "."
    parts.append("<div class='verdict %s'>%s<small>%s</small></div>" % (verdict, headline, detail))
    parts.append("<dl class='meta'><dt>Commit</dt><dd><code>%s</code> on <code>%s</code>%s</dd>"
                 "<dt>Run</dt><dd>%s</dd><dt>Machine</dt><dd>%s</dd></dl>" % (
                     e(meta["commit"][:12]), e(meta["branch"]),
                     " <span class='FAIL'>with uncommitted changes</span>" if meta["dirty"] else "",
                     e(meta["date"]), e(meta["host"])))

    if sitl:
        columns = limits or [None]
        names = []
        for t in sitl:
            if t["name"] not in names:
                names.append(t["name"])
        by = {(t["name"], t.get("limit")): t for t in sitl}
        parts.append("<h2>Results</h2>")
        parts.append("<p class='note'>Each cell is one SITL flight; click it for its plot and numbers. "
                     "<b>Sent home</b>: the speed limit asked for RTL, timed from its first update over the limit "
                     "(the SPDL log, 10 Hz). <b>Checked up to</b>: the highest EKF speed while the limit was live "
                     "in STABILIZE or ALT_HOLD, against the limit at that moment.</p>")
        groups = {}
        for n in names:
            groups.setdefault(requirement(n), []).append(n)
        order = [r for r, _ in REQUIREMENTS] + ["Other"]
        for title in order:
            if title not in groups:
                continue
            parts.append("<h3>%s</h3><table><tr><th>Test</th>%s</tr>" % (
                e(title), "".join("<th>%s</th>" % ("%g m/s" % c if c is not None else "Result") for c in columns)))
            for n in groups[title]:
                cells = []
                for c in columns:
                    t = by.get((n, c))
                    if t is None:
                        cells.append("<td>-</td>")
                        continue
                    note = key_summary(t.get("key") or {})
                    cells.append("<td class='res %s'><a href='#%s'>%s<small>%s</small></a></td>" % (
                        t["result"], anchor(t), t["result"], e(note)))
                desc = by.get((n, columns[0])) or next(t for t in sitl if t["name"] == n)
                parts.append("<tr><td><div class='tname'>%s</div><div class='tcode'>%s</div>"
                             "<div class='tdesc'>%s</div></td>%s</tr>" % (
                                 e(test_title(n)), e(n), e(desc["description"]), "".join(cells)))
            parts.append("</table>")

    if data["unit"]:
        bad = [c for c in unit_cases if c["result"] == "FAIL"]
        parts.append("<h2>Unit tests</h2>")
        parts.append("<details%s><summary>%d of %d passed, show all</summary>" % (
            " open" if bad else "", len(unit_cases) - len(bad), len(unit_cases)))
        parts.append("<table><tr><th>Test</th><th>Result</th><th>What it checks</th></tr>")
        for u in data["unit"]:
            for c in u["cases"]:
                detail = e(c["description"]) + ("<br><span class='FAIL'>%s</span>" % e(c["failure"]) if c["failure"] else "")
                parts.append("<tr><td><span class='tcode'>%s</span></td><td class='%s'>%s</td><td>%s</td></tr>" % (
                    e(c["name"]), c["result"], c["result"], detail))
        parts.append("</table></details>")

    if data["constants"]:
        parts.append("<h2>What the tests allow</h2><table><tr><th>Setting</th><th>Value</th><th>Meaning</th></tr>")
        for k, v in data["constants"].items():
            value = v["value"] + (" " + v.get("unit", "") if v.get("unit") else "")
            parts.append("<tr><td><span class='tcode'>%s</span></td><td>%s</td><td>%s</td></tr>" % (
                e(k), e(value), e(v["note"])))
        parts.append("</table>")

    sweeps = [r for t in sitl for r in t.get("sweep", [])]
    if sweeps:
        parts.append("<h2>Speed limiter gain sweep (D = 0)</h2>")
        parts.append("<p class='note'>Full forward stick, the limit stepped down 20%% and back at each setting. "
                     "Oscillating = the speed still swings by more than 4%% of the limit after 8 s.</p>")
        parts.append(sweep_plot(sweeps))
        parts.append("<table><tr><th>Mode</th><th>Limit</th><th>P</th><th>I</th><th>Overshoot after step</th>"
                     "<th>Mean</th><th>Speed swing</th><th>Pitch swing</th><th>Period</th><th>Verdict</th></tr>")
        for r in sweeps:
            verdict_s = r["verdict"]
            cls = "PASS" if verdict_s == "stable" else ("FAIL" if verdict_s == "oscillating" else "DISABLED")
            parts.append("<tr><td>%s</td><td>%s m/s</td><td>%s</td><td>%s</td><td>%+.2f m/s</td><td>%.2f m/s</td>"
                         "<td>%.2f m/s</td><td>%.1f&deg;</td><td>%.1f s</td><td class='%s'>%s</td></tr>" % (
                             e(r.get("mode", "")), e(r["limit"]), e(r["P"]), e(r["I"]), float(r["overshoot_mps"]),
                             float(r["mean_mps"]), float(r["p2p_mps"]), float(r["pitch_p2p_deg"]),
                             float(r["period_s"]), cls, e(verdict_s)))
        parts.append("</table>")

    if sitl:
        parts.append("<h2>Flights</h2>")
        parts.append(
            "<div class='legend'>"
            "<span><i style='width:20px;border-top:2px solid #1f5fbf'></i>true ground speed (simulator)</span>"
            "<span><i style='width:20px;border-top:2px solid #e08a2c'></i>EKF ground speed (what the limit "
            "checks)</span>"
            "<span><i style='width:20px;border-top:2px dashed #c0392b'></i>speed limit, while live</span>"
            "<span><i style='width:14px;height:12px;background:repeating-linear-gradient(45deg,#f2f2f2 0 3px,"
            "#c4c4c4 3px 5px)'></i>limit enabled but inactive (no GPS, or the pilot switch)</span>"
            "<span><i style='width:0;height:0;border-left:5px solid transparent;border-right:5px solid "
            "transparent;border-top:9px solid #c0392b'></i>RTL request</span>"
            "<span><i style='width:3px;height:10px;background:#d39b00'></i>warning</span>"
            "<span><i style='width:3px;height:10px;background:#c0392b'></i>over-limit alert</span>"
            "<span><i style='width:3px;height:10px;background:#7b4fc9'></i>GPS back heads-up</span>"
            "<span>Shaded bands are the flight modes, named along the top.</span></div>")
        for t in sitl:
            at = " at %g m/s" % t["limit"] if t.get("limit") is not None else ""
            parts.append("<section class='test%s' id='%s'><div class='head'><h4>%s%s</h4>"
                         "<span class='badge %s'>%s &middot; %.0f s</span></div>" % (
                             " failed" if t["result"] == "FAIL" else "", anchor(t), e(test_title(t["name"])), at,
                             t["result"], t["result"], t["seconds"]))
            parts.append("<p>%s <span class='tcode'>%s</span></p>" % (e(t["description"]), e(t["name"])))
            if t["reason"]:
                parts.append("<p class='reason'>Failed: %s</p>" % e(t["reason"]))
            chips = key_chips(t["key"]) if t.get("key") else []
            if chips:
                parts.append("<div class='chips'>%s</div>" % "".join(
                    "<div class='chip'><span>%s</span><b>%s</b></div>" % (e(a), e(b)) for a, b in chips))
            if t.get("plot"):
                parts.append(t["plot"])
            st = t.get("stability")
            if st:
                bits = ["max tilt %s&deg;" % st["max_tilt_deg"]]
                if st["roll_err_max_deg"] is not None:
                    bits.append("roll tracking error %s&deg; max, %s&deg; rms" % (st["roll_err_max_deg"],
                                                                                  st["roll_err_rms_deg"]))
                if st["yaw_err_max_deg"] is not None:
                    bits.append("yaw error %s&deg; max" % st["yaw_err_max_deg"])
                bits.append("rates up to %s / %s / %s &deg;/s (roll / pitch / yaw)" % tuple(st["max_rate_dps"]))
                if st["alt_band_m"]:
                    bits.append("height %s to %s m" % tuple(st["alt_band_m"]))
                parts.append("<details><summary>How steadily it flew</summary><p>%s</p></details>" %
                             " &middot; ".join(bits))
            if t["measurements"]:
                parts.append("<details><summary>What the test printed</summary><ul>%s</ul></details>" %
                             "".join("<li>%s</li>" % e(m) for m in t["measurements"]))
            folder = t.get("dir", "autotest/%s" % t["name"])
            links = ["<a href='%s/output.txt'>output.txt</a>" % e(folder)]
            links += ["<a href='%s/%s'>%s</a>" % (e(folder), e(f), e(f)) for f in t["logs"]]
            parts.append("<details><summary>Logs</summary><div class='files'>%s</div></details></section>" %
                         "".join(links))
    parts.append("<p class='note'>Generated by Tools/scripts/ma_test_evidence.py. The logs are in the autotest/ "
                 "folder next to this report; .BIN files open in Mission Planner or MAVExplorer.</p>")
    parts.append("</main></body></html>")
    open(path, "w").write("\n".join(parts))


def write_markdown(path, data):
    meta = data["meta"]
    unit_cases, sitl, failed = summary_counts(data)
    limits = limits_of(data)
    verdict = "failed" if failed else "passed"
    lines = ["## Speed limit test evidence: %s" % verdict, "",
             "SPD_LIM on the %s in SITL%s. Commit `%s` on `%s`%s, run %s." % (
                 meta.get("frame") or "T150",
                 ", limits " + ", ".join("%g" % v for v in limits) + " m/s" if limits else "",
                 meta["commit"][:10], meta["branch"], " (with uncommitted changes)" if meta["dirty"] else "",
                 meta["date"]), ""]
    if unit_cases:
        passed = sum(1 for c in unit_cases if c["result"] == "PASS")
        lines += ["Unit tests: %d of %d passed." % (passed, len(unit_cases)), ""]
    if sitl:
        columns = limits or [None]
        names = []
        for t in sitl:
            if t["name"] not in names:
                names.append(t["name"])
        by = {(t["name"], t.get("limit")): t for t in sitl}
        lines.append("| Test | %s |" % " | ".join("%g m/s" % c if c is not None else "Result" for c in columns))
        lines.append("| --- |" + " --- |" * len(columns))
        for n in names:
            cells = []
            for c in columns:
                t = by.get((n, c))
                if t is None:
                    cells.append("-")
                    continue
                note = key_summary(t.get("key") or {})
                cells.append("%s %s" % ("✅" if t["result"] == "PASS" else "❌", note))
            lines.append("| %s | %s |" % (test_title(n), " | ".join(cells)))
        lines.append("")
    bad = [t for t in sitl if t["result"] == "FAIL"]
    if bad:
        lines.append("**Failed**")
        lines.append("")
        for t in bad:
            at = " at %g m/s" % t["limit"] if t.get("limit") is not None else ""
            lines.append("- %s%s: %s" % (test_title(t["name"]), at, t["reason"] or "see output.txt"))
        lines.append("")
    for c in unit_cases:
        if c["result"] == "FAIL":
            lines.append("- unit test %s: %s" % (c["name"], c["failure"]))
    lines.append("The full report, with a plot and the key numbers for every flight, is report.html in the "
                 "evidence folder.")
    open(path, "w").write("\n".join(lines) + "\n")


# -- main -------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="evidence", help="folder to put the evidence folder in")
    parser.add_argument("--tests", nargs="*", help="autotests to run (default: SpeedLimit tests in testsMA)")
    parser.add_argument("--also", nargs="*", default=[],
                        help="autotests to run after the default list, e.g. SpeedLimitGainSweep")
    parser.add_argument("--skip", nargs="*", default=[], help="autotests to leave out of the default list")
    parser.add_argument("--pattern", default="SpeedLimit", help="name prefix of the autotests to pick up")
    parser.add_argument("--unit", nargs="*", help="gtest binaries to run (default: the speed limit ones that exist)")
    parser.add_argument("--skip-unit", action="store_true", help="don't run the unit tests")
    parser.add_argument("--skip-sitl", action="store_true", help="don't run the SITL autotests")
    parser.add_argument("--timeout", type=int, default=1200, help="seconds allowed per autotest")
    parser.add_argument("--zip", action="store_true", help="zip the evidence folder as well")
    parser.add_argument("--limits", nargs="*", type=float,
                        help="run every test at each of these speed limits (m/s), e.g. --limits 5 17 25")
    parser.add_argument("--merge", nargs="+", metavar="FOLDER",
                        help="OUT RUN [RUN ...]: put several earlier runs (e.g. one per limit) into one report in OUT")
    parser.add_argument("--report-only", metavar="FOLDER",
                        help="rebuild report.html and report.md for an earlier run, without running anything")
    args = parser.parse_args()

    if args.merge:
        if len(args.merge) < 2:
            raise SystemExit("--merge needs an output folder and at least one run")
        out, runs = args.merge[0], args.merge[1:]
        os.makedirs(out, exist_ok=True)
        merged = None
        for run_dir in runs:
            data = json.load(open(os.path.join(run_dir, "results.json")))
            for t in data["autotests"]:
                rel = t.get("dir", "autotest/%s" % t["name"])
                dest = os.path.join(out, rel)
                if not os.path.exists(dest):
                    shutil.copytree(os.path.join(run_dir, rel), dest)
                t["dir"] = rel
            if os.path.isdir(os.path.join(run_dir, "unit")) and not os.path.exists(os.path.join(out, "unit")):
                shutil.copytree(os.path.join(run_dir, "unit"), os.path.join(out, "unit"))
            if merged is None:
                merged = data
            else:
                merged["autotests"] += data["autotests"]
                merged["unit"] = merged["unit"] or data["unit"]
        merged["autotests"].sort(key=lambda t: (t.get("limit") or 0))
        json.dump(merged, open(os.path.join(out, "results.json"), "w"), indent=2)
        args.report_only = out

    if args.report_only:
        data = json.load(open(os.path.join(args.report_only, "results.json")))
        for t in data["autotests"]:
            folder = os.path.join(args.report_only, t.get("dir", "autotest/%s" % t["name"]))
            t["plot"], t["stability"], t["key"] = plot_for(folder, t["logs"])
            console = os.path.join(folder, "output.txt")
            if os.path.exists(console):
                t["measurements"] = measurements(open(console).read(), t["name"])
        saved = {k: v for k, v in data.items() if k != "autotests"}
        saved["autotests"] = [{k: v for k, v in t.items() if k != "plot"} for t in data["autotests"]]
        json.dump(saved, open(os.path.join(args.report_only, "results.json"), "w"), indent=2)
        write_html(os.path.join(args.report_only, "report.html"), data)
        write_markdown(os.path.join(args.report_only, "report.md"), data)
        print("report rebuilt in %s" % args.report_only)
        return 0

    if not os.path.exists(ARDUCOPTER):
        raise SystemExit("run this from the root of the repo")

    source = open(ARDUCOPTER).read()
    found = autotest_names(source, args.pattern)
    docs = dict(found)
    if args.tests:
        missing = [t for t in args.tests if t not in docs]
        if missing:
            raise SystemExit("not in %s: %s" % (ARDUCOPTER, ", ".join(missing)))
        names = args.tests
    else:
        group = ma_group(source)
        names = [n for n, _ in found if group is None or n in group]
    missing = [t for t in args.also if t not in docs]
    if missing:
        raise SystemExit("not in %s: %s" % (ARDUCOPTER, ", ".join(missing)))
    names += [t for t in args.also if t not in names]
    names = [t for t in names if t not in args.skip]

    commit = git("rev-parse", "--short=10", "HEAD") or "unknown"
    stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H%M")
    folder = os.path.join(args.out, "%s-%s" % (stamp, commit))
    os.makedirs(folder, exist_ok=True)
    data = {
        "meta": {
            "commit": git("rev-parse", "HEAD") or "unknown",
            "branch": git("rev-parse", "--abbrev-ref", "HEAD") or "unknown",
            "dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
            "date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
            "host": "%s, Python %s" % (platform.platform(), platform.python_version()),
            "frame": "T150",
        },
        "constants": constants(source),
        "unit": [],
        "autotests": [],
    }

    if not args.skip_unit:
        binaries = args.unit if args.unit else [b for b in UNIT_BINARIES if os.path.exists(b)]
        if not binaries:
            print("no speed limit unit test binary found, build it with ./waf --target tests/<name>")
        data["unit"] = run_unit_tests(binaries, os.path.join(folder, "unit"))

    if not args.skip_sitl:
        buildlogs = os.path.abspath(os.path.join(folder, ".buildlogs"))
        os.makedirs(buildlogs, exist_ok=True)
        runs = [(limit, name) for limit in (args.limits or [None]) for name in names]
        for i, (limit, name) in enumerate(runs):
            label = name + (" @ %g m/s" % limit if limit is not None else "")
            print("sitl %2d/%d %-55s" % (i + 1, len(runs), label), end="", flush=True)
            rel = os.path.join("autotest", "%gms" % limit, name) if limit is not None else os.path.join("autotest", name)
            test_dir = os.path.join(folder, rel)
            env = {}
            if limit is not None:
                env["MA_SPEED_LIMIT"] = "%g" % limit
            result = run_autotest(name, test_dir, args.timeout, buildlogs, env)
            result.update({"name": name, "description": docs.get(name, ""), "limit": limit, "dir": rel})
            result["plot"], result["stability"], result["key"] = plot_for(test_dir, result["logs"])
            data["autotests"].append(result)
            print("%s (%.0fs)" % (result["result"], result["seconds"]))
        shutil.rmtree(buildlogs, ignore_errors=True)

    plots = [t.pop("plot") for t in data["autotests"]]
    json.dump(data, open(os.path.join(folder, "results.json"), "w"), indent=2)
    for t, plot in zip(data["autotests"], plots):
        t["plot"] = plot
    write_html(os.path.join(folder, "report.html"), data)
    write_markdown(os.path.join(folder, "report.md"), data)
    if args.zip:
        shutil.make_archive(folder, "zip", args.out, os.path.basename(folder))
        print("zipped to %s.zip" % folder)

    failed = [c for u in data["unit"] for c in u["cases"] if c["result"] == "FAIL"]
    failed += [t for t in data["autotests"] if t["result"] == "FAIL"]
    failed += [u for u in data["unit"] if u["exit"] != 0 and not u["cases"]]
    print("evidence in %s/report.html, %s" % (folder, "%d failed" % len(failed) if failed else "everything passed"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
