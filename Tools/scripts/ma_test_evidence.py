#!/usr/bin/env python3
"""
Run the speed limiter tests and put together the evidence for them: what each test checks,
what it measured, whether it passed, which commit it ran on, and the logs it flew.

Run from the repo root on Ubuntu (WSL) with SITL built and the Python 3.10 venv active:

    Tools/scripts/ma_test_evidence.py                  # unit tests and the SpeedLimit autotests
    Tools/scripts/ma_test_evidence.py --tests SpeedLimitAttitudeAltHold SpeedLimitLoiter
    Tools/scripts/ma_test_evidence.py --skip-sitl      # unit tests only, takes seconds
    Tools/scripts/ma_test_evidence.py --zip            # also zip the folder to send on
    Tools/scripts/ma_test_evidence.py --report-only evidence/<folder>   # rebuild an earlier report

It writes evidence/<date>-<commit>/ with:

    report.html       the evidence report: summary, every unit test, every autotest with what it
                      measured and a plot of true speed against the limit from its flight log
    report.md         the same summary in Markdown, for a PR or a ticket
    results.json      everything above as data
    unit/             gtest XML and console output
    autotest/<Test>/  console output, dataflash logs (.BIN) and telemetry logs (.tlog)

The autotests are the SpeedLimit tests listed in testsMA (all SpeedLimit tests if there is no
testsMA list). Exits non-zero if any test failed, so it can be a CI step.
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

# lines worth keeping from a test's console output: the numbers the test measured and judged
MEASUREMENT_HINTS = ("true speed", "m/s", "past it", "course change", "heading change", "pitch rate",
                     "tilt", "EKF lost", "overshoot", "RTL after", "speeded up", "Exception caught")
NOISE_HINTS = ("Received wanted", "Sending set", " want=", " is now ", "PARAM_VALUE", "statustext=",
               "Waiting for", "Got: ", " AP: ", "Drained", "Set RC", "mavproxy", "Wait groundspeed",
               "groundspeed ", "SIM_STATE", "set_message_rate", "Arming", "Takeoff", "MAV_CMD")
MAX_MEASUREMENTS = 14

# copter mode numbers, for labelling the plots
COPTER_MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL",
                7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
                17: "BRAKE", 18: "THROW", 21: "SMART_RTL", 23: "FOLLOW", 24: "ZIGZAG", 27: "AUTO_RTL"}

# speed limit parameters, for both the attitude limiter and the RTL version
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
    '''a description for each TEST() in the binary's source file: the comment just above it, or its
    name written out plus the first comment in its body'''
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
    '''the pass bands the SITL tests use (SPEED_LIMIT = 5.0 and friends)'''
    found = {}
    for m in re.finditer(r"^    ((?:SPEED|FENCE|STICK)_[A-Z_]+) = ([0-9.]+|None)\s*(?:#\s*(.*))?$", source, re.M):
        found[m.group(1)] = {"value": m.group(2), "note": (m.group(3) or "").strip()}
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


def run_autotest(name, out_dir, timeout, buildlogs):
    os.makedirs(out_dir, exist_ok=True)
    clear_autotest_logs()
    env = dict(os.environ, BUILDLOGS=buildlogs)
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
        reason = "timed out after %ds" % timeout if timed_out else (m.group(1).strip() if m else "see output.txt")

    # passing runs leave their logs in Tools/autotest, failing ones are moved to BUILDLOGS
    logs = []
    for f in sorted(glob.glob(os.path.join(AUTOTEST_DIR, "logs", "*.BIN")) +
                    glob.glob(os.path.join(AUTOTEST_DIR, "autotest-*.tlog")) +
                    glob.glob(os.path.join(buildlogs, "*-%s-*" % name)) +
                    glob.glob(os.path.join(buildlogs, "*-%s.txt" % name))):
        dest = os.path.join(out_dir, os.path.basename(f))
        shutil.move(f, dest)
        logs.append(os.path.basename(dest))

    return {"result": "PASS" if passed else "FAIL", "seconds": seconds, "reason": reason,
            "measurements": measurements(output, name), "logs": logs}


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
    '''true and EKF horizontal speed, speed limit and mode changes from a dataflash log'''
    try:
        from pymavlink import DFReader
    except ImportError:
        return None
    try:
        log = DFReader.DFReader_binary(bin_path)
    except Exception:
        return None
    true_speed, ekf_speed, modes, params = [], [], [], {}
    while True:
        m = log.recv_match(type=["SIM2", "XKF1", "MODE", "PARM"])
        if m is None:
            break
        t = m.get_type()
        if t == "SIM2":
            true_speed.append((m.TimeUS * 1e-6, math.hypot(m.VN, m.VE)))
        elif t == "XKF1" and getattr(m, "C", 0) == 0:
            ekf_speed.append((m.TimeUS * 1e-6, math.hypot(m.VN, m.VE)))
        elif t == "MODE":
            modes.append((m.TimeUS * 1e-6, COPTER_MODES.get(m.ModeNum, str(m.ModeNum))))
        elif t == "PARM":
            if m.Name in LIMIT_PARAMS + ENABLE_PARAMS:
                params.setdefault(m.Name, []).append((m.TimeUS * 1e-6, m.Value))
    if not true_speed:
        return None
    return {"true": true_speed, "ekf": ekf_speed, "modes": modes, "params": params}


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


def svg_plot(series, width=760, height=230):
    '''a small self-contained SVG: true speed, EKF speed, the limit and the mode changes'''
    # just the flight: from a little before it starts moving to a little after it stops
    moving = [t for t, v in series["true"] if v > 0.3]
    if moving:
        t0, t_end = max(series["true"][0][0], moving[0] - 10), min(series["true"][-1][0], moving[-1] + 5)
    else:
        t0, t_end = series["true"][0][0], series["true"][-1][0]
    pts = [(t, v) for t, v in series["true"] if t0 <= t <= t_end]
    ekf = [(t, v) for t, v in series["ekf"] if t0 <= t <= t_end]
    span = max(t_end - t0, 1.0)
    steps = limit_steps(series)
    shown_limits = [v for t, v in steps if v is not None]
    vmax = max(max(v for _, v in pts), max([v for _, v in ekf] or [0]), min(shown_limits or [0]))
    tick = next(s for s in (0.5, 1, 2, 5, 10, 20) if vmax * 1.15 / s <= 6)
    vmax = tick * math.ceil(vmax * 1.15 / tick)
    left, right, top, bottom = 42, 10, 14, 26
    pw, ph = width - left - right, height - top - bottom

    def x(t):
        return left + (t - t0) / span * pw

    def y(v):
        return top + ph - v / vmax * ph

    def path(points):
        step = max(1, len(points) // 1500)
        return " ".join("%s%.1f,%.1f" % ("M" if i == 0 else "L", x(t), y(v))
                        for i, (t, v) in enumerate(points[::step]))

    out = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="100%%" role="img" '
           'aria-label="ground speed against the speed limit">' % (width, height)]
    out.append('<rect x="%d" y="%d" width="%d" height="%d" fill="#fff" stroke="#d0d4dc"/>' % (left, top, pw, ph))
    for i in range(0, int(round(vmax / tick)) + 1):
        v = tick * i
        out.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#eef0f3"/>' % (left, left + pw, y(v), y(v)))
        out.append('<text x="%d" y="%.1f" font-size="10" text-anchor="end" fill="#5a6474">%g</text>' %
                   (left - 4, y(v) + 3, round(v, 1)))
    for i in range(0, 6):
        t = t0 + span * i / 5
        out.append('<text x="%.1f" y="%d" font-size="10" text-anchor="middle" fill="#5a6474">%ds</text>' %
                   (x(t), height - 8, round(t - t0)))
    # mode changes: only the last one before the plot starts, and none that repeat the one before
    modes = []
    for t, name in series["modes"]:
        if t > t_end:
            break
        if t < t0 and modes and modes[-1][0] <= t0:
            modes[-1] = (t, name)
        elif not modes or modes[-1][1] != name:
            modes.append((t, name))
    # the label dropped a line when it would run into the one before
    last_label_x, row = -100.0, 0
    for t, name in modes:
        t = max(t, t0)
        out.append('<line x1="%.1f" x2="%.1f" y1="%d" y2="%d" stroke="#9aa3b2" stroke-dasharray="2,3"/>' %
                   (x(t), x(t), top, top + ph))
        row = row + 1 if x(t) - last_label_x < 60 else 0
        last_label_x = x(t)
        out.append('<text x="%.1f" y="%d" font-size="9" fill="#5a6474">%s</text>' % (x(t) + 2, top + 9 + 10 * row, name))
    # the limit while enabled
    for i, (t, v) in enumerate(steps):
        if v is None:
            continue
        t_next = steps[i + 1][0] if i + 1 < len(steps) else t_end
        a, b = min(max(t, t0), t_end), min(max(t_next, t0), t_end)
        if b > a:
            out.append('<line x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f" stroke="#c0392b" stroke-width="1.5" '
                       'stroke-dasharray="6,4"/>' % (x(a), x(b), y(v), y(v)))
    if ekf:
        out.append('<path d="%s" fill="none" stroke="#e08a2c" stroke-width="1.2" opacity="0.9"/>' % path(ekf))
    out.append('<path d="%s" fill="none" stroke="#1f5fbf" stroke-width="1.6"/>' % path(pts))
    out.append('<text x="%d" y="%d" font-size="10" fill="#5a6474" transform="rotate(-90 12 %d)">m/s</text>' %
               (12, top + ph // 2, top + ph // 2))
    out.append("</svg>")
    return "\n".join(out)


def plot_for(test_dir, logs):
    bins = [os.path.join(test_dir, f) for f in logs if f.endswith(".BIN")]
    if not bins:
        return None
    flight = max(bins, key=os.path.getsize)      # the first, small one is from boot
    series = flight_series(flight)
    return svg_plot(series) if series else None


# -- reports ----------------------------------------------------------------------------------

CSS = """
body { font-family: -apple-system, Segoe UI, Arial, sans-serif; color: #14213d; background: #f5f6f2;
       margin: 0; padding: 24px 16px; }
main { max-width: 1000px; margin: 0 auto; }
h1 { font-size: 26px; margin: 0 0 4px; } h2 { font-size: 19px; margin: 32px 0 10px; }
h3 { font-size: 15px; margin: 0; }
.meta { color: #3f4a5a; font-size: 13px; } .meta code { background: #e9ece4; padding: 1px 4px; }
.cards { display: flex; gap: 12px; flex-wrap: wrap; margin: 18px 0; }
.card { background: #fff; border: 1px solid #dcdfd6; border-radius: 10px; padding: 12px 16px; min-width: 150px; }
.card b { font-size: 24px; display: block; }
table { border-collapse: collapse; width: 100%; background: #fff; font-size: 13px; }
th, td { border: 1px solid #dcdfd6; padding: 6px 8px; text-align: left; vertical-align: top; }
th { background: #eef0ea; }
.PASS { color: #1e7b3a; font-weight: 600; } .FAIL { color: #b3261e; font-weight: 600; }
.DISABLED { color: #7a6a00; font-weight: 600; }
.test { background: #fff; border: 1px solid #dcdfd6; border-radius: 10px; padding: 14px 16px; margin: 12px 0; }
.test .head { display: flex; justify-content: space-between; gap: 12px; align-items: baseline; }
.test p { margin: 6px 0; font-size: 13px; color: #3f4a5a; }
.test ul { margin: 6px 0 8px 18px; padding: 0; font-size: 12.5px; font-family: Consolas, monospace; }
.files { font-size: 12px; } .files a { margin-right: 10px; }
.legend { font-size: 11.5px; color: #5a6474; }
.legend span { display: inline-block; width: 18px; height: 0; vertical-align: middle; margin: 0 4px 0 10px; }
@media print { body { background: #fff; } .test { break-inside: avoid; } }
"""


def write_html(path, data):
    e = html.escape
    meta = data["meta"]
    unit_cases = [c for u in data["unit"] for c in u["cases"]]
    sitl = data["autotests"]
    parts = ["<!doctype html><html lang='en'><head><meta charset='utf-8'>",
             "<meta name='viewport' content='width=device-width, initial-scale=1'>",
             "<title>Speed limiter test evidence</title><style>%s</style></head><body><main>" % CSS,
             "<h1>Speed limiter test evidence</h1>",
             "<div class='meta'>Commit <code>%s</code> on <code>%s</code>%s &middot; run %s &middot; %s</div>" % (
                 e(meta["commit"]), e(meta["branch"]),
                 " <b class='FAIL'>(uncommitted changes)</b>" if meta["dirty"] else "",
                 e(meta["date"]), e(meta["host"])),
             "<div class='cards'>"]
    for label, cases in (("unit tests", unit_cases), ("SITL autotests", sitl)):
        run_cases = [c for c in cases if c["result"] != "DISABLED"]
        passed = sum(1 for c in run_cases if c["result"] == "PASS")
        parts.append("<div class='card'><b class='%s'>%d / %d</b>%s passed</div>" % (
            "PASS" if passed == len(run_cases) else "FAIL", passed, len(run_cases), label))
    parts.append("</div>")

    if data["constants"]:
        parts.append("<h2>SITL pass bands</h2><table><tr><th>Setting</th><th>Value</th><th>Meaning</th></tr>")
        for k, v in data["constants"].items():
            parts.append("<tr><td>%s</td><td>%s</td><td>%s</td></tr>" % (e(k), e(v["value"]), e(v["note"])))
        parts.append("</table>")

    if data["unit"]:
        parts.append("<h2>Unit tests</h2><table><tr><th>Test</th><th>Result</th><th>What it checks</th></tr>")
        for c in unit_cases:
            detail = e(c["description"]) + ("<br><span class='FAIL'>%s</span>" % e(c["failure"]) if c["failure"] else "")
            parts.append("<tr><td>%s</td><td class='%s'>%s</td><td>%s</td></tr>" % (
                e(c["name"]), c["result"], c["result"], detail))
        parts.append("</table>")

    if sitl:
        parts.append("<h2>SITL autotests</h2>")
        parts.append("<div class='legend'>Plots, from each test's flight log:<span style='border-top:2px solid #1f5fbf'>"
                     "</span>true ground speed (simulator)<span style='border-top:2px solid #e08a2c'></span>EKF ground "
                     "speed (what the vehicle used)<span style='border-top:2px dashed #c0392b'></span>speed limit, while "
                     "enabled (still drawn while the pilot override switch is on). Dotted lines are mode changes.</div>")
        for t in sitl:
            parts.append("<section class='test'><div class='head'><h3>%s</h3><span class='%s'>%s &middot; %.0fs</span>"
                         "</div>" % (e(t["name"]), t["result"], t["result"], t["seconds"]))
            parts.append("<p>%s</p>" % e(t["description"]))
            if t["reason"]:
                parts.append("<p class='FAIL'>%s</p>" % e(t["reason"]))
            if t["measurements"]:
                parts.append("<ul>%s</ul>" % "".join("<li>%s</li>" % e(m) for m in t["measurements"]))
            if t.get("plot"):
                parts.append(t["plot"])
            links = ["<a href='autotest/%s/output.txt'>output.txt</a>" % e(t["name"])]
            links += ["<a href='autotest/%s/%s'>%s</a>" % (e(t["name"]), e(f), e(f)) for f in t["logs"]]
            parts.append("<div class='files'>%s</div></section>" % "".join(links))
    parts.append("<p class='meta'>Generated by Tools/scripts/ma_test_evidence.py. Logs are in the autotest/ "
                 "folder next to this report; .BIN files open in Mission Planner or MAVExplorer.</p>")
    parts.append("</main></body></html>")
    open(path, "w").write("\n".join(parts))


def write_markdown(path, data):
    meta = data["meta"]
    unit_cases = [c for u in data["unit"] for c in u["cases"] if c["result"] != "DISABLED"]
    lines = ["# Speed limiter test evidence", "",
             "Commit `%s` on `%s`%s, run %s." % (meta["commit"], meta["branch"],
                                                 " (with uncommitted changes)" if meta["dirty"] else "", meta["date"]),
             ""]
    if unit_cases:
        passed = sum(1 for c in unit_cases if c["result"] == "PASS")
        lines += ["**Unit tests: %d / %d passed**" % (passed, len(unit_cases)), ""]
    if data["autotests"]:
        passed = sum(1 for t in data["autotests"] if t["result"] == "PASS")
        lines += ["**SITL autotests: %d / %d passed**" % (passed, len(data["autotests"])), "",
                  "| Test | Result | What it checks | Measured |", "| --- | --- | --- | --- |"]
        for t in data["autotests"]:
            measured = "; ".join(t["measurements"][:2]) or t["reason"]
            lines.append("| %s | %s | %s | %s |" % (t["name"], t["result"], t["description"].replace("|", "/"),
                                                    measured.replace("|", "/")))
        lines.append("")
    failed = [c["name"] for c in unit_cases if c["result"] == "FAIL"] + \
             [t["name"] for t in data["autotests"] if t["result"] == "FAIL"]
    if failed:
        lines += ["**Failed:** " + ", ".join(failed), ""]
    lines.append("Full report with plots and logs: report.html in the evidence folder.")
    open(path, "w").write("\n".join(lines) + "\n")


# -- main -------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="evidence", help="folder to put the evidence folder in")
    parser.add_argument("--tests", nargs="*", help="autotests to run (default: SpeedLimit tests in testsMA)")
    parser.add_argument("--pattern", default="SpeedLimit", help="name prefix of the autotests to pick up")
    parser.add_argument("--unit", nargs="*", help="gtest binaries to run (default: the speed limit ones that exist)")
    parser.add_argument("--skip-unit", action="store_true", help="don't run the unit tests")
    parser.add_argument("--skip-sitl", action="store_true", help="don't run the SITL autotests")
    parser.add_argument("--timeout", type=int, default=1200, help="seconds allowed per autotest")
    parser.add_argument("--zip", action="store_true", help="zip the evidence folder as well")
    parser.add_argument("--report-only", metavar="FOLDER",
                        help="rebuild report.html and report.md for an earlier run, without running anything")
    args = parser.parse_args()

    if args.report_only:
        data = json.load(open(os.path.join(args.report_only, "results.json")))
        for t in data["autotests"]:
            t["plot"] = plot_for(os.path.join(args.report_only, "autotest", t["name"]), t["logs"])
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
        for i, name in enumerate(names):
            print("sitl %2d/%d %-45s" % (i + 1, len(names), name), end="", flush=True)
            test_dir = os.path.join(folder, "autotest", name)
            result = run_autotest(name, test_dir, args.timeout, buildlogs)
            result.update({"name": name, "description": docs.get(name, "")})
            result["plot"] = plot_for(test_dir, result["logs"])
            data["autotests"].append(result)
            print("%s (%.0fs)" % (result["result"], result["seconds"]))
        shutil.rmtree(buildlogs, ignore_errors=True)

    plots = {t["name"]: t.pop("plot") for t in data["autotests"]}
    json.dump(data, open(os.path.join(folder, "results.json"), "w"), indent=2)
    for t in data["autotests"]:
        t["plot"] = plots[t["name"]]
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
