#!/usr/bin/env python3
"""
Turn the gtest XML files from `./waf check` into a short Markdown summary.

CI appends the output to the job summary page, so a failing unit test shows up by name
without digging through the build log:

    GTEST_OUTPUT=xml:test-results/ ./waf check
    Tools/scripts/ma_test_summary.py test-results >> "$GITHUB_STEP_SUMMARY"

Always exits 0; the test step itself is what fails the job.
"""

import glob
import os
import sys
import xml.etree.ElementTree as ET


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else "test-results"
    files = sorted(glob.glob(os.path.join(folder, "*.xml")))
    if not files:
        print("### Unit tests\n\nNo test results found (did the build fail before the tests ran?)")
        return

    rows = []
    failures = []
    totals = {"tests": 0, "failures": 0, "disabled": 0}
    for path in files:
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError:
            # a binary that crashed part way leaves a truncated file behind
            rows.append((os.path.basename(path), "-", "-", "-", "unreadable"))
            continue
        binary = os.path.splitext(os.path.basename(path))[0]
        tests = int(root.get("tests", 0))
        failed = int(root.get("failures", 0)) + int(root.get("errors", 0))
        disabled = int(root.get("disabled", 0))
        totals["tests"] += tests
        totals["failures"] += failed
        totals["disabled"] += disabled
        rows.append((binary, tests, failed, disabled, "failed" if failed else "ok"))
        for case in root.iter("testcase"):
            for failure in case.findall("failure"):
                first_line = (failure.get("message") or "").strip().splitlines()[:1]
                failures.append("%s: %s.%s %s" % (binary, case.get("classname"), case.get("name"),
                                                   first_line[0] if first_line else ""))

    print("### Unit tests")
    print()
    print("%d tests, %d failed, %d disabled, in %d binaries" %
          (totals["tests"], totals["failures"], totals["disabled"], len(files)))
    print()
    # only list the binaries worth looking at, the rest is noise
    interesting = [r for r in rows if r[4] != "ok" or "speed_limit" in r[0]]
    if interesting:
        print("| Binary | Tests | Failed | Disabled | Result |")
        print("| --- | --- | --- | --- | --- |")
        for r in interesting:
            print("| %s | %s | %s | %s | %s |" % r)
        print()
    if failures:
        print("**Failures**")
        print()
        print("```")
        for line in failures:
            print(line)
        print("```")


if __name__ == "__main__":
    main()
