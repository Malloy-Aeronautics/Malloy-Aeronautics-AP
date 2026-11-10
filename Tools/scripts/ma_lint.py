#!/usr/bin/env python3
"""
Lint the lines a branch changes, not the whole tree.

The 4.3 tree has plenty of old style issues and compiler warnings that we are not going to
clean up, so every check here only reports problems on lines added or changed compared with
the base branch. New code has to be clean, old code is left alone.

Checks:
  hygiene     trailing whitespace, tabs, CRLF line endings and leftover conflict markers
  python      flake8 (repo .flake8 config) on changed Python files
  workflows   actionlint on changed GitHub workflow files, if actionlint is installed
  warnings    compiler warnings that point at changed lines (pass --build-log)

Usage:
  Tools/scripts/ma_lint.py                                   # compare with the release branch
  Tools/scripts/ma_lint.py --base origin/feature/other       # compare with something else
  Tools/scripts/ma_lint.py --build-log build.log             # also check compiler warnings

Uncommitted changes are included, so it can be run before committing too.
Exits non-zero if anything is found. Inside GitHub Actions the findings are also printed as
annotations so they show up against the lines in the PR.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys

DEFAULT_BASE = "origin/release/MA-4.3.0.17"

# files we never lint (generated or vendored)
SKIP_PREFIXES = ("modules/", "build/")

# where tabs are a mistake; Makefiles and similar need them
NO_TAB_EXTENSIONS = (".py", ".yml", ".yaml", ".cpp", ".c", ".h", ".hpp", ".sh")

CONFLICT_MARKER = re.compile(r"^(<{7}|>{7}|={7})( |$)")


def run(cmd, check=True):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, check=check).stdout


class Linter:

    def __init__(self, base):
        self.base = base
        self.findings = []
        self.changed = {}       # path -> set of changed line numbers (new file numbering)

    def report(self, check, path, line, message):
        self.findings.append((check, path, line, message))

    # -- working out what changed ------------------------------------------------------------

    def load_changes(self):
        merge_base = run(["git", "merge-base", self.base, "HEAD"]).strip()
        # diff against the working tree so uncommitted edits are covered as well
        diff = run(["git", "diff", "-U0", "--no-color", "--no-ext-diff", "--diff-filter=AM", merge_base])
        path = None
        for line in diff.splitlines():
            if line.startswith("+++ "):
                name = line[4:]
                path = name[2:] if name.startswith("b/") else None
                if path is not None and path.startswith(SKIP_PREFIXES):
                    path = None
                if path is not None:
                    self.changed.setdefault(path, set())
            elif line.startswith("@@") and path is not None:
                m = re.match(r"@@ -\S+ \+(\d+)(?:,(\d+))? @@", line)
                start = int(m.group(1))
                count = int(m.group(2)) if m.group(2) is not None else 1
                self.changed[path].update(range(start, start + count))
        return merge_base

    def is_changed(self, path, line):
        return line in self.changed.get(path, ())

    # -- checks --------------------------------------------------------------------------------

    def check_hygiene(self):
        for path, lines in self.changed.items():
            if not lines or not os.path.isfile(path):
                continue
            with open(path, "rb") as f:
                content = f.read()
            if b"\0" in content[:8000]:
                continue    # binary
            for number, raw in enumerate(content.split(b"\n"), start=1):
                if number not in lines:
                    continue
                text = raw.decode("utf-8", errors="replace")
                if text.endswith("\r"):
                    self.report("hygiene", path, number, "CRLF line ending (check your editor / core.autocrlf)")
                    text = text[:-1]
                if text != text.rstrip():
                    self.report("hygiene", path, number, "trailing whitespace")
                if "\t" in text and path.endswith(NO_TAB_EXTENSIONS):
                    self.report("hygiene", path, number, "tab character, use spaces")
                if CONFLICT_MARKER.match(text):
                    self.report("hygiene", path, number, "leftover merge conflict marker")

    def check_python(self):
        files = [p for p, lines in self.changed.items() if lines and p.endswith(".py") and os.path.isfile(p)]
        if not files:
            return
        if shutil.which("flake8") is None:
            self.report("python", files[0], 0, "flake8 not installed (python3 -m pip install flake8)")
            return
        out = run(["flake8", "--format=%(path)s:%(row)d:%(col)d: %(code)s %(text)s"] + files, check=False)
        for line in out.splitlines():
            m = re.match(r"(?:\./)?(.+?):(\d+):\d+: (\S+ .*)", line)
            if m and self.is_changed(m.group(1), int(m.group(2))):
                self.report("python", m.group(1), int(m.group(2)), m.group(3))

    def check_workflows(self):
        files = [p for p in self.changed
                 if p.startswith(".github/workflows/") and p.endswith((".yml", ".yaml")) and os.path.isfile(p)]
        if not files:
            return
        if shutil.which("actionlint") is None:
            print("note: actionlint not installed, workflow files not checked")
            return
        # workflow files are short, so report everything in a changed one rather than just
        # the changed lines; a bad matrix reference elsewhere in the file still breaks the run.
        # shellcheck and pyflakes inside run: blocks are left off for now, the existing
        # workflows would drown in them.
        out = run(["actionlint", "-shellcheck=", "-pyflakes=", "-oneline"] + files, check=False)
        for line in out.splitlines():
            m = re.match(r"(.+?):(\d+):\d+: (.*)", line)
            if m:
                self.report("workflows", m.group(1), int(m.group(2)), m.group(3))

    def check_build_log(self, log_path):
        warning = re.compile(r"^(?:\.\./)*(.+?):(\d+):\d+: warning: (.*)$")
        seen = set()
        with open(log_path, errors="replace") as f:
            for line in f:
                m = warning.match(line.strip())
                if not m:
                    continue
                path, number, message = m.group(1), int(m.group(2)), m.group(3)
                if (path, number, message) in seen:
                    continue
                seen.add((path, number, message))
                if self.is_changed(path, number):
                    self.report("warnings", path, number, message)

    # -- output --------------------------------------------------------------------------------

    def print_findings(self):
        in_actions = os.environ.get("GITHUB_ACTIONS") == "true"
        for check, path, line, message in sorted(self.findings, key=lambda f: (f[1], f[2])):
            print("%s:%d: [%s] %s" % (path, line, check, message))
            if in_actions and line > 0:
                print("::error file=%s,line=%d,title=%s::%s" % (path, line, check, message))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", default=os.environ.get("MA_LINT_BASE", DEFAULT_BASE),
                        help="branch or commit to compare with (default %(default)s)")
    parser.add_argument("--build-log", help="compiler output to check for warnings on changed lines")
    args = parser.parse_args()

    linter = Linter(args.base)
    try:
        merge_base = linter.load_changes()
    except subprocess.CalledProcessError as e:
        print("could not compare with %s: %s" % (args.base, e.stdout.strip()))
        print("fetch it first (git fetch origin) or pass --base")
        return 2

    changed_lines = sum(len(v) for v in linter.changed.values())
    print("linting %d changed lines in %d files against %s (%s)" %
          (changed_lines, len(linter.changed), args.base, merge_base[:10]))

    linter.check_hygiene()
    linter.check_python()
    linter.check_workflows()
    if args.build_log:
        linter.check_build_log(args.build_log)

    linter.print_findings()
    if linter.findings:
        print("%d problem(s) found" % len(linter.findings))
        return 1
    print("lint clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
