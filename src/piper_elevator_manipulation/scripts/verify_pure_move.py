#!/usr/bin/env python3
"""Prove that a "pure move" relocated code without changing it.

A pure move means: each symbol listed in the symbols file is byte-identical
to the version git has recorded. Only the file it lives in, and the leading
whitespace of each line, may differ.

That is a stronger check than a dry run. A dry run exercises one execution
path and therefore never reaches createRecoveryTask, the error branches of
readPressParamsFile, the timeout branch of waitForSettledJoints, or any of
executePressSequence's early returns. This compares every listed definition,
including the branches that never run.

Usage:
    python3 scripts/verify_pure_move.py --before HEAD
    python3 scripts/verify_pure_move.py --before HEAD --only /tmp/before.cpp

Exit code is 0 only when every symbol is IDENTICAL.
"""

import argparse
import glob
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
DEFAULT_SYMBOLS = os.path.join(PACKAGE_DIR, "docs",
                               "press_button_demo_split_symbols.txt")
DEFAULT_BEFORE = "src/piper_elevator_manipulation/src/press_button_demo.cpp"


# ── masking ───────────────────────────────────────────────────────────────

def mask(text):
    """Blank out comments and string/char literals, preserving length.

    Brace and paren matching must not be confused by a `{` inside a log
    message or a `)` inside a comment, so all scanning happens on the masked
    copy while the extracted text still comes from the original.
    """
    out = list(text)
    n = len(text)
    i = 0

    def blank(start, stop):
        for k in range(start, stop):
            if out[k] != "\n":
                out[k] = " "

    while i < n:
        c = text[i]
        if c == "/" and i + 1 < n and text[i + 1] == "/":
            j = i
            while j < n and text[j] != "\n":
                j += 1
            blank(i, j)
            i = j
        elif c == "/" and i + 1 < n and text[i + 1] == "*":
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            blank(i, j)
            i = j
        elif c in "\"'":
            quote = c
            j = i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == quote:
                    j += 1
                    break
                if text[j] == "\n":
                    break
                j += 1
            blank(i, j)
            i = j
        else:
            i += 1
    return "".join(out)


def match_pair(masked, start, open_ch, close_ch):
    """Index of the closer matching masked[start], or None."""
    depth = 0
    for i in range(start, len(masked)):
        if masked[i] == open_ch:
            depth += 1
        elif masked[i] == close_ch:
            depth -= 1
            if depth == 0:
                return i
    return None


# ── locating a definition ─────────────────────────────────────────────────

def line_start(text, pos):
    return text.rfind("\n", 0, pos) + 1


def extend_over_template(text, start):
    """Include a `template <...>` line that precedes the definition."""
    if start == 0:
        return start
    previous_end = start - 1                      # the newline just before `start`
    previous_start = text.rfind("\n", 0, previous_end) + 1
    if text[previous_start:previous_end].strip().startswith("template"):
        return previous_start
    return start


def _preceded_by_qualifier(masked, pos):
    """True when the name is reached through . -> or :: (a call, not a def)."""
    i = pos - 1
    while i >= 0 and masked[i] in " \t\n":
        i -= 1
    if i < 0:
        return False
    if masked[i] == ".":
        return True
    if i >= 1 and masked[i - 1:i + 1] == "->":
        return True
    if i >= 1 and masked[i - 1:i + 1] == "::":
        return True
    return False


def _assigned_since_boundary(masked, pos):
    """True when the declarator region before `pos` contains '='.

    That rejects calls and assignments such as
    `const auto target = waitForTarget(` or `x = solveWaypoint(`.
    """
    i = pos - 1
    while i >= 0 and masked[i] not in ";{}":
        i -= 1
    return "=" in masked[i + 1:pos]


TRAILING_QUALIFIERS = ("const", "noexcept", "override", "final")


def function_definition(masked, text, name):
    for m in re.finditer(r"\b" + re.escape(name) + r"\b", masked):
        pos = m.start()
        if _preceded_by_qualifier(masked, pos) or _assigned_since_boundary(masked, pos):
            continue
        j = m.end()
        while j < len(masked) and masked[j] in " \t\n":
            j += 1
        if j >= len(masked) or masked[j] != "(":
            continue
        close = match_pair(masked, j, "(", ")")
        if close is None:
            continue
        j = close + 1
        while True:
            while j < len(masked) and masked[j] in " \t\n":
                j += 1
            if any(masked.startswith(q, j) for q in TRAILING_QUALIFIERS):
                while j < len(masked) and masked[j] not in " \t\n{;":
                    j += 1
                continue
            if masked.startswith("->", j):
                while j < len(masked) and masked[j] not in "{;":
                    j += 1
                continue
            break
        if j >= len(masked) or masked[j] != "{":
            continue  # declaration, not a definition — try the next match
        end = match_pair(masked, j, "{", "}")
        if end is None:
            continue
        start = extend_over_template(text, line_start(text, pos))
        return text[start:end + 1]
    return None


def type_definition(masked, text, name):
    pattern = r"\b(?:class|struct)\s+" + re.escape(name) + r"\b"
    for m in re.finditer(pattern, masked):
        j = m.end()
        while j < len(masked) and masked[j] not in "{;":
            j += 1
        if j >= len(masked) or masked[j] != "{":
            continue  # forward declaration
        end = match_pair(masked, j, "{", "}")
        if end is None:
            continue
        start = extend_over_template(text, line_start(text, m.start()))
        return text[start:end + 1]
    return None


# ── comparison ────────────────────────────────────────────────────────────

def normalize(text):
    """Leading indentation and blank lines may differ; nothing else may."""
    return [line.strip() for line in text.splitlines() if line.strip()]


def first_difference(before, after):
    for i, (a, b) in enumerate(zip(before, after)):
        if a != b:
            return i, a, b
    if len(before) != len(after):
        i = min(len(before), len(after))
        a = before[i] if i < len(before) else "<missing>"
        b = after[i] if i < len(after) else "<missing>"
        return i, a, b
    return None


def after_files():
    files = sorted(glob.glob(os.path.join(PACKAGE_DIR, "src", "**", "*.cpp"),
                             recursive=True))
    files += sorted(glob.glob(os.path.join(PACKAGE_DIR, "include", "**", "*.hpp"),
                               recursive=True))
    return files


def read(path):
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--before", default="067d404",
                        help="git ref holding the pre-move version. Defaults to "
                             "the last commit before the split began; HEAD would "
                             "move as the split's own commits land.")
    parser.add_argument("--before-path", default=DEFAULT_BEFORE,
                        help="repo-relative path of the pre-move file")
    parser.add_argument("--symbols", default=DEFAULT_SYMBOLS,
                        help="file listing the symbols to check")
    parser.add_argument("--only", default=None,
                        help="compare against this single file instead of scanning the package")
    args = parser.parse_args()

    with open(args.symbols, "r", encoding="utf-8") as handle:
        raw = handle.read()
    functions, types = [], []
    for line in raw.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("@type"):
            types.append(line[len("@type"):].strip())
        else:
            functions.append(line)
    symbols = [("func", n) for n in functions] + [("type", n) for n in types]

    try:
        before_text = subprocess.run(
            ["git", "show", f"{args.before}:{args.before_path}"],
            cwd=PACKAGE_DIR, check=True, capture_output=True).stdout.decode("utf-8")
    except subprocess.CalledProcessError as error:
        print(f"无法从 git 取 {args.before}:{args.before_path}\n{error.stderr.decode()}",
              file=sys.stderr)
        return 2

    before_masked = mask(before_text)

    files = [args.only] if args.only else after_files()
    if not files:
        print("没有可扫描的文件", file=sys.stderr)
        return 2

    identical = different = missing = 0
    for kind, name in symbols:
        expected = (type_definition(before_masked, before_text, name) if kind == "type"
                    else function_definition(before_masked, before_text, name))
        if expected is None:
            print(f"{name:<28} NOT-IN-BEFORE  ⚠️ 脚本找不到原定义")
            missing += 1
            continue

        found = None
        where = None
        for path in files:
            text = read(path)
            masked = mask(text)
            got = (type_definition(masked, text, name) if kind == "type"
                   else function_definition(masked, text, name))
            if got is not None:
                found, where = got, os.path.relpath(path, PACKAGE_DIR)
                break

        if found is None:
            print(f"{name:<28} MISSING         ⚠️ 搬丢了")
            missing += 1
            continue

        before_lines, after_lines = normalize(expected), normalize(found)
        if before_lines == after_lines:
            print(f"{name:<28} IDENTICAL       {where}")
            identical += 1
        else:
            print(f"{name:<28} DIFFERENT       {where}")
            diff = first_difference(before_lines, after_lines)
            if diff is not None:
                index, want, got = diff
                print(f"{'':<28}   第 {index + 1} 行（规范化后）")
                print(f"{'':<28}   before: {want}")
                print(f"{'':<28}   after : {got}")
            different += 1

    print()
    print(f"{identical} identical, {different} different, {missing} missing")
    return 0 if different == 0 and missing == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
