#!/usr/bin/env python3
"""Seed the translation catalogs with the runtime picker names.

The shade-mode and view-preset names are C++ table data, not tr() calls, so
lupdate never sees them; yet at runtime both pickers look them up through
QCoreApplication::translate("ViewportStrip", name), which resolves a name only
if the catalog carries an entry for that exact source text in the ViewportStrip
context (the bare class name, as passed to translate — not lupdate's
namespaced "pose::ViewportStrip" context for the tr() literals).

This script parses the two tables (the single source of truth) and adds any
missing <message> entries to the ViewportStrip context of each catalog under
translations/, creating the context when lupdate never produced one. The .ts
file is parsed as XML, so context names are compared exactly — a namespaced
context can never be mistaken for the bare one — while the write-back is a
surgical text insert, keeping the rest of the lupdate layout (and the diff)
untouched. Existing entries are left alone, so hand-written translations
survive re-runs; a second run over an already-seeded catalog is a no-op.

Usage:  python3 tools/i18n/add_runtime_strings.py [catalog.ts ...]
        (no arguments = every *.ts under translations/)
"""

import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TS_CONTEXT = "ViewportStrip"

# (source file, table declaration line) — the first quoted string of every row.
TABLES = [
    ("src/viewport/scene/shademode.h", r"inline constexpr ShadeMode kShadeModes\[\] = \{"),
    ("src/viewport/viewportstrip.cpp", r"constexpr ViewEntry kViewEntries\[\] = \{"),
]
ROW_RE = re.compile(r'\{"([^"]+)"')


def collect_names():
    """The runtime picker names, in table order (duplicates dropped)."""
    names = []
    for rel_path, decl in TABLES:
        path = os.path.join(REPO_ROOT, rel_path)
        with open(path, encoding="utf-8") as f:
            lines = f.readlines()
        start = None
        for i, line in enumerate(lines):
            if re.search(decl, line):
                start = i + 1
                break
        if start is None:
            sys.exit(f"error: table declaration not found in {rel_path}")
        block = []
        for line in lines[start:]:
            if line.strip() == "};":
                break
            block.append(line)
        # finditer over the whole block: a row may pack two entries per line.
        for m in ROW_RE.finditer("".join(block)):
            if m.group(1) not in names:
                names.append(m.group(1))
    return names


def analyze(text):
    """(context index by exact <name> text, its existing sources)."""
    root = ET.fromstring(text)
    for i, c in enumerate(root.findall("context")):
        n = c.find("name")
        if n is not None and n.text == TS_CONTEXT:
            return i, {m.findtext("source") for m in c.findall("message")}
    return None, set()


def seed(ts_path, names):
    with open(ts_path, encoding="utf-8") as f:
        text = f.read()

    ctx_index, present = analyze(text)
    missing = [n for n in names if n not in present]
    if not missing:
        print(f"ok   {os.path.relpath(ts_path, REPO_ROOT)}: all {len(names)} runtime strings present")
        return 0

    lines = text.splitlines()
    entries = []
    for source in missing:
        entries += [
            "        <message>",
            f"            <source>{source}</source>",
            '            <translation type="unfinished"></translation>',
            "        </message>",
        ]

    if ctx_index is None:
        # lupdate never produced the bare context (no tr() literal uses it):
        # append the section after the last existing context, matching layout.
        last_ctx_close = max(i for i, l in enumerate(lines) if l.strip() == "</context>")
        at = last_ctx_close + 1
        lines[at:at] = ["", "    <context>", f"        <name>{TS_CONTEXT}</name>"] + entries + [
            "    </context>"
        ]
    else:
        # Insert right after the context's opening tag, before its first message.
        name_line = next(
            i for i, l in enumerate(lines) if l.strip() == f"<name>{TS_CONTEXT}</name>"
        )
        lines[name_line + 1:name_line + 1] = entries

    with open(ts_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"seed {os.path.relpath(ts_path, REPO_ROOT)}: +{len(missing)} runtime strings "
          f"({', '.join(missing)})")
    return len(missing)


def main():
    names = collect_names()
    print(f"{len(names)} runtime picker names from the source tables")

    if len(sys.argv) > 1:
        catalogs = sys.argv[1:]
    else:
        catalogs = sorted(glob.glob(os.path.join(REPO_ROOT, "translations", "*.ts")))
    if not catalogs:
        sys.exit("error: no catalogs found under translations/")

    total = sum(seed(p, names) for p in catalogs)
    print(f"done: {total} entries added")


if __name__ == "__main__":
    main()
