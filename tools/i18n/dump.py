#!/usr/bin/env python3
"""Dump a .ts catalog for manual review.

Prints every message as a source -> translation pair with control characters
made visible, so a human can read through the whole translation in one file:

    [MenuManager] Delete Selected Object\tDel
        -> Удалить выбранный объект\tDel

Usage:
    dump.py <catalog.ts> [--empty] [--context NAME] [--out FILE]

--empty      only messages with an empty translation (progress check)
--context    filter by context name (substring match)
--out        write to FILE instead of stdout
--audit    consistency check only; exit 1 if one source has several translations

The dump always ends with a consistency audit: source strings that occur
more than once but were translated differently are listed (the same UI
string must read the same everywhere)."""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def visible(text: str) -> str:
    return (text or "").replace("\t", "\\t").replace("\n", "\\n")


def find_divergent(root) -> dict:
    """Source strings with more than one distinct filled translation."""
    seen = {}
    for m in root.iter("message"):
        src = m.findtext("source") or ""
        t = m.find("translation")
        tr = (t.text or "").strip() if t is not None else ""
        if not tr:
            continue
        seen.setdefault(src, set()).add(tr)
    return {s: v for s, v in seen.items() if len(v) > 1}

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("catalog", type=Path, help="path to the .ts catalog")
    parser.add_argument("--empty", action="store_true",
                        help="only messages with an empty translation")
    parser.add_argument("--context", default=None,
                        help="filter by context name (substring match)")
    parser.add_argument("--out", type=Path, default=None,
                        help="write to FILE instead of stdout")
    parser.add_argument("--audit", action="store_true",
                        help="consistency check only; exit 1 on divergent translations")
    args = parser.parse_args()

    if not args.catalog.exists():
        sys.exit(f"catalog not found: {args.catalog}")
    root = ET.parse(args.catalog).getroot()

    divergent = find_divergent(root)
    if args.audit:
        if divergent:
            print(f"CONSISTENCY: {len(divergent)} source(s) translated differently:")
            for src in sorted(divergent):
                print(f"  {visible(src)!r}")
                for tr in sorted(divergent[src]):
                    print(f"      {visible(tr)}")
            sys.exit(1)
        print("consistency: OK (every source has a single translation)")
        return
    lines = []
    total = shown = empty = 0
    for ctx in root.iter("context"):
        name = ctx.findtext("name") or ""
        if args.context and args.context not in name:
            continue
        for m in ctx.findall("message"):
            src = m.findtext("source") or ""
            t = m.find("translation")
            tr = t.text if t is not None else ""
            total += 1
            is_empty = not (tr or "").strip()
            if is_empty:
                empty += 1
            if args.empty and not is_empty:
                continue
            shown += 1
            lines.append(f"[{name}] {visible(src)}")
            lines.append(f"    -> {visible(tr)}")
    if divergent:
        lines.append("")
        lines.append(f"CONSISTENCY: {len(divergent)} source(s) translated differently:")
        for src in sorted(divergent):
            lines.append(f"  {visible(src)!r}")
            for tr in sorted(divergent[src]):
                lines.append(f"      {visible(tr)}")
    footer = [f"{shown} of {total} messages shown, {empty} empty"]
    text = "\n".join(lines + ["", *footer])
    if args.out:
        args.out.write_text(text + "\n", encoding="utf-8")
        print(f"wrote {args.out} ({len(lines)} lines)")
    else:
        print(text)


if __name__ == "__main__":
    main()
