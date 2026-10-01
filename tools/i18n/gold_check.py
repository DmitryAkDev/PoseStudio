#!/usr/bin/env python3
"""Check a gold set against a .ts catalog (regression check for drift).

Every gold row pins a source string and its expected translation. This
script reads both, normalizes them the same way (HTML entities unescaped,
the two-character \\t of the TSV read back as a real tab), and reports:

- a row whose source is missing from the catalog (lupdate dropped it),
- a row whose catalog translation differs from expected_ru (a regression).

Usage:
    gold_check.py <catalog.ts> [--gold FILE]

The language is inferred from the catalog file name (Qt convention:
ru.ts -> ru) and the gold set defaults to gold/<lang>.gold.tsv next to
this script; --gold overrides the path. Exits 1 on any miss or
mismatch, 0 when every row matches."""

import argparse
import html
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent


def normalize(text: str) -> str:
    """Read a stored string back to its live form.

    The .ts and the TSV store strings with HTML entities as written
    (&amp;, couldn&apos;t) and embedded tabs/newlines escaped as two-character
    \\t / \\n (the escaping dump.py prints); unescape all of it before comparing."""
    return html.unescape(text or "").replace("\\t", "\t").replace("\\n", "\n")


def catalog_map(catalog: Path) -> dict:
    """source text -> translation text for every message in the catalog."""
    root = ET.parse(catalog).getroot()
    entries = {}
    for message in root.iter("message"):
        source_el = message.find("source")
        source = "".join(source_el.itertext()) if source_el is not None else ""
        translation_el = message.find("translation")
        value = ("".join(translation_el.itertext())
                 if translation_el is not None else "")
        entries.setdefault(source, value)
    return entries


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("catalog", type=Path, help="path to the .ts catalog")
    parser.add_argument("--gold", type=Path, default=None,
                        help="gold set path (default: gold/<lang>.gold.tsv "
                             "next to this script)")
    args = parser.parse_args()

    lang = args.catalog.stem
    gold_file = args.gold or (HERE / "gold" / f"{lang}.gold.tsv")
    if not gold_file.exists():
        sys.exit(f"gold set not found: {gold_file}")

    entries = catalog_map(args.catalog)
    lines = [line for line in gold_file.read_text(encoding="utf-8").splitlines()
             if line.strip()]
    rows = [line.split("\t") for line in lines[1:]]

    missing, mismatched = [], []
    for row in rows:
        if len(row) < 3:
            sys.exit(f"malformed gold row (need id/source/expected): {row}")
        gid, source, expected = row[0], normalize(row[1]), normalize(row[2])
        if source not in entries:
            missing.append((gid, row[1]))
            continue
        actual = normalize(entries[source])
        if actual != expected:
            mismatched.append((gid, row[1], expected, actual))

    print(f"gold {lang}: {len(rows)} rows checked against {args.catalog.name}")
    for gid, source in missing:
        print(f"  MISS     {gid}: source not in catalog: {source!r}")
    for gid, source, expected, actual in mismatched:
        print(f"  MISMATCH {gid}: {source!r}\n"
              f"             expected: {expected!r}\n"
              f"             actual:   {actual!r}")
    if missing or mismatched:
        sys.exit(1)
    print("all rows match")


if __name__ == "__main__":
    main()
