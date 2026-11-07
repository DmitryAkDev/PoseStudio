#!/usr/bin/env python3
"""Clear translations in a .ts catalog (keep the XML structure).

By default every translation is cleared — use before re-rolling a whole
catalog with an updated prompt or a different model (translate.py only
fills empty entries, so existing strings must be emptied first):

    clear.py translations/ru.ts
    python3 tools/i18n/translate.py translations/<lang>.ts

With --sources, only the listed sources are cleared (a targeted re-roll:
the reviewed bulk of the catalog survives). The worklist is a JSON array
of exact source texts, unescaped the way dump.py prints them (real
newlines, plain quotes):

    clear.py translations/ru.ts --sources worklist.json

Warning: clearing discards the affected translations, including any human
review or verification done on them. A re-run with a different model
produces different results (wording and terminology drift), so the new
catalog needs a fresh review pass before it ships.
"""

import argparse
import html
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("catalog", type=Path, help="path to the .ts catalog")
    parser.add_argument("--sources", type=Path, metavar="WORKLIST",
                        help="JSON array of exact source texts: clear only these, keep the rest")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt (for scripting)")
    args = parser.parse_args()

    if not args.catalog.exists():
        sys.exit(f"catalog not found: {args.catalog}")

    wanted = None
    if args.sources:
        wanted = {s.strip() for s in json.loads(args.sources.read_text(encoding="utf-8"))}
        if not wanted:
            sys.exit(f"empty worklist: {args.sources}")

    tree = ET.parse(args.catalog)
    messages = list(tree.getroot().iter("message"))
    # <source> text carries HTML entities (&apos;, &quot;, &amp;) — unescape before matching.
    targets = []
    leftover = set(wanted or ())
    for msg in messages:
        src = msg.find("source")
        if src is None:
            continue
        text = html.unescape(src.text or "")
        if wanted is None or text.strip() in wanted:
            targets.append(msg)
            leftover.discard(text.strip())
    if wanted is not None and leftover:
        sys.exit("worklist sources not found in the catalog:\n  " +
                 "\n  ".join(repr(s) for s in sorted(leftover)))

    filled = sum(1 for m in targets for t in m.iter("translation") if (t.text or "").strip())
    if filled and not args.yes:
        scope = (f"{len(targets)} message(s) from {args.sources.name}"
                 if wanted is not None else "ALL entries")
        print(f"{args.catalog}: {filled} of {scope} are translated.")
        print("Clearing discards them, including any human review work. A re-run")
        print("with a different model produces different results — the new")
        print("catalog needs a fresh review pass before it ships.")
        answer = input("Clear anyway? [y/N] ").strip().lower()
        if answer not in ("y", "yes"):
            sys.exit("aborted — catalog unchanged")

    cleared = 0
    for msg in targets:
        for t in msg.iter("translation"):
            t.text = ""
            t.set("type", "unfinished")
            cleared += 1
    tree.write(args.catalog, encoding="UTF-8", xml_declaration=True)
    print(f"{args.catalog}: cleared {cleared} translations")


if __name__ == "__main__":
    main()
