#!/usr/bin/env python3
"""Clear every translation in a .ts catalog (keep the XML structure).

Use before re-rolling a whole catalog with an updated prompt or a different
model — translate.py only fills empty entries, so existing strings must be
emptied first:

    clear.py translations/ru.ts
    python3 tools/i18n/translate.py translations/<lang>.ts

Warning: this discards every existing translation, including any human review
or verification done on them. A re-run with a different model produces
different results (wording and terminology drift), so the new catalog needs
a fresh review pass before it ships.
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("catalog", type=Path, help="path to the .ts catalog")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt (for scripting)")
    args = parser.parse_args()

    if not args.catalog.exists():
        sys.exit(f"catalog not found: {args.catalog}")
    tree = ET.parse(args.catalog)
    translations = list(tree.getroot().iter("translation"))
    filled = sum(1 for t in translations if (t.text or "").strip())

    if filled and not args.yes:
        print(f"{args.catalog}: {filled} of {len(translations)} entries are translated.")
        print("Clearing discards them, including any human review work. A re-run")
        print("with a different model produces different results — the new")
        print("catalog needs a fresh review pass before it ships.")
        answer = input("Clear anyway? [y/N] ").strip().lower()
        if answer not in ("y", "yes"):
            sys.exit("aborted — catalog unchanged")

    cleared = 0
    for t in translations:
        t.text = ""
        t.set("type", "unfinished")
        cleared += 1
    tree.write(args.catalog, encoding="UTF-8", xml_declaration=True)
    print(f"{args.catalog}: cleared {cleared} translations")


if __name__ == "__main__":
    main()
