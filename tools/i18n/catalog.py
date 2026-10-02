"""Shared Qt Linguist catalog loading for the i18n tools.

A .ts catalog is the single source of truth for the on-screen names of a
language: both the UI and the manual must quote those names exactly as the
catalog translates them, so every tool that needs a name's target-language
form resolves it here instead of carrying its own copy.

The catalog is data, not code - a new language is a new .ts file, and nothing
in these tools changes when it appears."""

import xml.etree.ElementTree as ET
from pathlib import Path


def load_catalog(ts_path: Path) -> dict:
    """Parse a translations/<lang>.ts file into a {source: translation} map.

    Returns an empty dict when the catalog is absent or unreadable - callers
    then skip the catalog-driven check (a build without catalogs keeps the
    English text anyway). Only messages with both a source and a non-empty
    translation are kept: an empty translation means "not translated yet" and
    must not be treated as a form to substitute."""
    if not ts_path.exists():
        return {}
    try:
        tree = ET.parse(ts_path)
    except ET.ParseError:
        return {}
    catalog = {}
    for msg in tree.getroot().iter("message"):
        src = (msg.findtext("source") or "").strip()
        tr = (msg.findtext("translation") or "").strip()
        if src and tr:
            catalog[src] = tr
    return catalog


def bare_form(source: str) -> str:
    """The on-screen form of a catalog source string.

    Menu entries carry their accelerator after a tab ("Pin Joint\tP"); only the
    part before the tab is shown, so that is the form the manual quotes. A
    source without an accelerator is returned as-is."""
    return source.split("\t", 1)[0]
