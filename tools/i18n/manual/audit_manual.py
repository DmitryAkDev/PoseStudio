#!/usr/bin/env python3
"""Audits a translated manual tree (docs/i18n/<lang>/manual/) against the English original.

The audit is the post-gate of tools/i18n/manual/translate_manual.py: a green run
passed both the per-page guards and this whole-tree check. It reports, and its
exit code equals the number of errors (0 = the tree is consistent):

    manifest parity   the language manifest repeats the English one: same ids and
                      files, in the same order; every title non-empty — except the
                      "What's New" entry, whose title stays English on purpose
    page presence     every listed page exists in the tree (and vice versa)
    heading parity    per page, the number and levels of the `##` headings match
                      the English original one for one
    anchor links      every link in the tree resolves against the tree's own
                      headings (the app renders anchors with HelpManual::slugFor,
                      so a RU anchor must exist as a RU heading slug)
    code parity       per page, the fenced code blocks and the inline code spans
                      are identical to the English original (code is never
                      translated)
    gold terminology  every row of tools/i18n/gold/<lang>.gold.tsv whose EN term
                      still occurs in the English manual must have its expected
                      RU form present somewhere in the tree

Usage:
    audit_manual.py [--lang ru] [--root DIR]
"""

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent                 # tools/i18n/manual
I18N = HERE.parent                                     # tools/i18n
GOLD_DIR = I18N / "gold"

HEADING_RE = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$")
FENCE_RE = re.compile(r"^\s*(```|~~~)")
LINK_RE = re.compile(r"(!?)\[([^\]]*)\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")

CHANGELOG_FILE = "changelog.md"


def slug_for(text: str) -> str:
    """HelpManual::slugFor (src/help/helpmanual.cpp): lower case; letters, digits,
    '_' and '-' kept; spaces -> '-'; the rest dropped. Mirrored here so the audit
    resolves anchors exactly as the app renders them."""
    text = text.replace("`", "").replace("*", "").strip().lower()
    out = []
    for c in text:
        if c.isalnum() or c in "_-":
            out.append(c)
        elif c.isspace():
            out.append("-")
    return "".join(out)


def headings_of(text: str):
    """[(level, text, slug)] of every ATX heading outside fenced code; slugs are
    numbered like the app's (the second occurrence gets '-1')."""
    out, seen, in_fence = [], {}, False
    for line in text.splitlines():
        if FENCE_RE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        m = HEADING_RE.match(line)
        if not m:
            continue
        htext = m.group(2).replace("`", "").replace("*", "")
        slug = slug_for(htext)
        n = seen.get(slug, 0)
        seen[slug] = n + 1
        if n:
            slug = f"{slug}-{n}"
        out.append((len(m.group(1)), htext, slug))
    return out


def links_of(text: str):
    """[(is_image, text, target)] of every link outside fenced code."""
    out, in_fence = [], False
    for line in text.splitlines():
        if FENCE_RE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        for m in LINK_RE.finditer(line):
            out.append((m.group(1) == "!", m.group(2), m.group(3)))
    return out


def fence_blocks(text: str):
    """The content of every fenced code block, in order."""
    blocks, current, in_fence = [], None, False
    for line in text.splitlines():
        if FENCE_RE.match(line):
            if in_fence and current is not None:
                blocks.append("\n".join(current))
                current = None
            in_fence = not in_fence
            continue
        if in_fence:
            if current is None:
                current = []
            current.append(line)
    if current:
        blocks.append("\n".join(current))
    return blocks


def inline_code(text: str):
    """The set of inline code spans (backtick content) outside fenced code."""
    out, in_fence = set(), False
    for line in text.splitlines():
        if FENCE_RE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        for m in re.finditer(r"`([^`]+)`", line):
            out.add(m.group(1))
    return out


def flatten(pages):
    """The manifest pages in order, children included."""
    out = []
    for page in pages:
        out.append(page)
        out.extend(flatten(page.get("children", [])))
    return out


def read_json(path: Path, errors: list, what: str):
    if not path.exists():
        errors.append(f"{what}: {path} does not exist")
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        errors.append(f"{what}: {path} does not parse: {exc}")
        return None


def audit_manifest(en_pages: list, ru_pages: list, ru_tree: Path,
                    en_titles: dict, errors: list) -> None:
    """5.1 — manifest parity: ids and files 1:1 in the same order; titles non-empty."""
    for i, (en_p, ru_p) in enumerate(zip(en_pages, ru_pages)):
        if en_p.get("id") != ru_p.get("id"):
            errors.append(f"manifest: entry {i} id differs ({en_p.get('id')} -> {ru_p.get('id')})")
        if en_p.get("file") != ru_p.get("file"):
            errors.append(f"manifest: entry {i} file differs ({en_p.get('file')} -> {ru_p.get('file')})")
    if len(en_pages) != len(ru_pages):
        errors.append(f"manifest: page count differs ({len(en_pages)} -> {len(ru_pages)})")

    for ru_p in ru_pages:
        file_name = ru_p.get("file", "")
        title = (ru_p.get("title") or "").strip()
        if not title:
            errors.append(f"manifest: page {ru_p.get('id')} has an empty title")
        elif file_name == CHANGELOG_FILE and title != en_titles.get(file_name, ""):
            errors.append(f"manifest: the changelog title must stay English "
                          f"({en_titles.get(file_name)!r}, got {title!r})")

    # Page presence in both directions.
    for ru_p in ru_pages:
        file_name = ru_p.get("file", "")
        # The changelog is English-only: it has no per-language copy, so HelpManual falls back to
        # the English tree at run time. Do not require it to exist in this language's folder.
        if file_name and file_name != CHANGELOG_FILE and not (ru_tree / file_name).exists():
            errors.append(f"manifest: page {ru_p.get('id')} lists {file_name}, which does not exist")
    listed = {p.get("file") for p in ru_pages}
    for path in sorted(ru_tree.glob("*.md")):
        if path.name != "README.md" and path.name not in listed:
            errors.append(f"{path.name} is in the tree but not in its manifest")


def audit_page_structure(en_text: str, ru_text: str, file_name: str, errors: list) -> None:
    """5.1 — per page: the `##` heading count and levels match the English original."""
    en_h = [h for h in headings_of(en_text) if h[0] >= 2]
    ru_h = [h for h in headings_of(ru_text) if h[0] >= 2]
    if len(en_h) != len(ru_h):
        errors.append(f"{file_name}: heading count differs ({len(en_h)} -> {len(ru_h)})")
        return
    for (el, et, _), (rl, rt, _) in zip(en_h, ru_h):
        if el != rl:
            errors.append(f"{file_name}: heading level differs at {et!r} ({el} -> {rl})")
            return


def audit_anchors(ru_files: dict, errors: list) -> None:
    """5.2 — every link in the tree resolves against the tree's own headings."""
    anchors = {name: {h[2] for h in headings_of(text)} for name, text in ru_files.items()}
    for name, text in ru_files.items():
        for is_image, _label, target in links_of(text):
            if re.match(r"^(https?|mailto|qrc):", target):
                continue
            file_part, _, anchor = target.partition("#")
            page = file_part or name
            # The changelog is English-only: a link to it resolves against the English tree at
            # run time (HelpManual's fallback), so it is not required to be a page of this tree.
            if page == CHANGELOG_FILE:
                continue
            if page not in ru_files:
                errors.append(f"{name}: link to {target}, which is not a page of the tree")
                continue
            if anchor and anchor not in anchors[page]:
                errors.append(f"{name}: link to {target}: no such heading in the tree "
                              f"(anchors: {', '.join(sorted(anchors[page])[:12])}"
                              f"{'…' if len(anchors[page]) > 12 else ''})")


def audit_code_parity(en_text: str, ru_text: str, file_name: str, errors: list) -> None:
    """5.2 — code is never translated: fences and inline spans are identical to EN."""
    if fence_blocks(en_text) != fence_blocks(ru_text):
        errors.append(f"{file_name}: fenced code blocks differ from the English original")
    en_spans, ru_spans = inline_code(en_text), inline_code(ru_text)
    if en_spans != ru_spans:
        lost = sorted(en_spans - ru_spans)
        extra = sorted(ru_spans - en_spans)
        errors.append(f"{file_name}: inline code spans differ (lost: {lost}, extra: {extra})")


def audit_gold(lang: str, en_corpus: str, ru_corpus: str, errors: list) -> None:
    """5.3 — gold terminology hits: a row whose EN term still occurs in the manual
    must have its expected RU form present in the tree."""
    gold = GOLD_DIR / f"manual_{lang}.gold.tsv"
    if not gold.exists():
        return  # an empty set is a valid state (nothing to check yet)
    for line in gold.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("id\t") or line.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) < 3:
            errors.append(f"gold: malformed row in {gold.name}: {line!r}")
            continue
        _row_id, source, expected_ru = parts[0], parts[1], parts[2]
        if not source or not expected_ru:
            continue
        if source not in en_corpus:
            continue  # the term no longer occurs in the English manual — the row is dormant
        if expected_ru not in ru_corpus:
            errors.append(f"gold {_row_id}: {source!r} -> {expected_ru!r} is missing from the tree")


MENU_TITLES = ("File", "Edit", "View", "Help")

# On-screen tool / menu / setting names that must appear in their RU catalog form when
# referenced in the manual prose (the same set the pipeline's deterministic pass and the
# translator prompt carry). Checked case-sensitively: a lowercase ordinary word is not a UI
# reference, and these are matched whole so "Reset Joint" does not fire inside "Reset All".
UI_LABELS = (
    ("Turn Joint with Mouse", "Поворачивать сустав мышью"),
    ("Reset Selected Joint", "Сбросить выбранный сустав"),
    ("Reset Limb", "Сбросить конечность"),
    ("Reset All", "Сбросить всё"),
    ("Reset Joint", "Сбросить сустав"),
    ("Mirror Pose", "Отразить позу"),
    ("Mirror Limb to Other Side", "Отразить конечность на другую сторону"),
    ("Show Skeleton", "Показать скелет"),
    ("Image-Based Lighting", "Освещение на основе изображений"),
    ("Preferences", "Настройки"),
    ("Undo", "Отменить"),
    ("Redo", "Повторить"),
    ("Import", "Импорт"),
    ("Bend", "Изгиб"),
    ("Twist", "Скручивание"),
    ("Side-Side", "Боковой изгиб"),
    ("Specular", "Блик"),
    ("Exposure", "Экспозиция"),
)


def load_catalog(root: Path, lang: str):
    """Parse translations/<lang>.ts into a {source: translation} map.

    Returns an empty dict when the catalog is absent or unreadable - the menu check is
    then skipped (a build without catalogs keeps the English manual anyway)."""
    ts = root / "translations" / f"{lang}.ts"
    if not ts.exists():
        return {}
    try:
        tree = ET.parse(ts)
    except ET.ParseError:
        return {}
    catalog = {}
    for msg in tree.getroot().iter("message"):
        src = msg.findtext("source") or ""
        tr = msg.findtext("translation") or ""
        if src and tr:
            catalog[src] = tr
    return catalog


def audit_menu_ui(ru_corpus: str, catalog: dict, errors: list) -> None:
    """5.4 - a top-level menu title in a menu-reference context must use the catalog's
    RU form, not the English one. Catches the drift where a page keeps "File → Save"
    while another writes "Файл": both are invisible to the corpus-level gold check.

    A menu-reference context is the same one the pipeline's deterministic pass rewrites:
    the start of an arrow chain, right after the word "меню"/"menu", before the word
    "menu", or in an enumeration of top-level titles."""
    if not catalog:
        return  # no catalog - nothing to compare against
    for en in MENU_TITLES:
        ru = catalog.get(en)
        if not ru:
            continue  # the item is not (yet) in the catalog - not a drift we can judge
        patterns = (
            r"\b" + en + r"(?=\s*(?:→|->))",
            r"(меню|menu)\s+" + en + r"\b",
            r"\b" + en + r"(?=\s+menu\b)",
            # Enumeration of top-level titles ("File, Edit, View and Help"): capitalized only
            # (no IGNORECASE) and not preceded by a letter/digit, so an ordinary word such as
            # "pose file." is not mistaken for the menu title.
            r"(?<![A-Za-z0-9])" + en + r"(?=\s*(?:,| and | and\b|\.))",
        )
        hits = 0
        for i, pat in enumerate(patterns):
            # The enum pattern (last) is case-sensitive: only the capitalized menu title counts.
            flags = 0 if i == len(patterns) - 1 else re.IGNORECASE
            hits += len(re.findall(pat, ru_corpus, flags=flags))
        if hits:
            errors.append(
                f"menu-ui: {en!r} appears {hits}x in a menu context; use the catalog "
                f"form {ru!r}")

def audit_ui_labels(ru_corpus: str, errors: list) -> None:
    """5.5 - an on-screen UI name in the prose must use its RU catalog form, not the English
    one. Catches drift the menu-title check (5.4) does not see: a tool or setting name left in
    English while the rest of the page is Russian.

    Each label is matched whole and case-sensitively, so a lowercase ordinary word ("bend",
    "twist") is not a UI reference and "Reset Joint" does not fire inside "Reset All". One error
    per label with its occurrence count."""
    for en, ru in UI_LABELS:
        hits = len(re.findall(r"\b" + re.escape(en) + r"\b", ru_corpus))
        if hits:
            errors.append(
                f"ui-label: {en!r} appears {hits}x in the prose; use the catalog form {ru!r}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Audit a translated manual tree against the English original")
    parser.add_argument("--lang", default="ru", help="language code (default: ru)")
    parser.add_argument("--root", type=Path, default=HERE.parent.parent.parent,
                        help="repository root (default: auto)")
    args = parser.parse_args()

    root = args.root.resolve()
    lang = args.lang.lower()
    errors: list = []

    en_tree = root / "docs" / "manual"
    ru_tree = root / "docs" / "i18n" / lang / "manual"

    en_doc = read_json(en_tree / "manual.json", errors, "EN manifest")
    ru_doc = read_json(ru_tree / "manual.json", errors, f"{lang} manifest")
    if en_doc is None or ru_doc is None:
        return finish(errors)

    en_pages = flatten(en_doc.get("pages", []))
    ru_pages = flatten(ru_doc.get("pages", []))
    en_titles = {p.get("file"): p.get("title", "") for p in en_pages}

    # 5.1 — manifest parity and page presence.
    audit_manifest(en_pages, ru_pages, ru_tree, en_titles, errors)

    # The pages themselves: EN original vs the RU translation.
    ru_files = {}
    for ru_p in ru_pages:
        file_name = ru_p.get("file", "")
        path = ru_tree / file_name
        if path.exists():
            ru_files[file_name] = path.read_text(encoding="utf-8")

    en_corpus, ru_corpus = "", ""
    for en_p in en_pages:
        file_name = en_p.get("file", "")
        en_path = en_tree / file_name
        if not en_path.exists():
            continue  # the EN tree is checked by tools/manual/check_manual.py
        en_text = en_path.read_text(encoding="utf-8")
        en_corpus += en_text + "\n"
        ru_text = ru_files.get(file_name)
        if ru_text is None:
            continue  # already reported by the manifest parity check
        ru_corpus += ru_text + "\n"
        audit_page_structure(en_text, ru_text, file_name, errors)
        audit_code_parity(en_text, ru_text, file_name, errors)

    # 5.2 — anchors resolve inside the tree.
    audit_anchors(ru_files, errors)

    # 5.3 — gold terminology.
    audit_gold(lang, en_corpus, ru_corpus, errors)

    # 5.4 - menu items in the prose use the catalog's RU form (menu x UI consistency).
    audit_menu_ui(ru_corpus, load_catalog(root, lang), errors)

    # 5.5 - on-screen UI names in the prose use their RU catalog form (tool / setting drift).
    audit_ui_labels(ru_corpus, errors)

    return finish(errors)


def finish(errors: list) -> int:
    for e in errors:
        print("ERROR: " + e)
    if not errors:
        print("audit: OK (tree consistent)")
    else:
        print(f"audit: {len(errors)} error(s)")
    return len(errors)


if __name__ == "__main__":
    sys.exit(main())
