#!/usr/bin/env python3
"""Translate the English user manual (docs/manual/) into a language tree.

Everything is driven from this directory and tools/i18n/ — the script reads
nothing else:

    docs/manual/manual.json     the English manifest (the single source of truth
                                for the page list, order and file names)
    docs/manual/<file>          the English pages (~60 lines each)
    .env / locales/<lang>/.env  model config (see tools/i18n/README.md)
    locales/<lang>/headings.md  system prompt: one heading at a time (pass 1)
    locales/<lang>/manual.md    system prompt: a whole page (pass 2)

Three passes per page, the third without an LLM:

    pass 1  HEADINGS — every `##` heading of the page, translated one by one
            (short strings, their own prompt); from the answers the slug map is
            built with the app's slug rules (HelpManual::slugFor: lower case;
            letters/digits/_/- kept; spaces to hyphens; duplicates numbered)
    pass 2  BODY — the whole page in one call; the prompt gets the English text
            plus this page's "EN heading -> RU heading" table, and the model
            does not invent headings. The answer is checked by deterministic
            markdown guards (link targets, code fences, heading set) before it
            is accepted; a failure goes back to the model with the reason
            (1-2 retries), after which the page is left untranslated and the
            run stops
    pass 3  LINKS — anchors rewritten deterministically from the slug map:
            `file.md#en-anchor` -> `file.md#ru-anchor`, same-page `#anchor` the
            same way; the file part never changes (file names are shared by
            the trees). An anchor missing from the map is left as-is — the
            audit reports it and a human decides.

Idempotency: the sidecar .source_hashes in the language tree stores, per page,
the SHA-256 of the normalized English source at translation time (line strip +
unified newlines — one function, used for both writing and checking). Only
missing or stale pages are translated; a rerun with an unchanged English
manual makes zero LLM calls. `--dry-run` lists the missing/stale pages without
a single LLM call.

The page "What's New" (changelog.md) is not translated: the release notes are
English-only, so each language tree falls back to the English changelog at run
time (HelpManual's fallback), and it is excluded from .source_hashes on purpose.

A run ends with audit_manual.py as a post-gate: a green run passed both the
guards and the audit.

Usage:
    translate_manual.py <lang> [--dry-run] [--limit N] [--model NAME]
                        [--no-audit] [--root DIR]

    <lang>              language code; prompts come from locales/<lang>/
    --dry-run           list missing/stale pages, zero LLM calls, exit 0
    --limit N           translate at most N pages this run (0 = all)
    --model NAME        override the MODEL from .env
    --no-audit          skip the audit post-gate (the run is not green then)
    --root DIR          repository root (default: two levels up from here)
"""

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent                 # tools/i18n/manual
I18N = HERE.parent                                     # tools/i18n
sys.path.insert(0, str(I18N))

from llm_pool import load_model_config, llm_chat      # noqa: E402

HEADING_RE = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$")
FENCE_RE = re.compile(r"^\s*(```|~~~)")
LINK_RE = re.compile(r"(!?)\[([^\]]*)\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")

CHANGELOG_FILE = "changelog.md"
HASHES_NAME = ".source_hashes"
GUARD_RETRIES = 2


# --------------------------------------------------------------------------
# Normalization and hashing (one function, both sides of the comparison)
# --------------------------------------------------------------------------

def normalize_source(text: str) -> str:
    """The canonical form of an English source page.

    Lines are stripped and joined with a single newline, so a cosmetic EN edit
    (trailing spaces, CRLF) does not trigger a re-translation. The same
    function hashes the source at write time and checks it at run time."""
    return "\n".join(line.strip() for line in text.splitlines())


def source_hash(text: str) -> str:
    """SHA-256 of the normalized English source (hex)."""
    return hashlib.sha256(normalize_source(text).encode("utf-8")).hexdigest()


# --------------------------------------------------------------------------
# Markdown structure (mirrors tools/manual/check_manual.py and the app's
# HelpManual::slugFor — keep the two in sync)
# --------------------------------------------------------------------------

def slug_for(text: str) -> str:
    """HelpManual::slugFor: lower case; letters, digits, '_' and '-' kept; spaces -> '-'; the rest dropped."""
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
    numbered like the app's (the second "Joint pins" gets 'joint-pins-2')."""
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
        # The text as the rendered block will carry it: inline code marks and emphasis dropped.
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


def link_targets(text: str):
    """The set of link target files (anchor parts dropped) outside fenced code."""
    return {t.split("#")[0] for _, _, t in links_of(text)}


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


# --------------------------------------------------------------------------
# The manifest and the page list
# --------------------------------------------------------------------------

def load_manifest(root: Path, lang: str):
    """(en_pages, en_nested, ru_pages) — three views of the manifests.

    en_pages is the English manifest flattened in order (no children key — it is the
    pipeline's work list); en_nested is the same pages with their original nesting
    (the shape sync_manifest() and ensure_tree() build from); ru_pages is the language
    tree's manifest, flattened. A missing or malformed English manifest exits — there is
    nothing to translate against."""
    def read(path: Path):
        if not path.exists():
            sys.exit(f"manifest not found: {path}")
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            sys.exit(f"manifest does not parse: {path}: {exc}")

    en_doc = read(root / "docs" / "manual" / "manual.json")
    ru_path = root / "docs" / "i18n" / lang / "manual" / "manual.json"
    ru_doc = read(ru_path) if ru_path.exists() else None

    def flatten(pages):
        out = []
        for page in pages:
            # A flat copy without the children key: sync_manifest() re-nests from this list,
            # and a leftover 'children' on the parent would duplicate the child entries.
            entry = {k: v for k, v in page.items() if k != "children"}
            out.append(entry)
            out.extend(flatten(page.get("children", [])))
        return out

    en_pages = flatten(en_doc.get("pages", []))
    en_nested = en_doc.get("pages", [])  # the original nesting, for sync_manifest()
    ru_pages = flatten(ru_doc.get("pages", [])) if ru_doc else []
    return en_pages, en_nested, ru_pages


def pending_pages(root: Path, en_pages: list, tree: Path) -> list:
    """The pages that need a translation: missing from the tree or stale
    (hash of the English source != the stored one). changelog.md is skipped —
    it is English-only and falls back to the English tree at run time."""
    hashes_path = tree / HASHES_NAME
    stored = {}
    if hashes_path.exists():
        try:
            stored = json.loads(hashes_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            sys.exit(f"sidecar does not parse (treat it as corrupt and rerun): {hashes_path}")
    pending = []
    for page in en_pages:
        file_name = page["file"]
        if file_name == CHANGELOG_FILE:
            continue
        src = root / "docs" / "manual" / file_name
        if not src.exists():
            sys.exit(f"English page listed in the manifest but missing: {src}")
        actual = source_hash(src.read_text(encoding="utf-8"))
        if not (tree / file_name).exists() or stored.get(file_name) != actual:
            pending.append((page, actual))
    return pending


# --------------------------------------------------------------------------
# Pass 1 — headings
# --------------------------------------------------------------------------

def translate_heading(cfg, prompt_text: str, heading: str, page_file: str):
    """One LLM call for one heading; the answer is a single line or None."""
    payload = {"source": heading, "context": f"heading on page {page_file}"}
    try:
        answer = llm_chat(cfg, prompt_text, json.dumps(payload, ensure_ascii=False))
    except Exception as exc:  # network, HTTP, JSON — all keep the page open
        print(f"  LLM error: {exc}", file=sys.stderr)
        return None
    answer = answer.strip()
    if len(answer) >= 2 and answer[0] == answer[-1] and answer[0] in "\"'":
        answer = answer[1:-1].strip()
    answer = answer.splitlines()[0].strip() if answer else ""
    return answer or None


def pass_headings(cfg, headings_prompt: str, page: dict, text: str):
    """Translate every `##` heading of the page; returns {en_slug: ru_text}.

    The `#` page title (level 1) is not translated here — it comes from the
    manifest's title. Duplicates are numbered exactly like the app, so the map
    keys match the anchors the rendered document carries."""
    result = {}
    headings = [h for h in headings_of(text) if h[0] >= 2]
    total = len(headings)
    for i, (level, en_text, en_slug) in enumerate(headings, start=1):
        print(f"    [{i}/{total}] {en_text!r}", flush=True)
        answer = translate_heading(cfg, headings_prompt, en_text, page["file"])
        if answer is None:
            return None
        result[en_slug] = answer
    return result


# --------------------------------------------------------------------------
# Pass 2 — the body, with markdown guards
# --------------------------------------------------------------------------

def heading_table_for(page_headings: dict, text: str) -> str:
    """The "EN heading -> RU heading" table for the page's prompt."""
    lines = ["| English | Russian |", "|---|---|"]
    for level, en_text, en_slug in headings_of(text):
        if level < 2 or en_slug not in page_headings:
            continue
        lines.append(f"| {en_text} | {page_headings[en_slug]} |")
    return "\n".join(lines)


def guard_body(en_text: str, ru_text: str, page_headings: dict, ru_titles: dict | None = None):
    """Deterministic markdown guards; returns a list of failure reasons (empty = pass).

    The structure must match the English original one for one: the set of link
    target files, every code fence block, and the heading set (levels and
    count — the texts are the translations from pass 1, so they are compared
    by position against the table, not against each other)."""
    problems = []
    en_targets, ru_targets = link_targets(en_text), link_targets(ru_text)
    if en_targets != ru_targets:
        lost = sorted(en_targets - ru_targets)
        extra = sorted(ru_targets - en_targets)
        problems.append(f"link target files changed (lost: {lost}, extra: {extra})")

    en_fences, ru_fences = fence_blocks(en_text), fence_blocks(ru_text)
    if en_fences != ru_fences:
        problems.append(f"code fences changed ({len(en_fences)} -> {len(ru_fences)} blocks or content)")

    # Inline code spans are content, not formatting: the set must match one for one.
    en_code, ru_code = inline_code(en_text), inline_code(ru_text)
    if en_code != ru_code:
        lost = sorted(en_code - ru_code)
        extra = sorted(ru_code - en_code)
        problems.append(f"inline code spans changed (lost: {lost}, extra: {extra})")

    # The visible text of a link to a manual page is prose: it must be translated,
    # not carried over from the English original (the target file stays verbatim).
    en_link_texts = {t for _, t, tg in links_of(en_text) if ".md" in tg.split("#")[0]}
    untrans = {}   # text -> target file, so the retry knows which page it names
    for _, t, tg in links_of(ru_text):
        fp = tg.split("#", 1)[0]
        if ".md" not in fp or t not in en_link_texts:
            continue
        # A title that is intentionally the same in both languages (the English
        # "What's New") needs no translation: the RU form IS the label.
        if ru_titles and ru_titles.get(fp) == t:
            continue
        untrans[t] = fp
    if untrans:
        hint = "; ".join(f"{t!r} (in a link to {f})" for t, f in sorted(untrans.items()))
        problems.append(
            "link texts left untranslated - translate the visible text to Russian, "
            "keep the target file as-is: " + hint)

    en_h = [h for h in headings_of(en_text)]
    ru_h = [h for h in headings_of(ru_text)]
    if len(en_h) != len(ru_h):
        problems.append(f"heading count changed ({len(en_h)} -> {len(ru_h)})")
    else:
        for (el, et, es), (rl, rt, rs) in zip(en_h, ru_h):
            if el != rl:
                problems.append(f"heading level changed at {et!r} ({el} -> {rl})")
                break

    # The `##` headings must be exactly the pass-1 translations, in order.
    if not problems:
        for (el, et, es), (rl, rt, rs) in zip(en_h, ru_h):
            if el < 2 or rl < 2:
                continue
            expected = page_headings.get(es)
            # Backticks are compared away: the two model passes may independently decide
            # whether a key name like Ctrl carries them, and that is a rendering detail,
            # not a structural difference (the rendered tree stays self-consistent).
            if expected is not None and rt.replace("`", "") != expected.replace("`", ""):
                problems.append(
                    f"heading {et!r} must be translated as {expected!r} (pass 1), got {rt!r}")
                break
    return problems


def pass_body(cfg, manual_prompt: str, page: dict, en_text: str,
              page_headings: dict, ru_titles: dict | None = None,
              en_pages: list | None = None) -> str | None:
    """Translate the whole page; guards + retry (GUARD_RETRIES), then None."""
    table = heading_table_for(page_headings, en_text)
    query = (f"Table of this page's headings (use the Russian column exactly where the "
             f"English heading stands):\n\n{table}\n\n"
             f"Translate this page:\n\n{en_text}")
    last_problem = None
    for attempt in range(GUARD_RETRIES + 1):
        try:
            answer = llm_chat(cfg, manual_prompt, query)
        except Exception as exc:
            print(f"  LLM error: {exc}", file=sys.stderr)
            return None
        answer = answer.strip()
        if answer.startswith("```"):
            # A model that wraps the page in a fence loses the first line; strip it.
            lines = answer.splitlines()
            if lines and lines[0].startswith("```"):
                lines = lines[1:]
            if lines and lines[-1].strip() == "```":
                lines = lines[:-1]
            answer = "\n".join(lines).strip()
        # Page-title links are deterministic: normalize them before the guard, so the
        # model is only held to translating prose labels (it treats page names as
        # proper nouns and will not translate them on retry either).
        if ru_titles and en_pages:
            answer = normalize_link_texts(answer, en_pages, ru_titles)
        problems = guard_body(en_text, answer, page_headings, ru_titles)
        if not problems:
            return answer
        last_problem = "; ".join(problems)
        print(f"  guard failed (attempt {attempt + 1}/{GUARD_RETRIES + 1}): {last_problem}",
              file=sys.stderr)
        query = (f"The previous translation broke the markdown structure:\n{last_problem}\n\n"
                 f"Fix it. The table and the page again:\n\n{table}\n\nTranslate this page:\n\n{en_text}")
    print(f"  guards exhausted for {page['file']}: {last_problem}", file=sys.stderr)
    return None


# --------------------------------------------------------------------------
# Pass 3 — deterministic anchor rewrite
# --------------------------------------------------------------------------

def build_anchor_maps(root: Path, en_pages: list, tree: Path):
    """(maps, texts) for every file whose RU page exists on disk and matches the
    English structure (heading count). Deterministic — no LLM.

    maps[file][en_slug] = ru_slug      — for rewriting link anchors
    texts[file][en_slug] = ru_heading  — for rewriting link labels that are the
                                      English heading text

    Cross-page links need the target page's map, not only the current page's:
    `importing.md#static-models-obj` from interface.md must resolve against the
    RU headings of importing.md. A file whose RU page is missing or structurally
    off stays out of the maps — its links are left for the audit to report."""
    maps, texts = {}, {}
    for page in en_pages:
        file_name = page["file"]
        en_path = root / "docs" / "manual" / file_name
        ru_path = tree / file_name
        if not (en_path.exists() and ru_path.exists()):
            continue
        en_h = headings_of(en_path.read_text(encoding="utf-8"))
        ru_h = headings_of(ru_path.read_text(encoding="utf-8"))
        if len(en_h) != len(ru_h):
            continue
        m, t = {}, {}
        for (el, et, es), (rl, rt, rs) in zip(en_h, ru_h):
            if el == rl:
                m[es] = rs
                t[es] = rt.replace("`", "")
        maps[file_name] = m
        texts[file_name] = t
    return maps, texts

def en_heading_texts(root: Path, en_pages: list) -> dict:
    """file -> set of EN heading texts (backticks stripped), for the whole tree.

    Used to spot link labels that are English headings (the model leaves them in
    English) so pass 3 can swap in the Russian heading. A heading's tail after its
    last colon is included too: `[Ctrl + drag the body]` names the heading
    `Moving the whole figure: Ctrl + drag the body`. Deterministic — no LLM."""
    out = {}
    for page in en_pages:
        file_name = page["file"]
        path = root / "docs" / "manual" / file_name
        if not path.exists():
            continue
        texts = set()
        for _, t, _ in headings_of(path.read_text(encoding="utf-8")):
            t = t.replace("`", "")
            texts.add(t)
            tail = t.rsplit(": ", 1)[-1]
            if tail != t:
                texts.add(tail)
        out[file_name] = texts
    return out


def rewrite_anchors(text: str, anchor_maps: dict, current_file: str,
                     en_headings_by_file: dict | None = None,
                     ru_heading_texts: dict | None = None) -> str:
    """`file.md#en-anchor` -> `file.md#ru-anchor`; same-page `#anchor` the same way.

    The file part never changes (file names are shared by the trees). An anchor
    missing from the map is left as-is — the audit reports it and a human
    decides. Links inside fenced code are untouched.

    When the anchor resolves, the visible label is checked too: if it is exactly
    the English heading text that the anchor names (a same-page "See [Heading]"
    " link), replace it with the Russian heading — the model leaves such labels
    in English otherwise. `en_headings_by_file` maps file -> set of EN heading
    texts (backticks stripped) for the whole tree; without it only anchors are
    rewritten."""
    def sub(match):
        bang, label, target = match.group(1), match.group(2), match.group(3)
        if "#" not in target:
            return match.group(0)
        file_part, anchor = target.split("#", 1)
        # A bare `#anchor` points at the page itself; a named file at its own map.
        key = file_part if file_part else current_file
        ru_anchor = anchor_maps.get(key, {}).get(anchor)
        if ru_anchor is None:
            return match.group(0)
        # Label == the English heading the anchor names -> use the RU heading text.
        ru_heading = (ru_heading_texts or {}).get(key, {}).get(anchor)
        if ru_heading and en_headings_by_file \
                and label.replace("`", "") in en_headings_by_file.get(key, ()):
            label = ru_heading
        return f"{bang}[{label}]({file_part}#{ru_anchor})"

    out, in_fence = [], False
    for line in text.splitlines():
        if FENCE_RE.match(line):
            in_fence = not in_fence
            out.append(line)
            continue
        if in_fence:
            out.append(line)
        else:
            out.append(LINK_RE.sub(sub, line))
    result = "\n".join(out)
    if text.endswith("\n"):
        result += "\n"
    return result


def normalize_link_texts(text: str, en_pages: list, ru_titles: dict) -> str:
    """Rewrite the visible text of page-title links to the Russian titles.

    The model treats page names as proper nouns and leaves `[Posing](posing.md)`
    in English. Where the link text is exactly the English title of its target
    page (or that title with a leading article), replace it with the manifest's
    Russian title — the same name the table of contents shows. Other visible
    texts (prose labels like "Ground button") are the model's job; the guard
    reports what it left untranslated. Deterministic — no LLM."""
    en_titles = {p["file"]: p["title"] for p in en_pages}

    # UI element names the model will not translate (it treats them as on-screen
    # proper nouns): map them to the Russian catalog / terminology-table forms.
    ui_labels = {
        "The Transform tab": "вкладка «Трансформация»",
        "Transform tab": "вкладка «Трансформация»",
        "Ground button": "кнопка Ground",
        "the Ground button": "кнопка Ground",
        "skeleton overlay": "оверлей скелета",
        "Asset Manager": "Менеджер ассетов",
        "pose files": "файлы поз",
        "pose file": "файл поз",
        # Half-translated calque the model produces for the same label.
        "pose файлы": "файлы поз",
    }

    def sub(match):
        bang, label, target = match.group(1), match.group(2), match.group(3)
        file_part = target.split("#", 1)[0]
        if file_part not in ru_titles:
            return match.group(0)
        en_title = en_titles.get(file_part)
        if en_title and label in (en_title, "The " + en_title):
            return f"{bang}[{ru_titles[file_part]}]({target})"
        if label in ui_labels:
            return f"{bang}[{ui_labels[label]}]({target})"
        return match.group(0)

    out, in_fence = [], False
    for line in text.splitlines():
        if FENCE_RE.match(line):
            in_fence = not in_fence
            out.append(line)
            continue
        if in_fence:
            out.append(line)
        else:
            out.append(LINK_RE.sub(sub, line))
    result = "\n".join(out)
    if text.endswith("\n"):
        result += "\n"
    return result

UI_NAME_REPLACEMENTS = (
    # (EN UI element name, RU form from the ru.ts catalog). Word-boundary
    # replacements over prose and headings: the model keeps on-screen names in
    # English, but the Russian app shows the catalog forms.
    ("Transform", "Трансформация"),
    ("Asset Manager", "Менеджер ассетов"),
    ("Environment", "Окружение"),
)

UI_LABEL_REPLACEMENTS = (
    # Exact-match app strings (shading modes, menu items, dialog labels).
    # Case-sensitive: they appear verbatim as on-screen names.
    ("PBR Shaded", "PBR-затенение"),
    ("Flat Texture Shaded", "Затенение текстурой (плоское)"),
    ("Texture Shaded", "Текстурное затенение"),
    ("Cartoon Shaded", "Мультяшное затенение"),
    ("Clay Shaded", "Глиняное затенение"),
    ("Lighting Only", "Только свет"),
    ("Hidden Line Wireframe", "Каркас с невидимыми линиями"),
    ("Wireframe", "Каркас"),
    ("Silhouette", "Силуэт"),
    ("Albedo", "Альбедо"),
    ("Ambient Occlusion", "Окружающая окклюзия (AO)"),
    ("Roughness Map", "Карта шероховатости"),
    ("Specular Only", "Только блики"),
    ("Normals", "Нормали"),
    ("UV Checker", "Проверка UV"),
    ("Send an anonymous install ping", "Отправлять анонимный сигнал об установке"),
    ("Check for updates at startup", "Проверять обновления при запуске"),
    ("Count camera changes as unsaved", "Считать смену камеры изменением сцены"),
    ("Add Asset Folder…", "Добавить папку ассетов…"),
    ("Remove Selected", "Удалить выделенное"),
    ("Factory Reset", "Сброс до заводских настроек"),
    ("Open Download Page", "Открыть страницу загрузки"),
    ("Skip This Version", "Пропустить эту версию"),
    ("Manage Asset Folders", "Управление папками ассетов"),
    ("New Collection", "Новая коллекция"),
    ("Find In Library", "Найти в библиотеке"),
    ("Browse Folder", "Обзор папки"),
    ("Add To Collection", "Добавить в коллекцию"),
    ("Move To Collection", "Переместить в коллекцию"),
    ("Copy To Collection", "Копировать в коллекцию"),
    ("Refresh", "Обновить"),
    ("Locate Content Folder…", "Найти папку контента…"),
    ("Content Folder Needed", "Нужна папка с контентом"),
    ("Reset Pose", "Сбросить позу"),
    ("Choose File…", "Выбрать файл…"),
    ("Discard Changes", "Отменить изменения"),
    ("Unpin All Joints", "Открепить все суставы"),
    ("Later", "Позже"),
)
def normalize_ui_names(text: str) -> str:
    """Replace EN UI element names with the RU catalog forms, word-boundary safe.

    Fenced code and inline code spans are content — untouched. Runs in pass 3,
    after anchors and link labels, so it also fixes headings; the audit then
    re-checks structure against the English original."""
    def sub_prose(line):
        # Protect markdown link targets: UI names must not rewrite file parts
        # ("environment.md" is a file name, not the Environment tab).
        targets = []
        def stash(m):
            bang, label, target = m.group(1), m.group(2), m.group(3)
            targets.append(target)
            return "%s[%s](\x00%d\x00)" % (bang, label, len(targets) - 1)
        line = re.sub(r"(!?)\[([^\]]*)\]\(([^)]+)\)", stash, line)
        for en, ru in UI_NAME_REPLACEMENTS:
            # Case-insensitive: the model may lowercase a name ("Вкладка environment").
            # Preserve the match's capitalization on the first letter.
            def repl(m, ru=ru):
                hit = m.group(0)
                if hit[0].islower():
                    return ru[0].lower() + ru[1:]
                return ru
            line = re.sub(r"\b" + re.escape(en) + r"\b", repl, line, flags=re.IGNORECASE)
        for en, ru in UI_LABEL_REPLACEMENTS:
            line = line.replace(en, ru)
        line = re.sub(
            r"(\x00\d+\x00)", lambda m: targets[int(m.group(1)[1:-1])], line)
        return line

    out, in_fence = [], False
    for line in text.splitlines():
        if FENCE_RE.match(line):
            in_fence = not in_fence
            out.append(line)
            continue
        if in_fence:
            out.append(line)
        else:
            # Protect inline code spans, then replace in the rest.
            parts, i = [], 0
            for m in re.finditer(r"`[^`]*`", line):
                parts.append(sub_prose(line[i:m.start()]))
                parts.append(m.group(0))
                i = m.end()
            parts.append(sub_prose(line[i:]))
            out.append("".join(parts))
    result = "\n".join(out)
    if text.endswith("\n"):
        result += "\n"
    return result

def set_page_title(text: str, title: str) -> str:
    """Rewrite the page's H1 line to `# <title>` (the translated manifest title).

    The first ATX heading of level 1 is replaced; a page without one gets the
    title inserted as its first line. Everything else is untouched."""
    lines = text.splitlines()
    for i, line in enumerate(lines):
        m = HEADING_RE.match(line)
        if m and len(m.group(1)) == 1:
            lines[i] = f"# {title}"
            return "\n".join(lines)
    if lines and lines[0].strip():
        # No H1 found: put the title first, then a blank line before the content.
        return f"# {title}\n\n" + "\n".join(lines)
    lines.insert(0, f"# {title}")
    return "\n".join(lines)

# --------------------------------------------------------------------------
# The language tree (manifest + sidecar)
# --------------------------------------------------------------------------

def ensure_tree(root: Path, lang: str, en_nested: list) -> Path:
    """Create docs/i18n/<lang>/manual/ with the manifest when absent.

    The manifest repeats the English one: same ids and files in the same order;
    titles are translated by the pipeline (a missing title is left for the
    next run). The "What's New" entry keeps its English title on purpose —
    the only conscious EN title in the tree's manifest."""
    tree = root / "docs" / "i18n" / lang / "manual"
    tree.mkdir(parents=True, exist_ok=True)
    manifest_path = tree / "manual.json"
    if manifest_path.exists():
        return tree
    def build(pages):
        out = []
        for page in pages:
            entry = {"id": page["id"], "title": "", "file": page["file"]}
            children = page.get("children")
            if children:
                entry["children"] = build(children)
            out.append(entry)
        return out
    doc = {
        "_comment": (f"Translated table of contents for the {lang} manual: ids and files repeat "
                     "the English manifest verbatim; titles are translated by tools/i18n/manual/"
                     "translate_manual.py. The 'What's New' entry keeps its English title on "
                     "purpose — the changelog is maintained in English."),
        "pages": build(en_nested),
    }
    manifest_path.write_text(json.dumps(doc, ensure_ascii=False, indent=2) + "\n",
                             encoding="utf-8")
    print(f"created {manifest_path.relative_to(root)} (titles to be translated)")
    return tree


def load_titles(tree: Path) -> dict:
    """file -> title from the language manifest (empty string when not set yet)."""
    manifest = tree / "manual.json"
    if not manifest.exists():
        return {}
    doc = json.loads(manifest.read_text(encoding="utf-8"))

    def flatten(pages):
        for page in pages:
            yield page["file"], page.get("title", "")
            yield from flatten(page.get("children", []))

    return dict(flatten(doc.get("pages", [])))


def translate_title(cfg, headings_prompt: str, title: str, page_id: str) -> str | None:
    """A manifest title is a heading of its own (the TOC entry); one LLM call."""
    payload = {"source": title, "context": f"table-of-contents entry for page {page_id}"}
    try:
        answer = llm_chat(cfg, headings_prompt, json.dumps(payload, ensure_ascii=False))
    except Exception as exc:
        print(f"  LLM error (title {title!r}): {exc}", file=sys.stderr)
        return None
    answer = answer.strip()
    if len(answer) >= 2 and answer[0] == answer[-1] and answer[0] in "\"'":
        answer = answer[1:-1].strip()
    answer = answer.splitlines()[0].strip() if answer else ""
    return answer or None


def sync_manifest(root: Path, lang: str, en_nested: list, titles: dict) -> bool:
    """Rewrite the language manifest from the English one (ids/files/order/nesting),
    keeping the translated titles. Returns True when anything changed."""
    tree = root / "docs" / "i18n" / lang / "manual"
    old = None
    manifest_path = tree / "manual.json"
    if manifest_path.exists():
        try:
            old = json.loads(manifest_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            old = None

    def build(pages):
        out = []
        for page in pages:
            entry = {"id": page["id"], "file": page["file"]}
            title = titles.get(page["file"], "")
            if not title and page["file"] == CHANGELOG_FILE:
                # The conscious EN title of the stub page.
                title = next(p.get("title", "") for p in en_nested if p["file"] == CHANGELOG_FILE)
            entry["title"] = title
            children = page.get("children")
            if children:
                entry["children"] = build(children)
            out.append(entry)
        return out

    new = {
        "_comment": (f"Translated table of contents for the {lang} manual: ids and files repeat "
                     "the English manifest verbatim; titles are translated by tools/i18n/manual/"
                     "translate_manual.py. The 'What's New' entry keeps its English title on "
                     "purpose — the changelog is maintained in English."),
        "pages": build(en_nested),
    }
    if old == new:
        return False
    manifest_path.write_text(json.dumps(new, ensure_ascii=False, indent=2) + "\n",
                             encoding="utf-8")
    print(f"updated {manifest_path.relative_to(root)}")
    return True


def prune_tree(tree: Path, en_pages: list) -> None:
    """Remove pages the English manifest no longer lists (file + hash row)."""
    keep = {p["file"] for p in en_pages} | {CHANGELOG_FILE}
    for path in tree.glob("*.md"):
        if path.name not in keep:
            path.unlink()
            print(f"removed {path.name} (no longer in the English manifest)")
    hashes_path = tree / HASHES_NAME
    if hashes_path.exists():
        stored = json.loads(hashes_path.read_text(encoding="utf-8"))
        pruned = {k: v for k, v in stored.items() if k in keep}
        if pruned != stored:
            hashes_path.write_text(json.dumps(pruned, indent=2) + "\n", encoding="utf-8")


def write_hashes(tree: Path, updated: dict) -> None:
    """Merge the freshly translated pages into .source_hashes (sorted keys)."""
    hashes_path = tree / HASHES_NAME
    stored = {}
    if hashes_path.exists():
        stored = json.loads(hashes_path.read_text(encoding="utf-8"))
    stored.update(updated)
    hashes_path.write_text(json.dumps(dict(sorted(stored.items())), indent=2) + "\n",
                           encoding="utf-8")


# --------------------------------------------------------------------------
# The changelog stub (hand-written, never through the LLM)
# --------------------------------------------------------------------------

def ensure_changelog_stub(tree: Path, lang: str) -> None:
    """The static "What's New" page: the changelog stays English.

    The link is absolute on purpose — a relative `changelog.md` from the
    language tree would resolve to the stub itself."""
    stub = tree / CHANGELOG_FILE
    text = ("# What's New\n"
            "\n"
            "The release notes are maintained in English and are not translated. "
            "Open the English page: [What's New](qrc:/manual/changelog.md)\n")
    if not stub.exists() or stub.read_text(encoding="utf-8") != text:
        stub.write_text(text, encoding="utf-8")


# --------------------------------------------------------------------------
# The run
# --------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Translate the English user manual into a language tree")
    parser.add_argument("lang", help="language code (prompts from locales/<lang>/)")
    parser.add_argument("--dry-run", action="store_true",
                        help="list missing/stale pages, zero LLM calls")
    parser.add_argument("--limit", type=int, default=0,
                        help="translate at most N pages this run (0 = all)")
    parser.add_argument("--model", default=None,
                        help="override the MODEL from .env")
    parser.add_argument("--no-audit", action="store_true",
                        help="skip the audit post-gate")
    parser.add_argument("--root", type=Path, default=HERE.parent.parent.parent,
                        help="repository root (default: auto)")
    args = parser.parse_args()

    root = args.root.resolve()
    lang = args.lang.lower()
    tree = root / "docs" / "i18n" / lang / "manual"
    en_manifest = root / "docs" / "manual" / "manual.json"
    if not en_manifest.exists():
        sys.exit(f"English manifest not found: {en_manifest}")

    en_pages, en_nested, _ru_pages = load_manifest(root, lang)

    # --dry-run: the list of missing/stale pages, zero LLM calls.
    if args.dry_run:
        pending = pending_pages(root, en_pages, tree)
        print(f"{len(pending)} page(s) to translate (missing or stale):")
        for page, _ in pending:
            state = "stale" if (tree / page["file"]).exists() else "missing"
            print(f"  {state:8s} {page['file']}  ({page['id']})")
        return 0

    # The tree and its manifest.
    ensure_tree(root, lang, en_nested)
    prune_tree(tree, en_pages)
    ensure_changelog_stub(tree, lang)

    pending = pending_pages(root, en_pages, tree)
    if args.limit > 0:
        pending = pending[:args.limit]

    # Keep the manifest in sync even when nothing needs translating (a rerun after a
    # manual edit of the language manifest must not leave it stale).
    sync_manifest(root, lang, en_nested, load_titles(tree))
    if not pending:
        print("Nothing to translate — the tree is up to date.")
        return finish(root, lang, args)

    headings_prompt_file = I18N / "locales" / lang / "headings.md"
    manual_prompt_file = I18N / "locales" / lang / "manual.md"
    for prompt_file in (headings_prompt_file, manual_prompt_file):
        if not prompt_file.exists():
            sys.exit(f"prompt file not found: {prompt_file}")
    headings_prompt = headings_prompt_file.read_text(encoding="utf-8")
    manual_prompt = manual_prompt_file.read_text(encoding="utf-8")

    cfg = load_model_config(lang)
    if args.model:
        cfg["model"] = args.model
    print(f"model: {cfg['model']} @ {cfg['api_base']}", file=sys.stderr)

    # Manifest titles for the pending pages (one call each, only when empty).
    titles = load_titles(tree)
    title_cfg_needed = [p for p, _ in pending if not titles.get(p["file"])]
    for page in title_cfg_needed:
        en_title = page["title"]
        print(f"  title {page['file']}: {en_title!r}")
        answer = translate_title(cfg, headings_prompt, en_title, page["id"])
        if answer is None:
            print(f"  could not translate the title of {page['file']}; left empty",
                  file=sys.stderr)
            continue
        titles[page["file"]] = answer
    sync_manifest(root, lang, en_nested, titles)

    updated_hashes = {}
    translated = []

    # Stage 1 — headings for every pending page, up front: the anchor maps of this
    # run's pages must exist before any body is rewritten (cross-page links).
    heading_maps = {}   # file -> {en_slug: ru_heading_text} for this run's pages
    for i, (page, _actual) in enumerate(pending, start=1):
        file_name = page["file"]
        en_path = root / "docs" / "manual" / file_name
        print(f"[headings {i}/{len(pending)}] {file_name}", flush=True)
        page_headings = pass_headings(cfg, headings_prompt, page,
                                     en_path.read_text(encoding="utf-8"))
        if page_headings is None:
            print(f"STOP: {file_name} — a heading translation failed; "
                  f"the page is left untranslated", file=sys.stderr)
            return 1
        heading_maps[file_name] = page_headings

    # Stage 2 — bodies, with guards; anchors rewritten against the whole tree.
    en_headings_by_file = en_heading_texts(root, en_pages)
    for i, (page, actual_hash) in enumerate(pending, start=1):
        file_name = page["file"]
        en_path = root / "docs" / "manual" / file_name
        print(f"[{i}/{len(pending)}] {file_name}", flush=True)
        en_text = en_path.read_text(encoding="utf-8")
        page_headings = heading_maps[file_name]

        # Pass 2 — the body, with guards.
        ru_text = pass_body(cfg, manual_prompt, page, en_text, page_headings,
                            titles, en_pages)
        if ru_text is None:
            print(f"STOP: {file_name} — the guards rejected every attempt; "
                  f"the page is left untranslated", file=sys.stderr)
            return 1

        # Pass 3 — deterministic. Order matters: UI names first (they settle the
        # final RU heading texts, hence the slugs), then anchors against maps
        # covering the whole tree, then link labels and the H1 title.
        ru_text = normalize_ui_names(ru_text)
        anchor_maps, heading_texts = build_anchor_maps(root, en_pages, tree)
        for f, m in heading_maps.items():
            anchor_maps[f] = {es: slug_for(rt) for es, rt in m.items()}
            heading_texts[f] = {es: rt.replace("`", "") for es, rt in m.items()}
        # This page's own headings were just settled by normalize_ui_names —
        # rebuild its map positionally from the final text (the EN heading list
        # is the index; the guards guarantee equal counts).
        en_h = headings_of(en_path.read_text(encoding="utf-8"))
        ru_h = headings_of(ru_text)
        if len(en_h) == len(ru_h):
            anchor_maps[file_name] = {es: rs for (_, _, es), (_, _, rs) in zip(en_h, ru_h)}
            heading_texts[file_name] = {es: rt.replace("`", "")
                                        for (_, _, es), (_, rt, _) in zip(en_h, ru_h)}
        ru_text = rewrite_anchors(ru_text, anchor_maps, file_name,
                                 en_headings_by_file, heading_texts)
        # Page-title links get the manifest's Russian titles (the model leaves
        # them in English; the guard reports whatever this does not cover).
        ru_text = normalize_link_texts(ru_text, en_pages, titles)
        title = titles.get(file_name, "")
        if title:
            ru_text = set_page_title(ru_text, title)
        (tree / file_name).write_text(ru_text + "\n", encoding="utf-8")
        updated_hashes[file_name] = actual_hash
        # Persist right away: a run stopped later on must keep its progress.
        write_hashes(tree, updated_hashes)
        translated.append(file_name)
        print(f"    -> {file_name} written ({len(page_headings)} headings)")

    sync_manifest(root, lang, en_nested, load_titles(tree))
    print(f"\nDone: {len(translated)} page(s) translated into {tree.relative_to(root)}/")
    return finish(root, lang, args)


def finish(root: Path, lang: str, args) -> int:
    """The audit post-gate: a green run passed the guards AND the audit."""
    if args.no_audit:
        print("audit skipped (--no-audit) — this run is not green", file=sys.stderr)
        return 0
    audit = HERE / "audit_manual.py"
    if not audit.exists():
        sys.exit(f"audit script not found: {audit}")
    print("\nRunning the audit post-gate...", flush=True)
    result = subprocess.run([sys.executable, str(audit), "--lang", lang,
                             "--root", str(root)])
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
