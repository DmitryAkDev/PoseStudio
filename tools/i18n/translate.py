#!/usr/bin/env python3
"""Self-contained translator for Qt Linguist .ts catalogs.

Everything lives in this directory — the script reads nothing outside it:

    translate.py        this script (stdlib only, no pip packages)
    .env                base model config (API_BASE, MODEL, TEMPERATURE, ...);
                        must exist — create it from .env.template
    locales/<lang>/     per-language folder: translator.md (system prompt) and
                        .env (optional overlay of the same config keys)
Parses the .ts with xml.etree, asks the LLM (one short request per
message) for every <message> whose translation is empty, and writes the answer
back into its <translation>, dropping the "unfinished" marker (lrelease skips
anything still marked). Filled entries are
never touched, so a rerun makes no LLM calls at all.

Usage:
    translate.py <catalog.ts> [system_prompt.md] [--model NAME]
                    [--dry-run] [--limit N]
    translate.py <catalog.ts> --check

--check validates placeholder sets (%1, %2, ...) of source vs translation
for every filled message and exits non-zero on mismatch.
"""

import argparse
import io
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from llm_pool import HERE, load_model_config, llm_chat
PLACEHOLDER_RE = re.compile(r"%\d+")
TAG_TOKEN_RE = re.compile(r"\{T(\d+)\}")
TAG_RE = re.compile(r"</?[a-zA-Z][^>]*>")

LOCALE_RE = re.compile(r"^[a-z]{2,3}[-_][a-z]{2}$|^[a-z]{2,3}$")


def resolve_lang(catalog: Path):
    """Language code from the catalog name (Qt convention: ru.ts -> 'ru')."""
    stem = catalog.stem
    if not LOCALE_RE.match(stem.lower()):
        return None
    # Qt locale codes mix case (zh_CN); resolve against the locale directories so the
    # canonical casing of locales/<lang>/ is preserved.
    locales = HERE / "locales"
    if locales.is_dir():
        for entry in locales.iterdir():
            if entry.is_dir() and entry.name.lower() == stem.lower():
                return entry.name
    return stem.lower()
# --------------------------------------------------------------------------
# Catalog logic
# --------------------------------------------------------------------------

def translation_text(message) -> str:
    el = message.find("translation")
    return (el.text or "") if el is not None else ""


def is_filled(message) -> bool:
    """True when the entry must be skipped: no source, or text already there."""
    source = (message.findtext("source") or "").strip()
    return not source or translation_text(message).strip() != ""


def placeholders(text: str) -> set:
    return set(PLACEHOLDER_RE.findall(text))


def tokenize_html(text: str):
    """Replace HTML tags with neutral {Tn} tokens; returns (text, tags)."""
    tags = []
    def sub(match):
        tags.append(match.group(0))
        return f"{{T{len(tags) - 1}}}"
    return TAG_RE.sub(sub, text), tags


def restore_html(text: str, tags: list) -> str:
    """Put the original tags back at their {Tn} positions."""
    def sub(match):
        idx = int(match.group(1))
        return tags[idx] if idx < len(tags) else match.group(0)
    return TAG_TOKEN_RE.sub(sub, text)


def restore_control_chars(source: str, answer: str) -> str:
    """Turn the model's literal \\t / \\n / \\r back into real control characters.

    The JSON query encodes the source's tabs and newlines as two-character
    escapes, and the model copies those escapes verbatim into its answer.
    For every control character the SOURCE actually contains, the escape is
    restored in the answer — a string with no real tab keeps a literal \\t
    (none of ours do), so the translation mirrors the source's line breaks
    and tab separators (Qt renders a real tab in a menu label as the gap
    before the key hint)."""
    for char, escape in (("\t", "\\t"), ("\n", "\\n"), ("\r", "\\r")):
        if char in source:
            answer = answer.replace(escape, char)
    return answer


def mirror_edge_spaces(source: str, answer: str) -> str:
    """Mirror the source's leading/trailing spaces onto the answer.

    Several UI strings are built by concatenating consecutive tr() fragments;
    each fragment's trailing space is what keeps the words apart at the join.
    The model strips those spaces, so re-attach them from the source."""
    if source.startswith(" ") and not answer.startswith(" "):
        answer = " " + answer
    if source.endswith(" ") and not answer.endswith(" "):
        answer = answer + " "
    return answer


def fit_line_skeleton(source: str, answer: str) -> str | None:
    """Snap the answer onto the source's line structure; None when impossible.

    The source is the skeleton: its real newlines split it into segments and
    each answer segment must land on the matching source segment. A model that
    drops or adds a newline leaves the join points in the wrong place, so:
    - fewer lines than the source -> impossible (a join point was lost);
    - more lines -> glue the surplus onto the previous line (the model split
      one paragraph across two lines; gluing keeps every character).
    Whitespace at each line edge is re-derived from the source so a stray
    space at a join cannot glue two words together."""
    # Leading/trailing newlines are structure, not prose — mirror them from
    # the source (the model drops them on trailing-fragment strings).
    lead = "\n" * (len(source) - len(source.lstrip("\n")))
    trail = "\n" * (len(source) - len(source.rstrip("\n")))
    answer = lead + answer.strip("\n") + trail
    src_lines = source.split("\n")
    ans_lines = answer.split("\n")
    if len(ans_lines) < len(src_lines):
        # The model collapsed a blank line ("\n\n" became "\n"): re-open the
        # gap at the source's join point when exactly one blank line is missing.
        if len(ans_lines) == len(src_lines) - 1 and sum(1 for s in src_lines if not s.strip()) == 1:
            gap = next(i for i, s in enumerate(src_lines) if not s.strip())
            if gap <= len(ans_lines):
                if gap == len(ans_lines):
                    ans_lines = ans_lines + [""]
                else:
                    ans_lines = ans_lines[:gap] + [""] + ans_lines[gap:]
        if len(ans_lines) < len(src_lines):
            return None
    if len(ans_lines) > len(src_lines):
        keep = ans_lines[: len(src_lines) - 1]
        tail = "".join(ans_lines[len(src_lines) - 1:])
        ans_lines = keep + [tail]
    fitted = []
    for src_line, ans_line in zip(src_lines, ans_lines):
        core = ans_line.strip()
        if not core:
            core = "\t" if "\t" in src_line else ""
        prefix = " " if src_line.startswith(" ") else ""
        suffix = " " if src_line.endswith(" ") and len(src_line) > 1 else ""
        fitted.append(prefix + core + suffix)
    return "\n".join(fitted)


def check_catalog(root) -> int:
    """Placeholder audit over all filled messages; returns mismatch count."""
    bad = 0
    for message in root.iter("message"):
        if not is_filled(message):
            continue
        source = message.findtext("source") or ""
        translation = translation_text(message)
        src_set, tr_set = placeholders(source), placeholders(translation)
        if src_set != tr_set:
            bad += 1
            missing = sorted(src_set - tr_set)
            extra = sorted(tr_set - src_set)
            print(f"MISMATCH {source!r}")
            if missing:
                print(f"    lost:    {', '.join(missing)}")
            if extra:
                print(f"    extra:   {', '.join(extra)}")
    return bad


def _requote(line: str) -> str:
    """lupdate escapes quotes in text as &quot;/&apos;; ElementTree writes
    them raw. Re-escape both in the text part of a tag line, or in a whole
    continuation line of a multi-line element (attribute quotes are safe:
    they sit before the first '>' / after the last '<')."""
    stripped = line.lstrip()
    if not stripped.startswith("<"):
        return line.replace('"', "&quot;").replace("'", "&apos;")
    open_end = line.find(">")
    close_start = line.rfind("<")
    if open_end == -1:
        return line
    # closing tag on the same line: text sits between the tags;
    # multi-line element: text runs from '>' to end of line
    inner = (line[open_end + 1:close_start] if close_start > open_end + 1
             else line[open_end + 1:])
    if not inner.strip():
        return line
    fixed = inner.replace('"', "&quot;").replace("'", "&apos;")
    if fixed == inner:
        return line
    if close_start > open_end + 1:
        return line[:open_end + 1] + fixed + line[close_start:]
    return line[:open_end + 1] + fixed


def write_catalog(path: Path, tree) -> None:
    """Serialize the catalog back to disk in lupdate's byte style.
    ElementTree keeps every untouched node verbatim (text/tail
    whitespace survives); only systematic serializer quirks are patched
    on the way out: the dropped XML declaration/DOCTYPE, the space
    inserted before '/' on empty tags, and raw quotes in text."""
    buf = io.BytesIO()
    tree.write(buf, encoding="utf-8", xml_declaration=False)
    body = buf.getvalue().decode("utf-8")
    body = "\n".join(
        _requote(re.sub(r"^(\s*</?[a-zA-Z][^>]*?)\s*/>$", r"\1/>", line))
        for line in body.split("\n"))
    path.write_text('<?xml version="1.0" encoding="utf-8"?>\n'
                    "<!DOCTYPE TS>\n" + body + "\n", encoding="utf-8")


def translate_one(cfg, prompt_text: str, payload: dict) -> str | None:
    """One LLM call; returns the translation line or None on any failure."""
    source = payload["source"]
    tagged_source, tags = tokenize_html(source)
    query_payload = {"source": tagged_source,
                     "comment": payload["comment"],
                     "context": payload["context"]}
    try:
        answer = llm_chat(cfg, prompt_text,
                          json.dumps(query_payload, ensure_ascii=False))
    except Exception as exc:  # network, HTTP, JSON — all keep the line open
        print(f"  LLM error: {exc}", file=sys.stderr)
        return None
    answer = answer.strip()
    # Tolerate a single pair of surrounding quotes.
    if len(answer) >= 2 and answer[0] == answer[-1] and answer[0] in "\"'":
        answer = answer[1:-1].strip()
    # A single-line source gets only the first line back (a trailing
    # explanation is dropped); a multi-line source keeps all of its lines,
    # so trailing fragments like "%1" / "Details: %2" survive. Trailing
    # newlines are part of the structure for multi-line sources — strip
    # spaces only, never \n/\t (the count guard below enforces parity).
    if "\n" in source:
        answer = answer.strip(" ")
    else:
        answer = answer.splitlines()[0].strip() if answer else ""
    if not answer:
        return None
    # The JSON query escaped the source's tabs/newlines as \\t / \\n; put the
    # real characters back before any validation sees the answer.
    answer = restore_control_chars(source, answer)
    # Concatenated tr() fragments live on their edge spaces — mirror them.
    answer = mirror_edge_spaces(source, answer)
    # Multi-line sources: snap the answer onto the source's line structure
    # (the model adds/drops newlines); unrepairable -> reject and retry.
    if "\n" in source:
        fitted = fit_line_skeleton(source, answer)
        if fitted is None:
            print(f"  line skeleton mismatch in {answer!r}", file=sys.stderr)
            return None
        answer = fitted
    # Structural guard: the control-character count must match the source
    # exactly — an extra \n breaks multi-line layout, a lost \t loses the
    # key-hint gap in menu labels.
    for char in ("\t", "\n", "\r"):
        if answer.count(char) != source.count(char):
            print(f"  control-char count mismatch in {answer!r}", file=sys.stderr)
            return None
    # Symbol guard: arrows and the degree sign are UI structure, not words —
    # they must survive verbatim ("Forward ›" keeps its arrow).
    for char in "\u2039\u203a\u00b0\u2192":
        if answer.count(char) < source.count(char):
            print(f"  lost symbol {char!r} in {answer!r}", file=sys.stderr)
            return None
    # Garbage guard: the model must not invent placeholders.
    if placeholders(answer) - placeholders(source):
        print(f"  LLM invented placeholders in {answer!r}", file=sys.stderr)
        return None
    # HTML strings: every tag token from the source must survive verbatim.
    src_tokens = sorted(TAG_TOKEN_RE.findall(tagged_source))
    if src_tokens and src_tokens != sorted(TAG_TOKEN_RE.findall(answer)):
        print(f"  LLM mangled HTML tags in {answer!r}", file=sys.stderr)
        return None
    return restore_html(answer, tags)


def main() -> int:
    parser = argparse.ArgumentParser(description="Self-contained .ts translator")
    parser.add_argument("catalog", type=Path, help="path to the .ts catalog")
    parser.add_argument("system_prompt", nargs="?", type=Path, default=None,
                        help="system prompt file (default: locales/<lang>/translator.md, "
                             "resolved from the catalog name; not needed with "
                             "--check/--dry-run)")
    parser.add_argument("--model", default=None,
                        help="override the MODEL from .env (the locale overlay "
                             "wins over the base config when both set it)")
    parser.add_argument("--dry-run", action="store_true",
                        help="list the messages that would be translated")
    parser.add_argument("--limit", type=int, default=0,
                        help="translate at most N messages this run (0 = all)")
    parser.add_argument("--check", action="store_true",
                        help="validate placeholder sets and exit")
    args = parser.parse_args()

    if not args.catalog.exists():
        sys.exit(f"catalog not found: {args.catalog}")
    tree = ET.parse(args.catalog)
    root = tree.getroot()
    # Normalize legacy entries: a filled translation must not carry a
    # type attribute — lrelease skips anything marked unfinished.
    for message in root.iter("message"):
        el = message.find("translation")
        if el is not None and (el.text or "").strip() and "type" in el.attrib:
            del el.attrib["type"]

    if args.check:
        bad = check_catalog(root)
        print(f"--check: {'OK, all placeholders intact' if bad == 0 else f'{bad} mismatches'}")
        return 1 if bad else 0

    pending = []
    for context in root.iter("context"):
        class_name = context.findtext("name") or ""
        for message in context.findall("message"):
            if is_filled(message):
                continue
            comment_el = message.find("comment")
            pending.append({
                "source": message.findtext("source") or "",
                "comment": (comment_el.text or "").strip() if comment_el is not None else "",
                "context": class_name,
                "_message": message,
            })

    lang = resolve_lang(args.catalog)
    if args.system_prompt:
        prompt_file = args.system_prompt
    elif lang:
        prompt_file = HERE / "locales" / lang / "translator.md"
    else:
        available = sorted(p.name for p in (HERE / "locales").iterdir() if p.is_dir())
        sys.exit(f"cannot infer the language from '{args.catalog.name}'; "
                 f"pass the prompt file explicitly. Available: {', '.join(available)}")
    if not prompt_file.exists():
        sys.exit(f"system prompt file not found: {prompt_file}")

    if args.dry_run:
        print(f"{len(pending)} messages would be translated:")
        for item in pending:
            print(f"  [{item['context']}] {item['source']}")
        return 0

    if not pending:
        print("Nothing to translate — catalog is complete.")
        return 0

    if args.limit > 0:
        pending = pending[:args.limit]

    prompt_text = prompt_file.read_text(encoding="utf-8")

    cfg = load_model_config(lang)
    if args.model:
        cfg["model"] = args.model
    print(f"model: {cfg['model']} @ {cfg['api_base']}", file=sys.stderr)

    done = []
    failed = []
    # Duplicate-source guard: one source text must map to one translation
    # (dump.py --audit enforces it). Reuse a translation that already
    # exists for the same source instead of asking the LLM again — two
    # independent calls would drift apart (e.g. 'Twist' -> two words).
    known: dict[str, str] = {}
    for context in root.iter("context"):
        for message in context.findall("message"):
            if is_filled(message):
                src = message.findtext("source") or ""
                known.setdefault(src, translation_text(message))
    for i, item in enumerate(pending, start=1):
        source = item["source"]
        print(f"[{i}/{len(pending)}] {source!r}", flush=True)
        answer = known.get(source)
        if answer is not None:
            print(f"    -> reused existing translation for this source")
        else:
            answer = translate_one(cfg, prompt_text,
                               {"source": item["source"],
                                "comment": item["comment"],
                                "context": item["context"]})
            if answer is not None:
                known[source] = answer
        if answer is None:
            failed.append(item)
            continue
        done.append(item)
        translation = item["_message"].find("translation")
        if translation is None:
            translation = ET.SubElement(item["_message"], "translation")
        if "type" in translation.attrib:
            del translation.attrib["type"]
        translation.text = answer
        print(f"    -> {answer!r}")

    if done:
        write_catalog(args.catalog, tree)

    if failed:
        # Keep the file writable for the next run; only save what succeeded.
        print(f"\n{len(failed)} messages left untranslated (rerun to pick up):")
        for item in failed:
            print(f"  [{item['context']}] {item['source']}")
        return 1

    print(f"\nDone: {len(done)} translations written to {args.catalog}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
