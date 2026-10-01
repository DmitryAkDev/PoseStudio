#!/usr/bin/env python3
"""Add a new language to the PoseStudio translation pipeline.

Usage:
    add_language.py <lang> [model]

        <lang>   Qt locale code (zh_CN, de, fr, ...)
        [model]  optional pool entry name from .env (default: the DEFAULT entry)

Does every mechanical step of the README "Adding a new language" section:

    1. CMakeLists.txt  — appends <lang> to POSESTUDIO_I18N_LANGS
    2. locales/<lang>/translator.md — created from locales/template.md
    3. locales/<lang>/.env  — writes MODEL=<model> (when given)
    4. reconfigures the build and runs PoseStudio_lupdate, which generates
       the empty translations/<lang>.ts

Left for a human: rewrite the terminology table in translator-<lang>.md,
add {"<lang>", "<Endonym>"} to Constants::AVAILABLE_LANGUAGES in src/constants.h
(the Preferences -> General language picker is driven by that list), then run
translate.py on the new catalog.
"""

import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent

LANG_NAMES = {
    "ru": "Russian", "de": "German", "fr": "French", "es": "Spanish",
    "it": "Italian", "pl": "Polish", "ja": "Japanese", "ko": "Korean",
    "zh_CN": "Simplified Chinese", "zh_TW": "Traditional Chinese",
    "pt_BR": "Brazilian Portuguese",
}


def lang_name(lang: str) -> str:
    return LANG_NAMES.get(lang, lang.replace("_", " ").title())

def add_to_cmake(lang: str) -> None:
    cmake = ROOT / "CMakeLists.txt"
    text = cmake.read_text(encoding="utf-8")
    pattern = re.compile(r"^(\s*set\(POSESTUDIO_I18N_LANGS\s+)(.*?)\)\s*$", re.M)
    match = pattern.search(text)
    if not match:
        sys.exit(f"POSESTUDIO_I18N_LANGS list not found in {cmake}")
    langs = match.group(2).split()
    if lang in langs:
        print(f"CMakeLists.txt: '{lang}' already in the language list — skipped")
        return
    langs.append(lang)
    replacement = f"{match.group(1)}{' '.join(langs)})"
    cmake.write_text(text[:match.start()] + replacement + text[match.end():],
                     encoding="utf-8")
    print(f"CMakeLists.txt: language list is now: {' '.join(langs)}")


def make_prompt_template(lang: str) -> None:
    source = HERE / "locales" / "template.md"
    if not source.exists():
        sys.exit(f"template not found: {source}")
    locale_dir = HERE / "locales" / lang
    target = locale_dir / "translator.md"
    if target.exists():
        print(f"{target.name}: already exists — skipped")
        return
    locale_dir.mkdir(parents=True, exist_ok=True)
    text = (source.read_text(encoding="utf-8")
            .replace("{{LANG_NAME}}", lang_name(lang))
            .replace("{{LANG_CODE}}", lang))
    target.write_text(text, encoding="utf-8")
    print(f"locales/{lang}/translator.md: created from locales/template.md (TODO: terminology table)")

def add_env_model(lang: str, model: str | None) -> None:
    """Write the per-language model selection into locales/<lang>/.env."""
    if not model:
        return
    env_file = HERE / "locales" / lang / ".env"
    text = env_file.read_text(encoding="utf-8") if env_file.exists() else ""
    if re.search(r"^MODEL=", text, re.M):
        print(f"locales/{lang}/.env: MODEL already present — skipped")
        return
    env_file.write_text(text.rstrip("\n") + f"\nMODEL={model}\n", encoding="utf-8")
    print(f"locales/{lang}/.env: added MODEL={model}")


def generate_catalog() -> None:
    for cmd in (["cmake", "-B", "build"],
                ["cmake", "--build", "build", "--target", "PoseStudio_lupdate"]):
        result = subprocess.run(cmd, cwd=ROOT)
        if result.returncode != 0:
            sys.exit(f"failed: {' '.join(cmd)}")


def main() -> int:
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        sys.exit(__doc__)
    lang, model = sys.argv[1], (sys.argv[2] if len(sys.argv) == 3 else None)
    if not re.fullmatch(r"[a-z]{2,3}([-_]?[A-Za-z]{2})?", lang):
        sys.exit(f"unusual locale code: {lang!r} (expected e.g. zh_CN, de, fr)")

    add_to_cmake(lang)
    make_prompt_template(lang)
    add_env_model(lang, model)
    generate_catalog()

    print(f"\nDone. Next steps:")
    print(f"  1. Fill the terminology table in tools/i18n/locales/{lang}/translator.md")
    print(f"  2. Add {{\"{lang}\", \"<Endonym>\"}} to Constants::AVAILABLE_LANGUAGES in src/constants.h")
    print(f"  3. python3 tools/i18n/translate.py translations/{lang}.ts")
    return 0


if __name__ == "__main__":
    sys.exit(main())
