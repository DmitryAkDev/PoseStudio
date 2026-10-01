#!/usr/bin/env python3
"""Shared LLM pipeline infrastructure for the i18n tools (stdlib only).

Model config lives in this directory:

    .env                base model config (API_BASE, MODEL, TEMPERATURE, ...);
                        must exist — create it from .env.template
    locales/<lang>/.env optional per-language overlay of the same keys

The locale overlay wins over the base config when both set a key.
Imported by translate.py —
keep this module free of CLI logic and side effects at import time.
"""

import json
import sys
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent


# --------------------------------------------------------------------------
# Config: base .env + locales/<lang>/.env overlay (same keys, overlay wins)
# --------------------------------------------------------------------------

def _parse_env_file(path: Path, env: dict) -> None:
    """Merge KEY=VALUE lines of one .env file into env."""
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        env[key.strip()] = value.strip().strip("'\"")


def load_model_config(lang: str | None) -> dict:
    """Merge the base .env and the <lang> overlay into one model config.

    The overlay wins on a key-by-key basis. Exits when API_BASE or MODEL
    is missing after the merge — the tool cannot run without both.
    """
    env = {}
    env_file = HERE / ".env"
    if not env_file.exists():
        sys.exit(f"config not found: {env_file}")
    _parse_env_file(env_file, env)
    if lang:
        locale_env = HERE / "locales" / lang / ".env"
        if locale_env.exists():
            _parse_env_file(locale_env, env)
    api_base = env.get("API_BASE", "").rstrip("/")
    model = env.get("MODEL", "")
    if not api_base or not model:
        sys.exit(
            "incomplete model config in .env (and locale overlay): "
            "API_BASE and MODEL are required"
        )
    return {
        "api_base": api_base,
        "model": model,
        "temperature": float(env.get("TEMPERATURE", 1.0)),
        "reasoning_effort": env.get("REASONING_EFFORT") or None,
        "no_thinking": env.get("NO_THINKING", "").strip().lower() in ("1", "true", "yes"),
    }

# --------------------------------------------------------------------------
# LLM call (OpenAI-compatible chat/completions, stdlib urllib)
# --------------------------------------------------------------------------

def llm_chat(cfg: dict, system_prompt: str, query: str) -> str:
    """One chat completion; returns the assistant text."""
    payload = {
        "model": cfg["model"],
        "temperature": cfg["temperature"],
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": query},
        ],
    }
    if cfg["reasoning_effort"] and cfg["reasoning_effort"] != "off":
        payload["reasoning_effort"] = cfg["reasoning_effort"]
    if cfg["no_thinking"]:
        # Chat-template switch for hybrid reasoning models: skips the
        # chain-of-thought pass entirely (10-50x faster per label, and the
        # CoT tends to override the terminology table with its own reasoning).
        payload["chat_template_kwargs"] = {"enable_thinking": False}
    request = urllib.request.Request(
        f"{cfg['api_base']}/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=300) as response:
        result = json.loads(response.read().decode("utf-8"))
    return result["choices"][0]["message"]["content"]
