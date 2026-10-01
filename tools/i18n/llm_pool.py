#!/usr/bin/env python3
"""Shared LLM pipeline infrastructure for the i18n tools (stdlib only).

Model pool and locale config live in this directory:

    .env                shared LLM model pool (LLM_MODEL_<NAME>_API_BASE/_MODEL/...);
                        must exist — create it from .env.template
    locales/<lang>/.env per-language overlay: MODEL=<name> picks the pool entry
                        that translates the language

The DEFAULT pool entry is used when a language has no MODEL= of its own.
Imported by translate.py —
keep this module free of CLI logic and side effects at import time.
"""

import json
import sys
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent


# --------------------------------------------------------------------------
# Config: top-level .env (model pool) + locales/<lang>/.env (MODEL selection)
# --------------------------------------------------------------------------

def _parse_env_file(path: Path, env: dict) -> None:
    """Merge KEY=VALUE lines of one .env file into env."""
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        env[key.strip()] = value.strip().strip("'\"")


def load_env(lang: str | None) -> dict:
    """Top-level .env (model pool), then the locale overlay for <lang>."""
    env = {}
    env_file = HERE / ".env"
    if not env_file.exists():
        sys.exit(f"config not found: {env_file}")
    _parse_env_file(env_file, env)
    if lang:
        locale_env = HERE / "locales" / lang / ".env"
        if locale_env.exists():
            _parse_env_file(locale_env, env)
    return env


def load_models(env: dict) -> dict:
    """Build the model pool from LLM_MODEL_<NAME>_* variables."""
    models = {}
    for key, value in env.items():
        if not key.startswith("LLM_MODEL_"):
            continue
        remainder = key[len("LLM_MODEL_"):]
        name, _, field = remainder.partition("_")
        name, field = name.upper(), field.upper()
        models.setdefault(name, {})
        models[name][field] = value.strip()
    for name, raw in models.items():
        models[name] = {
            "api_base": raw.get("API_BASE", ""),
            "model": raw.get("MODEL", ""),
            "temperature": float(raw.get("TEMPERATURE", 1.0)),
            "reasoning_effort": raw.get("REASONING_EFFORT") or None,
            "no_thinking": raw.get("NO_THINKING", "").strip().lower() in ("1", "true", "yes"),
        }
    if not models:
        sys.exit("no LLM_MODEL_* entries in .env")
    return models


def resolve_model(models, key=None):
    """Resolve a model name to its config; falls back to the DEFAULT entry.

    Returns (None, None) when neither the requested nor the DEFAULT entry
    exists — the caller skips the language.
    """
    name = (key or "DEFAULT").upper()
    if name in models:
        return name, models[name]
    return None, None


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
