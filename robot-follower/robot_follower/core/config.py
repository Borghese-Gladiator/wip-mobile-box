"""Config loading — shared across sim and hardware."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    """Load config YAML. Defaults to config/common.yaml relative to project root."""
    if path is None:
        path = Path(__file__).resolve().parent.parent.parent / "config" / "common.yaml"
    else:
        path = Path(path)
    with open(path) as f:
        return yaml.safe_load(f)


def merge_configs(*configs: dict) -> dict:
    """Deep-merge multiple config dicts (later overrides earlier)."""
    result: dict = {}
    for cfg in configs:
        _deep_merge(result, cfg)
    return result


def _deep_merge(base: dict, override: dict) -> None:
    for k, v in override.items():
        if k in base and isinstance(base[k], dict) and isinstance(v, dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v
