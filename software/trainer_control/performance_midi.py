"""Validation for the safe MIDI-facing performance vocabulary."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping
import yaml

from .performance import load_macros

FORBIDDEN_WORDS = {
    "consent",
    "arm",
    "throttle",
    "flight_mode",
    "trainer_enable",
    "safety",
    "aircraft",
    "transmitter",
}


def load_performance_midi(
    path: Path,
    *,
    macros_path: Path | None = None,
    recipes_dir: Path | None = None,
) -> list[dict[str, Any]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping) or not isinstance(
        data.get("bindings"), list
    ):
        raise ValueError("performance MIDI requires a bindings list")
    macros, _ = load_macros(macros_path) if macros_path else load_macros()
    bindings = []
    for index, binding in enumerate(data["bindings"]):
        if not isinstance(binding, Mapping):
            raise ValueError(f"binding {index} must be an object")
        if binding.get("type") not in {"cc", "note"}:
            raise ValueError(f"binding {index} has invalid MIDI type")
        number_key = (
            "cc_number" if binding.get("type") == "cc" else "note_number"
        )
        number = binding.get(number_key)
        if (
            isinstance(number, bool)
            or not isinstance(number, int)
            or not 0 <= number <= 127
        ):
            raise ValueError(f"binding {index} has invalid {number_key}")
        target = binding.get("target")
        if not isinstance(target, str) or ":" not in target:
            raise ValueError(
                f"binding {index} requires macro:<name> or recipe:<name>"
            )
        kind, target_id = target.split(":", 1)
        if any(word in target_id.lower() for word in FORBIDDEN_WORDS):
            raise ValueError(f"binding {index} targets forbidden control path")
        if kind == "macro":
            if target_id not in macros:
                raise ValueError(f"binding {index} names unknown macro")
        elif kind == "recipe":
            if not target_id.replace("-", "").replace("_", "").isalnum():
                raise ValueError(f"binding {index} has invalid recipe name")
            if (
                recipes_dir is not None
                and not (recipes_dir / f"{target_id}.yaml").is_file()
            ):
                raise ValueError(f"binding {index} names unavailable recipe")
        else:
            raise ValueError(f"binding {index} targets forbidden control path")
        bindings.append(dict(binding))
    return bindings
