#!/usr/bin/env python3
"""Validate mapping.yaml and all control-bridge recipes before flight."""
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from typing import Iterable, List

REPO_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_DIR = REPO_ROOT / "software" / "control-bridge"


def _load_module(module_name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to import {module_name} from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


_cv_mod = _load_module(
    "control_bridge_config_validation", BRIDGE_DIR / "config_validation.py"
)
_bridge_mod = _load_module(
    "control_bridge_osc_msp_bridge", BRIDGE_DIR / "osc_msp_bridge.py"
)

cv = _cv_mod
load_recipe = _bridge_mod.load_recipe

DEFAULT_MAPPING = REPO_ROOT / "config" / "mapping.yaml"
DEFAULT_MIDI_MAPPING = REPO_ROOT / "config" / "mappings" / "midi.yaml"
DEFAULT_RECIPE_DIR = REPO_ROOT / "config" / "recipes"


def _iter_recipe_paths(recipe_dir: Path) -> Iterable[Path]:
    if not recipe_dir.exists():
        return []
    return sorted(
        p
        for p in recipe_dir.iterdir()
        if p.suffix.lower() in {".yaml", ".yml", ".json"}
    )


def validate(
    mapping_path: Path,
    recipe_dir: Path,
    midi_path: Path,
    *,
    verbose: bool = False,
) -> List[Path]:
    mapping_path = mapping_path.resolve()
    recipe_dir = recipe_dir.resolve()
    midi_path = midi_path.resolve()
    if verbose:
        print(f"[validate] mapping → {mapping_path}")
    cv.validate_file(mapping_path, source_label="mapping")
    if verbose:
        print(f"[validate] midi    → {midi_path}")
    cv.validate_midi_file(midi_path, source_label="midi")
    failures: List[Path] = []
    for recipe in _iter_recipe_paths(recipe_dir):
        if verbose:
            print(f"[validate] recipe  → {recipe}")
        try:
            cv.validate_recipe(recipe, load_recipe)
        except cv.ValidationError as exc:  # noqa: BLE001
            failures.append(recipe)
            print(f"[validate] ✖ {recipe.name}: {exc}")
    if failures:
        raise cv.ValidationError(
            [f"{len(failures)} recipe(s) failed validation"]
        )
    if verbose:
        print("[validate] all clear.")
    return failures


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Sanity-check OSC/MSP bridge configs"
    )
    ap.add_argument(
        "--mapping",
        type=Path,
        default=DEFAULT_MAPPING,
        help="Path to mapping.yaml",
    )
    ap.add_argument(
        "--midi-map",
        type=Path,
        default=DEFAULT_MIDI_MAPPING,
        help="Path to MIDI mapping YAML",
    )
    ap.add_argument(
        "--recipes",
        type=Path,
        default=DEFAULT_RECIPE_DIR,
        help="Directory containing recipes",
    )
    ap.add_argument(
        "--quiet", action="store_true", help="Suppress success chatter"
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    try:
        validate(
            args.mapping, args.recipes, args.midi_map, verbose=not args.quiet
        )
    except cv.ValidationError as exc:  # noqa: BLE001
        if not args.quiet:
            print("[validate] config errors detected")
        for line in exc.errors:
            print(f"[validate] ✖ {line}")
        return 1
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
