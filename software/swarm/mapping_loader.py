"""Utility helpers for loading base/recipe control mappings.

A "recipe" is a lightweight overlay patch that tweaks the default gesture-to-
behavior mapping without having to clone or rewrite the full base mapping.
Recipes live in ``config/recipes`` alongside the canonical ``config/mapping.yaml``
file and only override the keys they care about.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict

import yaml


def _validate_mapping(obj: Any, label: str) -> Dict[str, Any]:
    """Ensure the loaded YAML object is a dict.

    Parameters
    ----------
    obj: Any
        Parsed YAML content.
    label: str
        Human-readable label for error messages (e.g., a file path).

    Returns
    -------
    Dict[str, Any]
        The validated mapping content.

    Raises
    ------
    ValueError
        If the parsed YAML is not a mapping/dictionary.
    """

    if obj is None:
        return {}
    if not isinstance(obj, dict):
        raise ValueError(f"Mapping file {label} must contain a top-level dictionary; got {type(obj)}")
    return obj


def _deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge ``overlay`` on top of ``base``.

    - Nested dictionaries are merged so that only the touched keys change.
    - Scalars and lists are replaced entirely by the recipe.
    - ``base`` is not mutated; a merged copy is returned.
    """

    merged = copy.deepcopy(base)
    for key, overlay_value in overlay.items():
        base_value = merged.get(key)
        if isinstance(base_value, dict) and isinstance(overlay_value, dict):
            merged[key] = _deep_merge(base_value, overlay_value)
        else:
            merged[key] = copy.deepcopy(overlay_value)
    return merged


def load_mapping(
    base_path: str = "config/mapping.yaml",
    recipe_name: str | None = None,
    recipes_dir: str = "config/recipes",
) -> Dict[str, Any]:
    """Load the base mapping and optionally overlay a named recipe.

    A mapping defines how OSC gesture signals are transformed into behavioral
    parameters for the swarm. A recipe is a patch that overrides slices of that
    mapping to create a different "mood" (ambient, agitated, avoidant, etc.).

    Parameters
    ----------
    base_path: str
        Path to the canonical mapping YAML file.
    recipe_name: str | None
        If provided, the loader will look for ``<recipes_dir>/<recipe_name>.yaml``
        and overlay it on top of the base mapping.
    recipes_dir: str
        Directory that contains recipe YAML overlays.

    Returns
    -------
    Dict[str, Any]
        The merged mapping dictionary.

    Raises
    ------
    FileNotFoundError
        If the base mapping file or the requested recipe file cannot be found.
    ValueError
        If YAML parsing fails or the parsed content is not a dictionary.
    """

    base_file = Path(base_path)
    if not base_file.is_file():
        raise FileNotFoundError(f"Base mapping file not found: {base_file}")

    try:
        with base_file.open("r", encoding="utf-8") as handle:
            base_mapping = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        raise ValueError(f"Unable to parse base mapping YAML at {base_file}: {exc}") from exc

    base_mapping = _validate_mapping(base_mapping, str(base_file))

    if recipe_name is None:
        return base_mapping

    recipe_file = Path(recipes_dir) / f"{recipe_name}.yaml"
    if not recipe_file.is_file():
        raise FileNotFoundError(f"Recipe '{recipe_name}' not found in {recipes_dir}")

    try:
        with recipe_file.open("r", encoding="utf-8") as handle:
            recipe_mapping = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        raise ValueError(f"Unable to parse recipe YAML at {recipe_file}: {exc}") from exc

    recipe_mapping = _validate_mapping(recipe_mapping, str(recipe_file))

    return _deep_merge(base_mapping, recipe_mapping)
