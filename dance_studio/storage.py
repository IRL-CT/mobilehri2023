"""Save/load dance sequences as JSON files."""

import json
import os
from pathlib import Path


DEFAULT_DANCES_DIR = Path(__file__).parent / "dances"


def list_dances(dances_dir: Path = DEFAULT_DANCES_DIR) -> list[str]:
    """List saved dance names (without .json extension)."""
    dances_dir.mkdir(exist_ok=True)
    return sorted(
        p.stem for p in dances_dir.glob("*.json")
    )


def load_dance(name: str, dances_dir: Path = DEFAULT_DANCES_DIR) -> dict:
    """Load a saved sequence dict by name."""
    path = dances_dir / f"{name}.json"
    with open(path) as f:
        return json.load(f)


def save_dance(name: str, sequence_dict: dict, dances_dir: Path = DEFAULT_DANCES_DIR) -> None:
    """Save a sequence dict to a JSON file."""
    dances_dir.mkdir(exist_ok=True)
    path = dances_dir / f"{name}.json"
    with open(path, "w") as f:
        json.dump(sequence_dict, f, indent=2)


def delete_dance(name: str, dances_dir: Path = DEFAULT_DANCES_DIR) -> bool:
    """Delete a saved dance. Returns True if deleted."""
    path = dances_dir / f"{name}.json"
    if path.exists():
        path.unlink()
        return True
    return False
