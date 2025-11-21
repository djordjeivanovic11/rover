from pathlib import Path
from typing import Dict, Tuple

import yaml


class WaypointManager:
    """Load and lookup named GPS waypoints from a YAML config."""

    def __init__(self, yaml_path: Path):
        self._yaml_path = Path(yaml_path)
        self._waypoints: Dict[str, Dict[str, float]] = {}
        self._load()

    def _load(self) -> None:
        if not self._yaml_path.exists():
            # Empty database is allowed; caller can handle missing names
            return

        with self._yaml_path.open("r") as f:
            data = yaml.safe_load(f) or {}

        self._waypoints = data.get("waypoints", {}) or {}

    def lookup(self, name: str) -> Tuple[float, float]:
        """Return (lat, lon) for a named waypoint."""
        if name not in self._waypoints:
            raise KeyError(f"Waypoint '{name}' not found in {self._yaml_path}")

        entry = self._waypoints[name]
        return float(entry["latitude"]), float(entry["longitude"])


