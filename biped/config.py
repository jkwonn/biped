"""loads, saves, and merges config files (yaml or json)"""
import json
import copy
import yaml
from pathlib import Path
from typing import Optional

DEFAULT_CONFIG_DIR = Path(__file__).parent.parent / "configs"
DEFAULT_CONFIG_PATH = DEFAULT_CONFIG_DIR / "default.yaml"


def load_config(path: Optional[str] = None) -> dict:
    """loads the default config and optionally merges in a user config on top"""
    cfg = _load_file(DEFAULT_CONFIG_PATH)
    if path:
        cfg = _deep_merge(cfg, _load_file(Path(path)))
    return cfg


def save_config(cfg: dict, path: str):
    """writes a config dict to yaml or json based on file extension"""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w") as f:
        if p.suffix in (".yaml", ".yml"):
            yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)
        else:
            json.dump(cfg, f, indent=2)
    print(f"config saved to {path}")


def _load_file(path: Path) -> dict:
    with open(path) as f:
        if path.suffix in (".yaml", ".yml"):
            return yaml.safe_load(f)
        return json.load(f)


def _deep_merge(base: dict, override: dict) -> dict:
    """recursively merges the override dict into the base dict"""
    result = copy.deepcopy(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = copy.deepcopy(v)
    return result
