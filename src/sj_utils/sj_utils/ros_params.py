import os
from typing import Any, Dict, Optional

import yaml


def load_ros_params(
    path: str,
    node_key: str,
    defaults: Dict[str, Any],
    logger: Optional[Any] = None,
) -> Dict[str, Any]:
    if not path:
        return defaults
    if not os.path.isfile(path):
        if logger:
            logger.warn(f"Config file not found: {path}")
        return defaults
    if logger:
        logger.info(f"Loading params from: {path}")
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    except Exception as exc:
        if logger:
            logger.warn(f"Failed to read config file {path}: {exc}")
        return defaults
    node_cfg = data.get(node_key, {})
    ros_params = node_cfg.get("ros__parameters", {})
    if not isinstance(ros_params, dict):
        if logger:
            logger.warn(f"Invalid ros__parameters in {path} for {node_key}")
        return defaults
    merged = defaults.copy()
    merged.update(ros_params)
    return merged
