#!/usr/bin/env python3

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import re
import time
from typing import Iterable, List, Optional


def sanitize_filename(value: str, default: str = 'unnamed') -> str:
    cleaned = re.sub(r'[^A-Za-z0-9_.-]+', '_', value.strip().strip('/'))
    cleaned = cleaned.strip('._')
    return cleaned or default


def parse_csv(value: object) -> List[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        items: Iterable[object] = value
    else:
        items = str(value).split(',')
    return [
        str(item).strip().strip('/')
        for item in items
        if str(item).strip().strip('/')
    ]


def default_run_id() -> str:
    return datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')


def discover_repo_root() -> Path:
    candidates = [Path.cwd(), Path(__file__).resolve()]
    home_repo = Path.home() / 'Multi-Robot-Nav'
    if home_repo.exists():
        candidates.append(home_repo)

    for candidate in candidates:
        start = candidate if candidate.is_dir() else candidate.parent
        for parent in [start, *start.parents]:
            if parent.name == 'Multi-Robot-Nav':
                return parent
            if (
                (parent / 'workspace').exists()
                and (parent / 'Swarm-SLAM').exists()
                and (parent / 'Multi-Robot-Graph-SLAM').exists()
            ):
                return parent

    return Path.cwd()


def resolve_results_root(configured_root: str) -> Path:
    if configured_root.strip():
        return Path(configured_root).expanduser().resolve()
    return discover_repo_root() / 'results'


def make_run_directory(
    results_root: str,
    run_type: str,
    run_id: Optional[str] = None,
) -> Path:
    resolved_run_id = (run_id or '').strip() or default_run_id()
    directory = (
        resolve_results_root(results_root)
        / sanitize_filename(run_type, 'run')
        / sanitize_filename(resolved_run_id, 'run')
    )
    directory.mkdir(parents=True, exist_ok=True)
    return directory


class FileEventLogger:
    def __init__(self, node, log_path: Path) -> None:
        self.node = node
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.log_path.open('a', encoding='utf-8', buffering=1)

    def info(self, message: str) -> None:
        self.node.get_logger().info(message)
        self._write('INFO', message)

    def warn(self, message: str) -> None:
        self.node.get_logger().warn(message)
        self._write('WARN', message)

    def error(self, message: str) -> None:
        self.node.get_logger().error(message)
        self._write('ERROR', message)

    def _write(self, level: str, message: str) -> None:
        timestamp = datetime.now(timezone.utc).isoformat()
        self._file.write(f'{timestamp} {level} {message}\n')

    def close(self) -> None:
        self._file.close()


def clock_msg_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def now_wall_time() -> float:
    return time.time()
