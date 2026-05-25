#!/usr/bin/env python3
"""Small Python equivalent of the CSLAM simulated rendezvous gate."""

from __future__ import annotations

import time
from pathlib import Path


class SimulatedRendezvous:
    """Gate communication using the same relative schedule format as CSLAM."""

    def __init__(self, node, enabled: bool, schedule_file: str, robot_id: int):
        self.node = node
        self.enabled = bool(enabled)
        self.robot_id = int(robot_id)
        self.initial_time = int(time.time())
        self.ranges: list[tuple[int, int]] = []

        if not self.enabled:
            return

        try:
            path = Path(schedule_file)
            with path.open("r", encoding="utf-8") as schedule:
                for raw_line in schedule:
                    line = raw_line.strip()
                    if not line or line.startswith("#"):
                        continue
                    fields = [field.strip() for field in line.split(",") if field.strip()]
                    if not fields or int(fields[0]) != self.robot_id:
                        continue
                    values = [int(field) for field in fields[1:]]
                    if len(values) % 2 != 0:
                        raise ValueError(
                            f"Robot {self.robot_id} rendezvous schedule has an odd number of time entries"
                        )
                    for start, end in zip(values[0::2], values[1::2]):
                        self.ranges.append(
                            (self.initial_time + start, self.initial_time + end)
                        )
                    self.node.get_logger().info(
                        f"Simulated rendezvous schedule of robot {line}"
                    )
                    break
        except Exception as exc:
            self.node.get_logger().error(
                f"Reading simulated rendezvous schedule failed: {exc}"
            )
            self.enabled = False

    def is_alive(self) -> bool:
        if not self.enabled:
            return True

        now = int(time.time())
        return any(start <= now <= end for start, end in self.ranges)
