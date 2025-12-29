from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from ..prim_utils import delete_prim

log = logging.getLogger(__name__)


@dataclass
class _DeferredDelete:
    prim_path: str
    remaining_s: float


@dataclass
class ThroughputStats:
    """
    Collect cycle times and compute throughput / uptime needs.
    Also handles deferred deletes (for animation/illusion convenience).
    """
    sim_time_s: float = 0.0
    cycles_s: List[float] = field(default_factory=list)
    deferred: List[_DeferredDelete] = field(default_factory=list)

    def tick_time(self, dt: float) -> None:
        self.sim_time_s += dt

    def record_cycle(self, start_s: float, end_s: float) -> None:
        self.cycles_s.append(max(0.0, end_s - start_s))

    def avg_cycle_s(self) -> float:
        if not self.cycles_s:
            return 0.0
        return sum(self.cycles_s) / len(self.cycles_s)

    def required_hours_for_units(self, units_per_day: int = 200) -> float:
        """
        If average cycle time is known, estimate hours of robot uptime to reach a daily target.
        """
        avg = self.avg_cycle_s()
        if avg <= 0:
            return 0.0
        total_seconds = avg * units_per_day
        return total_seconds / 3600.0

    def defer_delete(self, prim_path: str, delay_s: float) -> None:
        self.deferred.append(_DeferredDelete(prim_path=prim_path, remaining_s=delay_s))

    def step_deferred_deletes(self, dt: float) -> None:
        if not self.deferred:
            return
        for d in list(self.deferred):
            d.remaining_s -= dt
            if d.remaining_s <= 0:
                try:
                    delete_prim(d.prim_path)
                except Exception:
                    pass
                self.deferred.remove(d)
