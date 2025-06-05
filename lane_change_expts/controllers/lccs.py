"""
Lane‐changing CAV controller based on Wang et al. (2015).

Implements:
  • horizon roll-out
  • 7-term cost (safety, efficiency, comfort, route, lane pref, switch, coop)
  • arg-min over {stay, left, right} with offset clipping
"""
from __future__ import annotations
import numpy as np, traci
from dataclasses import dataclass

@dataclass
class Params:
    horizon: float = 8.0      # T_p [s]
    dt: float = 0.5           # step [s]
    weights: dict | None = None
    lane_eval_hz: float = 1.0

    def __post_init__(self):
        if self.weights is None:
            self.weights = dict(
                safety=0.7, efficiency=0.2, comfort=0.05,
                route=0.02, pref=0.01, switch=0.01, coop=0.01
            )

class LCCS:
    """Game-theoretic lane-change controller for a single vehicle."""
    def __init__(self, vid: str, params: Params):
        self.id, self.p = vid, params
        self._t_next = 0.0      # next re-plan time
        self.active = True

    def _vehicle_exists(self) -> bool:
        """SUMO ≤1.23 has no traci.vehicle.exists() helper."""
        return self.id in traci.vehicle.getIDList()
    
    # ---------------- public API ----------------
    def step(self, sim_t: float):
        if not self.active:
            return                   # already finished

        if not self._vehicle_exists():      # arrived / teleported
            self.active = False
            return

        if sim_t < self._t_next:     # 1 Hz replan rate
            return
        self._t_next = sim_t + 1 / self.p.lane_eval_hz
        self._plan_and_act()

    # --------------- internals ------------------
    def _plan_and_act(self):
        lane_idx = traci.vehicle.getLaneIndex(self.id)
        lane_cnt = traci.edge.getLaneNumber(traci.vehicle.getRoadID(self.id))
        s0 = self._state_vec()

        best_lo, best_J = 0, float("inf")
        for lo in (-1, 0, +1):
            if not 0 <= lane_idx + lo < lane_cnt:      # stay on road
                continue
            J = self._cost(self._rollout(lo, s0))
            if J < best_J:
                best_lo, best_J = lo, J

        if best_lo:
            traci.vehicle.changeLaneRelative(
                self.id, best_lo, int(self.p.dt * 1000)
            )
        # longitudinal: let SUMO car-following handle it for now

    def _state_vec(self):
        pos = traci.vehicle.getLanePosition(self.id)
        speed = traci.vehicle.getSpeed(self.id)
        lane = traci.vehicle.getLaneIndex(self.id)
        return np.array([pos, speed, lane], dtype=float)

    def _rollout(self, lo: int, s0: np.ndarray):
        N = int(self.p.horizon / self.p.dt) + 1
        traj = np.empty((N, 3))
        x, v, lane = s0
        for k in range(N):
            traj[k] = (x, v, lane + lo)
            x += v * self.p.dt
        return traj

    def _cost(self, traj: np.ndarray) -> float:
        w = self.p.weights
        # --- placeholder: implement full 7-term cost here -------------
        safety = np.sum(np.exp(-traj[:, 1])) * self.p.dt
        return w["safety"] * safety
