from __future__ import annotations
import math, traci
from dataclasses import dataclass
import numpy as np


# ─────────────────────────── parameters ────────────────────────────
@dataclass
class Weights:
    safety: float = 0.70
    efficiency: float = 0.20
    comfort: float = 0.05
    route: float = 0.02          # soft lane pref / exit penalty
    switch: float = 0.02
    coop: float = 0.01           # for Experiments 4–6


@dataclass
class MandatoryCfg:
    exit_edge: str = ""          # ""  ⇒ discretionary only
    goal_lane: int = 0
    trigger_dist: float = 120.0
    k_penalty: float = 0.25      # steeper ⇒ stronger push
    hard_deadline: bool = False  # True ⇒ infinite cost at exit


@dataclass
class Params:
    horizon: float = 8.0
    dt: float = 0.5
    replanning_hz: float = 1.0
    w: Weights = Weights()
    mandatory: MandatoryCfg = MandatoryCfg()


# ───────────────────────── unified controller ──────────────────────
class UnifiedLC:
    """
    One-size-fits-all lane-change controller following Wang et al.:
        • discretionary overtaking if no exit constraint
        • mandatory merge when exit_edge != "" and distance < trigger
        • escalating penalty (soft) or hard constraint (hard_deadline)
    """

    def __init__(self, vid: str, p: Params):
        self.id, self.p = vid, p
        self._next_plan = 0.0
        self.active = True

    # ───────── public entry point ─────────
    def step(self, sim_t: float):
        if not self.active:
            return
        if not self._exists():
            self.active = False
            return
        if sim_t < self._next_plan:
            return
        self._next_plan = sim_t + 1 / self.p.replanning_hz
        self._plan_and_act()

    # ───────── internals ─────────
    def _exists(self) -> bool:
        return self.id in traci.vehicle.getIDList()

    def _plan_and_act(self):
        lane_idx = traci.vehicle.getLaneIndex(self.id)
        lane_cnt = traci.edge.getLaneNumber(traci.vehicle.getRoadID(self.id))
        s0 = self._state_vec()

        best_lo, bestJ = 0, float("inf")
        for lo in (-1, 0, +1):
            if not 0 <= lane_idx + lo < lane_cnt:
                continue
            J = self._cost(self._rollout(lo, s0), lane_idx + lo)
            if J < bestJ:
                bestJ, best_lo = J, lo

        # hard-deadline override (forces lane change even if cost↑)
        if self._must_merge_now(lane_idx):
            best_lo = -1 if lane_idx > self.p.mandatory.goal_lane else 0

        if best_lo:
            traci.vehicle.changeLaneRelative(
                self.id, best_lo, int(self.p.dt * 1000))

    def _state_vec(self):
        pos = traci.vehicle.getLanePosition(self.id)
        v   = traci.vehicle.getSpeed(self.id)
        ln  = traci.vehicle.getLaneIndex(self.id)
        return np.array([pos, v, ln])

    def _rollout(self, lo: int, s0: np.ndarray):
        N = int(self.p.horizon / self.p.dt) + 1
        out = np.empty((N, 3))
        x, v, ln = s0
        for k in range(N):
            out[k] = (x, v, ln + lo)
            x += v * self.p.dt
        return out

    # ─────── cost function incl. mandatory term ───────
    def _cost(self, traj: np.ndarray, cand_lane: int) -> float:
        w = self.p.w
        safety   = np.sum(np.exp(-traj[:, 1])) * self.p.dt                # toy
        eff      = -traj[-1, 1]                                           # want v high
        comfort  = np.var(np.diff(traj[:, 1])) if len(traj) > 1 else 0.0
        switch   = 1.0 * (cand_lane != traj[0, 2])
        coop     = 0.0                                                    # to-do

        route_pen = 0.0
        mand = self.p.mandatory
        if mand.exit_edge:
            dist = self._dist_to_exit()
            if cand_lane != mand.goal_lane:
                # logistic penalty that explodes as dist→0
                route_pen = mand.k_penalty * (
                    1 / (1 + math.exp( 0.1 * (dist - mand.trigger_dist))) )
                if mand.hard_deadline and dist < 5.0:
                    route_pen = 1e6

        return (w.safety   * safety   +
                w.efficiency * eff     +
                w.comfort   * comfort +
                w.route     * route_pen +
                w.switch    * switch  +
                w.coop      * coop)

    def _dist_to_exit(self) -> float:
        """Distance along current edge until divergence node (if same edge)."""
        mand = self.p.mandatory
        cur_edge = traci.vehicle.getRoadID(self.id)
        pos = traci.vehicle.getLanePosition(self.id)
        if mand.exit_edge.startswith(cur_edge):
            return traci.edge.getLength(cur_edge) - pos
        return float("inf")

    def _must_merge_now(self, lane_idx: int) -> bool:
        mand = self.p.mandatory
        if not mand.exit_edge or lane_idx == mand.goal_lane:
            return False
        return mand.hard_deadline and self._dist_to_exit() < mand.trigger_dist
