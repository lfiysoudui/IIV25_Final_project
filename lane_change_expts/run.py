#!/usr/bin/env python3
import argparse, os, yaml, traci
from pathlib import Path
from controllers.lccs import LCCS, Params

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", required=True,
                    help="experiments/expXX.yaml")
    args = ap.parse_args()

    cfg_path = Path(args.config).resolve()
    cfg = yaml.safe_load(cfg_path.read_text())
    cfg_dir = cfg_path.parent

    # ---- SUMO cmd: always GUI, autostart, verbose -------------------
    sumo_gui = Path(os.environ["SUMO_HOME"]) / "bin" / "sumo-gui"
    sumocfg  = (cfg_dir / cfg["sumocfg"]).resolve()
    cmd = [str(sumo_gui), "--delay", "160",
           "-c", str(sumocfg),
           "--step-length", str(cfg["step_length"]),
           "-v"]

    traci.start(cmd)
    cavs = {vid: LCCS(vid, Params()) for vid in cfg["cavs"]}

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        sim_t = traci.simulation.getTime()

        # iterate over *copy* so we can pop finished ones
        for vid, ctrl in list(cavs.items()):
            ctrl.step(sim_t)
            if not ctrl.active:
                cavs.pop(vid)

    print("✅ done")

if __name__ == "__main__":
    main()
