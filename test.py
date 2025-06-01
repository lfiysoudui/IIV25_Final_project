import traci
import sumolib
import time

SUMO_BINARY = "sumo-gui"  # or "sumo" for CLI
CONFIG_FILE = "crossroads.sumocfg"
sumo_cmd = [SUMO_BINARY, "-c", CONFIG_FILE, "--step-length", "0.5"] # , "--log", "SUMO.log", "--start"

detection_offset = 10.0

arrival_order = []
managed_vehicles = set()

def run():
    step = 0
    traci.start(sumo_cmd)

    crossroads = traci.junction.getPosition("J_crossroads")
    print(f"Crossroads position: {crossroads}")

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1

        vehicles = traci.vehicle.getIDList()
        print(f"Step {step}: Vehicles in simulation: {vehicles}")

        for veh_id in vehicles:
            if veh_id not in arrival_order:
                pos = traci.vehicle.getPosition(veh_id)
                dx = pos[0] - crossroads[0]
                dy = pos[1] - crossroads[1]
                dist = (dx**2 + dy**2)**0.5
                if dist < 2 and veh_id:
                    arrival_order.append(veh_id)
                    managed_vehicles.add(veh_id)

        if arrival_order:
            first = arrival_order[0]
            for veh_id in vehicles:
                if veh_id == first:
                    traci.vehicle.setSpeed(veh_id, 10)
                else:
                    traci.vehicle.setSpeed(veh_id, 0)

    traci.close()


if __name__ == "__main__":
    run()
