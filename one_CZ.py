import traci
import sumolib
import time

SUMO_BINARY = "sumo-gui"  # or "sumo" for CLI
CONFIG_FILE = "crossroads.sumocfg"
sumo_cmd = [SUMO_BINARY, "-c", CONFIG_FILE, "--step-length", "0.5", "--start"] # , "--log", "SUMO.log"

detection_offset = 10.0

arrival_order = []
managed_vehicles = set()
current_intersection = "J_crossroads"
current_intersection_ID = "J_crossroads_0_0"

def print_vehicle_info(veh_id):
    print(f"--- Info for vehicle {veh_id} ---")
    print("Position:", traci.vehicle.getPosition(veh_id))
    print("Speed:", traci.vehicle.getSpeed(veh_id))
    print("Angle:", traci.vehicle.getAngle(veh_id))
    print("Lane ID:", traci.vehicle.getLaneID(veh_id))
    print("Route:", traci.vehicle.getRoute(veh_id))
    print("Type ID:", traci.vehicle.getTypeID(veh_id))
    print("Acceleration:", traci.vehicle.getAcceleration(veh_id))
    print("Color:", traci.vehicle.getColor(veh_id))
    print("Waiting time:", traci.vehicle.getWaitingTime(veh_id))
    print("Distance:", traci.vehicle.getDistance(veh_id))
    print("Edge ID:", traci.vehicle.getRoadID(veh_id))
    print("-------------------------------")

def run():
    step = 0
    traci.start(sumo_cmd)

    crossroads = traci.junction.getPosition(current_intersection)
    print(f"Current crossroads position: {crossroads}")

    net = sumolib.net.readNet("./crossroads.net.xml")
    junction = net.getNode(current_intersection)

    incoming_edges = [edge.getID() for edge in junction.getIncoming()]
    Outgoing_edges = [edge.getID() for edge in junction.getOutgoing()]
    print("Incoming edges to junction:", incoming_edges)
    print("Outgoing edges from junction:", Outgoing_edges)
    
    current_vehicle = {"ID":None, "loc":None}
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1

        vehicles = traci.vehicle.getIDList()
        print(f"Step {step}: Vehicles in simulation: {vehicles}")

        for veh_id in vehicles:
            pos_on_lane = traci.vehicle.getLanePosition(veh_id)
            lane_id = traci.vehicle.getLaneID(veh_id)
            lane_length = traci.lane.getLength(lane_id)
            stop_line_pos = lane_length - 4  # adjust if you know the stop line is before the lane end
            if pos_on_lane >= stop_line_pos:
                if veh_id not in managed_vehicles:
                    print(f"Vehicle {veh_id} has reached the stop line at position {pos_on_lane} on lane {lane_id}.")
                    managed_vehicles.add(veh_id)
                    arrival_order.append(veh_id)
                    
        # check if the current vehicle finished crossing the junction
        print(f"Arrival order: {arrival_order}")
        if current_vehicle["ID"]:
            for edges in Outgoing_edges:
                if current_vehicle["loc"].startswith(edges):
                    print(f"Current vehicle {current_vehicle['ID']} is leaving the junction on edge {current_vehicle['loc']}.")
                    current_vehicle["ID"] = None
                    arrival_order.pop(0)
                    break
        
        if arrival_order:
            first = arrival_order[0]
            for veh_id in arrival_order:
                if veh_id == current_vehicle["ID"] or veh_id == first:
                    traci.vehicle.setSpeed(veh_id, 10)
                    current_vehicle["ID"] = veh_id
                    current_vehicle["loc"] = traci.vehicle.getLaneID(veh_id)
                    print(f"Current vehicle set to {current_vehicle} with speed 10.")
                else:
                    traci.vehicle.setSpeed(veh_id, 0)
                    print(f"Vehicle {veh_id} stopped.")
        
        if current_vehicle["ID"]:
            print(f"{current_vehicle}:{traci.vehicle.getLaneID(current_vehicle["ID"])}")

        print("----------------------------------------")
    traci.close()


if __name__ == "__main__":
    run()
