import os
import sys
import traci
import random

# Step 1: Ensure SUMO_HOME is set
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 2: Define SUMO configuration
Sumo_config = [
    'sumo-gui',
    '-c', 'ang4m.sumocfg',
    '--step-length', '1',
    '--delay', '150',
    '--lateral-resolution', '0.1'
]

traci.start(Sumo_config)

# Step 3: Define Variables
junctions = traci.trafficlight.getIDList()
default_phase_duration = 30
optimization_interval = 300  # Check every 300 steps
step = 0
min_phase_duration = 10  # Minimum phase duration to prevent rapid switching
max_phase_duration = 60  # Maximum phase duration
critical_queue_threshold = 20  # If queue exceeds this, override other rules

def get_vehicle_data(junction_id):
    """Retrieve vehicle counts and waiting times per lane."""
    lanes = traci.trafficlight.getControlledLanes(junction_id)
    vehicle_counts, waiting_times = {}, {}
    
    for lane in lanes:
        vehicles = traci.lane.getLastStepVehicleIDs(lane)
        stopped_vehicles = [veh for veh in vehicles if traci.vehicle.getSpeed(veh) == 0]
        
        vehicle_counts[lane] = len(stopped_vehicles)
        waiting_times[lane] = sum(traci.vehicle.getWaitingTime(veh) for veh in vehicles)
    
    return vehicle_counts, waiting_times

def jaya_algorithm(junctions):
    """Jaya algorithm to optimize traffic signal timing."""
    def objective_function(phases):
        return sum(phases.values())
    
    population_size, max_iterations = 10, 100
    population = [{j: random.uniform(min_phase_duration, max_phase_duration) for j in junctions} for _ in range(population_size)]
    
    for _ in range(max_iterations):
        fitness = [objective_function(ind) for ind in population]
        best_index = fitness.index(min(fitness))
        best = population[best_index]
        
        for i in range(population_size):
            if i != best_index:
                for j in junctions:
                    r1, r2 = random.random(), random.random()
                    new_value = population[i][j] + r1 * (best[j] - abs(population[i][j])) - r2 * (max_phase_duration - abs(population[i][j]))
                    population[i][j] = max(min_phase_duration, min(max_phase_duration, new_value))
    
    return population[best_index]

def adjust_traffic_signals(optimal_phases):
    """Adjust all traffic signal phases based on optimization."""
    for junction in junctions:
        traci.trafficlight.setPhaseDuration(junction, optimal_phases[junction])
        print(f'Adjusted traffic light {junction} to duration {optimal_phases[junction]}')

def dynamically_adjust_signals():
    """Dynamically adjust signals based on real-time congestion."""
    for junction in junctions:
        lanes = traci.trafficlight.getControlledLanes(junction)
        queue_lengths = {lane: traci.lane.getLastStepVehicleNumber(lane) for lane in lanes}
        total_queue = sum(queue_lengths.values())

        if total_queue > 0:
            max_queue_lane = max(queue_lengths, key=queue_lengths.get)
            max_queue = queue_lengths[max_queue_lane]
            queue_threshold = max_queue * 0.3  # Instead of 50%, use dynamic 30%
            
            green_time = max(min_phase_duration, min(max_phase_duration, int((max_queue / total_queue) * max_phase_duration)))
            
            if max_queue > critical_queue_threshold:
                traci.trafficlight.setPhaseDuration(junction, max_phase_duration)
                print(f"Critical congestion at {junction}, extending green to {max_phase_duration}s")
            else:
                traci.trafficlight.setPhaseDuration(junction, green_time)
                print(f"Adjusted {junction}: Green for {green_time}s (Queue: {max_queue})")
            
            while queue_lengths[max_queue_lane] > queue_threshold:
                traci.simulationStep()
                queue_lengths[max_queue_lane] = traci.lane.getLastStepVehicleNumber(max_queue_lane)
            
            current_phase = traci.trafficlight.getPhase(junction)
            new_phase = (current_phase + 1) % len(traci.trafficlight.getAllProgramLogics(junction)[0].phases)
            traci.trafficlight.setPhase(junction, new_phase)
            print(f"Switched phase for {junction} to {new_phase}")

# Step 5: Simulation Loop
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    
    if step % optimization_interval == 0:
        optimal_phases = jaya_algorithm(junctions)
        adjust_traffic_signals(optimal_phases)
    
    dynamically_adjust_signals()
    step += 1

traci.close()
