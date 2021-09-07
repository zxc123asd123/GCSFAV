import random

from Environment import environment
from generate_scenario.trajectory_solver import path_gen
import os
import math
from generate_scenario.spawn import *
from path_gen import get_district


def generate_apollo_case():
    SIMULATOR_HOST = os.environ.get("SIMULATOR_HOST", "127.0.0.1")
    SIMULATOR_PORT = int(os.environ.get("SIMULATOR_PORT", 8181))
    BRIDGE_HOST = os.environ.get("BRIDGE_HOST", "127.0.0.1")
    BRIDGE_PORT = int(os.environ.get("BRIDGE_PORT", 9090))
    print("Vehicle Follow: Connecting to the Simulator")
    # Connects to the simulator instance at the ip defined by LGSVL__SIMULATOR_HOST, default is localhost or 127.0.0.1
    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)

    # Loads the named map in the connected simulator. The available maps can be set up in web interface
    if sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco:
        sim.reset()
    else:
        sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco)

        # define environment parameter: pairwise and importance sampling
        enumerative = environment()
        i = random.randint(0, 74)
        allparameter = enumerative[i]
        if allparameter[2] == 1 or allparameter[6] == 1 or allparameter[7] == 1:
            visibility = random.uniform(0.05, 0.45)
            visibility = round(visibility, 2)
        else:
            visibility = importance_sampling(0.15, 1)
        rain, fog, wetness, cloudiness = 0, 0, 0, 0
        if allparameter[0] == 'on':
            signal = sim.get_controllable(lgsvl.Vector(180, 4.7, -38), "signal")
            control_policy = "trigger=110;green=50;yellow=10;red=50;loop"
            signal.control(control_policy)
        if allparameter[1] == 1:
            rain = random.uniform(0.75, 0.95)
            rain = round(rain, 2)
        if allparameter[2] == 1:
            fog = random.uniform(0.45, 0.95)
            fog = round(fog, 2)
        cloudiness = 1 - visibility
        sim.weather = lgsvl.WeatherState(rain=rain, fog=fog, wetness=wetness, cloudiness=cloudiness)

    # define ego route and bridge to Apollo
    district_id = random.randint(0, 2)
    dest_disrtict = get_district(district_id)
    ego_destination_x = random.randint(dest_disrtict[0][0], dest_disrtict[0][1])
    ego_destination_yy = random.randint(dest_disrtict[1][0], dest_disrtict[1][1])
    ego_destination_y = ego_destination_yy + 586
    ego_spawn_x = random.randint(-130, -50)
    ego_spawn_y = random.randint(-39, -34)
    ego_spawn = [ego_spawn_x, ego_spawn_y + 596]
    # ego_destination = [-6, -55 + 596]
    ego_destination = [ego_destination_x, ego_destination_y]
    ego_speed = random.randint(10, 15)
    ego_state, _ = spawn_direct(sim, ego_spawn[0], ego_spawn[1], ego_speed)
    # choose map and road to set up the scenario
    ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular, lgsvl.AgentType.EGO, ego_state)
    ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)
    dv = lgsvl.dreamview.Connection(sim, ego, BRIDGE_HOST)
    dv.set_hd_map('SanFrancisco')
    dv.set_vehicle('Lincoln2017MKZ_LGSVL')
    modules = [
        'Localization',
        'Perception',
        'Transform',
        'Routing',
        'Prediction',
        'Planning',
        'Control'
    ]
    dv.disable_apollo()
    dv.setup_apollo(ego_destination[0], ego_destination[1], modules)

    # monitor safety violation of ego vehicle
    ego.on_collision(on_collision)

    ego_time = math.ceil(
        get_point_distance(ego_spawn[0], ego_spawn[1], ego_destination[0], ego_destination[1]) / ego_speed)

    # define participants and their trajectories
    first_pattern = random.randint(0, 10)
    trajectories = path_gen.main(ego_spawn, [ego_destination_x, ego_destination_yy], ego_speed, ego_time, district_id,
                                 first_pattern)
    for i in range(len(trajectories)):
        trajectory = trajectories[i]
        vehicle, rotation = spawn(sim, trajectory[0][0], trajectory[0][1] + 596, 0)
        print(trajectory)
        agent = sim.add_agent("Sedan", lgsvl.AgentType.NPC, vehicle)
        if trajectory[1][0] == 0:
            if trajectory[0][0] < dest_disrtict[0][0] or trajectory[0][0] > dest_disrtict[0][0]:
                waypoints = [[trajectory[0][0], trajectory[0][1] + 596, 15],
                             [trajectory[0][2], trajectory[0][3] + 596, 10],
                             [trajectory[0][4], trajectory[0][5] + 596, 5],
                             [trajectory[0][6], trajectory[0][7] + 596, 0]]
                waypoints = get_waypoints_time(sim, waypoints, rotation,
                                               ego_time - trajectory[1][1] + 7 * (ego_spawn_x / (-130)))
                agent.follow(waypoints)
        else:
            waypoints = [[trajectory[0][0], trajectory[0][1] + 596, trajectory[1][0]],
                         [trajectory[0][2], trajectory[0][3] + 596, trajectory[1][0]],
                         [trajectory[0][4], trajectory[0][5] + 596, trajectory[1][0]],
                         [trajectory[0][6], trajectory[0][7] + 596, trajectory[1][0]]]
            waypoints = get_waypoints_time(sim, waypoints, rotation,
                                           ego_time - trajectory[1][1] + 8.5 * (ego_spawn_x / (-130)))
            agent.follow(waypoints)

    # execute the scenario
    for i in range(70):
        ego_position = ego.state.position
        print(ego_position)

        sim.run(0.5)

if __name__ == '__main__':
    generate_apollo_case()
