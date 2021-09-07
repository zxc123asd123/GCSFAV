import math

import lgsvl


def spawn(sim, x, y, speed, reverse=False):
    state = lgsvl.AgentState()
    state.transform = get_position(sim, x, y)
    if reverse:
        state.transform.rotation = state.transform.rotation - lgsvl.Vector(0, 180, 0)
    forward = lgsvl.utils.transform_to_forward(state.transform)
    state.velocity = speed * forward

    return state, state.transform.rotation


def spawn_direct(sim, x, y, speed, reverse=False):
    state = lgsvl.AgentState()
    transform = get_position(sim, x, y)
    state.transform.position = lgsvl.Vector(x, 10.2, y)
    state.transform.rotation = transform.rotation
    if reverse:
        state.transform.rotation = state.transform.rotation - lgsvl.Vector(0, 180, 0)
    forward = lgsvl.utils.transform_to_forward(state.transform)
    state.velocity = speed * forward

    return state, state.transform.rotation


def get_position(sim, x, y):
    position = lgsvl.Vector(x, 10.2, y)
    return sim.map_point_on_lane(position)


def get_waypoints_time(sim, waypoints, rotation=None, idle=0):
    npc_waypoints = []
    for i in range(len(waypoints)):
        transform = get_position(sim, waypoints[i][0], waypoints[i][1])
        if i == 0:
            if rotation is not None:
                npc_waypoints.append(lgsvl.DriveWaypoint(transform.position, waypoints[i][2], rotation, idle, False))
            else:
                npc_waypoints.append(
                    lgsvl.DriveWaypoint(transform.position, waypoints[i][2], transform.rotation, idle, False))
        else:
            transform_former = get_position(sim, waypoints[i-1][0], waypoints[i-1][1])
            rotation = transform.rotation - transform_former.rotation
            rotation = transform.rotation
            npc_waypoints.append(
                lgsvl.DriveWaypoint(transform.position, waypoints[i][2], rotation, 0, False))

    return npc_waypoints


def get_waypoints_distance(sim, waypoints, rotation=None, distance=0):
    npc_waypoints = []
    for i in range(len(waypoints)):
        transform = get_position(sim, waypoints[i][0], waypoints[i][1])
        if rotation is not None:
            npc_waypoints.append(lgsvl.DriveWaypoint(transform.position, waypoints[i][2], rotation, 0, False,
                                                     trigger_distance=distance))
        else:
            npc_waypoints.append(lgsvl.DriveWaypoint(transform.position, waypoints[i][2], transform.rotation, 0, False,
                                                     trigger_distance=distance))

    return npc_waypoints


def get_waypoints_direct(sim, waypoints, rotation=None, distance=0):
    npc_waypoints = []
    for i in range(len(waypoints)):
        transform_map = get_position(sim, waypoints[i][0], waypoints[i][1])
        transform = lgsvl.Vector(waypoints[i][0], 10.2, waypoints[i][1])
        if rotation is not None:
            npc_waypoints.append(lgsvl.DriveWaypoint(transform, waypoints[i][2], rotation, 0, False,
                                                     trigger_distance=distance))
        else:
            npc_waypoints.append(lgsvl.DriveWaypoint(transform, waypoints[i][2], transform_map.rotation, 0, False,
                                                     trigger_distance=distance))

    return npc_waypoints


def get_point_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))


def on_collision(agent1, agent2, contact):
    name1 = "STATIC OBSTACLE" if agent1 is None else agent1.name
    name2 = "STATIC OBSTACLE" if agent2 is None else agent2.name
    print("{} collided with {} at {}".format(name1, name2, contact))
