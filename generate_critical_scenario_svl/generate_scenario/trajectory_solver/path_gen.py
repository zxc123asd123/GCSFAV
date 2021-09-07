import random
from math import tan, atan, sqrt

import path_gen_1
import path_gen_2
import path_gen_3
from generate_scenario import BehaviorPattern


def cal_coord(x1, y1, x2, y2, theta1, theta2):
    x = ((y2 - y1) - (x2 * tan(theta2) - x1 * tan(theta1))) / (tan(theta1) - tan(theta2))
    y = ((x2 - x1) - (y2 / tan(theta2) - y1 / tan(theta1))) / (1 / tan(theta1) - 1 / tan(theta2))
    return x, y


def cal_TTC(init_ego, dst_ego, speed_ego, t_ego, traj, speed_p, t_m, behavior, region_id):
    if behavior in ['Cut in', 'Change Lane'] and region_id in [2, 3, 4]:
        init_ego[0] = max((t_ego - t_m) * speed_ego + init_ego[0], -20)
        x1, y1 = [init_ego[0], init_ego[1]]
        x2, y2 = [traj[0], traj[1]]
        theta_ego = 0
        theta_p = atan((traj[2] - traj[0]) / (traj[3] - traj[1]))

        x, y = cal_coord(x1, y1, x2, y2, theta_ego, theta_p)
        dist_ego = sqrt((x - x1) ** 2 + (y - y1) ** 2)
        dist_p = sqrt((x - x2) ** 2 + (y - y2) ** 2)
        TTC = abs(dist_ego / speed_ego - dist_p / speed_p)

    else:
        x1, y1 = [-20, -39]
        x2, y2 = [traj[2], traj[3]]
        theta_ego = atan((dst_ego[0] + 20) / (dst_ego[1] + 39))
        theta_p = atan((traj[6] - traj[2]) / (traj[7] - traj[3]))

        x, y = cal_coord(x1, y1, x2, y2, theta_ego, theta_p)
        dist_ego = sqrt((x - x1) ** 2 + (y - y1) ** 2)
        dist_p = sqrt((x - x2) ** 2 + (y - y2) ** 2)
        if speed_p == 0:
            TTC = 100
        else:
            TTC = abs(dist_ego / speed_ego - dist_p / speed_p)

    return TTC


def get_district(id):
    critical_district = [[[-10, -2], [-60, -50]], [[-2, 6], [-25, -15]], [[3, 13], [-41, -33]]]
    return critical_district[id]


def traj_gen(behavior, region_id, init_ego, dst_ego, t_ego, D_id):
    if D_id == 0:
        SMT_Solver = path_gen_1.SMT_Solver
    elif D_id == 1:
        SMT_Solver = path_gen_2.SMT_Solver
    else:
        SMT_Solver = path_gen_3.SMT_Solver

    traj = SMT_Solver(init_ego, dst_ego, t_ego, dst_ego, region_id, behavior)
    traj[0].append(dst_ego[0])
    traj[0].append(dst_ego[1])

    return traj


def get_region(dst_ego, D):
    region_id = []
    if D[0][0] <= dst_ego[0] <= D[0][1] and D[1][0] <= dst_ego[1] <= D[1][1]:
        region_id = [[1], [2, 3, 4], [4, 10], [3, 4], [6], [3, 4, 8], [12], ['D'], [5, 7, 9, 11], [6],
                     ['SideWalk']]
    elif D[0][0] <= dst_ego[0] <= D[0][1] and D[1][0] <= dst_ego[1] <= D[1][1]:
        region_id = [[1], [2, 3, 4], [4, 6], [3, 4], [10], [3, 4, 8], [12], ['D'], [5, 7, 9, 11], [10],
                     ['SideWalk']]
    elif D[0][0] <= dst_ego[0] <= D[0][1] and D[1][0] <= dst_ego[1] <= D[1][1]:
        region_id = [[1], [2, 3, 4], [4, 8], [3, 4], [2, 4], [6, 10], [12], ['D'], [5, 7, 9, 11], [4],
                     ['SideWalk']]

    return region_id


def get_id(behavior, behaviors, region_id):
    index = random.randint(0, len(region_id[behaviors.index(behavior)]) - 1)
    return region_id[behaviors.index(behavior)][index]


def main(init_ego, dst_ego, ego_speed, t_ego, dest_d, pattern_id):
    pattern = BehaviorPattern.get_behavior_pattern(pattern_id)
    region = dict()

    d = get_district(dest_d)
    behaviors = ['Follow Vehicle', 'Follow Lane', 'Change Lane', 'Cut In', 'Vehicle Cross', 'Turn Around', 'Brake',
                 'Park', 'Retrograde', 'Pedestrian Cross', 'Pedestrian Walk']

    region_id = get_region(dst_ego, d)
    for i in range(len(behaviors)):
        region[behaviors[i]] = region_id[i]

    patterns = BehaviorPattern.get_all_behaviors()

    TTC = 100
    behavior_list = []
    vis = dict()
    for p in patterns:
        vis[p] = False
    vis[(pattern[0], pattern[1])] = True
    traj = []

    for p in patterns:
        if not vis[p] and (pattern[0] in p or pattern[1] in p):
            behavior_list.append(p)
            vis[p] = True
    print(behavior_list)
    flag = False

    while TTC > 2 and behavior_list:
        traj1 = traj_gen(behavior_list[0][0], get_id(behavior_list[0][0], behaviors, region_id), init_ego, dst_ego,
                         t_ego, dest_d)
        traj2 = traj_gen(behavior_list[0][1], get_id(behavior_list[0][1], behaviors, region_id), init_ego, dst_ego,
                         t_ego, dest_d)
        TTC1 = cal_TTC(init_ego, dst_ego, ego_speed, t_ego, traj1[0], traj1[1][0], traj1[1][1], behavior_list[0][0],
                          region_id)
        TTC2 = cal_TTC(init_ego, dst_ego, ego_speed, t_ego, traj2[0], traj2[1][0], traj2[1][1], behavior_list[0][1],
                          region_id)
        if flag:
            TTC = min(TTC1, TTC2)
        else:
            TTC = max(TTC1, TTC2)
            flag = True
        traj.extend([traj1, traj2])
        del behavior_list[0]

    return traj
