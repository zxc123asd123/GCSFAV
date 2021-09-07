from z3 import *
import random


def SMT_Solver(init_ego, dst_ego, time_ego, dst_p, region_id, behavior):
    speed, t_e, t_m = Ints('speed t_e t_m')
    init_y, init_x, dst_x, dst_y = Ints('init_y init_x dst_x dst_y')
    adjust_wp_y = [Int("adjust_wp_y_%s" % i) for i in range(2)]
    adjust_wp_x = [Int("adjust_wp_x_%s" % i) for i in range(2)]
    dist = [Real('dist_%s' % i) for i in range(3)]
    tot_dist = Int('tot_dist')

    init_condition = [
        dst_x == dst_p[0],
        dst_y == dst_p[1]
    ]

    init_c = [
        If(region_id in [1, 2], init_y == -39,
           If(region_id in [3, 4], init_y == -35,
              If(region_id in [5, 8], Or(init_y == -27, init_y == -31),
                 If(region_id == 9, Or(init_y == -39, init_y == -35),
                    If(region_id in [6, 7], And(-13 <= init_y, init_y <= 30),
                       If(region_id in [10, 11], And(-100 <= init_y, init_y <= -50),
                          If(region_id == 12, And(-39 <= init_y, init_y <= -27),
                             And(-25 <= init_y, init_y <= -15)))))))),
        If(region_id in [1, 3], And(-150 <= init_x, init_x < init_ego[0]),
           If(region_id in [2, 4], And(init_ego[0] < init_x, init_x <= -21),
              If(region_id == 5, And(-150 <= init_x, init_x <= -21),
                 If(region_id in [8, 9], And(15 <= init_x, init_x <= 50),
                    If(region_id in [6, 11], Or(init_x == -8, init_x == -4),
                       If(region_id in [7, 10],
                          If(behavior == 'Pedestrian Cross', init_x == 6, Or(init_x == 0, init_x == 4)),
                          If(region_id == 12, And(-8 <= init_x, init_x <= 4), And(-2 <= init_x, init_x <= 6))))))))
    ]

    wp_c = [
        If(behavior == 'Follow Vehicle', And(adjust_wp_x[0] == -21, adjust_wp_y[0] == -39),
           If(behavior == 'Follow Lane', And(adjust_wp_x[0] == -21, adjust_wp_y[0] == init_y),
              If(behavior == 'Change Lane', If(region_id == 4, And(adjust_wp_y[0] == init_y - 4, adjust_wp_x[0] == -21),
                                               And(adjust_wp_y[0] == -14, adjust_wp_x[0] == -2)),
                 If(behavior == 'Cut In', And(adjust_wp_y[0] == init_y - 4, adjust_wp_x[0] == -21),
                    If(behavior == 'Vehicle Cross', And(adjust_wp_x[0] == init_x, adjust_wp_y[0] == -14),
                       If(behavior == 'Turn Around',
                          And(adjust_wp_y[0] == init_y,
                              If(region_id in [3, 4], adjust_wp_x[0] == -21, adjust_wp_x[0] == 15)),
                          If(behavior == 'Retrograde',
                             If(region_id == 5, And(adjust_wp_y[0] == init_y, adjust_wp_x[0] == -21),
                                If(region_id == 7, And(adjust_wp_x[0] == init_x, adjust_wp_y[0] == -13),
                                   If(region_id == 9, And(adjust_wp_y[0] == init_y, adjust_wp_x[0] == 15),
                                      And(adjust_wp_x[0] == init_x, adjust_wp_y[0] == -50)))),
                             If(behavior == 'Pedestrian Cross', And(adjust_wp_y[0] == -50, adjust_wp_x[0] == init_x),
                                And(adjust_wp_x[0] == init_x, adjust_wp_y[0] == init_y))))))))),
        If(behavior == 'Pedestrian Cross', And(adjust_wp_x[1] == init_x, adjust_wp_y[1] == init_y),
           If(And(behavior == 'Change Lane', region_id == 6),
              And(And(dst_ego[1] <= adjust_wp_y[1], adjust_wp_y[1] <= -14), adjust_wp_x[1] == -2),
              And(If(region_id == 7, adjust_wp_y[1] == -14, adjust_wp_y[1] == -24), adjust_wp_x[1] == dst_ego[0])))
    ]

    dist_c = [
        dist[0] ** 2 == (adjust_wp_y[0] - init_y) ** 2 + (adjust_wp_x[0] - init_x) ** 2,
        dist[1] ** 2 == (adjust_wp_y[0] - adjust_wp_y[1]) ** 2 + (adjust_wp_x[0] - adjust_wp_x[1]) ** 2,
        dist[2] ** 2 == (adjust_wp_y[1] - dst_x) ** 2 + (adjust_wp_x[1] - dst_y) ** 2,
        And([d >= 0 for d in dist]),
        tot_dist == ToInt(Sum([d for d in dist])),
    ]

    time_c = [
        t_e == time_ego,
        If(behavior in ['Pedestrian Cross', 'Pedestrian Walk'], And(0 < speed, speed <= 5),
           And(0 < speed, speed <= 20)),
        And(0 <= t_m, t_m <= t_e),
        Or(And(speed * t_m >= tot_dist, speed * t_m - tot_dist <= speed),
           And(speed * t_m < tot_dist, tot_dist - speed * t_m <= speed)),
        # speed * t_m == tot_dist
    ]

    def get_num(val):
        val = str(val)
        num = ''
        for ch in val:
            if '0' <= ch <= '9' or ch == '-':
                num += ch
        if num != '':
            return int(num)
        if val in ['speed', 't_m']:
            return 0
        return val

    def n_solutions(cnt):
        s = Solver()
        s.add(init_condition + init_c)
        if behavior not in ['Park', 'Brake', 'Pedestrian Cross', 'Pedestrian Walk']:
            s.add(wp_c + dist_c + time_c)
        i = 0
        # print(s.assertions())
        print(s.check())
        set_option(rational_to_decimal=True)
        set_option(precision=0)
        res = []
        while s.check() == sat and i < cnt:
            m = s.model()
            # print([m.evaluate(init_x), m.evaluate(init_y), m.evaluate(adjust_wp_x[0]), m.evaluate(adjust_wp_y[0])] +
            #       [m.evaluate(adjust_wp_x[1]), m.evaluate(adjust_wp_y[1])] + [get_num(m.evaluate(tot_dist))] +
            #       [m.evaluate(speed), m.evaluate(t_m)])
            res.append([[get_num(m.evaluate(init_x)), get_num(m.evaluate(init_y)), get_num(m.evaluate(adjust_wp_x[0])),
                         get_num(m.evaluate(adjust_wp_y[0])), get_num(m.evaluate(adjust_wp_x[1])),
                         get_num(m.evaluate(adjust_wp_y[1]))], [get_num(m.evaluate(speed)), get_num(m.evaluate(t_m))]])
            # print(res[-1])
            fml = And([And(init_y == m.evaluate(init_y), init_x == m.evaluate(init_x)) for i in
                       range(2)])
            s.add(Not(fml))
            i += 1
        if len(res) > 0:
            return res[random.randint(0, len(res) - 1)]
        return []

    return n_solutions(100)
