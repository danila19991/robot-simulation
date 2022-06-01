from task_planner.correct_solutions.max_flow import get_less


def check_plan(scenario, p):
    carts = scenario.cart_robs
    manips = scenario.manip_robs
    manup_times = [0]*len(manips)
    cart_times = list()
    for i, cart in enumerate(carts):
        print(manup_times, '?????????')
        s0 = cart.state
        t0 = 0
        l0 = s0.composite

        s1 = l0.next_choose
        t1 = t0 + l0.t1[0] + l0.t2[0]
        l1 = s1.next[p.carts[i][0]]

        s2 = l1.next_choose
        t2 = t1 + l1.t1[0] + l1.t2[0]
        l2 = s2.next[p.carts[i][1]]

        t3 = max(t2 + l2.t1[0], manup_times[s2.next[p.carts[i][1]].manip_id] + l2.t1[1]) + l2.t2[0]
        cart_times.append(t3)
        manup_times[s2.next[p.carts[i][1]].manip_id] = \
            max(t2 + l2.t1[0], manup_times[s2.next[p.carts[i][1]].manip_id] + l2.t1[1]) + l2.t2[1]

    return cart_times, manup_times


def check_half_plan(scenario, hp):
    carts = scenario.cart_robs
    manips = scenario.manip_robs
    manup_times = [0]*len(manips)
    cart_times = list()
    md = dict()
    for i, cart in enumerate(carts):
        # print(manup_times, '............')
        s0 = cart.state
        t0 = 0
        l0 = s0.composite

        s1 = l0.next_choose
        t1 = t0 + l0.t1[0] + l0.t2[0]
        l1 = s1.next[hp[i]]

        k, val = get_less(md, hp[i], s1, t1)

        s2 = l1.next_choose
        t2 = t1 + l1.t1[0] + l1.t2[0]
        l2 = s2.next[k]

        t3 = val + l2.t2[0]
        cart_times.append(t3)
        manup_times[s2.next[k].manip_id] = val + l2.t2[1]
        md[s2.id] = val + l2.t2[1]

    return cart_times, manup_times


def check_half_plan_with_time(scenario, hp):
    carts = scenario.cart_robs
    manips = scenario.manip_robs
    manup_times = [0]*len(manips)
    cart_times = list()
    md = dict()
    mt = dict()
    ct = list()
    for i in range(max(hp)+1):
        mt[i] = list()
    for i, cart in enumerate(carts):
        # print(manup_times, '............')
        s0 = cart.state
        t0 = 0
        l0 = s0.composite

        s1 = l0.next_choose
        t1 = t0 + l0.t1[0] + l0.t2[0]
        l1 = s1.next[hp[i]]

        k, val = get_less(md, hp[i], s1, t1)

        s2 = l1.next_choose
        t2 = t1 + l1.t1[0] + l1.t2[0]
        l2 = s2.next[k]

        t3 = val + l2.t2[0]
        cart_times.append(t3)
        manup_times[s2.next[k].manip_id] = val + l2.t2[1]
        md[s2.id] = val + l2.t2[1]

        ct.append((t1, t2, val-l2.t1[0], val, val+l2.t2[0]))
        mt[s2.next[k].manip_id].append((val-l2.t1[1], val, val+l2.t2[1]))

    return ct, mt

