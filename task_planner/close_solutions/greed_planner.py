from task_planner.scenarios import ExampleScenario
from task_planner.correct_solutions.dynamic import calc_time
import random


def get_loc_lens(dist, cart_time, prev, prev_time, ns):
    curr_time = [0] * len(dist)
    for i, t in zip(prev, prev_time):
        curr_time[i] = max(curr_time[i], t)
    vals = list()
    for key, man_d in enumerate(dist):
        vals.append(calc_time(curr_time[key], man_d, ns))
    return vals


def generate_min_ex_one(dist, cart_time, prev, prev_time, ns):
    vals = get_loc_lens(dist, cart_time, prev, prev_time, ns)
    nv = min(vals)
    loc_res = list()
    for i, v in enumerate(vals):
        if v == nv:
            loc_res.append(i)

    # print(len(loc_res))
    return random.choice(loc_res), nv


def generate_greed_solutions(scenario: ExampleScenario, k: int):
    ns = scenario.cart_robs[0].state.composite.next_choose.next[0].next_choose
    res = list()
    vals = list()
    for cart_len in scenario.cart_lengths:
        if k == 0:
            q, v = generate_min_ex_one(scenario.manip_lengths, cart_len, list(), list(), ns)
        elif len(res) < k:
            q, v = generate_min_ex_one(scenario.manip_lengths, cart_len, res, vals, ns)
        else:
            q, v = generate_min_ex_one(scenario.manip_lengths, cart_len, res[-k:], vals[-k:], ns)
        res.append(q)
        vals.append(v)
    # print("----", res)
    return res
