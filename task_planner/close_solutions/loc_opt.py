from task_planner.scenarios import ExampleScenario
from task_planner.correct_solutions.dynamic import calc_time, generate_plan
import random


def generate_loc_opt_one(dist, cart_time, prev, prev_time, ns):
    curr_time = [0] * len(dist)
    for i, t in zip(prev, prev_time):
        curr_time[i] = max(curr_time[i], t)
    vals = list()
    for key, man_d in enumerate(dist):
        vals.append(calc_time(curr_time[key], man_d, ns))

    # print(len(loc_res))
    q = generate_plan(cart_time, dist, curr_time, ns)
    return q[0], vals[q[0]]


def generate_loc_opt_solutions(scenario: ExampleScenario, k: int):
    ns = scenario.cart_robs[0].state.composite.next_choose.next[0].next_choose
    res = list()
    vals = list()
    for i, cart_len in enumerate(scenario.cart_lengths):
        if i + k < len(scenario.cart_lengths):
            cart_lens = scenario.cart_lengths[i:i+k+1]
        else:
            cart_lens = scenario.cart_lengths[i:]
        if k == 0:
            q, v = generate_loc_opt_one(scenario.manip_lengths, cart_lens, list(), list(), ns)
        elif len(res) < k:
            q, v = generate_loc_opt_one(scenario.manip_lengths, cart_lens, res, vals, ns)
        else:
            q, v = generate_loc_opt_one(scenario.manip_lengths, cart_lens, res[-k:], vals[-k:], ns)
        res.append(q)
        vals.append(v)
    # print("----", res)
    return res
