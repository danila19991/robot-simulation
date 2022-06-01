import numpy as np

from task_planner.scenarios import ExampleScenario
from task_planner.close_solutions.greed_planner import get_loc_lens
import random


def generate_min_prob_one(dist, cart_time, prev, prev_time, ns, p, a):
    vals = np.array(get_loc_lens(dist, cart_time, prev, prev_time, ns))
    vals_orig = vals.copy()
    vals = vals - np.min(vals)
    vals = np.max(vals) - vals
    vals = vals**a
    vals = vals - np.min(vals)
    if np.sum(vals) > 1e-3:
        vals = vals / np.sum(vals)
    else:
        vals = np.array([1/len(vals)]*len(vals))

    # print(vals)
    q = np.random.choice(list(range(len(dist))), p=vals)
    # print(q, vals_orig[q], vals_orig, vals)
    return q, vals_orig[q]


def generate_prob_solutions(scenario: ExampleScenario, k: int, p, a):
    ns = scenario.cart_robs[0].state.composite.next_choose.next[0].next_choose
    res = list()
    vals = list()
    for cart_len in scenario.cart_lengths:
        if k == 0:
            q, v = generate_min_prob_one(scenario.manip_lengths, cart_len, list(), list(), ns, p, a)
        elif len(res) < k:
            q, v = generate_min_prob_one(scenario.manip_lengths, cart_len, res, vals, ns, p, a)
        else:
            q, v = generate_min_prob_one(scenario.manip_lengths, cart_len, res[-k:], vals[-k:], ns, p, a)
        res.append(q)
        vals.append(v)
    # print("----", res)
    return res
