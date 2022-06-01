import numpy as np

from task_planner.plan import ExecutionPlan
from task_planner.scenarios import ExampleScenario


def calc_time(curr_manip_time, start_cart_time, ns):
    v1 = max(start_cart_time + ns.next[0].t1[0], curr_manip_time + ns.next[0].t1[1]) + ns.next[0].t2[1]
    v2 = max(start_cart_time + ns.next[1].t1[0], curr_manip_time + ns.next[1].t1[1]) + ns.next[1].t2[1]
    # print(v1, v2)
    return min(v1, v2)


def calc_one_manip_time(cart_lengths, ns, manip_time, start_manip_time):
    l = (1 << len(cart_lengths))+1
    res = [start_manip_time]*l
    for i in range(len(cart_lengths)):
        for id in range(0, l-1, 1 << (i+1)):
            for j in range(1 << i, 1 << (i+1)):
                res[id+j] = calc_time(res[id+j], cart_lengths[i] + manip_time, ns)
        # print(np.array(res))
    return res[:-1]


def generate_plan(cart_len, manip_len, start_manip_time, ns):
    p = [0] * len(cart_len)
    one_maip_len = list()
    for i, v in enumerate(manip_len):
        one_maip_len.append(list(calc_one_manip_time(cart_len, ns, v, start_manip_time[i])))
    l = (1 << len(cart_len))

    dp = [[0]*l for _ in manip_len]
    prev = [[-1]*l for _ in manip_len]

    for i, v in enumerate(manip_len):
        for mask in range(l):
            dp[i][mask] = one_maip_len[i][mask]
            prev[i][mask] = mask
            if i != 0:
                submask = mask
                while submask > 0:
                    submask = (submask - 1) & mask
                    nv1 = max(one_maip_len[i][submask], dp[i-1][mask ^ submask])
                    # print(i, submask, mask, nv1, one_maip_len[i][submask], dp[i-1][mask ^ submask], mask ^ submask)
                    if nv1 < dp[i][mask]:
                        dp[i][mask] = nv1
                        prev[i][mask] = submask

                    # nv2 = max(one_maip_len[i][mask ^ submask], dp[i-1][submask])
                    # # print(i, submask, mask, nv1, nv2)
                    # if nv2 < dp[i][mask]:
                    #     dp[i][mask] = nv2
                    #     prev[i][mask] = mask ^ submask

    # for line in one_maip_len:
    #     print(line)
    # for line in dp:
    #     print(line)
    # for line in prev:
    #     print(line)

    masks = [0] * len(manip_len)
    curr_mask = l-1
    for i, _ in enumerate(manip_len):
        j = len(manip_len) - 1 - i
        masks[j] = prev[j][curr_mask]
        # print(curr_mask, prev[j][curr_mask], dp[j][curr_mask], one_maip_len[j][prev[j][curr_mask]], j)
        curr_mask = curr_mask ^ masks[j]
        for q, _ in enumerate(cart_len):
            # print(masks[j], (1 << q), bool(masks[j] & (1 << q)))
            if masks[j] & (1 << q):
                p[q] = j
    # print(masks)

    return p


def generate_solutions(scenario: ExampleScenario):
    ns = scenario.cart_robs[0].state.composite.next_choose.next[0].next_choose
    cart_len = scenario.cart_lengths
    manip_len = scenario.manip_lengths

    return generate_plan(cart_len, manip_len, [0]*len(manip_len), ns)
