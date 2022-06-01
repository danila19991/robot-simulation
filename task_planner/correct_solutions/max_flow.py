from task_planner.plan import ExecutionPlan
from virtual_modeling.data_saver import DataSingleton


def get_less(md, key, s, t):
    ns = s.next[key].next_choose
    v1 = max(s.next[key].t1[0] + s.next[key].t2[0] + t + ns.next[0].t1[0], md.get(ns.id, 0) + ns.next[0].t1[1])
    v2 = max(s.next[key].t1[0] + s.next[key].t2[0] + t + ns.next[1].t1[0], md.get(ns.id, 0) + ns.next[1].t1[1])
    # print(ns.id, t, v1, v2, '----')
    # print(s.next[key].t1[0] + s.next[key].t2[0] + t + ns.next[0].t1[0], md.get(ns.id, 0) + ns.next[0].t1[1],
    #       s.next[key].t1[0] + s.next[key].t2[0] + t + ns.next[1].t1[0], md.get(ns.id, 0) + ns.next[1].t1[1])
    if v1 < v2:
        return 0, v1
    else:
        return 1, v2


class ManipTime:

    def __init__(self, plan):
        self.manip_time = dict()
        self.plan = plan
        self.sol_variants = dict()
        self.manip_comp = dict()
        self.sol_comp = dict()

    def fix_sol(self, i, k=None):
        if k is None:
            k = list(self.sol_variants[i].keys())[0]

        res = self.sol_variants[i][k]
        self.plan.carts[i].append(res[0])
        self.plan.carts[i].append(res[1])
        self.plan.manips[k].append(res[1])
        self.manip_time[res[3]] = res[2]
        del self.sol_variants[i]
        if i in self.sol_comp:
            del self.sol_comp[i]

        to_fix_sol = list()
        for k1, v1 in self.sol_variants.items():
            if k in v1:
                del v1[k]
            if len(v1) == 1:
                to_fix_sol.append(k1)

        for k1 in to_fix_sol:
            self.fix_sol(k1)

        self.check_comps()

    def fix_comp(self, carts, manips):
        for i, cart in enumerate(carts):
            if cart in self.sol_variants:
                self.fix_sol(cart)

        for manip in manips:
            del self.manip_comp[manip]

    def check_comps(self):
        comp_sz1 = dict()
        for man_k, comp_v in self.manip_comp.items():
            if comp_v not in comp_sz1:
                comp_sz1[comp_v] = list()
            comp_sz1[comp_v].append(man_k)

        comp_sz2 = dict()
        for car_k, comp_v in self.sol_comp.items():
            if comp_v not in comp_sz2:
                comp_sz2[comp_v] = list()
            comp_sz2[comp_v].append(car_k)

        for k1, v1 in comp_sz2.items():
            if len(v1) == len(comp_sz1[k1]):
                self.fix_comp(v1, comp_sz1[k1])

    def add_sol(self, cart_i, cur_s, key1_l, key2_l, val_l, manip_l):
        val_min = min(val_l)
        print(val_l, val_min)

        res_d = dict()
        for j, v in enumerate(val_l):
            if v == val_min:
                ns = cur_s.next[key1_l[j]].next_choose
                res_d[manip_l[j]] = (key1_l[j], key2_l[j], v + ns.next[key2_l[j]].t2[1], ns.id)

        print(key1_l, key2_l, val_l, manip_l)
        self.sol_variants[cart_i] = res_d
        print(res_d)
        print(self.sol_variants)

        if len(self.sol_variants[cart_i].keys()) == 1:
            self.fix_sol(cart_i)
            return

        comps = list()
        for e1, v in res_d.items():
            q = self.manip_comp.get(e1, 0)
            if q != 0 and q not in comps:
                comps.append(q)
        if not comps:
            curr_comps = list(self.manip_comp.values())
            for i, _ in enumerate(self.plan.manips):
                if i+1 not in curr_comps:
                    comps.append(i+1)
                    break
        curr_comp = comps[0]
        if len(comps) > 1:
            for old_comp in comps[1:]:
                for man_k, comp_v in self.manip_comp.items():
                    if comp_v == old_comp:
                        self.manip_comp[man_k] = curr_comp
        for new_man_k in res_d.keys():
            self.manip_comp[new_man_k] = curr_comp
        self.sol_comp[cart_i] = curr_comp

        self.check_comps()


def generate_solutions(scenario):
    carts = scenario.cart_robs
    manips = scenario.manip_robs
    p = ExecutionPlan(len(carts), len(manips))
    manip_time = ManipTime(p)

    for i, cart in enumerate(carts):
        print('--------', manip_time.manip_time)
        s0 = cart.state

        l0 = s0.composite
        s1 = l0.next_choose
        t1 = l0.t1[0] + l0.t2[0]

        c_key2_l = list()
        c_val_l = list()
        c_key_l = list(s1.next.keys())
        c_manip_id_l = list()
        for key in c_key_l:
            key2, val = get_less(manip_time.manip_time, key, s1, t1)
            c_key2_l.append(key2)
            c_val_l.append(val)
            c_manip_id_l.append(s1.next[key].next_choose.next[0].manip_id)

        manip_time.add_sol(i, s1, c_key_l, c_key2_l, c_val_l, c_manip_id_l)
    return p
