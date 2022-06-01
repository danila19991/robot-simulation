import numpy as np
import scipy.optimize as opt

from task_planner.scenarios import ExampleScenario
from task_planner.close_solutions.check_plan import check_half_plan
from task_planner.close_solutions.greed_planner import generate_greed_solutions
from task_planner.close_solutions.prob_planer import generate_prob_solutions
from task_planner.close_solutions.prob_lin_planner import generate_prob_lin_solutions
from geneticalgorithm import geneticalgorithm as ga


def generate_stat_data_greed(scenario: ExampleScenario, n: int, k=None):
    if k is not None:
        res = list()
        for _ in range(n):
            res.append(max(check_half_plan(scenario, generate_greed_solutions(scenario, k))[1]))
        return np.array(res)
    res = dict()
    for k in range(1, len(scenario.manip_lengths)+1):
        loc_line = list()
        for _ in range(n):
            loc_line.append(max(check_half_plan(scenario, generate_greed_solutions(scenario, k))[1]))
        res[k] = np.array(loc_line)
    return res


def generate_stat_data_prob(scenario: ExampleScenario, n: int, p, a, k=None):
    if k is not None:
        res = list()
        for _ in range(n):
            res.append(max(check_half_plan(scenario, generate_prob_solutions(scenario, k, p, a))[1]))
        return np.array(res)
    res = dict()
    for k in range(1, len(scenario.manip_lengths)+1):
        loc_line = list()
        for _ in range(n):
            loc_line.append(max(check_half_plan(scenario, generate_prob_solutions(scenario, k, p, a))[1]))
        res[k] = np.array(loc_line)
    return res


def teach_prob(scenario, n, k):

    def func(inp):
        # print(inp)
        p = inp[:-1]
        a = inp[-1]
        data = generate_stat_data_prob(scenario, n, p, a, k)
        return np.mean(data)

    start = np.array([0.8739, 0.5813, 0.9474, 0.8368, 4.4158])
    params = {
        'max_num_iteration': 500, # None
        'population_size': 100,
        'mutation_probability': 0.1,
        'elit_ratio': 0.01,
        'crossover_probability': 0.5,
        'parents_portion': 0.3,
        'crossover_type': 'uniform',
        'max_iteration_without_improv': 10 # None
    }

    bounds = np.array(((0., 1.), (0., 1.), (0., 1.), (0., 1.), (0., 7.)))
    model = ga(function=func, dimension=5, variable_type='real', variable_boundaries=bounds, algorithm_parameters=params)
    print(model.param)
    model.run()
    solution = model.output_dict
    print(solution)
    # res = opt.minimize(func, start, method='nelder-mead', options={'xtol': 1e-1, 'maxiter': 300}, bounds=bounds)
    res = solution['variable']
    print(res)
    return res[:-1], res[-1]


def generate_stat_data_prob_lin(scenario: ExampleScenario, n: int, a, k=None):
    if k is not None:
        res = list()
        for _ in range(n):
            res.append(max(check_half_plan(scenario, generate_prob_lin_solutions(scenario, k, a))[1]))
        return np.array(res)
    res = dict()
    for k in range(1, len(scenario.manip_lengths)+1):
        loc_line = list()
        for _ in range(n):
            loc_line.append(max(check_half_plan(scenario, generate_prob_lin_solutions(scenario, k, a))[1]))
        res[k] = np.array(loc_line)
    return res


def teach_prob_lin(scenario, n, k):

    def func(inp):
        # print(inp)
        inp = inp.reshape((4, 4))
        data = generate_stat_data_prob_lin(scenario, n, inp, k)
        return np.mean(data)

    start = np.array([
        [1.1974, 0.1930, 0.0253, 0.4445],
        [0.2309, 1.0431, 0.0218, 0.5087],
        [0.1501, 0.2415, 1.2717, 0.6027],
        [0.2334, 0.3655, 0.3352, 0.8921]
    ])
    params = {
        'max_num_iteration': 500, # None
        'population_size': 100,
        'mutation_probability': 0.1,
        'elit_ratio': 0.01,
        'crossover_probability': 0.5,
        'parents_portion': 0.3,
        'crossover_type': 'uniform',
        'max_iteration_without_improv': 10 # None
    }

    bond = (0., 1.8)
    bounds = np.array((bond, bond, bond, bond,
                       bond, bond, bond, bond,
                       bond, bond, bond, bond,
                       bond, bond, bond, bond,))
    model = ga(function=func, dimension=16, variable_type='real', variable_boundaries=bounds, algorithm_parameters=params)
    print(model.param)
    model.run()
    solution = model.output_dict
    print(solution)
    # res = opt.minimize(func, start, method='nelder-mead', options={'xtol': 1e-1, 'maxiter': 300}, bounds=bounds)
    res = solution['variable']
    print(res)
    return res.reshape(4, 4)

