import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import seaborn as sns

from virtual_modeling.model_ctx import ModelCtx
from virtual_modeling.model_runner import ModelRunner
from robots.two_wheel.simulation import TwoWheelRobot
from virtual_modeling.data_type import show_plot, show_half_plan, show_heatmap
from task_planner.splines import *
from robots.two_wheel.fsm_states import TwoWheelState
from robots.two_wheel.fsm_actions import CircleMoveGenerator, LinMoveGenerator
from virtual_modeling.data_saver import DataSingleton
from task_planner.fsm_state import *
from task_planner.composite_task import *
from task_planner.scenarios import ExampleScenario
from task_planner.correct_solutions.max_flow import generate_solutions as flow_sol
from task_planner.correct_solutions.dynamic import generate_solutions as dp_sol
from task_planner.close_solutions.check_plan import check_plan, check_half_plan
from task_planner.close_solutions.greed_planner import generate_greed_solutions
from task_planner.close_solutions.loc_opt import generate_loc_opt_solutions
from task_planner.close_solutions.prob_planer import generate_prob_solutions
from task_planner.close_solutions.prob_lin_planner import generate_prob_lin_solutions
from tests.utils.stat_data import generate_stat_data_greed, generate_stat_data_prob, teach_prob, \
    generate_stat_data_prob_lin, teach_prob_lin


def main():
    ctx = ModelCtx()

    dist_vars = [
        (
            [500, 900, 1000], # manip
            [1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000], # cart
            "smoke" # name
        ),
        (
            [500, 800], # manip
            [500, 1200, 1500, 2200, 2500], # cart
            "sync" # name
        ),
        (
            [500, 900, 1000],  # manip
            [1000, 1500, 2000],  # cart
            "small one"  # name
        ),
        (
            [500, 900, 1100, 1400],  # manip
            [1000, 1200, 1400, 1600, 1800, 2000,
             2200, 2400, 2600, 2800, 3000, 3200,
             3400, 3800, 4000, 4200],  # cart
            "big one"  # name
        ),
        (
            [500, 900, 1100, 1400, 1700, 2000, 2300, 2600, 2900],  # manip
            [1000, 1200, 1400, 1600, 1800, 2000,
             2200, 2400, 2600, 2800, 3000, 3200,
             3400],  # cart
            "big one more manips"  # name
        ),
    ]
    check_id = 4

    es = ExampleScenario(
        dist_vars[check_id][1],
        dist_vars[check_id][0],
        ctx
    )

    p = [3, 2, 1, 0, 0, 3, 2, 1, 0, 0, 1, 2, 3, 1, 2, 3]
    # p = dp_sol(es)
    # print(p)
    resc, resm = check_half_plan(es, p)
    best = max(resm)
    print(resc, resm, max(resm))
    # show_half_plan(es, p, name='correct')

    # p2 = generate_loc_opt_solutions(es, 4)
    # # print(p2)
    # resc, resm = check_half_plan(es, p2) # 7900
    # print(resc, resm, max(resm))
    # show_half_plan(es, p2, name='local')

    # p2 = generate_greed_solutions(es, 0)
    # resc, resm = check_half_plan(es, p2) # 7900
    # print(resc, resm, max(resm))
    # show_half_plan(es, p2, name='greed')

    # p2 = generate_prob_solutions(es, 4, [0, 0, 0, 0], 4)
    # resc, resm = check_half_plan(es, p2)
    # print(resc, resm, max(resm))
    # show_half_plan(es, p2, name='random')

    # data = generate_stat_data_greed(es, 100)
    # print(np.max(data[2]), np.mean(data[2]), np.min(data[2]))
    # print(np.max(data[3]), np.mean(data[3]), np.min(data[3])) # 8500
    # print(np.max(data[4]), np.mean(data[4]), np.min(data[4])) # 8800

    # data = generate_stat_data_prob(es, 500, [0.8739, 0.5813, 0.9474, 0.8368], 4.4158, 3) # 8450
    # data = generate_stat_data_prob(es, 10000, [0., 0., 0., 0.], 1, 4) # 8500
    # print(np.max(data), np.mean(data), np.min(data))
    # plt.hist(data, bins=15)
    # plt.show()
    # p, a = teach_prob(es, 60, 3)
    # print(p, a)
    # data = generate_stat_data_prob(es, 20, p, a, 4)
    # print(np.max(data), np.mean(data), np.min(data))

    # p2 = generate_prob_solutions(es, 4, [0.25, 0.25, 0.25, 0.25], 1)
    # resc, resm = check_half_plan(es, p2)
    # print(resc, resm)
    # data = generate_stat_data_prob(es, 500, [0.8739, 0.5813, 0.9474, 0.8368], 4.4158, 3) # 8450
    # print(np.max(data), np.mean(data), np.min(data))
    # p, a = teach_prob(es, 60, 3)
    # print(p, a)
    # data = generate_stat_data_prob(es, 20, p, a, 4)
    # print(np.max(data), np.mean(data), np.min(data))

    # a = np.array([
    #     [1.1974, 0.1930, 0.0253, 0.4445],
    #     [0.2309, 1.0431, 0.0218, 0.5087],
    #     [0.1501, 0.2415, 1.2717, 0.6027],
    #     [0.2334, 0.3655, 0.3352, 0.8921]
    # ])
    # p2 = generate_prob_lin_solutions(es, 3, a)
    # resc, resm = check_half_plan(es, p2)
    # print(resc, resm)
    # data = generate_stat_data_prob_lin(es, 500, a, 3) # 9070
    # print(np.max(data), np.mean(data), np.min(data))
    # a = teach_prob_lin(es, 20, 3)
    # print(a)
    # data = generate_stat_data_prob_lin(es, 500, a, 3)
    # print(np.max(data), np.mean(data), np.min(data))

    # data = [list() for _ in range(7)]
    # for i in range(11):
    #     data[0].append(max(check_half_plan(es, generate_greed_solutions(es, i))[1]))
    #     data[1].append(np.mean(generate_stat_data_prob(es, 1000, [0., 0., 0., 0.], 0.5, i)))
    #     data[2].append(np.mean(generate_stat_data_prob(es, 1000, [0., 0., 0., 0.], 1, i)))
    #     data[3].append(np.mean(generate_stat_data_prob(es, 1000, [0., 0., 0., 0.], 2, i)))
    #     data[4].append(np.mean(generate_stat_data_prob(es, 1000, [0., 0., 0., 0.], 3, i)))
    #     data[5].append(np.mean(generate_stat_data_prob(es, 1000, [0., 0., 0., 0.], 4, i)))
    #     data[6].append(np.mean(generate_stat_data_prob(es, 1000, [0., 0., 0., 0.], 5, i)))
    # data = np.array(data)
    # print(data)
    # show_plot(data[0, :], data[1, :], data[2, :], data[3, :], data[4, :], data[5, :], data[6, :],
    #           legend=['greed', 'p=0.5', "p=1", "p=2", "p=3", "p=4", "p=5"])

    # data = list()
    # for j in range(1, 7):
    #     sc = ExampleScenario(
    #         dist_vars[check_id][1],
    #         dist_vars[check_id][0][:j],
    #         ctx
    #     )
    #     line = list()
    #     b = max(check_half_plan(sc, dp_sol(sc))[1])
    #     for i in range(10):
    #         # g = max(check_half_plan(sc, generate_greed_solutions(sc, i))[1])
    #         g = np.mean(generate_stat_data_prob(sc, 500, [0., 0., 0., 0.], 1, i))
    #         line.append(np.round(g/b, 2))
    #     data.append(line)
    # data = np.array(data)
    # print(data)
    # show_heatmap(data)

    t = 6
    data = list()
    data2 = list()
    for j in range(1, 7):
        line = list()
        line2 = list()
        for i in range(1, 12):
            sc = ExampleScenario(
                dist_vars[check_id][1][:i],
                dist_vars[check_id][0][:j],
                ctx
            )
            b = max(check_half_plan(sc, dp_sol(sc))[1])
            g1 = max(check_half_plan(sc, generate_greed_solutions(sc, t))[1])
            g2 = np.mean(generate_stat_data_prob(sc, 500, [0., 0., 0., 0.], 1, t))
            line.append(np.round(g1/b, 2))
            line2.append(np.round(g2/b, 2))
        data.append(line)
        data2.append(line2)
    data = np.array(data)
    show_heatmap(data)
    data2 = np.array(data2)
    show_heatmap(data2)

    print(dist_vars[check_id][2])


if __name__ == '__main__':
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)
    main()
