import numpy as np

from virtual_modeling.model_ctx import ModelCtx
from virtual_modeling.model_runner import ModelRunner
from robots.two_wheel.simulation import TwoWheelRobot
from virtual_modeling.data_type import show_plot
from task_planner.splines import *
from robots.two_wheel.fsm_states import TwoWheelState
from robots.two_wheel.fsm_actions import CircleMoveGenerator, LinMoveGenerator
from virtual_modeling.data_saver import DataSingleton


def main():
    ctx = ModelCtx()
    rob = TwoWheelRobot(ctx)
    ctx.with_gui = True
    runner = ModelRunner(ctx)
    runner.robots.append(rob)
    runner.initialise()

    v = 0.002

    s1 = TwoWheelState(0, 0, 0, 0)
    # s2 = TwoWheelState(1, 1, np.pi/2, 0)
    s2 = TwoWheelState(1, 1, np.pi/2, v)
    s3 = TwoWheelState(1, 2, np.pi/2, v)
    s4 = TwoWheelState(0, 3, np.pi, 0)

    s5 = TwoWheelState(1, 0.1, 0, 0)

    rob.state = s1

    s1.default = CircleMoveGenerator(np.array([0, 1]), s1, s2, v)
    # s2.default = LinMoveGenerator(s2, s3, v)
    # s3.default = CircleMoveGenerator(np.array([0, 2]), s3, s4, v)

    # s1.default = LinMoveGenerator(s1, s5, v)

    t_steps = 1000

    runner.run(400, t_steps)
    runner.deinitialise()

    data2 = DataSingleton.get().get_ds("1")
    data2 = np.array(data2)
    show_plot((data2[:, 0], data2[:, 1]), name='diff')
    data3 = DataSingleton.get().get_ds("2")
    data3 = np.array(data3)
    show_plot(data3[:, 0], name='d_l')
    show_plot(data3[:, 1], name='d_teta')

    data4 = DataSingleton.get().get_ds("3")
    data4 = np.array(data4)
    show_plot((data4[:,0], data4[:,1]), (data4[:,2], data4[:,3]), name='position', legend=('ppos', 'cpos'))


    data = np.array(rob.position_story_joint)

    print(data)
    # show_plot((data[:, 0], data[:, 1]), name='x y')
    # show_plot(data[:, 2], name='teta')
    # show_plot(np.linalg.norm(data[:-1, :2] - data[1:, :2], axis=1), name='vel')


if __name__ == '__main__':
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)
    main()
