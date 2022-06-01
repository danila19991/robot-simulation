from virtual_modeling.model_ctx import ModelCtx
from virtual_modeling.model_runner import ModelRunner
from robots.manipuator.simulation import ManipulatorRobot
from task_planner.splines import *
from scipy.spatial.transform import Rotation as R
from robots.manipuator.fsm_elements import ManipulatorState, LinMoveWorldGenerator


def main():
    ctx = ModelCtx()
    rob = ManipulatorRobot(ctx)
    ctx.with_gui = True
    ctx.gravity = np.array([0, 0, 0])
    runner = ModelRunner(ctx)
    runner.robots.append(rob)
    runner.initialise()

    orient = np.array([0, 1, 0, 0])
    orient_m = R.from_quat(orient).as_matrix()

    s1 = ManipulatorState(np.array([-0.5, -0.6, 0.6]), orient_m)
    s2 = ManipulatorState(np.array([0.3, -0.6, 0.6]), orient_m)
    s3 = ManipulatorState(np.array([-0.1, -0.7, 0.6]), orient_m)

    s1.default = LinMoveWorldGenerator(300, s1, s2)
    s2.default = LinMoveWorldGenerator(100, s2, s3)
    s3.default = LinMoveWorldGenerator(200, s3, s1)

    rob.set_state(s1)

    t_steps = 5000

    runner.run(400, t_steps)
    runner.deinitialise()


if __name__ == '__main__':
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)
    main()
