from virtual_modeling.camera import Camera
from virtual_modeling.model_ctx import ModelCtx
from virtual_modeling.model_runner import ModelRunner
from robots.two_wheel.simulation import TwoWheelRobot
from virtual_modeling.data_type import get_angle_x, show_plot
from task_planner.splines import *


def main2():
    rob = TwoWheelRobot()
    fov = 126.8
    sz = {
        'width': 2000,
        'height': 2000
    }
    cam = Camera(size=sz, fov=fov)
    ctx = ModelCtx()

    runner = ModelRunner(ctx)
    runner.robots.append(rob)
    runner.cameras.append(cam)

    runner.initialise()
    runner.run(200, 30, 20)
    runner.deinitialise()

    wasp1 = list()
    wasp2 = list()
    was1 = list()
    was2 = list()

    for act, det in zip(rob.position_story_actual, rob.position_story_detected):
        print('++++++++')
        print(act)
        print(det)
        was1.append(get_angle_x(act))
        was2.append(get_angle_x(det))
        wasp1.append((act[0][3], act[1][3], act[2][3]))
        wasp2.append((det[0][3], det[1][3], det[2][3]))

    wasp1 = np.array(wasp1)
    wasp2 = np.array(wasp2)
    was1 = np.array(was1)
    was2 = np.array(was2)
    print(was1)
    print(was2)
    show_plot((wasp1[:,0], wasp1[:,1]), (wasp2[:,0], wasp2[:,1]), name='position', legend=('pybullet', 'Aruco'))
    show_plot(wasp1[:,2], wasp2[:,2], name='high', legend=('pybullet', 'Aruco'))
    show_plot((was1[:,0], was1[:,1]), (was2[:,0], was2[:,1]), name='angle circ', legend=('pybullet', 'Aruco'))
    # show_plot(was1[:,0], was2[:,0], name='anglex', legend=('pybullet', 'Aruco'))
    # show_plot(was1[:,1], was2[:,1], name='angley', legend=('pybullet', 'Aruco'))
    # show_plot(was1[:,2], was2[:,2], name='anglez', legend=('pybullet', 'Aruco'))


if __name__ == '__main__':
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)
    main2()
