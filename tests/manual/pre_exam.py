from virtual_modeling.model_ctx import ModelCtx
from virtual_modeling.model_runner import ModelRunner
from robots.manipuator.simulation import ManipulatorRobot
from task_planner.splines import *
import pybullet as pb
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


def main():
    def get_control(js, je, ts, te, func=None):
        def control(t):
            if t < ts:
                return js
            if t > te:
                return je
            # res = lin_interp(js, je, t-ts, te-ts)
            # res = cubic_interp(js, je, t-ts, te-ts)
            # res = trap_interp(js, je, t-ts, te-ts)
            # res = s_curve_interp(js, je, t-ts, te-ts)
            res = func(js, je, t-ts, te-ts)
            return res

        return control

    def split_traj(func, traj, T):
        def control(rob, t):
            for i, _ in enumerate(traj[:-1]):
                if int(traj[i][1] * T) <= t <= int(traj[i + 1][1] * T):
                    return func(traj[i][0], traj[i + 1][0], t - int(traj[i][1] * T),
                                int(traj[i + 1][1] * T) - int(traj[i][1] * T))

        return control

    ctx = ModelCtx()
    rob = ManipulatorRobot(ctx)
    ctx.with_gui = True
    ctx.gravity = np.array([0, 0, 0])
    runner = ModelRunner(ctx)
    runner.robots.append(rob)
    runner.initialise()

    orient = np.array([0, 1, 0, 0])
    orient_m = R.from_quat(orient).as_matrix()
    ws = np.array([-0.5, -0.6, 0.6])
    w1 = np.array([-0.1, -0.5, 0.5])
    w2 = np.array([0.5, 0.3, 0.6])
    w3 = np.array([-0.5, 0.3, 0.9])
    we = np.array([-0.5, -0.8, 0.9])
    wc1 = np.array([-0.5, -0.8, 0.3])
    wc2 = np.array([-0.1, -0.5, 0.3])
    wt1 = np.array([-0.5, -0.8, 0.6])
    wt2 = np.array([-0.1, -0.5, 0.6])
    wl = np.array([0.5, -0.6, 0.6])
    js = np.array([0, 0, np.pi/2, 0, np.pi/2, 0])
    js = rob.get_world_from_joint(to_homogeneous(ws, orient_m))
    je = rob.get_world_from_joint(to_homogeneous(wc2, orient_m))
    traj_j = [
        (rob.get_world_from_joint(to_homogeneous(ws, orient_m)), 0),
        (rob.get_world_from_joint(to_homogeneous(wc1, orient_m)), 0.2),
        (rob.get_world_from_joint(to_homogeneous(wt1, orient_m)), 0.4),
        (rob.get_world_from_joint(to_homogeneous(wt2, orient_m)), 0.6),
        (rob.get_world_from_joint(to_homogeneous(wc2, orient_m)), 0.8),
        (rob.get_world_from_joint(to_homogeneous(we, orient_m)), 1),
    ]
    traj_w = [
        (to_homogeneous(ws, orient_m), 0),
        (to_homogeneous(wc1, orient_m), 0.2),
        (to_homogeneous(wt1, orient_m), 0.4),
        (to_homogeneous(wt2, orient_m), 0.6),
        (to_homogeneous(wc2, orient_m), 0.8),
        (to_homogeneous(we, orient_m), 1)
    ]
    traj_w2 = [
        (to_homogeneous(ws, orient_m), 0),
        (to_homogeneous(wl, orient_m), 1)
    ]
    rob.start_joints = js
    rob.curr_world = to_homogeneous(ws, orient_m)
    t_skip = 0
    t_steps = 500
    t_in_step = 1
    t_array = np.array(list(range(t_steps*t_in_step)))*ctx.dt
    t_array = t_array[t_skip:]

    # rob.control_joint_func = get_control(js, je, 0, t_steps*t_in_step)
    # rob.control_joint_func = split_traj(lin_interp, traj_j, t_steps*t_in_step)
    # rob.control_joint_func = split_traj(cubic_interp, traj_j, t_steps*t_in_step)
    # rob.control_joint_func = split_traj(trap_interp, traj_j, t_steps*t_in_step)
    # rob.control_joint_func = split_traj(s_curve_interp, traj_j, t_steps*t_in_step)
    # rob.control_joint_func = split_traj(p2p_lin_screw, traj_w, t_steps*t_in_step)
    # rob.control_joint_func = split_traj(p2p_lin_decoupled, traj_w, t_steps*t_in_step)
    # rob.control_joint_func = split_traj(p2p_s_curve_decoupled, traj_w2, t_steps*t_in_step)
    rob.control_world_func = get_control(to_homogeneous(ws, orient_m), to_homogeneous(wl, orient_m), 0, t_steps*t_in_step, p2p_s_curve_decoupled)

    pb.loadURDF("models/preexam.urdf")
    # pb.loadURDF("models/preexam.urdf")

    runner.run(400, t_steps, t_in_step)
    runner.deinitialise()

    dataw0 = np.array(rob.position_story_actual)

    dataw = dataw0[:, 0, 3]
    fig, ax = plt.subplots(3, 3)
    ax[0, 0].plot(dataw)
    ax[0, 0].grid()
    ax[0, 0].set(title="pos")
    dataw = (dataw[:-1] - dataw[1:])/ctx.dt
    ax[1, 0].plot(dataw)
    ax[1, 0].grid()
    ax[1, 0].set(title="vel")
    dataw = (dataw[:-1] - dataw[1:])/ctx.dt
    ax[2, 0].plot(dataw)
    ax[2, 0].grid()
    ax[2, 0].set(title="vel")

    dataw = dataw0[:, 1, 3]
    print(dataw[:50])
    ax[0, 1].plot(dataw)
    ax[0, 1].grid()
    ax[0, 1].set(title="pos")
    dataw = (dataw[:-1] - dataw[1:])/ctx.dt
    print(dataw[:50])
    ax[1, 1].plot(dataw)
    ax[1, 1].grid()
    ax[1, 1].set(title="vel")
    dataw = (dataw[:-1] - dataw[1:])/ctx.dt
    print(dataw[:50])
    ax[2, 1].plot(dataw)
    ax[2, 1].grid()
    ax[2, 1].set(title="vel")

    dataw = dataw0[:, 2, 3]
    ax[0, 2].plot(dataw)
    ax[0, 2].grid()
    ax[0, 2].set(title="pos")
    dataw = (dataw[:-1] - dataw[1:])/ctx.dt
    ax[1, 2].plot(dataw)
    ax[1, 2].grid()
    ax[1, 2].set(title="vel")
    dataw = (dataw[:-1] - dataw[1:])/ctx.dt
    ax[2, 2].plot(dataw)
    ax[2, 2].grid()
    ax[2, 2].set(title="vel")
    plt.show()


    data = np.array(rob.position_story_joint)
    pos = data[t_skip:, :, 0]
    vel = data[t_skip:, :, 1]
    # acc = np.concatenate((np.array([[0,0,0,0,0,0]]), (vel[:-1,:] - vel[1:,:])/ctx.dt), axis=0)
    acc = (vel[:-1,:] - vel[1:,:])/ctx.dt

    def show_subplots(t, data, name):
        fig, ax = plt.subplots(3, 2,
                               sharex='col',
                               sharey='row',
                               num=name)
        for row in range(3):
            for col in range(2):
                num = row + 3 * col
                ax[row, col].plot(t, data[:, num])
                ax[row, col].grid()
                ax[row, col].set(title="Joint" + str(num + 1))
        plt.show()
    show_subplots(t_array, pos, 'joints')
    # show_subplots(t_array, vel, 'velocity')
    # show_subplots(t_array[1:], acc, 'acceleration')


if __name__ == '__main__':
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)
    main()
