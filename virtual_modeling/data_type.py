from matplotlib.patches import Rectangle
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from task_planner.close_solutions.check_plan import check_half_plan_with_time


def to_homogeneous(pos, rot):
    X = np.concatenate((rot, np.array([pos]).T), axis=1)
    return np.concatenate((X, np.array([[0, 0, 0, 1]])), axis=0)


def get_pos(hom):
    return np.array(hom[0:3, 3])


def get_orient_mat(hom):
    return hom[0:3, 0:3]


def get_orient_quat(hom):
    m = hom[0:3, 0:3]
    return R.from_matrix(m).as_quat()


def inverse_homogeneous(mat):
    R = mat[:3, :3]
    return to_homogeneous(-np.matmul(R.T, mat[:3, 3]).T, R.T)


def get_angle_x(mat):
    R = mat[:3, :3]
    x_angle = np.matmul(R, np.array([[1], [0], [0]])).reshape(3)
    return np.arctan2(x_angle[1], x_angle[0])


def normalise_angle(a):
    while a < -np.pi:
        a += 2*np.pi
    while a > np.pi:
        a -= 2*np.pi
    return a


def show_plot(*data, name=None, legend=None, labx=None, laby=None):
    if legend is not None and len(legend) != len(data):
        raise ValueError
    fig, ax = plt.subplots()
    colors = ['r-', 'b-', 'g-', 'k-', 'c-', 'm-', 'y-']
    for i, d in enumerate(data):
        l = legend[i] if legend else None
        if isinstance(d, tuple):
            ax.plot(*d, colors[i], label=l)
        else:
            ax.plot(d, colors[i], label=l)

    ax.grid()
    if labx:
        ax.set_xlabel(labx)
    if laby:
        ax.set_ylabel(laby)
    if legend:
        ax.legend()
    if name:
        ax.set_title(name)
    plt.show()


def show_half_plan(es, p, name=None):
    ct, mt = check_half_plan_with_time(es, p)
    fig, ax = plt.subplots(2, 1)
    mxv = 0

    for i, line in mt.items():
        for s, c, e in line:
            mxv = max(mxv, e)
            rect = Rectangle((s, i), c-s, 1, linewidth=0, edgecolor='none', facecolor='yellow')
            ax[0].add_patch(rect)
            rect = Rectangle((c, i), e-c, 1, linewidth=0, edgecolor='none', facecolor='olive')
            ax[0].add_patch(rect)

    for i, line in enumerate(ct):
        t1, t2, s, c, e = line
        mxv = max(mxv, e)
        rect = Rectangle((0, i), t1, 1, linewidth=0, edgecolor='none', facecolor='gray')
        ax[1].add_patch(rect)
        rect = Rectangle((t1, i), t2-t1, 1, linewidth=0, edgecolor='none', facecolor='cyan')
        ax[1].add_patch(rect)
        rect = Rectangle((s, i), c-s, 1, linewidth=0, edgecolor='none', facecolor='yellow')
        ax[1].add_patch(rect)
        rect = Rectangle((c, i), e-c, 1, linewidth=0, edgecolor='none', facecolor='olive')
        ax[1].add_patch(rect)

    ax[0].set_xlim([0, mxv])
    ax[0].set_ylim([0, len(mt.keys())])
    ax[0].grid()
    ax[0].set_ylabel('manipulators')
    ax[1].set_xlim([0, mxv])
    ax[1].set_ylim([0, len(ct)])
    ax[1].grid()
    ax[1].set_ylabel('carts')
    ax[1].set_xlabel('time')
    if name:
        fig.suptitle(name)
    plt.show()


def show_heatmap(data):
    fruits = list(range(1, data.shape[0]+1))
    Countries = list(range(1, data.shape[1]+1))

    fig, ax = plt.subplots()
    im = ax.imshow(data)

    # Setting the labels
    ax.set_xticks(np.arange(len(Countries)))
    ax.set_yticks(np.arange(len(fruits)))
    # labeling respective list entries
    ax.set_xticklabels(Countries)
    ax.set_yticklabels(fruits)

    # Rotate the tick labels and set their alignment.
    # plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
    #          rotation_mode="anchor")

    # Creating text annotations by using for loop
    for i in range(len(fruits)):
        for j in range(len(Countries)):
            text = ax.text(j, i, data[i, j],
                           ha="center", va="center", color="w")

    # ax.set_title("Growth of Fruits in Different Countries (in tons/year)")
    fig.tight_layout()
    plt.show()

