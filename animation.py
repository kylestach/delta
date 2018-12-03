import numpy as np
import matplotlib.pylab as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import math

def get_points(filename):
    with open(filename, "r") as points_file:
        return np.array(
            [[float(p) for p in line.split()] for line in points_file]
        ).transpose()

def main(filename):
    fig = plt.figure()
    ax = plt.axes(projection = '3d')

    points = get_points(filename)

    upper_arms = [plt.plot([], [], [], 'r', linewidth=2)[0] for _ in range(3)]
    lower_arms = [plt.plot([], [], [], 'b', linewidth=2)[0] for _ in range(3)]
    manipulator, = ax.plot(points[0, :1], points[1, :1], points[2, :1], 'og')

    ax.set_xlim(-4, 4)
    ax.set_xlabel('z')
    ax.set_ylim(-4, 4)
    ax.set_ylabel('z')
    ax.set_zlim(-8, 0)
    ax.set_zlabel('z')
    ax.set_aspect('equal')

    base_radius = 2.0
    upper_arm_length = 2.854
    manipulator_radius = 1.0

    def update(t):
        mx = points[0][t]
        my = points[1][t]
        mz = points[2][t]

        angles = [points[3][t], points[4][t], points[5][t]]

        manipulator.set_data([mx], [my])
        manipulator.set_3d_properties([mz])

        for i in range(3):
            theta = 2 * math.pi / 3.0 * i
            direction = np.array([math.cos(theta), math.sin(theta)])
            base_joint = base_radius * direction
            mid_joint = base_joint + (
                upper_arm_length * math.cos(angles[i]) * direction)
            upper_arms[i].set_data(
                [base_joint[0], mid_joint[0]],
                [base_joint[1], mid_joint[1]]
            )
            upper_arms[i].set_3d_properties(
                [0.0, -upper_arm_length * math.sin(angles[i])]
            )
            lower_arms[i].set_data(
                [mid_joint[0], mx + math.cos(theta)],
                [mid_joint[1], my + math.sin(theta)]
            )
            lower_arms[i].set_3d_properties(
                [-upper_arm_length * math.sin(angles[i]), mz]
            )

    ts = range(0, points.shape[1])
    anim = animation.FuncAnimation(fig, update, frames = ts, interval = 30)

    plt.show()

if __name__ == '__main__':
    main('test.csv')
