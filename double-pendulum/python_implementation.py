import matplotlib.animation as animation
import matplotlib.pyplot as plt
from numpy import sqrt, cos, sin
import numpy as np
import scipy.integrate as integrate

'''
See Dynamics homework 4, problem 1 for equations
'''

# Set up constants
L1 = 1                      # Length (m)
L2 = 1                      # Length (m)
M1 = 2                      # Mass (Kg)
M2 = 1                      # Mass (Kg)
G = 9.8                     # Gravity acceleration (m/s**2)

def main():

    # Set up initial conditions
    theta1_0 = np.pi / 5        # Initial angle of arm 1 (rad)
    theta1_dot_0 = 0            # Initial angular velocity of arm 1 (rad/s)
    theta2_0 = 5 * np.pi / 8    # Initial angle of arm 2 (rad)
    theta2_dot_0 = 0            # Initial angular velocity of arm 2 (rad/s)

    initial = np.array([theta1_0, theta1_dot_0, theta2_0, theta2_dot_0])
    t_span = np.linspace(0, 10, 10 * 30)
    zout = integrate.odeint(double_pendulum_ode, initial, t_span)

    # Process for XYZ
    xy = []  # (x, y) position (m), of shape (N, 4) where it goes
             # [x for mass 1, y for mass 1, x for mass 2, y for mass 2]
    v  = []  # Speed w/o direction (m/2), of shape (N, 2) where it goes
             # [speed for mass 1, speed for mass 2]
    for i in range(len(t_span)):
        x1 = -sin(zout[i, 0]) * L1
        y1 = -cos(zout[i, 0]) * L1
        xy.append([x1,
                   y1,
                   x1 - sin(zout[i, 2]) * L2,
                   y1 - cos(zout[i, 2]) * L2])
        v1 = -L1 * zout[0, 1]
        v.append([v1,
                  sqrt((cos(zout[i, 2] - zout[i, 0]) * v1 - L2 * zout[i, 3])**2 +\
                  (-sin(zout[i, 2] - zout[i, 0]) * v1)**2)
                  ])

    xy = np.array(xy)
    v = np.array(v)

    animate_pendulum(t_span, xy)


def double_pendulum_ode(Z, t):
    ode_theta1_dot = Z[1]
    ode_theta2_dot = Z[3]
    out_vec = calculate_LHS_vec(Z)
    zout = np.array([ode_theta1_dot, out_vec[0], ode_theta2_dot, out_vec[1]])
    return zout


def calculate_LHS_vec(Z):
    # Unpack variables
    ode_theta1     = Z[0]
    ode_theta1_dot = Z[1]
    ode_theta2     = Z[2]
    ode_theta2_dot = Z[3]

    # Make the M-Matrix
    M = np.array([[1, 0, 0, sin(ode_theta1 - ode_theta2) / (M1 * L1)],
                  [0, 0, -1, cos(ode_theta1 - ode_theta2)],
                  [cos(ode_theta2 - ode_theta1), L2 / L1, 0, 0],
                  [sin(ode_theta2 - ode_theta1), 0, 0, 1 / (M2 * L1)]])

    # These 'elements' are those in the given RHS vector from the PSet
    el_1 = -(G / L1) * sin(ode_theta1)
    el_2 = -(cos(ode_theta1) * M1 * G) - (M1 * L1 * ode_theta1_dot**2)
    el_3 = -((G / L1) * sin(ode_theta2)) - (ode_theta1_dot**2 * sin(ode_theta2 - ode_theta1))
    el_4 =  ((G / L1) * cos(ode_theta2)) + ((L2 / L1) * ode_theta2_dot**2) + (ode_theta1_dot**2 * cos(ode_theta2 - ode_theta1))
    RHS_vec = np.array([[el_1, el_2, el_3, el_4]]).T

    # Solve for the accelerations
    lhs_vec = np.linalg.inv(M).dot(RHS_vec)
    return lhs_vec.squeeze()


def animate_pendulum(t_span, xy, scale=1.0):
    '''
    Takes the points in time ((N,) array) and the (x, y) points of each mass
    ((N, 4) array) and creates a plot that shows the pendulum motion.

    scale is a float that will scale the playback speed. At 2 the playback will
    happen at twice the actual speed

    Used this example :)
    matplotlib.org/gallery/animation/double_pendulum_animated_sgskip.html
    '''
    assert 0.01 < scale < 100
    dt = np.average(np.diff(t_span)) / scale

    figure = plt.figure()
    axes = figure.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
    axes.set_aspect('equal')
    axes.grid()

    line, = axes.plot([], [], 'o-', lw=2)
    mass1_point, = axes.plot([], [], 'o', markersize=M1 * 10)
    mass2_point, = axes.plot([], [], 'o', markersize=M2 * 10)
    time_text = axes.text(0.05, 0.9, '', transform=axes.transAxes)

    def initialize_animation():
        line.set_data([], [])
        mass1_point.set_data([], [])
        mass2_point.set_data([], [])
        time_text.set_text('')
        return line, mass1_point, mass2_point, time_text

    def animate(i):
        mass1 = [0, xy[i, 0], xy[i, 2]]
        mass2 = [0, xy[i, 1], xy[i, 3]]

        line.set_data(mass1, mass2)
        mass1_point.set_data(xy[i, 0], xy[i, 1])
        mass2_point.set_data(xy[i, 2], xy[i, 3])
        time_text.set_text('time={:.1f}s'.format(i * dt))
        return line, mass1_point, mass2_point, time_text

    pendulumAnimation = animation.FuncAnimation(
        figure,
        animate,
        np.arange(1, xy.shape[0]),
        interval=int(dt * 1e3),
        blit=True,
        init_func=initialize_animation)

    plt.show()


if __name__ == '__main__':
    main()
