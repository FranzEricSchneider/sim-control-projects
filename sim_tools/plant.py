import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate as integrate


# Acceleration due to gravity (m/s^2)
_G = 9.8


class Kettle(object):
    """
    A simulated brewing kettle.

    Args:
        diameter: Kettle diameter in centimeters.
        volume: Content volume in liters.
        initial_temp: Initial content temperature in degree celsius.
        density: Content density.
        heater_power: Kilo-wattage of the heater for scaling the PID output (PID
            output is a percentage)
        ambient_temp: See docstring for _cool
        heat_loss_factor: See docstring for _cool
    """
    # specific heat capacity of water: c = 4.182 kJ / kg * K
    SPECIFIC_HEAT_CAP_WATER = 4.182

    # thermal conductivity of steel: lambda = 15 W / m * K
    THERMAL_CONDUCTIVITY_STEEL = 15

    def __init__(self, initial, constants):

        self._temp = initial.get('kettle_temp', 40.0)

        volume = constants.get('volume', 70.0)
        self._mass = (volume * constants.get('density', 1.0))
        self._heater_power = constants.get('heater_power', 6.0)
        self._ambient_temp = constants.get('ambient_temp', 20.0)
        self._heat_loss_factor = constants.get('heat_loss_factor', 1.0)

        # radius in cm
        radius = constants.get('diameter', 50.0) / 2
        # height in cm
        height = (volume * 1000) / (np.pi * np.power(radius, 2))
        # surface in m^2
        self._surface = (2 * np.pi * np.power(radius, 2) + 2 * np.pi * radius * height) / 10000

    @property
    def sensable_state(self):
        """Get the content's sensable state, which for the kettle is
        temperature
        """
        return self._temp

    def update(self, output, duration):
        '''
        Update the internal state of the kettle based on the controller output
        (power), the simulation constants (duration), and some external factors
        (ambient_temp)

        Args:
            See arguments of heat and cool
        '''
        power = (output / 100) * self._heater_power
        self._heat(power, duration)
        self._cool(duration, self._ambient_temp, self._heat_loss_factor)

    def _heat(self, power, duration, efficiency=0.98):
        """Heat the kettle's content.

        Args:
            power: The power in kW.
            duration: The duration in seconds.
            efficiency: The efficiency as number between 0 and 1.
        """
        self._temp += self._get_deltaT(power * efficiency, duration)
        return self._temp

    def _cool(self, duration, ambient_temp, heat_loss_factor=1):
        """Make the content loose heat.

        Args:
            duration: The duration in seconds.
            ambient_temp: The ambient temperature in degree celsius.
            heat_loss_factor: Increase or decrease the heat loss by a
            specified factor.
        """
        # Q = k_w * A * (T_kettle - T_ambient)
        # P = Q / t
        power = ((Kettle.THERMAL_CONDUCTIVITY_STEEL * self._surface
                 * (self._temp - ambient_temp)) / duration)

        # W to kW
        power /= 1000
        self._temp -= self._get_deltaT(power, duration) * heat_loss_factor
        return self._temp

    def _get_deltaT(self, power, duration):
        # P = Q / t
        # Q = c * m * delta T
        # => delta(T) = (P * t) / (c * m)
        return ((power * duration) / (Kettle.SPECIFIC_HEAT_CAP_WATER * self._mass))


# TODO: Read this:
# http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
class InvertedPendulum(object):
    """
    A simulated inverted pendulum on a cart.

    Args:
        length: Length of pendulum arm from pivot to point mass (m)
        mass: Mass of point mass at end of pendulum (kg)
        x0: Initial cart position (m)
        x_dot0: Initial cart velocity (m/s)
        theta0: Initial arm angle (rad)
        theta_dot0: Initial arm angular velocity (rad/s)
    """

    def __init__(self, initial, constants):
        self._length = constants.get('length', 1.0)
        self._mass = constants.get('mass', 0.25)
        self._x = initial.get('x0', 0.0)
        self._x_dot = initial.get('x_dot0', 0.0)
        self._theta = initial.get('theta0', 0.0)
        self._theta_dot = initial.get('theta_dot0', 0.0)
        self._elapsed_time = 0.0
        self._state_history = np.array([[
            self._elapsed_time,
            self._x,
            self._x_dot,
            self._theta,
            self._theta_dot,
        ]])

    @property
    def sensable_state(self):
        """
        Get the content's sensable state, which for the kettle is
        temperature
        """
        return self._theta

    def update(self, acceleration, duration):
        '''
        Update the internal state of the cart based on the given acceleration
        TODO: Break this down further into more things? Like motor voltage
              or wheel slippage or something?
        TODO: Add a disturbance force? Or noise in the motor output?
        TODO: Prevent acceleration from being applied past certain speeds?

        Args:
            acceleration: Applied cart acceleration (m/s^2)
            duration: Amount of time to step forward with odeint (s)
        '''
        t_span = np.linspace(self._elapsed_time,
                             self._elapsed_time + duration,
                             num=10)
        state_vector = [self._x, self._x_dot, self._theta, self._theta_dot]
        # updated_state_vector contains [x, x_dot, theta, theta_dot] over the linspace timespan
        updated_state_vector = integrate.odeint(
            func=self._pendulum_ode,
            y0=state_vector,
            t=t_span,
            args=(acceleration, )
        )
        # Save the state history as fine-grain as possible
        self._save_state(np.hstack((t_span.reshape(t_span.shape[0], 1),
                                    updated_state_vector)))
        # Set the current state
        self._elapsed_time = t_span[-1]
        self._x, self._x_dot, self._theta, self._theta_dot = updated_state_vector[-1]

    def plot_state_history(self):
        '''
        Plots the available state variables in _state_history
        '''
        history = self._state_history.copy()
        fig, axes = plt.subplots(2)
        axes[0].plot(history[:, 0], history[:, 1], label='cart x')
        axes[0].plot(history[:, 0], history[:, 2], label='cart x_dot')
        axes[1].plot(history[:, 0], history[:, 3], label='theta')
        axes[1].plot(history[:, 0], history[:, 4], label='theta_dot')
        axes[0].legend()
        axes[1].legend()
        axes[1].set_xlabel('Time (s)')
        axes[0].set_ylabel('m, m/s')
        axes[1].set_ylabel('rad, rad/s')
        axes[0].set_title('Trackable state history')
        plt.show()

    def plot_energy(self):
        '''
        Plots the KE, PE, and total energy in the pendulum over all of the
        pendulum states stored in _state_history. This is expected to be
        constant if the cart is not moving, and is a good check of the pendulum
        simulation
        '''
        history = self._state_history.copy()
        # Potential energy = m * g * cos(theta) * L
        PE = self._mass * _G * self._length * np.cos(history[:, 3])
        # Kinetic energy = (1/2) * m * (L * theta_dot)^2
        KE = (self._mass * np.power(self._length * history[:, 4], 2)) / 2
        total = PE + KE
        plt.plot(history[:, 0], PE, linewidth=2, label='PE')
        plt.plot(history[:, 0], KE, linewidth=2, label='KE')
        plt.plot(history[:, 0], total, linewidth=2, label='total')
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Energy (joules?)')
        plt.title('Energy in the pendulum point mass')
        plt.show()

    def animate_system(self):
        '''
        TODO: Take from double_pendulum
        '''
        # TODO: Maybe make this based on dt?
        downsample = 50
        scaling = 0.5

        history = self._state_history.copy()

        t_span = history[::downsample, 0]
        dt = np.average(np.diff(t_span)) * scaling
        print 'dt', dt

        # Create an (n,) and (n, 2) set of arrays
        cart_x = history[::downsample, 1]
        pendulum_xy = np.array([
            history[::downsample, 1] - np.sin(history[::downsample, 3]),
            np.cos(history[::downsample, 3]),
        ]).T

        figure = plt.figure()
        axes = figure.add_subplot(111, autoscale_on=False, xlim=(-10, 10), ylim=(-2, 2))
        axes.set_aspect('equal')
        axes.grid()

        line, = axes.plot([], [], 'o-', lw=2)
        cart_point, = axes.plot([], [], 'o', markersize=20)
        pendulum_point, = axes.plot([], [], 'o', markersize=10)
        time_text = axes.text(0.05, 0.9, '', transform=axes.transAxes)

        def initialize_animation():
            line.set_data([], [])
            cart_point.set_data([], [])
            pendulum_point.set_data([], [])
            time_text.set_text('')
            return line, cart_point, pendulum_point, time_text

        def animate(i):
            line_x_values = [cart_x[i], pendulum_xy[i, 0]]
            line_y_values = [0.0, pendulum_xy[i, 1]]

            line.set_data(line_x_values, line_y_values)
            cart_point.set_data(cart_x[i], 0.0)
            pendulum_point.set_data(pendulum_xy[i, 0], pendulum_xy[i, 1])
            time_text.set_text('time={:.1f}s'.format(t_span[i]))
            return line, cart_point, pendulum_point, time_text

        pendulumAnimation = animation.FuncAnimation(
            figure,
            animate,
            np.arange(1, t_span.shape[0]),
            interval=int(dt * 1e3),
            blit=True,
            init_func=initialize_animation)

        plt.show()

    def _save_state(self, updated_state_vector_w_time):
        self._state_history = np.vstack((self._state_history,
                                         updated_state_vector_w_time))

    def _pendulum_ode(self, state_vector, t, command_x_ddot):
        '''
        Constructs a derivatized vector that can be passed into an integrator.
        Here, it will output a vector [x_dot, x_ddot, theta_dot, theta_ddot]

        Args:
            state_vector: contains the non-derivative values of the equation.
                In this case it contains [x, x_dot, theta, theta dot]
            t: time along the time span
            command_x_ddot: The commanded x_ddot by the controller
        '''
        x_dot = state_vector[1]
        x_ddot = command_x_ddot
        theta_dot = state_vector[3]
        # Calculate theta_ddot from the current situation
        gravity_component = _G * np.sin(state_vector[2])
        cart_component = x_ddot * np.cos(state_vector[2])
        theta_ddot = (gravity_component + cart_component) / self._length
        # Return a derivatized state_vector
        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])
