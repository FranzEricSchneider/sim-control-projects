import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate as integrate


# Acceleration due to gravity (m/s^2)
_G = 9.8


class Kettle(object):
    """
    A simulated brewing kettle.

    Args:
        diameter (float): Kettle diameter in centimeters.
        volume (float): Content volume in liters.
        initial_temp (float): Initial content temperature in degree celsius.
        density (float): Content density.
    """
    # specific heat capacity of water: c = 4.182 kJ / kg * K
    SPECIFIC_HEAT_CAP_WATER = 4.182

    # thermal conductivity of steel: lambda = 15 W / m * K
    THERMAL_CONDUCTIVITY_STEEL = 15

    def __init__(self, diameter, volume, initial_temp, density=1):
        self._mass = volume * density
        self._temp = initial_temp
        radius = diameter / 2

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

    def update(self, power, duration, ambient_temp, efficiency=0.98,
               heat_loss_factor=1):
        '''
        Update the internal state of the kettle based on the controller output
        (power), the simulation constants (duration), and some external factors
        (ambient_temp)

        Args:
            See arguments of heat and cool
        '''
        self.heat(power, duration)
        self.cool(duration, ambient_temp, heat_loss_factor)

    def heat(self, power, duration, efficiency=0.98):
        """Heat the kettle's content.

        Args:
            power (float): The power in kW.
            duration (float): The duration in seconds.
            efficiency (float): The efficiency as number between 0 and 1.
        """
        self._temp += self._get_deltaT(power * efficiency, duration)
        return self._temp

    def cool(self, duration, ambient_temp, heat_loss_factor=1):
        """Make the content loose heat.

        Args:
            duration (float): The duration in seconds.
            ambient_temp (float): The ambient temperature in degree celsius.
            heat_loss_factor (float): Increase or decrease the heat loss by a
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

    def __init__(self, length, mass, x0, x_dot0, theta0, theta_dot0):
        self._length = length
        self._mass = mass
        self._x = x0
        self._x_dot = x_dot0
        self._theta = theta0
        self._theta_dot = theta_dot0
        self._state_history = np.array([[x0, x_dot0, theta0, theta_dot0]])

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

        Args:
            acceleration: Applied cart acceleration (m/s^2)
            duration: Amount of time to step forward with odeint (s)
        '''
        t_span = np.linspace(0, duration, 10)
        state_vector = [self._x, self._x_dot, self._theta, self._theta_dot]
        # updated_state_vector contains [x, x_dot, theta, theta_dot] over the linspace timespan
        updated_state_vector = integrate.odeint(
            func=self.pendulum_ode,
            y0=state_vector,
            t=t_span,
            args=(acceleration, )
        )
        # Save the state history as fine-grain as possible
        self._save_state(updated_state_vector)
        # Set the current state
        self._x, self._x_dot, self._theta, self._theta_dot = updated_state_vector[-1]

    def plot_state_history(self):
        history = self._state_history.copy()
        for i in range(history.shape[1]):
            plt.plot(history[:, i], label='{}'.format(i))
        plt.legend()
        plt.show()

    def plot_energy(self):
        history = self._state_history.copy()
        # Potential energy = m * g * cos(theta) * L
        PE = self._mass * _G * self._length * np.cos(history[:, 2])
        # Kinetic energy = (1/2) * m * (L * theta_dot)^2
        KE = (self._mass * np.power(self._length * history[:, 3], 2)) / 2
        total = PE + KE
        plt.plot(PE, linewidth=2, label='PE')
        plt.plot(KE, linewidth=2, label='KE')
        plt.plot(total, linewidth=2, label='total')
        plt.legend()
        plt.show()

    def _save_state(self, updated_state_vector):
        self._state_history = np.vstack((self._state_history,
                                         updated_state_vector))

    def pendulum_ode(self, state_vector, t, command_x_ddot):
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
