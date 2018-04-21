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
        x_ddot0: Initial cart acceleration (m/s^2)
        theta0: Initial arm angle (rad)
        theta_dot0: Initial arm angular velocity (rad/s)
        theta_ddot0: Initial arm angular acceleration (rad/s^2)
    """

    def __init__(self, length, mass, x0, x_dot0, x_ddot0, theta0, theta_dot0,
                 theta_ddot0):
        self._length = length
        self._mass = mass
        self._x = x0
        self._x_dot = x_dot0
        self._x_ddot = x_ddot0
        self._theta = theta0
        self._theta_dot = theta_dot0
        self._theta_ddot = theta_ddot0

        self._state_history = []

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
        TODO: Should update be based on some sort of ODE thing instead of
              a linear approximation?

        Args:
            Applied cart acceleration (m/s^2)
        '''
        self._save_state()
        self._update_x(acceleration, duration)
        self._update_theta(acceleration, duration)

    def plot_state_history(self):
        history = np.array(self._state_history)
        for i in range(history.shape[1]):
            plt.plot(history[:, i], label='{}'.format(i))
        plt.legend()
        plt.show()

    def plot_energy(self):
        history = np.array(self._state_history)
        # Potential energy = m * g * cos(theta) * L
        PE = self._mass * _G * self._length * np.cos(history[:, 3])
        KE = (self._mass * np.power(self._length * history[:, 4], 2)) / 2
        total = PE + KE
        plt.plot(PE, linewidth=2, label='PE')
        plt.plot(KE, linewidth=2, label='KE')
        plt.plot(total, linewidth=2, label='total')
        plt.legend()
        plt.show()

    def _save_state(self):
        self._state_history.append([
            self._x, self._x_dot, self._x_ddot,
            self._theta, self._theta_dot, self._theta_ddot
        ])

    def _update_x(self, acceleration, duration):
        '''
        TODO: comment when tested
        '''
        self._x_ddot = acceleration

        delta_velocity = self._x_ddot * duration
        self._x += ((self._x_dot * duration) +\
                    ((self._x_dot + delta_velocity) * duration)) / 2
        self._x_dot += delta_velocity

    def _update_theta(self, acceleration, duration):
        '''
        TODO: comment when tested

        L * theta_ddot = g * sin(theta) + x_ddot * cos(theta)
        '''
        # import ipdb; ipdb.set_trace()
        gravity_component = _G * np.sin(self._theta)
        cart_component = acceleration * np.cos(self._theta)
        self._theta_ddot = (gravity_component + cart_component) / self._length

        delta_velocity = self._theta_ddot * duration 
        self._theta += ((self._theta_dot * duration) +\
                        ((self._theta_dot + delta_velocity) * duration)) / 2
        self._theta_dot += delta_velocity
