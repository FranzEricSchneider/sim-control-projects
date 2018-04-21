import argparse
from collections import deque, namedtuple
import logging
import math
import matplotlib.pyplot as plt
import sys

from controller import PIDArduino
from plant import InvertedPendulum, Kettle

LOG_FORMAT = '%(name)s: %(message)s'
Simulation = namedtuple(
    'Simulation',
    ['name', 'controller', 'plant', 'delayed_states', 'timestamps',
     'plant_states', 'sensor_states', 'outputs'])


# def simulate_system(args):
#     timestamp = 0  # seconds
#     delayed_samples_len = max(1, round(args.delay / args.sampletime))

#     # Create a simulation for the tuple pid(kp, ki, kd)
#     sim = Simulation(
#         name='Cart PID',
#         controller=PIDArduino(
#             sampletime=args.sampletime,
#             kp=float(args.pid[0]),
#             ki=float(args.pid[1]),
#             kd=float(args.pid[2]),
#             out_min=args.out_min,
#             out_max=args.out_max,
#             time=lambda: timestamp),
#         plant=Kettle(diameter=args.diameter,
#                      volume=args.volume,
#                      initial_temp=args.kettle_temp),
#         delayed_states=deque(maxlen=delayed_samples_len),
#         timestamps=[],
#         plant_states=[],
#         sensor_states=[],
#         outputs=[],
#     )

#     # Init delayed_states deque for each simulation
#     sim.delayed_states.extend(sim.delayed_states.maxlen * [args.kettle_temp])

#     # Run simulation for specified interval. The (x60) is because args.interval
#     # is in minutes and we want seconds
#     while timestamp < (args.interval * 60):
#         timestamp += args.sampletime

#         # Calculates controller reaction
#         output = sim.controller.calc(sim.delayed_states[0], args.setpoint)
#         output = max(output, 0)
#         output = min(output, 100)
#         # Calculates the effects of the controller output on the next sensor
#         # reading
#         simulation_update(sim, timestamp, output, args)

#     title = 'PID simulation, {0:.1f}l kettle, {1:.1f}kW heater, {2:.1f}s delay'.format(
#         args.volume, args.heater_power, args.delay)
#     plot_simulation(sim, title)


# def simulation_update(simulation, timestamp, output, args):
#     simulation.plant.update(args.heater_power * (output / 100),
#                             args.sampletime,
#                             args.ambient_temp,
#                             heat_loss_factor=args.heat_loss_factor)
#     # Add a state reading to the delayed_states queue, which bumps an element
#     # off the front
#     simulation.delayed_states.append(simulation.plant.sensable_state)
#     # Make the simulation read the delayed state value
#     simulation.sensor_states.append(simulation.delayed_states[0])
#     # For the following values just append them to lists of values over time
#     simulation.timestamps.append(timestamp)
#     simulation.outputs.append(output)
#     simulation.plant_states.append(simulation.plant.sensable_state)


def simulate_system(args):
    timestamp = 0  # seconds
    delayed_samples_len = max(1, round(0.03 / 0.001))

    # Create a simulation for the tuple pid(kp, ki, kd)
    sim = Simulation(
        name='Cart PID',
        controller=PIDArduino(
            sampletime=0.001,
            kp=3.0,
            ki=1.0,
            kd=0.0,
            out_min=-15.0,
            out_max=15.0,
            time=lambda: timestamp),
        plant=InvertedPendulum(length=1.0,
                               mass=0.25,
                               x0=0.0,
                               x_dot0=0.0,
                               x_ddot0=0.0,
                               theta0=0.0505,
                               theta_dot0=0.0,
                               theta_ddot0=0.0),
        delayed_states=deque(maxlen=delayed_samples_len),
        timestamps=[],
        plant_states=[],
        sensor_states=[],
        outputs=[],
    )

    # Init delayed_states deque for each simulation
    sim.delayed_states.extend(sim.delayed_states.maxlen * [0.0505])

    # Run simulation for specified interval. The (x60) is because args.interval
    # is in minutes and we want seconds
    while timestamp < (0.25 * 60):
        timestamp += 0.001

        # Calculates controller reaction
        output = sim.controller.calc(input_val=sim.delayed_states[0],
                                     setpoint=3.1415)
        output = max(output, -15.0)
        output = min(output, 15.0)
        output = 0.0
        # Calculates the effects of the controller output on the next sensor
        # reading
        simulation_update(sim, timestamp, output, args)

    title = 'PID simulation, {0:.1f}l kettle, {1:.1f}kW heater, {2:.1f}s delay'.format(
        args.volume, args.heater_power, args.delay)
    plot_simulation(sim, title)
    sim.plant.plot_state_history()
    sim.plant.plot_energy()


def simulation_update(simulation, timestamp, output, args):
    simulation.plant.update(output, duration=0.001)
    # Add a state reading to the delayed_states queue, which bumps an element
    # off the front
    simulation.delayed_states.append(simulation.plant.sensable_state)
    # Make the simulation read the delayed state value
    simulation.sensor_states.append(simulation.delayed_states[0])
    # For the following values just append them to lists of values over time
    simulation.timestamps.append(timestamp)
    simulation.outputs.append(output)
    simulation.plant_states.append(simulation.plant.sensable_state)


# def plot_simulation(simulation, title):
#     lines = []
#     fig, ax1 = plt.subplots()
#     upper_limit = 0

#     # # Try to limit the y-axis to a more relevant area if possible
#     # m = max(simulation.sensor_states) + 1
#     # upper_limit = max(upper_limit, m)

#     # if upper_limit > args.setpoint:
#     #     lower_limit = args.setpoint - (upper_limit - args.setpoint)
#     #     ax1.set_ylim(lower_limit, upper_limit)

#     # Create x-axis and first y-axis
#     ax1.plot()
#     ax1.set_xlabel('time (s)')
#     ax1.set_ylabel('sensed value')
#     ax1.grid(axis='y', linestyle=':', alpha=0.5)

#     # Draw setpoint line
#     lines += [plt.axhline(
#         y=args.setpoint, color='r', linestyle=':', linewidth=0.9, label='setpoint')]

#     # Create second y-axis (power)
#     ax2 = ax1.twinx()
#     ax2.set_ylabel('power (%)')

#     # Plot sensor and output values
#     color = 'b'
#     lines += ax1.plot(
#         simulation.timestamps, simulation.sensor_states, color=color,
#         alpha=1.0, label='{0}: sensor state.'.format(simulation.name))
#     lines += ax2.plot(
#         simulation.timestamps, simulation.outputs, '--', color=color,
#         linewidth=1, alpha=0.7, label='{0}: output'.format(simulation.name))

#     # Create legend
#     labels = [l.get_label() for l in lines]
#     offset = math.ceil(4 / 3) * 0.05
#     ax1.legend(lines, labels, loc=9, bbox_to_anchor=(
#         0.5, -0.1 - offset), ncol=3)
#     fig.subplots_adjust(bottom=0.2 + offset)

#     # Set title
#     plt.title(title)
#     fig.canvas.set_window_title(title)
#     plt.show()

def plot_simulation(simulation, title):
    lines = []
    fig, ax1 = plt.subplots()
    upper_limit = 0

    # # Try to limit the y-axis to a more relevant area if possible
    # m = max(simulation.sensor_states) + 1
    # upper_limit = max(upper_limit, m)

    # if upper_limit > args.setpoint:
    #     lower_limit = args.setpoint - (upper_limit - args.setpoint)
    #     ax1.set_ylim(lower_limit, upper_limit)

    # Create x-axis and first y-axis
    ax1.plot()
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('sensed value')
    ax1.grid(axis='y', linestyle=':', alpha=0.5)

    # Draw setpoint line
    lines += [plt.axhline(
        y=0.0, color='r', linestyle=':', linewidth=0.9, label='setpoint')]

    # Create second y-axis (power)
    ax2 = ax1.twinx()
    ax2.set_ylabel('output')

    # Plot sensor and output values
    color = 'b'
    lines += ax1.plot(
        simulation.timestamps, simulation.sensor_states, color=color,
        alpha=1.0, label='{0}: sensor state.'.format(simulation.name))
    lines += ax2.plot(
        simulation.timestamps, simulation.outputs, '--', color=color,
        linewidth=1, alpha=0.7, label='{0}: output'.format(simulation.name))

    # Create legend
    labels = [l.get_label() for l in lines]
    offset = math.ceil(4 / 3) * 0.05
    ax1.legend(lines, labels, loc=9, bbox_to_anchor=(
        0.5, -0.1 - offset), ncol=3)
    fig.subplots_adjust(bottom=0.2 + offset)

    # Set title
    plt.title(title)
    fig.canvas.set_window_title(title)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-p', '--pid', nargs=3, metavar=('kp', 'ki', 'kd'),
        default=None, help='simulate a PID controller')

    parser.add_argument(
        '-v', '--verbose', default=False, action='store_true',
        help='be verbose')

    parser.add_argument(
        '-t', '--temp', dest='kettle_temp', metavar='T', default=40.0,
        type=float, help='initial kettle temperature in C (default: 40)')
    parser.add_argument(
        '-s', '--setpoint', metavar='T', default=45.0, type=float,
        help='target temperature in C (default: 45)')
    parser.add_argument(
        '--ambient', dest='ambient_temp', metavar='T', default=20.0,
        type=float, help='ambient temperature in C (default: 20)')

    parser.add_argument(
        '-i', '--interval', metavar='t', default=20, type=int,
        help='simulated interval in minutes (default: 20)')
    parser.add_argument(
        '-d', '--delay', metavar='t', default=15.0, type=float,
        help='system response delay in seconds (default: 15)')
    parser.add_argument(
        '--sampletime', metavar='t', default=5.0, type=float,
        help='sensor sample time in seconds (default: 5)')

    parser.add_argument(
        '--volume', metavar='V', default=70.0, type=float,
        help='kettle content volume in liters (default: 70)')
    parser.add_argument(
        '--diameter', metavar='d', default=50.0, type=float,
        help='kettle diameter in cm (default: 50)')

    parser.add_argument(
        '--power', dest='heater_power', metavar='P', default=6.0,
        type=float, help='heater power in kW (default: 6)')
    parser.add_argument(
        '--heatloss', dest='heat_loss_factor', default=1.0,
        type=float, help='kettle heat loss factor (default: 1)')

    parser.add_argument(
        '--minout', dest='out_min', default=0.0,
        type=float, help='minimum PID controller output (default: 0)')
    parser.add_argument(
        '--maxout', dest='out_max', default=100.0,
        type=float, help='maximum PID controller output (default: 100)')

    if len(sys.argv) == 1:
        parser.print_help()
    else:
        args = parser.parse_args()

        if args.verbose:
            logging.basicConfig(stream=sys.stderr, format=LOG_FORMAT, level=logging.DEBUG)
        if args.pid is not None:
            simulate_system(args)
