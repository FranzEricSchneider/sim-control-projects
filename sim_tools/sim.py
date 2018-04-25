import argparse
from ast import literal_eval
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


def simulate_system(args):
    timestamp = 0  # seconds
    delayed_samples_len = max(1, round(args.delay / args.sampletime))

    initial = literal_eval(args.initial_values)
    assert isinstance(initial, dict)
    initial['kettle_temp'] = initial.get('kettle_temp', 40.0)
    constants = literal_eval(args.constant_values)
    assert isinstance(constants, dict)
    constants['diameter'] = constants.get('diameter', 50.0)
    constants['volume'] = constants.get('volume', 70.0)
    constants['heater_power'] = constants.get('heater_power', 6.0)
    constants['ambient_temp'] = constants.get('ambient_temp', 20.0)
    constants['heat_loss_factor'] = constants.get('heat_loss_factor', 1.0)

    # Create a simulation for the tuple pid(kp, ki, kd)
    sim = Simulation(
        name='Cart PID',
        controller=PIDArduino(
            sampletime=args.sampletime,
            kp=float(args.pid[0]),
            ki=float(args.pid[1]),
            kd=float(args.pid[2]),
            out_min=args.out_min,
            out_max=args.out_max,
            time=lambda: timestamp),
        plant=Kettle(diameter=constants['diameter'],
                     volume=constants['volume'],
                     initial_temp=initial['kettle_temp'],
                     heater_power=constants['heater_power'],
                     ambient_temp=constants['ambient_temp'],
                     heat_loss_factor=constants['heat_loss_factor']),
        delayed_states=deque(maxlen=delayed_samples_len),
        timestamps=[],
        plant_states=[],
        sensor_states=[],
        outputs=[],
    )

    # Init delayed_states deque for each simulation
    sim.delayed_states.extend(sim.delayed_states.maxlen * [initial['kettle_temp']])

    # Run simulation for specified interval. The (x60) is because args.interval
    # is in minutes and we want seconds
    while timestamp < (args.interval * 60):
        timestamp += args.sampletime

        # Calculates controller reaction
        output = sim.controller.calc(sim.delayed_states[0], args.setpoint)
        output = max(output, args.out_min)
        output = min(output, args.out_max)
        # Calculates the effects of the controller output on the next sensor
        # reading
        simulation_update(sim, timestamp, output, args)

    title = 'PID simulation, {0:.1f}l kettle, {1:.1f}kW heater, {2:.1f}s delay'.format(
        constants['volume'], constants['heater_power'], args.delay)
    plot_simulation(sim, title)


def simulation_update(simulation, timestamp, output, args):
    simulation.plant.update(output, duration=args.sampletime)
    # Add a state reading to the delayed_states queue, which bumps an element
    # off the front
    simulation.delayed_states.append(simulation.plant.sensable_state)
    # Make the simulation read the delayed state value
    simulation.sensor_states.append(simulation.delayed_states[0])
    # For the following values just append them to lists of values over time
    simulation.timestamps.append(timestamp)
    simulation.outputs.append(output)
    simulation.plant_states.append(simulation.plant.sensable_state)


def plot_simulation(simulation, title):
    lines = []
    fig, ax1 = plt.subplots()
    upper_limit = 0

    # Create x-axis and first y-axis
    ax1.plot()
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('sensed value')
    ax1.grid(axis='y', linestyle=':', alpha=0.5)

    # Draw setpoint line
    lines += [plt.axhline(
        y=args.setpoint, color='r', linestyle=':', linewidth=0.9, label='setpoint')]

    # Create second y-axis (power)
    ax2 = ax1.twinx()
    ax2.set_ylabel('power (%)')

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


# def simulate_system(args):
#     timestamp = 0.0  # Beginning time (seconds)
#     delayed_samples_len = max(1, round(args.delay / args.sampletime))

#     # TODO: comment
#     initial = literal_eval(args.initial_values)
#     assert isinstance(initial, dict)

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
#         plant=InvertedPendulum(length=1.0,
#                                mass=0.25,
#                                x0=initial.get('x0', 0.0),
#                                x_dot0=initial.get('x_dot0', 0.0),
#                                theta0=initial.get('theta0', 0.0),
#                                theta_dot0=initial.get('theta_dot0', 0.0)),
#         delayed_states=deque(maxlen=delayed_samples_len),
#         timestamps=[],
#         plant_states=[],
#         sensor_states=[],
#         outputs=[],
#     )

#     # Init delayed_states deque for each simulation
#     sim.delayed_states.extend(
#         sim.delayed_states.maxlen * [initial.get('theta0', 0.0)]
#     )

#     # Run simulation for specified interval. The (x60) is because args.interval
#     # is in minutes and we want seconds
#     while timestamp < (args.interval * 60):
#         timestamp += args.sampletime

#         # Calculates controller reaction
#         output = sim.controller.calc(input_val=sim.delayed_states[0],
#                                      setpoint=args.setpoint)
#         output = max(output, args.out_min)
#         output = min(output, args.out_max)
#         # Calculates the effects of the controller output on the next sensor
#         # reading
#         simulation_update(sim, timestamp, output, args)

#     title = 'PID simulation'
#     plot_simulation(sim, title)
#     sim.plant.plot_state_history()
#     sim.plant.plot_energy()


# def plot_simulation(simulation, title):
#     lines = []
#     fig, ax1 = plt.subplots()

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
#     ax2.set_ylabel('output')

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
#     ax1.legend(lines, labels, loc=9, ncol=3,
#                bbox_to_anchor=(0.5, -0.1 - offset))
#     fig.subplots_adjust(bottom=0.2 + offset)

#     # Set title
#     plt.title(title)
#     fig.canvas.set_window_title(title)
#     plt.show()


# Kettle
# python sim_tools/sim.py --pid 104 0.8 205 --out-min -0.0 --out-max 100.0 --sampletime 1 --delay 15.0 --setpoint 45.0 --interval 20 --initial-values "{'kettle_temp': 40.0}" --constant-values "{'ambient_temp': 20.0, 'volume': 70.0, 'diameter': 50.0, 'heater_power': 6.0, 'heat_loss_factor': 1.0}"
# Pendulum:
# python sim_tools/sim.py --pid 15 10 0 --out-min -15.0 --out-max 15.0 --sampletime 0.01 --delay 0.03 --setpoint 3.1415 --interval 0.5 --initial-values "{'theta0': 0.1}"
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-p', '--pid', nargs=3, metavar=('kp', 'ki', 'kd'),
        default=None, help='simulate a PID controller')

    parser.add_argument(
        '-v', '--verbose', default=False, action='store_true',
        help='be verbose')

    parser.add_argument(
        '-s', '--setpoint', metavar='T', default=45.0, type=float,
        help='target sensor value (default: 45)')

    parser.add_argument(
        '-i', '--interval', metavar='t', default=20, type=float,
        help='simulated interval in minutes (default: 20)')
    parser.add_argument(
        '-d', '--delay', metavar='t', default=15.0, type=float,
        help='system response delay in seconds (default: 15)')
    parser.add_argument(
        '--sampletime', metavar='t', default=5.0, type=float,
        help='sensor sample time in seconds (default: 5)')

    parser.add_argument(
        '--out-min', default=0.0,
        type=float, help='minimum PID controller output (default: 0)')
    parser.add_argument(
        '--out-max', default=100.0,
        type=float, help='maximum PID controller output (default: 100)')

    parser.add_argument(
        '--constant-values', default='{}', action='store',
        help='Pass in a dictionary of constants values used throughout the'
        ' simulation as a string, specific to each plant')
    parser.add_argument(
        '--initial-values', default='{}', action='store',
        help='Pass in a dictionary of initial values as a string, specific'
        ' to each plant')

    if len(sys.argv) == 1:
        parser.print_help()
    else:
        args = parser.parse_args()

        if args.verbose:
            logging.basicConfig(stream=sys.stderr, format=LOG_FORMAT, level=logging.DEBUG)
        if args.pid is not None:
            simulate_system(args)
