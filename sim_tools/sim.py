import argparse
from ast import literal_eval
from collections import deque, namedtuple
import logging
import math
import matplotlib.pyplot as plt
import sys

from controller import PIDArduino
import plant

LOG_FORMAT = '%(name)s: %(message)s'
Simulation = namedtuple(
    'Simulation',
    ['name', 'controller', 'plant', 'delayed_states', 'timestamps',
     'plant_states', 'sensor_states', 'outputs'])


def simulate_system(args):
    timestamp = 0  # seconds
    delayed_samples_len = max(1, round(args.delay / args.sampletime))

    assert hasattr(plant, args.plant)
    plantClass = getattr(plant, args.plant)

    initial = literal_eval(args.initial_values)
    assert isinstance(initial, dict)
    constants = literal_eval(args.constant_values)
    assert isinstance(constants, dict)

    # Create a simulation for the tuple pid(kp, ki, kd)
    sim = Simulation(
        name='{} PID'.format(args.plant),
        controller=PIDArduino(
            sampletime=args.sampletime,
            kp=float(args.pid[0]),
            ki=float(args.pid[1]),
            kd=float(args.pid[2]),
            out_min=args.out_min,
            out_max=args.out_max,
            time=lambda: timestamp),
        plant=plantClass(initial, constants),
        delayed_states=deque(maxlen=delayed_samples_len),
        timestamps=[],
        plant_states=[],
        sensor_states=[],
        outputs=[],
    )

    # Init delayed_states deque for each simulation
    sim.delayed_states.extend(
        sim.delayed_states.maxlen * [sim.plant.sensable_state]
    )

    # Run simulation for specified interval. The (x60) is because args.interval
    # is in minutes and we want seconds
    while timestamp < (args.interval * 60):
        timestamp += args.sampletime

        # Calculates controller reaction
        if args.supress_output:
            output = 0.0
        else:
            output = sim.controller.calc(sim.delayed_states[0], args.setpoint)
            output = max(output, args.out_min)
            output = min(output, args.out_max)
        # Calculates the effects of the controller output on the next sensor
        # reading
        simulation_update(sim, timestamp, output, args)

    title = '{} simulation, {:.1f}s delay, {:.1f}s sampletime'.format(
        sim.name, args.delay, args.sampletime
    )
    plot_simulation(sim, title)

    # Do if implemented for this plant
    try:
        sim.plant.plot_state_history()
    except AttributeError:
        pass
    try:
        sim.plant.plot_energy()
    except AttributeError:
        pass
    try:
        sim.plant.animate_system()
    except AttributeError:
        pass


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
    ax2.set_ylabel('output')

    # Plot sensor and output values
    color = 'b'
    lines += ax1.plot(
        simulation.timestamps, simulation.sensor_states, color=color,
        alpha=1.0, label='sensor state')
    lines += ax2.plot(
        simulation.timestamps, simulation.outputs, '--', color=color,
        linewidth=1, alpha=0.7, label='output')

    # Create legend
    labels = [l.get_label() for l in lines]
    offset = math.ceil(4 / 3) * 0.05
    ax1.legend(lines,
               labels,
               loc=9,
               bbox_to_anchor=(0.5, -0.1 - offset),
               ncol=3)
    fig.subplots_adjust(bottom=(0.2 + offset))

    # Set title
    plt.title(title)
    fig.canvas.set_window_title(title)
    plt.show()


# Kettle
# python sim_tools/sim.py --pid 104 0.8 205 --out-min -0.0 --out-max 100.0 --sampletime 5 --delay 15.0 --setpoint 45.0 --interval 20 --initial-values "{'kettle_temp': 40.0}" --constant-values "{'ambient_temp': 20.0, 'volume': 70.0, 'diameter': 50.0, 'heater_power': 6.0, 'heat_loss_factor': 1.0}" --plant Kettle
# Pendulum:
# python sim_tools/sim.py --pid 2 0 0 --out-min -3.0 --out-max 3.0 --sampletime 0.05 --delay 0.0 --setpoint 3.141592653589793 --interval 2 --initial-values "{'theta0': 0.1}" --constant-values "{'length': 0.5}" --plant InvertedPendulum
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-P', '--plant', default='Kettle',
        help='the class from plant.py to simulate (e.g. InvertedPendulum)')
    parser.add_argument(
        '-p', '--pid', nargs=3, metavar=('kp', 'ki', 'kd'),
        default=None, help='simulate a PID controller')

    parser.add_argument(
        '-v', '--verbose', action='store_true',
        help='be verbose')

    parser.add_argument(
        '-s', '--setpoint', metavar='T', default=45.0, type=float,
        help='target sensor value (default: 45)')
    parser.add_argument(
        '-S', '--supress-output', action='store_true',
        help='sets output to 0.0 so you can see system steady state')

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
