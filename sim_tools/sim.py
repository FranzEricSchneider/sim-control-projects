from pid import PIDArduino
from kettle import Kettle
from collections import deque, namedtuple
import sys
import math
import logging
import argparse
import matplotlib.pyplot as plt

#### Removed
#!/bin/python3

LOG_FORMAT = '%(name)s: %(message)s'
Simulation = namedtuple(
    'Simulation',
    ['name', 'sut', 'kettle', 'delayed_temps', 'timestamps',
     'heater_temps', 'sensor_temps', 'outputs'])


def parser_add_args(parser):
    parser.add_argument(
        '-p', '--pid', dest='pid', nargs=4, metavar=('name', 'kp', 'ki', 'kd'),
        default=None, action='append', help='simulate a PID controller')
    parser.add_argument(
        '-a', '--atune', dest='autotune', default=False,
        action='store_true', help='simulate autotune')

    parser.add_argument(
        '-v', '--verbose', dest='verbose', default=0,
        action='count', help='be verbose')
    parser.add_argument(
        '-e', '--export', dest='export', default=False,
        action='store_true', help='export data to a .csv file')
    parser.add_argument(
        '-n', '--noplot', dest='noplot', default=False,
        action='store_true', help='do not plot the results')

    parser.add_argument(
        '-t', '--temp', dest='kettle_temp', metavar='T', default=40.0,
        type=float, help='initial kettle temperature in C (default: 40)')
    parser.add_argument(
        '-s', '--setpoint', dest='setpoint', metavar='T', default=45.0,
        type=float, help='target temperature in C (default: 45)')
    parser.add_argument(
        '--ambient', dest='ambient_temp', metavar='T', default=20.0,
        type=float, help='ambient temperature in C (default: 20)')

    parser.add_argument(
        '-i', '--interval', dest='interval', metavar='t', default=20,
        type=int, help='simulated interval in minutes (default: 20)')
    parser.add_argument(
        '-d', '--delay', dest='delay', metavar='t', default=15.0,
        type=float, help='system response delay in seconds (default: 15)')
    parser.add_argument(
        '--sampletime', dest='sampletime', metavar='t', default=5.0,
        type=float, help='temperature sample time in seconds (default: 5)')

    parser.add_argument(
        '--volume', dest='volume', metavar='V', default=70.0,
        type=float, help='kettle content volume in liters (default: 70)')
    parser.add_argument(
        '--diameter', dest='diameter', metavar='d', default=50.0,
        type=float, help='kettle diameter in cm (default: 50)')

    parser.add_argument(
        '--power', dest='heater_power', metavar='P', default=6.0,
        type=float, help='heater power in kW (default: 6)')
    parser.add_argument(
        '--heatloss', dest='heat_loss_factor', metavar='x', default=1.0,
        type=float, help='kettle heat loss factor (default: 1)')

    parser.add_argument(
        '--minout', dest='out_min', metavar='x', default=0.0,
        type=float, help='minimum PID controller output (default: 0)')
    parser.add_argument(
        '--maxout', dest='out_max', metavar='x', default=100.0,
        type=float, help='maximum PID controller output (default: 100)')


def write_csv(sim):
    filename = sim.name + '.csv'
    with open(filename, 'w+') as csv:
        csv.write('timestamp;output;sensor_temp;heater_temp\n')

        for i in range(0, len(sim.timestamps)):
            csv.write('{0};{1:.2f};{2:.2f};{3:.2f}\n'.format(
                sim.timestamps[i], sim.outputs[i], sim.sensor_temps[i], sim.heater_temps[i]))


def sim_update(sim, timestamp, output, args):
    sim.kettle.heat(timestamp, args.heater_power * (output / 100), args.sampletime)
    sim.kettle.cool(timestamp, args.sampletime, args.ambient_temp, args.heat_loss_factor)
    sim.delayed_temps.append(sim.kettle.temperature)
    sim.timestamps.append(timestamp)
    sim.outputs.append(output)
    sim.sensor_temps.append(sim.delayed_temps[0])
    sim.heater_temps.append(sim.kettle.temperature)


def plot_simulations(simulations, title):
    lines = []
    fig, ax1 = plt.subplots()
    upper_limit = 0

    # # Try to limit the y-axis to a more relevant area if possible
    # for sim in simulations:
    #     m = max(sim.sensor_temps) + 1
    #     upper_limit = max(upper_limit, m)

    # if upper_limit > args.setpoint:
    #     lower_limit = args.setpoint - (upper_limit - args.setpoint)
    #     ax1.set_ylim(lower_limit, upper_limit)

    # Create x-axis and first y-axis (temperature)
    ax1.plot()
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('temperature (C)')
    ax1.grid(axis='y', linestyle=':', alpha=0.5)

    # Draw setpoint line
    lines += [plt.axhline(
        y=args.setpoint, color='r', linestyle=':', linewidth=0.9, label='setpoint')]

    # Create second y-axis (power)
    ax2 = ax1.twinx()
    ax2.set_ylabel('power (%)')

    # Plot temperature and output values
    for sim in simulations:
        color = 'b'
        lines += ax1.plot(
            sim.timestamps, sim.sensor_temps, color=color,
            alpha=1.0, label='{0}: temp.'.format(sim.name))
        lines += ax2.plot(
            sim.timestamps, sim.outputs, '--', color=color,
            linewidth=1, alpha=0.7, label='{0}: output'.format(sim.name))

    # Create legend
    labels = [l.get_label() for l in lines]
    offset = math.ceil((1 + len(simulations) * 2) / 3) * 0.05
    ax1.legend(lines, labels, loc=9, bbox_to_anchor=(
        0.5, -0.1 - offset), ncol=3)
    fig.subplots_adjust(bottom=0.2 + offset)

    # Set title
    plt.title(title)
    fig.canvas.set_window_title(title)
    plt.show()


def simulate_pid(args):
    timestamp = 0  # seconds
    delayed_temps_len = max(1, round(args.delay / args.sampletime))
    sims = []

    #### Set up multiple simulations ##########################################
    #### Fun facts:
    ######## Simulation is a named tuple, not a class
    ######## I'm not sure what "sut" is, but it's PID Arduino
    # Create a simulation for each tuple pid(name, kp, ki, kd)
    for pid in args.pid:
        sim = Simulation(
            pid[0],
            PIDArduino(
                args.sampletime, float(pid[1]), float(pid[2]), float(pid[3]),
                args.out_min, args.out_max, lambda: timestamp),
            Kettle(args.diameter, args.volume, args.kettle_temp),
            deque(maxlen=delayed_temps_len),
            [], [], [], []
        )
        sims.append(sim)

    #### Oh! This is to do time delay! ########################################
    ######## The system starts off with some delayed data, then later uses
    ############ delayed temperature to set the output
    ######## Because delayed_temps is a deque with a fixed length, extending
    ############ data onto the end pushes old data off the front. This saves
    ############ bookkeeping. The function adds a new data point and delay[0]
    ############ automatically becomes up to date "delayed" data
    # Init delayed_temps deque for each simulation
    for sim in sims:
        sim.delayed_temps.extend(sim.delayed_temps.maxlen * [args.kettle_temp])

    # Run simulation for specified interval
    #### The (x60) is because args.interval is in minutes and we want seconds
    while timestamp < (args.interval * 60):
        timestamp += args.sampletime

        for sim in sims:
            #### Calculates the reaction of the controller
            output = sim.sut.calc(sim.delayed_temps[0], args.setpoint)
            output = max(output, 0)
            output = min(output, 100)
            #### Calculates the effects of the controller output on the next
            ######## sensor reading
            sim_update(sim, timestamp, output, args)

            if args.verbose > 0:
                print('time:    {0} sec'.format(timestamp))
                print('{0}: {1:.2f}%'.format(sim.name, output))
                print('temp sensor:    {0:.2f}C'.format(sim.sensor_temps[-1]))
                print('temp heater:    {0:.2f}C'.format(sim.heater_temps[-1]))
        if args.verbose > 0:
            print()

    if args.export:
        for sim in sims:
            write_csv(sim)

    if not args.noplot:
        title = 'PID simulation, {0:.1f}l kettle, {1:.1f}kW heater, {2:.1f}s delay'.format(
            args.volume, args.heater_power, args.delay)
        plot_simulations(sims, title)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser_add_args(parser)

    if len(sys.argv) == 1:
        parser.print_help()
    else:
        args = parser.parse_args()

        if args.verbose > 1:
            logging.basicConfig(stream=sys.stderr, format=LOG_FORMAT, level=logging.DEBUG)
        if args.autotune:
            simulate_autotune(args)
        if args.pid is not None:
            simulate_pid(args)
