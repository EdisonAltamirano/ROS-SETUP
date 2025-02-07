#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA MyWeather Storm Weather Simulation:

Connect to a CARLA Simulator instance and control a custom dust storm weather.
The script updates the sun position smoothly and simulates a dust storm by 
increasing airborne dust (using the cloudiness parameter), wind, and haze.
"""

import glob
import os
import sys
import math
import argparse

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))


class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds
        self.azimuth %= 360.0
        self.altitude = (70 * math.sin(self._t)) - 20

    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f)' % (self.altitude, self.azimuth)


class MyWeather(object):
    """
    Combines the Sun simulation with your Weather
    """
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)

    def tick(self, delta_seconds):
        self._sun.tick(delta_seconds)
        # Update CARLA weather parameters:
        self.weather.cloudiness = #Aqui el codigo de la practica5
        self.weather.precipitation = 0.0              # No rain during a storm.
        self.weather.precipitation_deposits = 0.0
        self.weather.wind_intensity = #Aqui el codigo de la practica5
        self.weather.fog_density = #Aqui el codigo de la practica5
        self.weather.wetness = 0.0
        self.weather.sun_azimuth_angle = #Aqui el codigo de la practica5
        self.weather.sun_altitude_angle = #Aqui el codigo de la practica5

    def __str__(self):
        return '%s %s' % (self._sun)


def main():
    argparser = argparse.ArgumentParser(
        description="CARLA MyWeather Storm Weather Simulation")
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--speed',
        metavar='FACTOR',
        default=1.0,
        type=float,
        help='rate at which the weather changes (default: 1.0)')
    args = argparser.parse_args()

    speed_factor = args.speed
    update_freq = 0.1 / speed_factor

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    # Start with the current weather and apply our custom storm simulation.
    weather = MyWeather(world.get_weather())

    elapsed_time = 0.0

    while True:
        timestamp = world.wait_for_tick(seconds=30.0).timestamp
        elapsed_time += timestamp.delta_seconds
        if elapsed_time > update_freq:
            weather.tick(speed_factor * elapsed_time)
            world.set_weather(weather.weather)
            sys.stdout.write('\r' + str(weather) + ' ' * 12)
            sys.stdout.flush()
            elapsed_time = 0.0


if __name__ == '__main__':
    main()
