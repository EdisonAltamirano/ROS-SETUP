#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Dust Storm Weather Simulation:

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


class DustStorm(object):
    """
    Simulates a dust storm effect by dynamically modifying a few weather parameters.
    In a dust storm, we simulate a high dust concentration (using cloudiness),
    high wind intensity, and a hazy atmosphere (using fog density). There is no rain.
    """
    def __init__(self):
        self._t = 0.0
        self._increasing = True
        self.clouds = 0.0  # Used to simulate dust concentration.
        self.wind = 0.0
        self.fog = 0.0

    def tick(self, delta_seconds):
        # Increase or decrease the dust storm intensity over time.
        delta = (20.0 if self._increasing else -20.0) * delta_seconds
        self._t = clamp(self._t + delta, 0.0, 100.0)
        if self._t >= 100.0:
            self._increasing = False
        elif self._t <= 0.0:
            self._increasing = True

        # Map the intensity to weather parameters.
        self.clouds = self._t  # Dust concentration (0 to 100).
        # Wind increases with intensity (base wind plus a factor).
        self.wind = clamp(30.0 + 0.7 * self._t, 0.0, 100.0)
        # Fog density simulates haze from dust; capped to a moderate value.
        self.fog = clamp(self._t * 0.5, 0.0, 50.0)

    def __str__(self):
        return 'DustStorm(intensity=%d, clouds=%d, wind=%d, fog=%d)' % (
            int(self._t), int(self.clouds), int(self.wind), int(self.fog))


class DustStormWeather(object):
    """
    Combines the Sun simulation with the DustStorm effect to update CARLA's weather.
    Note that precipitation and wetness are set to zero for a dry dust storm.
    """
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self._dust_storm = DustStorm()

    def tick(self, delta_seconds):
        self._sun.tick(delta_seconds)
        self._dust_storm.tick(delta_seconds)
        # Update CARLA weather parameters:
        self.weather.cloudiness = self._dust_storm.clouds
        self.weather.precipitation = 0.0              # No rain during a dust storm.
        self.weather.precipitation_deposits = 0.0
        self.weather.wind_intensity = self._dust_storm.wind
        self.weather.fog_density = self._dust_storm.fog
        self.weather.wetness = 0.0
        self.weather.sun_azimuth_angle = self._sun.azimuth
        self.weather.sun_altitude_angle = self._sun.altitude

    def __str__(self):
        return '%s %s' % (self._sun, self._dust_storm)


def main():
    argparser = argparse.ArgumentParser(
        description="CARLA Dust Storm Weather Simulation")
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

    # Start with the current weather and apply our custom dust storm simulation.
    weather = DustStormWeather(world.get_weather())

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
