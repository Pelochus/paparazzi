#!/usr/bin/env python
#
# Copyright (C) 2024 Hector Garcia de Marina <hgdemarina@gmail.com>
#                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING. If not, see
# <http://www.gnu.org/licenses/>.


# Segment formation made by Pelochus
# TODO: integrate circularFormation and segmentFormation in one script?
# Think carefully, since segmentFormation only works with rotorcrafts, 
# perhaps is best having fixedwing and rotorcrafts separated

'''
Centralized parallel segments formations employing guidance vector fields (GVF)
'''

import sys
import math
import numpy as np
import json
from time import sleep
from os import path, getenv

# This is needed before importing pprzlink
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python/")
sys.path.append(PPRZ_SRC + "/sw/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings_xml_parse import PaparazziACSettings

# Found in conf/messages.xml
raw_to_meters_factor = 0.003906 # Valid for INS and ROTORCRAFT_FP telemetry

# Recordatorio de que hacer
# Formula sera:

# vf1 = vi + k * (p2 - p1 - p*)
# vf2 = vi + k * (p1 - p2 - p*)

# Ademas para 3 seria (por ejemplo para vehiculo 1):
# vf1 = vi + k * ((p2 - p1 - p*21) + (p3 - p1 - p*31))
# Esto ultimo se puede ver mejor con Kuramoto, pero lo dejo como aclaracion que estaba dudoso que hacer con p*

# Con vf es velocidad final, vi velocidad inicial, k ganancia, p1 y p2 posiciones de los drones, p* posicion deseada
# Puede que haya que hacer ajustes (algun signo cambiado etc, pensar)
# Importante previamente parametrizar p de -1 a +1
# Comparar formula con Kuramoto ahora, se puede ver mejor (el sumatorio es si hubiera mas)

class Aircraft:
    def __init__(self, ac_id):
        self.initialized_gvf = False
        self.initialized_nav = False
        self.id = ac_id

        # Position 0 of array will be X, Position 1 will be Y
        self.XY = np.zeros(2)
        self.WP1 = np.zeros(2)
        self.WP2 = np.zeros(2)
        self.speed = 0 # Will be overwritten by telemetry
        self.index = 0

class FormationControl:
    def __init__(self, config, freq=10., verbose=False):
        self.config = config
        self.step = 1. / freq
        self.verbose = verbose
        self.ids = self.config['ids']
        self.first_iteration = True # Temporary solution for reading nominal speed 

        self.nominal_speed = self.config['nominal_speed'] 
        self.k = np.array(self.config['gain'])
        self.offset_desired = self.config['desired_normalized_offset'] # Should be between -1 and 1
        self.aircraft = [Aircraft(i) for i in self.ids]
        self.deltas = np.zeros(len(self.aircraft))
        self.mapped_pos = np.zeros(len(self.aircraft))

        print("Debugging: ", self.ids, self.k, self.offset_desired, self.aircraft) 

        for ac in self.aircraft:
            settings = PaparazziACSettings(ac.id)
            list_of_indexes = ['speed']

            for setting in list_of_indexes:
                try:
                    index = settings.name_lookup[setting].index
                    if setting == 'speed':
                        ac.index = index
                except Exception as e:
                    print(e)
                    print(setting + " setting not found, are you using a rotorcraft with GVF?")

        # Start IVY interface
        self._interface = IvyMessagesInterface("Segments Formation")

        # Read X and Y position (relative to home) and speeds
        def nav_cb(ac_id, msg):
            if ac_id in self.ids:
                if msg.name == "ROTORCRAFT_FP": 
                    ac = self.aircraft[self.ids.index(ac_id)]
                    ac.XY[0] = float(msg.get_field(0)) * raw_to_meters_factor
                    ac.XY[1] = float(msg.get_field(1)) * raw_to_meters_factor
                    vnorth = float(msg.get_field(2)) * raw_to_meters_factor
                    veast = float(msg.get_field(3)) * raw_to_meters_factor
                    ac.speed = math.sqrt(vnorth ** 2 + veast ** 2)
                    ac.initialized_nav = True

        self._interface.subscribe(nav_cb, PprzMessage("telemetry", "ROTORCRAFT_FP"))

        # Read waypoints
        def gvf_cb(ac_id, msg):
            if ac_id in self.ids and msg.name == "SEGMENT":
                ac = self.aircraft[self.ids.index(ac_id)]
                # TODO: Read current waypoints correctly, this will change periodically
                # ac.WP1[0] = float(msg.get_field(0))
                # ac.WP1[1] = float(msg.get_field(1))
                # ac.WP2[0] = float(msg.get_field(2))
                # ac.WP2[1] = float(msg.get_field(3))

                # Temporarily hardcoded from bebop2_gvf.xml
                # Bebop 2_2 (id 9) shall use segment 2 for correct results 
                ac.WP1[0] = 0
                ac.WP1[1] = 1
                ac.WP2[0] = 0
                ac.WP2[1] = 5
                if ac_id == 9:
                    ac.WP1[0] += 2
                    ac.WP2[0] += 2
                ac.initialized_gvf = True

        self._interface.subscribe(gvf_cb, PprzMessage("telemetry", "SEGMENT"))

    def __del__(self):
        self.stop()

    def stop(self):
        # Stop IVY interface
        if self._interface is not None:
            self._interface.shutdown()

    def segment_formation(self):
        '''
        Parallel segments formation control algorithm
        '''

        ready = True
        for ac in self.aircraft:
            if (not ac.initialized_nav) or (not ac.initialized_gvf):
                if self.verbose:
                    print("Waiting for state of aircraft ", ac.id)
                ready = False

        if not ready:
            return
        
        # If this is the first iteration, read the nominal speed for the drones
        # This speed shall be equal for each drone  
        # if self.first_iteration:
            # self.nominal_speed = 
            # self.first_iteration = False
        
        # Segment normalization
        i = 0
        for ac in self.aircraft:
            # Calculate x and y distances, from segment and current pos
            max_dist_x = ac.WP2[0] - ac.WP1[0]
            max_dist_y = ac.WP2[1] - ac.WP1[1]
            dist_x = ac.XY[0] - ac.WP1[0]
            dist_y = ac.XY[1] - ac.WP1[1]

            # Vector modulus (Pythagoras)
            max_dist = math.sqrt(max_dist_x**2 + max_dist_y**2)
            dist = math.sqrt(dist_x**2 + dist_y**2)

            # Having this, we can normalize the distance from 0 to 1
            norm_dist = dist / max_dist

            # Map segment values from range(0, 1) to range(-1, 1)
            def map_range(d, in_min, in_max, out_min, out_max):
                if (in_max == in_min):
                    return 0
                else:
                    return (d - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
            
            self.mapped_pos[i] = map_range(norm_dist, 0, 1, -1, 1)
            i += 1


        # TODO: Do properly, with nested for loops. Check Kuramoto model
        # Now we calculate the error (using the mapped positions in mapped_pos)
        i = 0
        for ac in self.aircraft:
            self.deltas[i] = self.mapped_pos[i] - self.mapped_pos[0] - self.offset_desired
            
            # Change direction of nominal speed when required 
            if ac.speed < 0:
                vn = -self.nominal_speed
            else:
                vn = self.nominal_speed
            
            vf = vn + self.k * self.deltas[i]

            if self.verbose:
                print("Inter-vehicle line offset: ", str(self.mapped_pos[i] - self.mapped_pos[0]).replace('[','').replace(']',''))
                print("Calculated speed: ", vf)

            # Send the modified speed (speed + s) to the aircraft
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = ac.id
            msg['index'] = ac.index
            msg['value'] = vf

            self._interface.send(msg)

            i += 1

    def run(self):
        try:
            # The main loop
            while True:
                # TODO: make better frequency managing
                sleep(self.step)

                # Run the formation algorithm
                self.segment_formation()

        except KeyboardInterrupt:
            self.stop()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Parallel segments formation for rotorcrafts")
    parser.add_argument('config_file', help="JSON configuration file")
    parser.add_argument('-f', '--freq', dest='freq', default=5, type=int, help="control frequency")
    parser.add_argument('-v', '--verbose', dest='verbose', default=False, action='store_true', help="display debug messages")
    args = parser.parse_args()

    with open(args.config_file, 'r') as f:
        conf = json.load(f)
        if args.verbose:
            print(json.dumps(conf))

        fc = FormationControl(conf, freq=args.freq, verbose=args.verbose)
        fc.run()