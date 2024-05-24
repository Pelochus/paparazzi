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
# TODO: integrate circularFormation and segmentFormation in one script
# Think carefully, since segmentFormation only works with rotorcrafts, 
# perhaps is best having fixedwing and rotorcrafts separated

'''
Centralized parallel segments formations employing guidance vector fields (GVF)
'''

import sys
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

# Recordatorio de que hacer
# Formula sera:

# vf1 = vi + k * (p2 - p1 - p*)
# vf2 = vi + k * (p1 - p2 - p*)

# Con vf es velocidad final, vi velocidad inicial, k ganancia, p1 y p2 posiciones de los drones, p* posicion deseada
# Puede que haya que hacer ajustes (algun signo cambiado etc, pensar)
# Importante previamente parametrizar p de -1 a +1
# Comparar formula con Kuramoto ahora, se puede ver mejor (el sumatorio es si hubiera mas)

class Aircraft:
    def __init__(self, ac_id):
        self.initialized_gvf = False
        self.initialized_nav = False
        self.id = ac_id

        self.XY = np.zeros(2)
        self.XYc = np.zeros(2)
        self.a = 0
        self.b = 0
        self.s = 1
        self.sigma = 0
        self.a_index = 0

class FormationControl:
    def __init__(self, config, freq=10., verbose=False):
        self.config = config
        self.step = 1. / freq
        self.verbose = verbose
        self.ids = self.config['ids']

        self.k = np.array(self.config['gain'])
        self.offset_desired = self.config['desired_normalized_offset'] # Should be between -1 and 1
        self.aircraft = [Aircraft(i) for i in self.ids]

        for ac in self.aircraft:
            settings = PaparazziACSettings(ac.id)
            list_of_indexes = ['speed']

            for setting in list_of_indexes:
                try:
                    index = settings.name_lookup[setting].index
                    if setting == 'speed':
                        ac.a_index = index
                except Exception as e:
                    print(e)
                    print(setting + " setting not found, are you using a rotorcraft with GVF?")

        # Start IVY interface
        self._interface = IvyMessagesInterface("Segments Formation")

        # Read X and Y position (relative)
        def nav_cb(ac_id, msg):
            if ac_id in self.ids:
                if msg.name == "INS":
                    ac = self.aircraft[self.ids.index(ac_id)]
                    ac.XY[0] = float(msg.get_field(0))
                    ac.XY[1] = float(msg.get_field(1))
                    ac.initialized_nav = True

        self._interface.subscribe(nav_cb, PprzMessage("telemetry", "INS"))

        # Read current waypoints
        def gvf_cb(ac_id, msg):
            if ac_id in self.ids and msg.name == "GVF":
                # If trajectory is a segment
                if int(msg.get_field(1)) == 0:
                    ac = self.aircraft[self.ids.index(ac_id)]
                    param = msg.get_field(4)
                    ac.XYc[0] = float(param[0])
                    ac.XYc[1] = float(param[1])
                    ac.a = float(param[2])
                    ac.b = float(param[3])
                    ac.s = float(msg.get_field(2))
                    ac.initialized_gvf = True

        self._interface.subscribe(gvf_cb, PprzMessage("telemetry", "GVF"))

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
        
        # Map segment values from -1 to 1
        

        i = 0
        for ac in self.aircraft:
            ac.sigma = np.arctan2(ac.XY[1]-ac.XYc[1], ac.XY[0]-ac.XYc[0])
            self.sigmas[i] = ac.sigma
            i = i + 1

        inter_sigma = self.B.transpose().dot(self.sigmas)
        error_sigma = inter_sigma - self.Zdesired

        for i in range(0, np.size(error_sigma)):
            if error_sigma[i] > np.pi:
                error_sigma[i] = error_sigma[i] - 2*np.pi
            elif error_sigma[i] <= -np.pi:
                error_sigma[i] = error_sigma[i] + 2*np.pi

        u = -self.aircraft[0].s * self.k * self.B.dot(error_sigma)

        if self.verbose:
            print("Inter-vehicle line offset: ", str("TODO").replace('[','').replace(']',''))

        # Send the modified speed (speed + s) to the aircraft
        i = 0
        for ac in self.aircraft:
            msg = PprzMessage("ground", "DL_SETTING")
            msg['ac_id'] = ac.id
            msg['index'] = ac.a_index
            msg['value'] = self.radius + u[i]

            self._interface.send(msga)

            i = i + 1

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
