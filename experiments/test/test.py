# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Simple example that connects to the first Crazyflie found, ramps up/down
the motors and disconnects.
"""

import logging
import time, sys
from threading import Thread

#FIXME: Has to be launched from within the example folder
sys.path.append("../lib")
import cflib
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie

import logging
logging.basicConfig(level=logging.ERROR)

class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""
    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print "Connecting to %s" % link_uri

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Initially we go up and have done nothing with the barometer log
        self._thrust_mult = 1
        self._baro_count = 0;
        self._initial_baro_average = 0;
        self._baro_average = 0;

        # Start the barometer logging
        # The definition of the logconfig can be made before connecting
        self._lg_baro = LogConfig(name = "Baro", period_in_ms = 100);
        self._lg_baro.add_variable("baro.aslLong", "float");
        self._lg_baro.add_variable("stabilizer.roll", "float")
        self._lg_baro.add_variable("stabilizer.pitch", "float")
        self._lg_baro.add_variable("stabilizer.yaw", "float")
        self._lg_baro.add_variable("stabilizer.thrust", "uint16_t")

        # Initialize the stabilizer params
        self._roll = 0
        self._thrust = 0
        self._yaw = 0
        self._pitch = 0

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        self._cf.log.add_config(self._lg_baro)
        if self._lg_baro.valid:
            # This callback will receive the data
            self._lg_baro.data_received_cb.add_callback(self._baro_log_data)
            # This callback will be called on errors
            self._lg_baro.error_cb.add_callback(self._baro_log_error)
            # Start the logging
            self._lg_baro.start()
        else:
            print("Could not add baro logconfig: some variables not in TOC")

        # Wait for the baseline barometer to be set
        time.sleep(1.0)

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._ramp_motors).start()

    def _baro_log_error(self, logconf, msg):
        """Callback from the baro log API when an error occurs"""
        print "Error when logging baro %s: %s" % (logconf.name, msg)

    def _baro_log_data(self, timestamp, data, logconf):
        """Callback from the baro log API when data arrives"""
        # Initial barometer period is when we establish we are on the ground.
        # We calculate a rolling average of the barometer.
        self._baro_count = self._baro_count + 1

        print data
        self._roll = data['stabilizer.roll']
        self._thrust = data['stabilizer.thrust']
        self._yaw = data['stabilizer.yaw']
        self._pitch = data['stabilizer.pitch']

        if self._baro_count == 1:
            self._baro_average = data['baro.aslLong']
        else:
            self._baro_average = (self._baro_average * 0.9) + (data['baro.aslLong'] * 0.1)

        if self._baro_count <= 10:
            # Still in initial phase
            self._initial_baro_average = self._baro_average
        elif abs(self._initial_baro_average - self._baro_average) > 2.0:
            # In testing phase have climbed enough.
            self._thrust_mult = -1
            print "Flipping"

#        print "[%d][%s]: %f %f %f" % (timestamp, logconf.name,
#                                      data['baro.aslLong'],
#                                      self._initial_baro_average,
#                                      self._baro_average)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print "Disconnected from %s" % link_uri

    def _ramp_motors(self):
        thrust_step = 750
        thrust = 20000
        pitch = 0
        roll = 0
        yawrate = 0

        # New zeroes
        roll_zero  = -1.3407979011535645
        pitch_zero = -0.5004292726516724

        # Trim values
        roll_trim  = -roll_zero
        pitch_trim = -pitch_zero

        # offsets from the trim value
        roll_offset = 5.0
        pitch_offset = 5.0

        #Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        while thrust >= 20000:
        #while True:
            #if self._roll < roll_zero:
            #  roll = roll_trim + roll_offset
            #else:
            #  roll = roll_trim - roll_offset

            #if self._pitch < pitch_zero:
            #  pitch = pitch_trim + pitch_offset
            #else:
            #  pitch = pitch_trim - pitch_offset

            # calculate new values
            roll  = 0
            pitch = 0
            yaw = 170

            kill = False
            cutoff = 20.0
            cutoff_en = True

            if (cutoff_en and (abs(self._roll)  > cutoff or
                abs(self._pitch) > cutoff)):  #or
                #abs(self._yaw)   > cutoff)):
              kill = True

            if thrust >= 45000:
              kill = True

            # If kill is set, bring it down gracefully, else climb
            if not kill:
              print "Killswitch!"
              thrust += thrust_step
            else:
              thrust -= 2 * thrust_step

            print 'Current data', (self._roll, self._pitch,
                                  self._yaw,   self._thrust)
            print 'Sending data', (roll, pitch, yaw, thrust)

            self._cf.commander.send_setpoint(roll, pitch, yaw, thrust)
            time.sleep(0.1)

        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print "Scanning interfaces for Crazyflies..."
    available = cflib.crtp.scan_interfaces()
    print "Crazyflies found:"
    for i in available:
        print i[0]

    if len(available) > 0:
        le = MotorRampExample(available[0][0])
    else:
        print "No Crazyflies found, cannot run example"
