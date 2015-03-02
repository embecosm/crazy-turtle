# -*- coding: utf-8 -*-
#
#  Copyright (C) 2014 Bitcraze AB
#  Copyright (C) 2015 Embecosm Limited
#
#  Contributor  Jeremy Bennett <jeremy.bennett@embecosm.com>
#
#  Make the CrazyFlie hop.  Based on the ramp.py example from the Crazyflie
#  Nano Quadcopter Client
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
Simple example that connects to the first Crazyflie found, powers up the motors for 500ms and disconnects.
"""

import os
import logging
import time, sys
from threading import Thread

# If it's not on the path already, put the Crazyflie client library in the
# CFLIB environment variable.  This is the "lib" subdirectory of the Crazyflie
# client library.
try:
    sys.path.append(os.environ['CFLIB'])
except:
    # Just catch and ignore
    print "CFLIB not found"

import cflib
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie

import logging
logging.basicConfig(level=logging.ERROR)

class MotorJumpExample:
    """Class to make a Crazyflie hop

    There are no public classes or instance variables.  The behavior is
    invoked by instantiating the class.
    """

    def __init__(self, link_uri):
        """ Initialize and invoke behavior with the specified link_uri

        Keyword argument:
        link_uri -- the URI for the Crazyflie (no default)
        """

        # Instantiate a Crazyflie and set its callbacks
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Opening the connection will cause the behavior to happen
        self._cf.open_link(link_uri)

        print "Connecting to %s" % link_uri


    def _connected(self, link_uri):
        """Callback on connection to a Crazyflie.

        This method is called from the Crazyflie API when a Crazyflie has been
        connected and the TOCs have been downloaded.

        It sets up some logging for the barometer, then after a short pause
        starts a separate thread to drive the motors.

        Keyword argument:
        link_uri -- the URI for the Crazyflie (no default)
        """

        # Initially we go up and have done nothing with the barometer log
        self._thrust_mult = 1
        self._baro_count = 0;
        self._initial_baro_average = 0;
        self._baro_average = 0;

        # Start the barometer logging.

        # The definition of the logconfig could have been made before
        # connecting, but we do it here, since the rest of the intialization
        # can't be done until we are connected.
        self._lg_baro = LogConfig(name = "Baro", period_in_ms = 100);
        self._lg_baro.add_variable("baro.aslLong", "float");

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
            print("Could not add baro logconfig")

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._jump_motors).start()


    def _baro_log_error(self, logconf, msg):
        """Callback from the baro log API when an error occurs

        We just note that an error has occurred.

        Keyword arguments:
        logconf -- configuration data for this log
        msg     -- message to print out.
        """

        print "Error when logging baro %s: %s" % (logconf.name, msg)


    def _baro_log_data(self, timestamp, data, logconf):
        """Callback from the baro log API when data arrives

        We print out the data from the barameter.

        Keyword arguments:
        timestamp -- when the log data occurred
        data      -- the actual data
        logconf   -- configuration data for this log


        """
        print "[%d][%s]: %f" % (timestamp, logconf.name, data['baro.aslLong'])


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails

        We just print out a message.

        Keyword arguments:
        link_uri -- the URI for the Crazyflie
        msg      -- message to print out
        """
        print "Connection to %s failed: %s" % (link_uri, msg)

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made

        This typically occurs if Crazyflie moves out of range.  We just print
        a message.

        Keyword arguments:
        link_uri -- the URI for the Crazyflie
        msg      -- message to print out
        """
        print "Connection to %s lost: %s" % (link_uri, msg)


    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected

        This is normal behaviour, and we just print a message.

        Keyword arguments:
        link_uri -- the URI for the Crazyflie
        """
        print "Disconnected from %s" % link_uri


    def _jump_motors(self):
        """Make the Crazyflie jump

        Just a burst of the motors.
        """

        # Parameters
        thrust_step = 500
        thrust_start = 20000
        thrust_stop  = 40000
        pitch = 15
        roll = 0
        yawrate = 0

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Rapidly wind up
        for t in range(thrust_start, thrust_stop, thrust_step):
            self._cf.commander.send_setpoint(roll, pitch, yawrate, t)
            time.sleep(0.01)

        # Wait a little bit
        time.sleep(1.0)

        # Turn the motors off
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()


# Initialize the instance to make the motor jump
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
        le = MotorJumpExample(link_uri = available[0][0])
    else:
        print "No Crazyflies found, cannot run example"
