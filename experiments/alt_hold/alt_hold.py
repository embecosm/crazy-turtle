# -*- coding: utf-8 -*-
#
#  Copyright (C) 2014 Bitcraze AB
#  Copyright (C) 2015 Embecosm Limited
#
#  Contributor  Jeremy Bennett <jeremy.bennett@embecosm.com>
#
#  Make the CrazyFlie jump to a controlled height using the built-in altitude
#  controller.
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
Module to make the Crazyflie jump to a controlled height, then rise gently

This code implements the library "altitude hold" function.

This simple example connects to the first Crazyflie found, powers up the
motors and attempts to rise to a fixed barometric pressure without drifting.

Classes exported:
AltHoldJumpExample -- Instantiate to make the crazyflie jump to a height
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
    print "CFLIB not specified"

import cflib
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie

import logging
logging.basicConfig(level=logging.ERROR)


class AltHoldJumpExample:
    """Class to make a Crazyflie jump to a controlled height.

    The only public instance is the constructor.  The behavior is invoked by
    instantiating the class.
    """

    def __init__(self, link_uri, debug = 0):
        """ Initialize and invoke behavior with the specified link_uri

        Keyword arguments:
        link_uri -- the URI for the Crazyflie (no default)
        debug    -- control debug messages.  Default to zero (no messages)
        """

        # Record debug state
        self._debug = debug

        # Instantiate a Crazyflie and set its callbacks
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Log instantaneous data
        self._timestamp = 0
        self._asl       = 0.0
        self._roll      = 0.0
        self._pitch     = 0.0
        self._yaw       = 0.0
        self._thrust    = 0

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

        # Start the logging.

        # The definition of the logconfig could have been made before
        # connecting, but we do it here, since the rest of the intialization
        # can't be done until we are connected.
        self._lg = LogConfig(name = "Log", period_in_ms = 10);
        self._lg.add_variable("baro.aslLong", "float");
        self._lg.add_variable("stabilizer.roll", "float")
        self._lg.add_variable("stabilizer.pitch", "float")
        self._lg.add_variable("stabilizer.yaw", "float")
        self._lg.add_variable("stabilizer.thrust", "uint16_t")

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        self._cf.log.add_config(self._lg)
        if self._lg.valid:
            # This callback will receive the data
            self._lg.data_received_cb.add_callback(self._log_data)
            # This callback will be called on errors
            self._lg.error_cb.add_callback(self._log_error)
            # Start the logging
            self._lg.start()
        else:
            print "Could not add logconfig"

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._alt_hold_motors).start()


    def _log_error(self, logconf, msg):
        """Callback from the log API when an error occurs

        We just note that an error has occurred.

        Keyword arguments:
        logconf -- configuration data for this log
        msg     -- message to print out.
        """

        print "Error when logging %s: %s" % (logconf.name, msg)


    def _log_data(self, timestamp, data, logconf):
        """Callback from the log API when data arrives

        We print out the data from the log.

        Keyword arguments:
        timestamp -- when the log data occurred
        data      -- the actual data
        logconf   -- configuration data for this log
        """

        #  Log data. Remember pitch and yaw are negative!
        self._timestamp = timestamp
        self._asl       = data['baro.aslLong']
        self._roll      = data['stabilizer.roll']
        self._pitch     = - data['stabilizer.pitch']
        self._yaw       = - data['stabilizer.yaw']
        self._thrust    = data['stabilizer.thrust']

        if self._debug > 0:
            print "[%d]: %f (%f %f %f) %d"  % (self._timestamp,
                                               self._asl,
                                               self._roll,
                                               self._pitch,
                                               self._yaw,
                                               self._thrust)


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


    def _alt_hold_motors(self):
        """Make the Crazyflie jump to a controlled height

        Just a burst of the motors followed by altitude hold flight mode.
        """

        # Kill parameters
        kill = False
        cutoff = 20.0
        cutoff_en = True

        # Initial values for attitude
        # Roll  - negative makes it go left
        # Pitch - positive makes it go forward
        # Yaw   - ? makes it go which way?
        rollInit  = -2.3
        pitchInit =  7.0
        yawInit   =  0.0

        # Parameters to get off the ground.
        thrust_step  =   500
        thrust_start = 20000
        thrust_stop  = 45000

        # Timing
        dt = 0.01
        jumptime = 5
        jumpsteps = int(float(jumptime) / dt)

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Rapidly wind up
        for t in range(thrust_start, thrust_stop, thrust_step):
            self._cf.commander.send_setpoint(rollInit, pitchInit, yawInit, t)
            time.sleep(0.01)

        time.sleep (0.9)
        kill = False

        # Magic command to set altitude hold mode.  Now "thrust" parameter is
        # an indicator of what to do
        # < 32767 -- drift down
        # = 32767 -- stay level
        # > 32767 -- drift up
        self._cf.param.set_value('flightmode.althold', 'True')

        fly_time = 3
        dt = 0.1
        count = int(fly_time / dt)

        print "steady"

        for i in range(1, count):
            if (cutoff_en and (abs(self._roll)  > cutoff or
                               abs(self._pitch) > cutoff)):
                print "Cutoff attitude (%f %f)" % (self._roll, self._pitch)
                kill = True

            if kill:
                # Die politely
                print "Kill switch"
                self._cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(1.0)
                self._cf.close_link()
                return
            else:
                self._cf.commander.send_setpoint(rollInit, pitchInit, yawInit,
                                                 32767)
                time.sleep(dt)

        print "climbing"

        for i in range(1, count):
            if (cutoff_en and (abs(self._roll)  > cutoff or
                               abs(self._pitch) > cutoff)):
                print "Cutoff attitude (%f %f)" % (self._roll, self._pitch)
                kill = True

            if kill:
                # Die politely
                print "Kill switch"
                self._cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(1.0)
                self._cf.close_link()
                return
            else:
                self._cf.commander.send_setpoint(rollInit, pitchInit, yawInit,
                                                 64000)
                time.sleep(dt)

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(1.0)
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
        le = AltHoldJumpExample(link_uri = available[0][0], debug = 0)
    else:
        print "No Crazyflies found, cannot run example"
