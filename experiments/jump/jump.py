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
Module to make the Crazyflie hop

Simple example that connects to the first Crazyflie found, powers up the
motors for 500ms and disconnects.

Classes exported:
MotorJumpExample -- Instantiate to make the crazyflie hop
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

        # Log instantaneous data
        self._timestamp = 0
        self._asl       = 0.0
        self._roll      = 0.0
        self._pitch     = 0.0
        self._yaw       = 0.0
        self._thrust    = 0

        # Log rolling averages
        self._asl_av   = 0.0
        self._roll_av  = 0.0
        self._pitch_av = 0.0
        self._yaw_av   = 0.0

        # Rolling average is done as an infinite impulse response filter
        self._iir_coeff = 0.8

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
        self._lg = LogConfig(name = "Log", period_in_ms = 20);
        self._lg.add_variable("baro.aslLong", "float");
        self._lg.add_variable("stabilizer.roll", "float")
        self._lg.add_variable("stabilizer.pitch", "float")
        self._lg.add_variable("stabilizer.yaw", "float")
        self._lg.add_variable("stabilizer.thrust", "uint16_t")

        self._first_datapoint = True;

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
        Thread(target=self._jump_motors).start()


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

        #  Log data. Remember pitch is negative!
        self._asl = data['baro.aslLong']
        self._roll    = data['stabilizer.roll']
        self._pitch   = - data['stabilizer.pitch']
        self._yaw     = - data['stabilizer.yaw']
        self._thrust  = data['stabilizer.thrust']

        # Rolling averages. Start off with the value we have.
        self._timestamp   = timestamp
        if self._first_datapoint:
            self._asl_av  = self._asl
            self._roll_av     = self._roll
            self._pitch_av    = self._pitch
            self._yaw_av      = self._yaw

            self._first_datapoint = False;
        else:
            self._asl_av   = self._iir_filter(self._asl_av,   self._asl)
            self._roll_av  = self._iir_filter(self._roll_av,  self._roll)
            self._pitch_av = self._iir_filter(self._pitch_av, self._pitch)
            self._yaw_av   = self._iir_filter(self._yaw_av,   self._yaw)

        print "[%d]: %f (%f %f %f)"  % (timestamp,
                                        self._asl_av,
                                        self._roll_av,
                                        self._pitch_av,
                                        self._yaw_av)

    def _iir_filter(self, point, prev_res):
        """Apply the a simple infinite impulse response filter

          X[n] = (1-a).x[n] + a.X[n-1e]

        Keyword arguments:
        point    - the data point to add
        prev_res - the result prior to this point.

        Returns the updated IIR filter result
        """

        return (1.0 - self._iir_coeff) * point + prev_res * self._iir_coeff


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

        # Kill parameters
        kill = False
        cutoff = 20.0
        cutoff_en = True

        # Parameters
        # Roll - negative makes it go left
        # Pitch - positive makes it go forward
        thrust_step  =   500
        thrust_start = 20000
        thrust_stop  = 37000
        thrust_limit = 50000

        # Target as a fraction of the take off ASL
        asl_frac = 0.995

        roll  = -2
        pitch = 5
        yaw   = 0

        step_period = 0.05
        jumptime = 3
        jumpsteps = int(float(jumptime) / step_period)

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Rapidly wind up
        for t in range(thrust_start, thrust_stop, thrust_step):
            self._cf.commander.send_setpoint(roll, pitch, yaw, t)
            time.sleep(0.01)

        self._aslTarget = self._asl * asl_frac
        print "Target ASL %f" % (self._aslTarget)

        thrust = thrust_stop
        dircount = 0
        prev_asl = self._asl_av

        # Then new figures every second
        for i in range (1, jumpsteps, 1):
            if (cutoff_en and (abs(self._roll_av)  > cutoff or
                               abs(self._pitch_av) > cutoff)):
                print "Cutoff (%f %f)" % (self._roll_av, self._pitch_av)
                kill = True

            if kill:
                # Die politely
                print "Kill switch"
                self._cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(1.0)
                self._cf.close_link()
            else:
                print "Correcting: (%f %f %f) %d"  % (roll  - self._roll_av,
                                                      pitch - self._pitch_av,
                                                      yaw   - self._yaw_av,
                                                      thrust)

                self._cf.commander.send_setpoint(roll  - self._roll_av,
                                                 pitch - self._pitch_av,
                                                 yaw   - self._yaw_av,
                                                 thrust)

            # Wait a little bit
            time.sleep(step_period)

        # Turn the motors off
        self._cf.commander.send_setpoint(0, 0, 0, 0)

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
        le = MotorJumpExample(link_uri = available[0][0])
    else:
        print "No Crazyflies found, cannot run example"
