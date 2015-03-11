# -*- coding: utf-8 -*-
#
#  Copyright (C) 2014 Bitcraze AB
#  Copyright (C) 2015 Embecosm Limited
#
#  Contributor  Jeremy Bennett <jeremy.bennett@embecosm.com>
#
#  Make the CrazyFlie jump to a controlled height using a PID controller.
#  Based on the jump.py example from this repository.
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
Module to make the Crazyflie jump to a controlled height

This code implements a proportional-integral-derivative controller (PID
controller) to control roll, pitch, yaw and barometric pressure.  See
http://en.wikipedia.org/wiki/PID_controller for an explanation of PID
controllers.

This simple example connects to the first Crazyflie found, powers up the
motors and attempts to rise to a fixed barometric pressure without drifting.

Classes exported:
PidController       -- Instantiate for each variable to be controlled
MotorPidJumpExample -- Instantiate to make the crazyflie jump to a height
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


class PidController:
    """Class to provide PID Control for a variable

    Note that we assume the control variable is floating point.

    The following public methods are available:
    __init__ -- constructor which defines the setpoint and initial tuning params
    control  -- perform one step of the control loop
    setpoint -- change the target point
    kp       -- set the proportional gain
    ki       -- set the integral gain
    kd       -- set the derivative gain
    """

    def __init__(self, setpoint = 0.0, kp = 0.0, ki = 0.0, kd = 0.0):
        """ Initialize the PID control

        The setpoint and gain parameters all have defaults of zero.

        Keyword parameters:
        setpoint  -- the setpoint
        kp        -- the proportional gain
        ki        -- the integral gain
        kd        -- the derivative gain
        """

        # Store the instance parameters
        self._setpoint = float(setpoint)
        self._kp       = float(kp)
        self._ki       = float(ki)
        self._kd       = float(kd)

        # Initialize the control variables for the derivative and integral
        # terms.
        self._prev_err = 0.0
        self._integral = 0.0


    def control(self, mv, dt):
        """ Carry out one step of the PID control loop

        Keyword parameters:
        mv -- the measured value
        dt -- the time step

        Returns the output to drive
        """

        err = self._setpoint - float(mv)
        self._integral = self._integral + err * float(dt)
        deriv = (err - self._prev_err) / float(dt)
        output = self._kp * err + self._ki * self._integral + self._kd * deriv
        self._prev_err = err

        return  output


    def setpoint(self, setpoint):
        """ Set the setpoint

        Update the target value for the controller

        Keyword parameters:
        setpoint -- the new setpoint
        """

        self._setpoint = float(setpoint)


    def kp(self, kp):
        """ Set the proportional gain

        Keyword parameters:
        kp -- the new proportional gain
        """

        self._kp = float(kp)


    def ki(self, ki):
        """ Set the integral gain

        Keyword parameters:
        ki -- the new integral gain
        """

        self._ki = float(ki)


    def kd(self, kd):
        """ Set the derivative gain

        Update the parameter for the controller

        Keyword parameters:
        kd -- the new derivative gain
        """

        self._kd = float(kd)


class MotorPidJumpExample:
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

        It sets up some logging for the parameters which concern us, then
        after a short pause starts a separate thread to drive the motors.

        For height we use the raw ASL (height above sea level) value.  This
        appears to be roughly in metres.  However it relies on a constant for
        pressure at sea level, so weather conditions make a big difference.
        We normalize the value.

        Keyword argument:
        link_uri -- the URI for the Crazyflie (no default)

        """

        # Start the logging.

        # The definition of the logconfig could have been made before
        # connecting, but we do it here, since the rest of the intialization
        # can't be done until we are connected.
        self._lg = LogConfig(name = "Log", period_in_ms = 10);
        self._lg.add_variable("baro.aslRaw", "float");
        self._lg.add_variable("stabilizer.roll", "float")
        self._lg.add_variable("stabilizer.pitch", "float")
        self._lg.add_variable("stabilizer.yaw", "float")
        self._lg.add_variable("stabilizer.thrust", "uint16_t")

        # We want to snag the ASL base as the average over the first 10 calls.
        self._aslBaseCount = 10
        self._aslBase = 0.0

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
        Thread(target=self._pid_jump_motors).start()


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

        #  Log data. Remember pitch and yaw are negative!  First time capture
        #  the ASL starting value
        if self._aslBaseCount > 0:
            self._aslBase      += data['baro.aslRaw'] / 10
            self._aslBaseCount -= 1

        self._timestamp = timestamp
        self._asl       = data['baro.aslRaw'] - self._aslBase
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


    def _pid_jump_motors(self):
        """Make the Crazyflie jump to a controlled height

        Just a burst of the motors with some control
        """

        # Kill parameters
        kill = False
        cutoff = 20.0
        cutoff_en = True
        thrust_limit = 50000

        # Initial values for attitude
        # Roll  - negative makes it go left
        # Pitch - positive makes it go forward
        # Yaw   - ? makes it go which way?
        # ASL   - height in metres above sea level
        rollInit  =  -1.5
        pitchInit =  8.0
        yawInit   =  0.0
        aslInit   =  1.5

        # PID controllers for attitude
        rollCtrl   = PidController(setpoint = rollInit,  kp = 0)
        pitchCtrl  = PidController(setpoint = pitchInit, kp = 0)
        yawCtrl    = PidController(setpoint = yawInit,   kp = 0)

        # PID controller for altitude.
        aslCtrl = PidController(setpoint = aslInit, kp = 0)

        # Parameters to get off the ground.
        thrust_step  =   500
        thrust_start = 20000
        thrust_stop  = 38000

        # Timing
        dt = 0.01
        jumptime = 5
        jumpsteps = int(float(jumptime) / dt)

        # Limits for thrust to avoid extreme oscillation
        thrust_min = 34000
        thrust_max = 48000

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Rapidly wind up
        for t in range(thrust_start, thrust_stop, thrust_step):
            self._cf.commander.send_setpoint(rollInit, pitchInit, yawInit, t*0.0)
            time.sleep(0.01)

        roll   = rollInit
        pitch  = pitchInit
        yaw    = yawInit
        thrust = thrust_stop

        # Control loop
        for i in range (1, jumpsteps, 1):
            roll   = rollInit  - rollCtrl.control(self._roll, dt)
            pitch  = pitchInit - pitchCtrl.control(self._pitch, dt)
            yaw    = yawInit   - yawCtrl.control(self._yaw, dt)

            thrust = thrust_stop - int(aslCtrl.control(self._asl, dt))

            if thrust > thrust_max:
                thrust = thrust_max

            if thrust < thrust_min:
                thrust = thrust_min

            if self._debug > 0:
                print "Correcting: (%f %f %f) %d"  % (roll, pitch, yaw,
                                                      thrust)

            now = float(self._timestamp) / 1000.0
            print "%f,%f,%f,%f,%f,%f,%f,%f,%d" % (now, self._roll, roll,
                                                  self._pitch, pitch,
                                                  self._yaw, yaw,
                                                  self._asl,
                                                  thrust)

            if (cutoff_en and (abs(self._roll)  > cutoff or
                               abs(self._pitch) > cutoff)):
                print "Cutoff attitude (%f %f)" % (self._roll, self._pitch)
                kill = True

            if (cutoff_en and (thrust > thrust_limit or
                               thrust < 0)):
                print "Cutoff thrust (%d)" % (thrust)
                kill = True;

            if kill:
                # Die politely
                print "Kill switch"
                self._cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(1.0)
                self._cf.close_link()
            else:
                self._cf.commander.send_setpoint(roll,
                                                 pitch,
                                                 yaw,
                                                 thrust * 0.0)

            # Wait a little bit
            time.sleep(dt)

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
        le = MotorPidJumpExample(link_uri = available[0][0], debug = 0)
    else:
        print "No Crazyflies found, cannot run example"
