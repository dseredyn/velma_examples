#!/usr/bin/env python

# Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('velma_examples')

import rospy
import tf

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np
import copy

from velma import Velma

class ExampleControl:

    def __init__(self):
        pass

    def spin(self):

        print "creating interface for Velma..."
        # create the interface for Velma robot
        velma = Velma()
        rospy.sleep(0.5)
        print "done."

        hv = [1.2, 1.2, 1.2, 1.2]
        ht = [3000, 3000, 3000, 3000]
        velma.moveHandLeft([15.0/180.0*math.pi, 15.0/180.0*math.pi, 15.0/180.0*math.pi, 0], hv, ht, 5000, True)
        print "wainting for moveHandLeft complete..."
        result = velma.waitForHandLeft()
        print "result:", result.error_code

        velma.moveHandRight([15.0/180.0*math.pi, 15.0/180.0*math.pi, 15.0/180.0*math.pi, 0], hv, ht, 5000, True)
        print "wainting for moveHandRight complete..."
        result = velma.waitForHandRight()
        print "result:", result.error_code

        velma.moveHandLeft([70.0/180.0*math.pi, 70.0/180.0*math.pi, 70.0/180.0*math.pi, 0], hv, ht, 5000, True)
        print "wainting for moveHandLeft complete..."
        result = velma.waitForHandLeft()
        print "result:", result.error_code

        velma.switchToCart()

        velma.updateTransformations()
        T_B_Wr1 = velma.T_B_Wr
        T_B_Wr2 = PyKDL.Frame(PyKDL.Vector(0.2,0,0)) * T_B_Wr1

        velma.moveWristRight(T_B_Wr2, 3.0, Wrench(Vector3(40,40,40), Vector3(14,14,14)), start_time=0.1)
        print "wainting for moveWristRight complete..."
        result = velma.waitForWristRight()
        print "result:", result.error_code

        velma.moveWristRight(T_B_Wr1, 3.0, Wrench(Vector3(40,40,40), Vector3(14,14,14)), start_time=0.1)
        print "wainting for moveWristRight complete..."
        result = velma.waitForWristRight()
        print "result:", result.error_code

        velma.updateTransformations()
        T_B_Wl1 = velma.T_B_Wl
        T_B_Wl2 = PyKDL.Frame(PyKDL.Vector(0,0,0.2)) * T_B_Wl1

        velma.moveWristLeft(T_B_Wl2, 3.0, Wrench(Vector3(40,40,40), Vector3(14,14,14)), start_time=0.1)
        print "wainting for moveWristLeft complete..."
        result = velma.waitForWristLeft()
        print "result:", result.error_code

        velma.moveWristLeft(T_B_Wl1, 3.0, Wrench(Vector3(40,40,40), Vector3(14,14,14)), start_time=0.1)
        print "wainting for moveWristLeft complete..."
        result = velma.waitForWristLeft()
        print "result:", result.error_code

        raw_input("Press ENTER to exit...")

if __name__ == '__main__':

    rospy.init_node('example_control')

    task = ExampleControl()
    task.spin()


