#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryActionGoal

import os, time
import struct

from machinekit import hal
import linuxcnc
from linuxcnc_control import LinuxcncControl

# print ring properties
def print_ring(r):
    print "name=%s size=%d reader=%d writer=%d scratchpad=%d" % (r.name,r.size,r.reader,r.writer,r.scratchpad_size),
    print "use_rmutex=%d use_wmutex=%d type=%d in_halmem=%d" % (r.rmutex_mode, r.wmutex_mode,r.type,r.in_halmem)

# retrieve list of ring names
rings = hal.rings()
print "rings: ", rings

# Message details:
msg_fmt  = '=2l6d'
msg_size = struct.calcsize(msg_fmt)

# Global variable for ring handle
w = 0
pi2deg = 180.0 / 3.14159265359
e = LinuxcncControl(1)
e.g_raise_except = False
if (e.ok_for_mdi() == False):
    print "ERROR: not ready for MDI, quit ..."
    quit()
else:
    print "Prepare for MDI ..."
    e.prepare_for_mdi()
print "ready to issue MDI commands ..."

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print 'In callback'
    global e
    if (e.ok_for_mdi() == False):
        print "ERROR: not ready for MDI, quit ..."
        quit()
    else:
        print "Prepare for MDI ..."
        e.prepare_for_mdi()
    print "ready to issue MDI commands ..."
    e.g("G21 F1500")
    # print data.goal.trajectory.points.time_from_start.secs
    # print data.goal.trajectory.points.time_from_start.nsecs
    # print data.goal.trajectory.points[0].positions
    prev_t = data.goal.trajectory.points[0].time_from_start.to_sec()
    gcode_array = []
    for point in data.goal.trajectory.points:
        # print point.time_from_start
        # print point.time_from_start.secs
        # print point.time_from_start.nsecs
        # print ["{0:0.2e}".format(i) for i in point.positions]
        print "dt: %.5f" % float(point.time_from_start.to_sec() - prev_t)
        global pi2deg
        # print "G1 X%.5f Y%.5f Z%.5f A%.5f B%.5f C%.5f" % tuple([p * pi2deg for p in point.positions])
        gcode = "G1 X%.5f Y%.5f Z%.5f A%.5f B%.5f C%.5f" % tuple([p * pi2deg for p in point.positions])
        gcode_array.append(gcode)
        prev_t = point.time_from_start.to_sec()
        
    for gcode in gcode_array:
        print gcode
        e.g(gcode)

    # for pose in data.points:
    #     # print pose.positions[0]
    #     print pose.time_from_start
    #     print ["{0:0.2e}".format(i) for i in pose.positions]

    #     if w.available > msg_size:
    #         msg = struct.pack(msg_fmt, pose.time_from_start.secs,
    #                                    pose.time_from_start.nsecs,
    #                                    pose.positions[0],
    #                                    pose.positions[1],
    #                                    pose.positions[2],
    #                                    pose.positions[3],
    #                                    pose.positions[4],
    #                                    pose.positions[5] )
    #         w.write(msg)

    # # investigate scratchpad region if one is defined
    # if w.scratchpad_size:
    #     print "scratchpad:%d = '%s'" % (w.scratchpad_size, w.scratchpad.tobytes())


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    if "jointpos" in rings:

        # attach to existing ring
        global w
        w = hal.Ring("jointpos")

        # see what we have
        print_ring(w)

    # rospy.Subscriber("/joint_path_command", JointTrajectory, callback)
    rospy.Subscriber("/rrbot/position_trajectory_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, callback)
    print 'after Subscriber'

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    print 'after spin'

if __name__ == '__main__':
    print 'HelloThere'
    listener()
