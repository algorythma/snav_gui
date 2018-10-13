#!/usr/bin/env python

import rospy
import roslaunch

import time

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

noDrones = 1
with open('/home/vradhakrishnan/workspace/configs/noMasters.cfg') as fle:
    noDrones = int(fle.readline())

waypointMarkerCmd = []
lfWaypoint = []
lfArgs = []
for i in range(noDrones):
    waypointMarkerCmd = ['snav_gui', 'swarm_waypoint.launch', 'nameSpace:=drone'+ '%02d' % (i+1,), 'nodeName:=dr' + '%02d' % (i+1,)]
    lfWaypoint.append(roslaunch.rlutil.resolve_launch_arguments(waypointMarkerCmd))
    lfArgs.append(waypointMarkerCmd[2:])
lf = []

print ("!!!!!!!!!!!!!!!!!!!!!1")
print (roslaunch.rlutil.resolve_launch_arguments(['snav_gui', 'swarm_waypoint.launch', 'nameSpace:=/drone'+ "%02d" % (0+1,)]))

for i in range(noDrones):
    lf.append((lfWaypoint[i][0], lfArgs[i]))
    print("AT: " + str(i))
    print ((lfWaypoint[i]))
    print ((lfArgs[i]))

print(lf)

parent = roslaunch.parent.ROSLaunchParent(uuid, lf)
print("Going to start")

parent.start()
while not rospy.is_shutdown():
    time.sleep(1)
print("Start Finished")