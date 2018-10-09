#!/usr/bin/env python

#      /pose - Current pose
#      /go_to_waypoint/goal - Setpoint
#      /on_ground - To check if its flying



import rospy
from geometry_msgs.msg import PoseStamped as ps
from geometry_msgs.msg import Pose as p
from geometry_msgs.msg import Point as po
from geometry_msgs.msg import Quaternion as q
from std_msgs.msg import Header as h

from snav_msgs.msg import GoToWaypointActionGoal as wp
from snav_msgs.msg import TakeoffActionGoal as tag
from snav_msgs.msg import LandActionGoal as lag

import math
import numpy as np
from matplotlib import pyplot as plt
import time

class errorLogger:
    def __init__(self):

        self.nh = rospy.init_node("errorCheck")

        self. errorPub    = rospy.Publisher("/error/current", ps, queue_size=10)
        self.errorMeanPub = rospy.Publisher("/error/mean", ps, queue_size=10)
        self.errorStdPub  = rospy.Publisher("/error/STD", ps, queue_size=10)
        self.currentPub   = rospy.Publisher("/error/sp", ps, queue_size=10)

        self.currentSP = ps()


        self.toMode = 0 #                            0 - Landed, 1- Flying

        self.dataSize = 0
        self.prevErrors = []

        self.error  = []
        self.mean   = []
        self.std    = []
        self.setPnt = []
        self.state  = []
        self.hedrs  = []

        rospy.Subscriber("/pose", ps, self.poseCB)
        rospy.Subscriber("/go_to_waypoint/goal", wp, self.setPointCB)
        rospy.Subscriber("/takeoff/goal", lag, self.takeoffCB)
        rospy.Subscriber("/land/goal", tag, self.landCB)

        rospy.spin()


    def poseCB(self, data):
        if self.toMode == 1:
            errorPos   = ps() #keeps the header the same
            errMeanPos = ps() #we can use the timestamp to identify which packet it came from
            errStdPos  = ps()
            currPos    = ps()

            errorPos.header = data.header
            errMeanPos.header = data.header
            errStdPos.header = data.header
            currPos.header = data.header

            currPos.pose.position.x = data.pose.position.x#self.currentSP.pose.position.x
            currPos.pose.position.y = data.pose.position.y#self.currentSP.pose.position.y
            currPos.pose.position.z = data.pose.position.z#self.currentSP.pose.position.z

            errorPos.pose.position.x = data.pose.position.x - self.currentSP.pose.position.x
            errorPos.pose.position.y = data.pose.position.y - self.currentSP.pose.position.y
            errorPos.pose.position.z = data.pose.position.z - self.currentSP.pose.position.z

            self.prevErrors.append(errorPos)
            self.dataSize += 1

            rospy.logerr(self.dataSize)

            sumX = []#np.array()
            sumY = []#np.array()
            sumZ = []#np.array()
            for prevErr in self.prevErrors:
                sumX.append(prevErr.pose.position.x)
                sumY.append(prevErr.pose.position.y)
                sumZ.append(prevErr.pose.position.z)

            sumX = np.array(sumX)
            sumY = np.array(sumY)
            sumZ = np.array(sumZ)

            errMeanPos.pose.position.x = np.mean(sumZ)
            errMeanPos.pose.position.y = np.mean(sumZ)
            errMeanPos.pose.position.z = np.mean(sumZ)

            errStdPos.pose.position.x = np.std(sumX)
            errStdPos.pose.position.y = np.std(sumY)
            errStdPos.pose.position.z = np.std(sumZ)

            self.state.append(currPos.pose.position)
            self.setPnt.append(self.currentSP.pose.position)
            self.error.append(errorPos.pose.position)
            self.mean.append(errMeanPos.pose.position)
            self.std.append(errStdPos.pose.position)
            self.hedrs.append(data.header)

            self.errorPub.publish(errorPos)
            self.errorMeanPub.publish(errMeanPos)
            self.errorStdPub.publish(errStdPos)
            self.currentPub.publish(self.currentSP)

        return

    def setPointCB(self, data):
        print("Got Setpoint")
        self.currentSP.pose.position = data.goal.position
        del self.prevErrors[:]
        return

    def takeoffCB(self, data):
        del self.prevErrors[:]
        del self.error[:]
        del self.mean[:]
        del self.std[:]
        del self.setPnt[:]
        del self.state[:]
        self.toMode = 1
        print("Got takeoff")
        rospy.logerr("Got takeoff")
        self.currentSP.pose.position.x = 0.0
        self.currentSP.pose.position.y = 0.0
        self.currentSP.pose.position.z = 1.5
        self.dataSize = 0



        return

    def landCB(self, data):
        print("Got Land")
        rospy.logerr("Got land")
        self.toMode = 0
        time.sleep(1)
        self.plotData()
        return

    def plotData(self):
        posX = []
        posY = []
        posZ = []
        rospy.logerr("POS: " + str(len(self.state)))
        for pos in self.state:
            posX.append(pos.x)
            posY.append(pos.y)
            posZ.append(pos.z)

        spX = []
        spY = []
        spZ = []
        rospy.logerr("SP: " + str(len(self.setPnt)))
        for sp in self.setPnt:
            spX.append(sp.x)
            spY.append(sp.y)
            spZ.append(sp.z)

        errX = []
        errY = []
        errZ = []
        rospy.logerr("ERR: " + str(len(self.error)))
        for err in self.error:
            errX.append(err.x)
            errY.append(err.y)
            errZ.append(err.z)

        mnX = []
        mnY = []
        mnZ = []
        rospy.logerr("Mean: " + str(len(self.mean)))
        for mn in self.mean:
            mnX.append(mn.x)
            mnY.append(mn.y)
            mnZ.append(mn.z)

        stdX = []
        stdY = []
        stdZ = []
        rospy.logerr("std: " + str(len(self.std)))
        for std in self.std:
            stdX.append(std.x)
            stdY.append(std.y)
            stdZ.append(std.z)

        tmr = []
        rospy.logerr("tmr: " + str(len(self.hedrs)))
        for tme in self.hedrs:
            tmr.append((tme.stamp.secs + tme.stamp.nsecs / 1000000000) - (self.hedrs[0].stamp.secs + self.hedrs[0].stamp.nsecs / 1000000000))


        rospy.logerr("posX: " + str(len(posX)) + " tmr: " + str(len(tmr)))


        plt.figure(1)
        plt.subplot(211)
        plt.plot(tmr, posZ, 'k', label='Pos Z')
        plt.plot(tmr, spZ, 'r--', label='SP Z')
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
        plt.subplot(212)
        plt.plot(tmr, errZ, 'b--', label='err Z')
        plt.plot(tmr, mnZ, 'k', label='mean Z')
        plt.plot(tmr, stdZ, 'r--', label='STD Z')
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)

        plt.figure(2)
        plt.subplot(211)
        plt.plot(tmr, posY, 'k', label='Pos Y')
        plt.plot(tmr, spY, 'r--', label='SP Y')
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
        plt.subplot(212)
        plt.plot(tmr, errY, 'b--', label='err Y')
        plt.plot(tmr, mnY, 'k', label='mean Y')
        plt.plot(tmr, stdY, 'r--', label='STD Y')
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)

        plt.figure(3)
        plt.subplot(211)
        plt.plot(tmr, posX, 'k', label='Pos X')
        plt.plot(tmr, spX, 'r--', label='SP X')
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
        plt.subplot(212)
        plt.plot(tmr, errX, 'b--', label='err X')
        plt.plot(tmr, mnX, 'k', label='mean X')
        plt.plot(tmr, stdX, 'r--', label='STD X')
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)

        plt.show()

el = errorLogger()


