import py_trees
import rclpy

from geometry_msgs.msg import Point
from hexapod2.controller.wakeup import InitSleepPos

class WeightLifting(InitSleepPos):
    def __init__(self, node, name="WeightLifting"):
        super().__init__(node, name)
        self.sleepPos = []

        for i in range(6):
            point = Point()
            point.x = 0.0
            point.y = 0.08
            point.z = -0.01
            self.sleepPos.append(point)
        
        self.sleepPos[0].x = self.sleepPos[3].x = 0.04
        self.sleepPos[2].x = self.sleepPos[5].x = -0.04

        self.node = node

        self.topHeight = -0.03
        self.bottomHeight = -0.06
        self.interval = rclpy.duration.Duration(seconds=1)
        self.movingUp = True
        

    def initialise(self):
        self.movementStart = self.node.get_clock().now()
        
    def update(self):
        duration = self.node.get_clock().now() - self.movementStart
        diff = (self.topHeight - self.bottomHeight) / self.interval.nanoseconds * duration.nanoseconds

        if(self.movingUp):
            currentHeight = self.bottomHeight + diff
        else:
            currentHeight = self.topHeight - diff 

        print(currentHeight)
        
        if(duration > self.interval):
            self.movementStart = self.node.get_clock().now()
            self.movingUp = not(self.movingUp)

        for i,pub in enumerate(self.legPublishers):
            self.sleepPos[i].z = currentHeight
            pub.publish(self.sleepPos[i])
        
        return py_trees.common.Status.RUNNING