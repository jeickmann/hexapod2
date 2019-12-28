import py_trees

from geometry_msgs.msg import Point

class InitSleepPos(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="GoToSleep"):
        super().__init__(name)
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

    def setup(self):
        self.legPublishers = []
        legNames = ['fl', 'ml', 'rl', 'fr', 'mr', 'rr']
        for legName in legNames:
            self.legPublishers.append(self.node.create_publisher(Point, legName + '/position_controller/command', 10))
        
    def update(self):
        for i,pub in enumerate(self.legPublishers):
            pub.publish(self.sleepPos[i])
        
        return py_trees.common.Status.SUCCESS