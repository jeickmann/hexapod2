import py_trees

import tf2_ros
import rclpy

class ReadLegPos(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="ReadLegPos"):
        super().__init__(name)
        self.node = node

    def setup(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, node=self.node)
        
    def update(self):
        try:
            trans = self.tfBuffer.lookup_transform('fl_coxa', 'fl_foot', rclpy.time.Time())
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)

        return py_trees.common.Status.RUNNING