import py_trees

import rclpy
from rclpy.node import Node

from .wakeup import InitSleepPos
from .weightlifting import WeightLifting

class ControllerNode(Node):
    def __init__(self):
        super().__init__('hexapod_controller')

        self.tree = py_trees.composites.Sequence()

        self.tree.add_child(py_trees.decorators.OneShot(InitSleepPos(self)))
        self.tree.add_child(WeightLifting(self))

        self.tree.setup_with_descendants()
        
        self.timer = self.create_timer(0.01, self.tree.tick_once)

def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()