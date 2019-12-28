import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float64, Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from serial import Serial, serialutil


from hexapod2.hexi import hardware, kinematics

class MockSerial:
    def write(self, param):
        pass

    def flush(self):
        pass

class LegSubNode():
    def __init__(self, name, node, leg):
        self.name = name
        self.hardwareLeg = leg
        self.kinematicLeg = kinematics.KinematicLeg(leg)

        self.subscription_coxa = node.create_subscription(
            Float64,
            name + '/coxa_controller/command',
            self.commandCallbackCoxa,
            10)

        self.subscription_femur = node.create_subscription(
            Float64,
            name + '/femur_controller/command',
            self.commandCallbackFemur,
            10)

        self.subscription_tibia = node.create_subscription(
            Float64,
            name + '/tibia_controller/command',
            self.commandCallbackTibia,
            10)

        self.subscription_position = node.create_subscription(
            Point,
            name + '/position_controller/command',
            self.commandCallbackPosition,
            10
        )

    def commandCallbackCoxa(self, msg):
        self.hardwareLeg.coxa_angle = msg.data
    
    def commandCallbackFemur(self, msg):
        self.hardwareLeg.femur_angle = msg.data
    
    def commandCallbackTibia(self, msg):
        self.hardwareLeg.tibia_angle = msg.data

    def commandCallbackPosition(self, msg):
        self.kinematicLeg.setPosition(msg.x, msg.y, msg.z)

class HardwareNode(Node):
    def __init__(self):
        super().__init__('hexapod_hardware')
        self.legSubNodes = []
        self.set_parameters_callback(self.parametersChanged)

        self.declare_parameter('comport')

        comPort = self.get_parameter('comport').value

        if(comPort == None):
            serial = MockSerial()
        else:
            try:
                serial = Serial(comPort, 115200, timeout=0)
            except serialutil.SerialException as e:
                serial = MockSerial()
        
        self.hardware = hardware.Hardware(serial)

        legNames = ['fl', 'ml', 'rl', 'fr', 'mr', 'rr']
        
        for i,leg in enumerate(self.hardware.legs):
            self.legSubNodes.append(LegSubNode(legNames[i], self, leg))
            self.declare_parameter(legNames[i] + "_coxa_offset", 0.0)
            self.declare_parameter(legNames[i] + "_femur_offset", 0.0)
            self.declare_parameter(legNames[i] + "_tibia_offset", 0.0)
            self.declare_parameter(legNames[i] + "_coxa_dir", 1)
            self.declare_parameter(legNames[i] + "_femur_dir", 1)
            self.declare_parameter(legNames[i] + "_tibia_dir", 1)

        self.sensorPub = self.create_publisher(JointState, 'joint_states', 10)

        self.create_timer(0.01, self.updateHardware)
        self.create_timer(0.01, self.publishJointStates)


    def parametersChanged(self, parameterList):
        r = SetParametersResult(successful=True)
        for param in parameterList:
            legName = param.name[:2]
            for leg in self.legSubNodes:
                if(legName == leg.name):
                    try:
                        #print("Trying to set " + param.name[3:] + " base: " + param.name)
                        getattr(leg.hardwareLeg, param.name[3:])
                        setattr(leg.hardwareLeg, param.name[3:], param.value)
                    except AttributeError as error:
                        pass
                    break
        return r

    def updateHardware(self):
        self.hardware.update()

    def publishJointStates(self):
        jointStateMsg = JointState()
        jointStateMsg.header = Header()
        jointStateMsg.header.stamp = self.get_clock().now().to_msg()
        for leg in self.legSubNodes:
            jointStateMsg.name.append("base_to_" + leg.name + "_coxa")
            jointStateMsg.position.append(leg.hardwareLeg.coxa_angle)
            jointStateMsg.name.append(leg.name + "_coxa_to_femur")
            jointStateMsg.position.append(leg.hardwareLeg.femur_angle)
            jointStateMsg.name.append(leg.name + "_femur_to_tibia")
            jointStateMsg.position.append(leg.hardwareLeg.tibia_angle)
            
        self.sensorPub.publish(jointStateMsg)
        pass

def main(args=None):
    rclpy.init(args=args)

    hardwareNode = HardwareNode()

    rclpy.spin(hardwareNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hardwareNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()