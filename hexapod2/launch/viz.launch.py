import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from ros2_launch_util import *

def generate_launch_description():
    urdfFile = xacro_to_urdf("hexapod2", "urdf", "hexapod.urdf.xacro")
    
    file = open(urdfFile,mode='r')
    urdf = file.read()
    file.close()

    ld =  launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2', node_executable='rviz2',
            node_name='rviz2'),

        #launch_ros.actions.Node(
        #    package='joint_state_publisher', node_executable='joint_state_publisher', output='screen',
        #    node_name='joint_state_publisher', parameters=[{'robot_description': urdf, 'use_gui': True }]),
    ])

    return ld