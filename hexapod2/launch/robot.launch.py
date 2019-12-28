import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os.path

from ros2_launch_util import *

def generate_launch_description():
    params_path = os.path.join(get_package_share_directory('hexapod2'), 'config', 'hexapod_hardware.yaml')
    print(params_path)
    ld =  launch.LaunchDescription([
        launch_ros.actions.Node(
            package='hexapod2', node_executable='hardwarenode',
            node_name='hexapod_hardware', parameters=[params_path]),

        #launch_ros.actions.Node(
        #    package='joint_state_publisher', node_executable='joint_state_publisher', output='screen',
        #    node_name='joint_state_publisher', parameters=[{'robot_description': urdf, 'use_gui': True }]),
    ])

    return ld