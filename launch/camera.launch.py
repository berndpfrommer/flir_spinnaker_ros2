#  ----------------------------------------------------------------------------
#  2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#
from launch import LaunchDescription
from launch_ros.actions import Node

#            prefix=['gdb -ex=r --args'],
def generate_launch_description():
    return LaunchDescription([
        Node(package='flir_spinnaker_ros2',
             namespace='flir',
             executable='flir_spinnaker_ros2',
             name='camera_node')])
