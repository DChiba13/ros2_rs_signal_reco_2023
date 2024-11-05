from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  pkg_prefix = get_package_share_directory('ros2_rs_signal_reco_2023')
  g_signal_node = Node(
    package = 'ros2_rs_signal_reco_2023',
    executable='rs_signal_reco_sub',
    parameters = [
      join(pkg_prefix, 'cfg/parameter.yaml')
    ],
    remappings=[
      ('camera1/image', 'signal_test')
    ],
  )
  return LaunchDescription([
    g_signal_node
  ])
