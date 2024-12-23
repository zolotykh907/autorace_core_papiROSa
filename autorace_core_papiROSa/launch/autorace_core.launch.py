import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_autorace_core = get_package_share_directory('autorace_core_papiROSa')
    pkg_autorace_camera = get_package_share_directory('autorace_camera')

    
    camera_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_autorace_camera, 'launch', 'extrinsic_camera_calibration.launch.py')))
    
    base_config = os.path.join(pkg_autorace_core, 'config', 'base.yaml')
    intersection_config = os.path.join(pkg_autorace_core, 'config', 'intersection.yaml')
    obstacles_config = os.path.join(pkg_autorace_core, 'config', 'obstacles.yaml')

    sign_detection = Node(
        package = 'autorace_core_papiROSa',
        executable = 'sign_detection',
        name = 'sign_detection',
        parameters=[
            {'model_path': os.path.join(pkg_autorace_core, 'model')},
            base_config])
    
    lane_detect = Node(
        package = 'autorace_core_papiROSa',
        executable = 'line_detect',
        name = 'line_detect',
        parameters = [base_config])
    
    lane_follow = Node(
        package = 'autorace_core_papiROSa',
        executable = 'line_follow',
        name = 'line_follow',
        parameters = [base_config])
    
    robot_rotator = Node(
        package = 'autorace_core_papiROSa',
        executable = 'robot_rotator',
        name = 'robot_rotator')
    
    traffic_light = Node(
        package = 'autorace_core_papiROSa',
        executable = 'traffic_light',
        name = 'traffic_light')
    
    intersection = Node(
        package = 'autorace_core_papiROSa',
        executable = 'intersection',
        name = 'intersection',
        parameters = [intersection_config])


    return LaunchDescription([
        camera_calibration,
        sign_detection,
        lane_detect,
        lane_follow,
        robot_rotator,
        traffic_light,
        intersection,


     ])
