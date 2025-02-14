#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def set_traj_defaults(context, *args, **kwargs):
    # Read trajectory_type from launch configuration:
    traj = LaunchConfiguration('trajectory_type').perform(context)
    if traj == 'circle':
        x_val = '0.0'
        y_val = '5.0'
        yaw_val = '0'
    elif traj == 'epitrochoid':
        x_val = '3.0'
        y_val = '0.01'
        yaw_val = '-1.57'
    elif traj == 'square':
        x_val = '0.0'
        y_val = '0.0'
        yaw_val = '1.57075'
    elif traj == 'infinite':
        x_val = '10.0'
        y_val = '0.0'
        yaw_val = '1.57075'
    else:
        x_val = '0.0'
        y_val = '0.0'
        yaw_val = '0'
    # Return new declarations for x_pos, y_pos and yaw.
    return [
        DeclareLaunchArgument('x_pos', default_value=x_val),
        DeclareLaunchArgument('y_pos', default_value=y_val),
        DeclareLaunchArgument('yaw', default_value=yaw_val)
    ]

def generate_launch_description():
    ld = LaunchDescription()

    # Global parameter for simulation time
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))

    # Declare core launch arguments.
    ld.add_action(DeclareLaunchArgument(
        'controller', default_value='mpc',
        description='opt: dwa, mpc, pure_pursuit'))
    ld.add_action(DeclareLaunchArgument(
        'model', default_value='serving_bot',
        description='opt: serving_bot'))
    ld.add_action(DeclareLaunchArgument(
        'trajectory_type', default_value='circle',
        description='opt: circle, epitrochoid, square, infinite'))
    ld.add_action(DeclareLaunchArgument('gui', default_value='false'))

    # Simulator/Gazebo related arguments
    ld.add_action(DeclareLaunchArgument('z_pos', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('roll', default_value='0'))
    ld.add_action(DeclareLaunchArgument('pitch', default_value='0'))

    # Declare dummy default values for x_pos, y_pos, and yaw.
    ld.add_action(DeclareLaunchArgument('x_pos', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y_pos', default_value='5.0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0'))

    # Adjust x_pos, y_pos, yaw based on the trajectory_type argument.
    ld.add_action(OpaqueFunction(function=set_traj_defaults))

    # Node: Trajectory Generation (only if controller is "mpc")
    controller = LaunchConfiguration('controller')
    ld.add_action(Node(
        package='mpc_ros',
        executable='mpc_trajectory_generation_ros2.py',
        name='mpc_trajectory_generation',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'mpc'"])),
        parameters=[{'trajectory_type': LaunchConfiguration('trajectory_type')}]
    ))

    # Node: MPC Tracking Node (only if controller is "mpc")
    # In ROS 2, the parameter file should be given via its full path.
    pkg_share = get_package_share_directory('mpc_ros')
    mpc_param_file = os.path.join(pkg_share, 'params', 'mpc_local_params.yaml')
    ld.add_action(Node(
        package='mpc_ros',
        executable='tracking_reference_trajectory',
        name='MPC_tracking',
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'mpc'"])),
        parameters=[mpc_param_file]
    ))

    # Node: RViz visualization
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz_navigation.rviz')
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config]
    ))

    return ld

if __name__ == '__main__':
    generate_launch_description()
