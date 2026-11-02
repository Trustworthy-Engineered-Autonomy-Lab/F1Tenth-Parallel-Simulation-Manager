"""Run the complete ego-defense/opponent-overtake pipeline with visible failures."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def pipeline_node(**kwargs):
    return Node(output='screen', **kwargs)


def generate_launch_description():
    params = '/sim_ws/src/lidar_processing/config/params_dynamic.yaml'
    nodes = [
        pipeline_node(package='frenet_frame_conv', executable='frenet_node'),
        pipeline_node(package='overtake_controller', executable='imm_filter'),
        pipeline_node(package='overtake_controller', executable='interceptor'),
        pipeline_node(package='overtake_controller', executable=LaunchConfiguration('controller')),
        pipeline_node(package='lidar_processing', executable='sim_opp_frenet.py'),
        pipeline_node(package='lidar_processing', executable='imm.py',
             remappings=[('/imm_path', '/defensive_imm_path')]),
        pipeline_node(package='lidar_processing', executable='generate_candidate_trajectories.py'),
        pipeline_node(package='lidar_processing', executable='path_converter_node.py', parameters=[params]),
        pipeline_node(package='lidar_processing', executable='dynamic_mppi_node.py', parameters=[params]),
        pipeline_node(package='env_manager', executable='main'),
        pipeline_node(package='data_logger', executable='data_logger'),
    ]
    actions = [DeclareLaunchArgument('controller', default_value='overtake')]
    # A dead planner must not leave the cars silently circling on fallback control.
    for node in nodes:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='A maneuver pipeline node exited; inspect its log.'))])))
    return LaunchDescription(actions + nodes)
