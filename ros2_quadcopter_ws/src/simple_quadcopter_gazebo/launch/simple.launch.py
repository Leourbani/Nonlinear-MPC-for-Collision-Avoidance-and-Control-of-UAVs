import launch
import launch.actions
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    pkg_share = FindPackageShare("simple_quad")    

    gazebo_model_path_env = launch.actions.SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            PathJoinSubstitution([pkg_share, 'models']),
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ]
    )



    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]),
    )

    robot_description = {
        "robot_description": launch.substitutions.Command(
            ["xacro ", PathJoinSubstitution([pkg_share, "urdf", "simple_quad.urdf.xacro"])]
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "simple_quad", "-topic", "robot_description", "-z", "0.0"],
        output="screen",
    )

    spawn_sphere_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "my_moving_sphere",
            "-file", PathJoinSubstitution([pkg_share, "models", "my_moving_sphere", "model", "model.sdf"]), 
            "-x", "15.0", "-y", "0.0", "-z", "2.0"], # TO MODIFY IF SPHERE STARTING POSITION CHANGES 
        output="screen",
    )


    return LaunchDescription(
        [
            gazebo_model_path_env,
            gazebo,
            rsp,
            spawn_entity,
            spawn_sphere_entity
        ]
    )
