import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='agent_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    rspm = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','online_async_launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params= os.path.join(
                  get_package_share_directory(package_name),'config','twist_mux.yaml')
    
    # twist_mux_params = '/home/ubuntu-ros/ros_ws/src/agent_bot/config/twist_mux.yaml'

    twist_mux =Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_params_path = os.path.join(
                  get_package_share_directory(package_name),'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
            )

    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #          )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'agent_bot'],
                        output='screen')
    
    diff_drive_spawner = Node(package="controller_manager",executable="spawner",arguments=["diff_cont"],)

    joint_broad_spawner = Node(package="controller_manager",executable="spawner",arguments=["joint_broad"],)

    rviz_launch =  Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join('/home/ubuntu-ros/ros_ws/src/agent_bot', 'config', 'map.rviz')]],
            output = 'screen'
        )
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        # nav2_go_to_point,
        gazebo,
        spawn_entity,
        rspm,
        diff_drive_spawner,
        joint_broad_spawner,
        twist_mux,
        rviz_launch,
    ])
