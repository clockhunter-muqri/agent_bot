import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='agent_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    # rspm = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','online_async_launch.py'
    #             )]), launch_arguments={'use_sim_time': 'false'}.items()
    # )

    # Include the Gazebo launch file, provided by the gazebo_ros package

    # gazebo_params_path = os.path.join(
    #               get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #                 launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path }.items()
    #         )

    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #          )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'my_bot'],
    #                     output='screen')
    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params = os.path.join(get_package_share_directory('agent_bot'),
                                     'config','my_controllers.yaml')
    
    controller_manager = Node(package='controller_manager',
                              executable='ros2_control_node',
                              parameters=[{'robot_description': robot_description},
                                          controller_params],)
    
    delayed_controller_manager = TimerAction(period=1.0,
                                             actions=[controller_manager])


    diff_drive_spawner = Node(package="controller_manager",
                              executable="spawner",
                              arguments=["diff_cont"],)
    
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(package="controller_manager",
                               executable="spawner",
                               arguments=["joint_broad"],)
    
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        # gazebo,
        # spawn_entity,
        # rspm,
        # diff_drive_spawner,
        # joint_broad_spawner,
    ])
