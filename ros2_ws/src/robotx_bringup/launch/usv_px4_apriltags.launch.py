from launch import LaunchDescription
from pathlib import Path
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def launch(context, *args, **kwargs):
    # Add the arguments required for both the USV-px4 launcher and the apritlag spawner:
    # Competation and PX4 launching:
    world_name = LaunchConfiguration('world').perform(context)
    urdf = LaunchConfiguration('urdf').perform(context)
    px4_model = LaunchConfiguration('px4_model').perform(context)
    px4_model_pose = LaunchConfiguration('px4_model_pose').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    # AprilTag Implementation:
    position_apriltag = LaunchConfiguration('pos_apriltag').perform(context)
    apriltag_name = LaunchConfiguration('apriltag_name').perform(context)

    # Define a vector contianing all laucnh process:
    launch_process = []
    # Node to add tHE APRILtAGS:
    add_april = Node(
        package='robotx_bringup',
        executable='prep_wamv_with_tags',
        output='screen',
        parameters=[{
            'add_apriltags': LaunchConfiguration('add_apriltags'),
            'num_apriltags':  LaunchConfiguration('num_apriltags'),
            'position_respect_wamv_center': position_apriltag,
            'apriltag_name': apriltag_name,
            'urdf': urdf,
            'sim_mode': sim_mode,
        }]
    )
    launch_process.append(add_april)


    # Create the main launch file
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.getenv('AMENT_PREFIX_PATH').split(':')[0],  # or use get_package_share_directory(...)
                'share', 'robotx_bringup', 'launch', 'main_usv_px4.launch.py'
            )
        ),
        launch_arguments={'world': world_name,
                          'px4_model': px4_model,
                          'px4_model_pose':px4_model_pose,
                          'urdf': urdf,
                          'sim_mode': sim_mode}.items(),
    )
    # Make the launch file run after the NOde shutdowns:
    start_launch = RegisterEventHandler(
        OnProcessExit(
            target_action=add_april,
            on_exit=[main_launch]
        )
    )
    launch_process.append(start_launch)

    # Return the launch processes:
    return launch_process


    



# Create the laucnh description:
def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments:
        # world name
        DeclareLaunchArgument(
            'world',
            default_value='sydney_regatta',
            description='Name of world'),
        # PX4 model:
        DeclareLaunchArgument(
            'px4_model',
            default_value='x500_mono_cam',
            description='MOdel used from the PX4 library.'
        ),
        # PX4 position:
        DeclareLaunchArgument(
            'px4_model_pose',
            default_value='-540,140,1.76,0,0,0',
            description='PX4_GZ_MODEL_POSE as "x,y,z,roll,pitch,yaw"'
        ),
        # URDF file of WAM-V:
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='URDF model used for the USV in the competition launch'
        ),
        # WAM-V simulation mode:
        DeclareLaunchArgument(
            'sim_mode',
            default_value='sim',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'
        ),
        # Tell if the system needs to apply the apriltags:
        DeclareLaunchArgument(
            'add_apriltags',
            default_value='true',
            description='Boolean to add the AprilTags'
        ),
        # How many Apriltags are going to be added:
        DeclareLaunchArgument(
            'num_apriltags',
            default_value='1',
            description='NUmber of AprilTags that are going to be added'
        ),
        # Define the position of the AprilTags:
        DeclareLaunchArgument(
            'pos_apriltag',
            default_value='0.28, 0, 1.76',
            description='Position ofhte AprilTags if there is more than one the positions can be subdivided by ;'
        ),
        # Name of the apriltag dae:
        DeclareLaunchArgument(
            'apriltag_name',
            default_value='marker',
            description='Name of the .dae file of the AprilTags in the wamv_descirpiton folder'
        ),
        


        # Run the launch function:
        OpaqueFunction(function=launch)
    ])