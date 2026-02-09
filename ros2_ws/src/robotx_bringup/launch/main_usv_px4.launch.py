from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node

# Create hte launch logic:
def launch(context, *args, **kwargs):
    # World and WAM-V name Argument:
    world_name = LaunchConfiguration('world').perform(context)
    urdf = LaunchConfiguration('urdf').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    # PX4 MOdel alcunh arguments:
    px4_model = LaunchConfiguration('px4_model').perform(context)
    px4_model_pose = LaunchConfiguration('px4_model_pose').perform(context)
    px4_dir    = LaunchConfiguration('px4_dir').perform(context)
    # Define a vector contianing all laucnh process:
    launch_process = []

    # Define the vrx_gz package to use its launch files:
    package_share_directory_vrx = get_package_share_directory('vrx_gz')
    # Open the VRX simulation of competition:
    vrx_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share_directory_vrx, 'launch', 'competition.launch.py')
        ),
        launch_arguments={'world': world_name,
                          'urdf': urdf,
                          'sim_mode': sim_mode}.items(),
    )
    # Add it to teh launch processes:
    launch_process.append(vrx_gz)

    # In case you are only using the sim mode to reduce the 
    # computational power create a bridge that the landing control requires:
    if sim_mode == "sim":
        req_bridges = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                # clock
                f"/world/{world_name}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                # USV GPS
                f"/world/{world_name}/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
            ],
            remappings=[
                # Rename ROS side topics to what you want
                (f"/world/{world_name}/clock", "/clock"),
                (f"/world/{world_name}/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat", "/wamv/sensors/gps/gps/fix"),
            ],
        )
        launch_process.append(req_bridges)
    
    # Temporary Node to represent the  USV position before adding the AprilTag detection:
    model_name_wamv = "wamv"
    wamv_pose_pub = Node(
        package="px4_exec",  
        executable="model_pose_pub", 
        name=f"{model_name_wamv}_pose_pub",
        output="screen",
        parameters=[{
            "world_name": world_name,
            "model_name": model_name_wamv,                 # or "wamv/base_link"
            "frame_id": world_name,               # e.g. "sydney_regatta"
            "gz_topic": f"/world/{world_name}/pose/info",
        }],
    )
    launch_process.append(wamv_pose_pub)
    
    # Start the micro XRCE-DDS agent to send teh PX4 commands:
    xrce_agent = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'micro-xrce-dds-agent udp4 -p 8888 '
            '|| MicroXRCEAgent udp4 -p 8888'
        ],
        output='screen'
    )
    launch_process.append(xrce_agent)

    # Launch the PX4 x500 drone:
    px4_cmd = ExecuteProcess(
        cmd=['bash', '-lc', f'make px4_sitl gz_{px4_model}'],
        cwd=px4_dir,  # Change to the px4 directory.
        output='screen',
        additional_env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': world_name,
            'PX4_GZ_MODEL_POSE': px4_model_pose,
        },
    )

    # Define the launch helping function:
    wait_for_gz = Node(
        package='robotx_bringup',
        executable='wait_for_gazebo',
        output = 'screen',
        name = 'wait_for_gazebo',
    )
    # Add the waiter for the launch_process:
    launch_process.append(wait_for_gz)

    # Launch the PX4 adter some time to wait for hte Gazebo to open:
    launch_px4 = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_gz,
            on_exit=[px4_cmd]
        )
    )
    launch_process.append(launch_px4)

    # Create the gazebo bridge of hte UAV px4 messages:
    uav_bridges = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # UAV GPS
            f"/world/{world_name}/model/{px4_model}_0/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
            # UAV camera IMages:
            f"/camera@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[
            # Rename ROS side topics to what you want
            (f"/world/{world_name}/model/{px4_model}_0/link/base_link/sensor/navsat_sensor/navsat", f"/{px4_model}/sensors/gps/gps/fix"),
            (f"/camera", f"/uav/camera"),
        ],
    )
    launch_process.append(uav_bridges)
    

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
        # PX4 directory:
        DeclareLaunchArgument(
            'px4_dir',
            default_value=os.path.expanduser('/home/adcl/Repository-RobotX/PX4-Autopilot'),
            description='Path to PX4-Autopilot folder (where you run make)'
        ),
        # PX4 model:
        DeclareLaunchArgument(
            'px4_model',
            default_value='x500_mono_cam',
            description='MOdel used from the PX4 library.'
        ),
        # PX4 position:
        DeclareLaunchArgument(
            'px4_model_pose',
            default_value='"-540,140,1.76,0,0,0"',
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
            default_value='full',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'
        ),
        

        # Run the launch function:
        OpaqueFunction(function=launch)
    ])