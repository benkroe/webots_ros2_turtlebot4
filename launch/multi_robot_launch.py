import os
import launch
import shutil
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node


# Variables for multi-robot configuration
NUM_ROBOTS = 3  # Number of robots (set to 3 to match robot_launch.py)
#ROBOT_NAMES = ["Turtlebot4_0", "Turtlebot4_1", "Turtlebot4_2"]  # Explicit namespaces for robots
ROBOT_NAMES = ["Turtlebot4_0"]
#SPAWN_POSITIONS = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]  # Explicit spawn positions (x, y, z)
SPAWN_POSITIONS = [(0.0, 0.0, 0.0)] 

# Copy the meshes manually to the shared directory
def copy_meshes_to_shared(context, *args, **kwargs):
    package_dir = get_package_share_directory('webots_ros2_turtlebot4')
    src_meshes_dir = os.path.join(package_dir, 'meshes')

    shared_env = os.environ.get('WEBOTS_SHARED_FOLDER')
    shared_dir = shared_env.split(':')[-1]  # Pick the VM path

    if not os.path.exists(shared_dir):
        os.makedirs(shared_dir)

    # Copy turtlebot4 meshes
    src_turtlebot4 = os.path.join(src_meshes_dir, 'turtlebot4')
    dest_turtlebot4 = os.path.join(shared_dir, 'meshes', 'turtlebot4')
    if os.path.exists(dest_turtlebot4):
        shutil.rmtree(dest_turtlebot4)
    shutil.copytree(src_turtlebot4, dest_turtlebot4)

    # Copy create3 meshes
    src_create3 = os.path.join(src_meshes_dir, 'create3')
    dest_create3 = os.path.join(shared_dir, 'meshes', 'create3')
    if os.path.exists(dest_create3):
        shutil.rmtree(dest_create3)
    shutil.copytree(src_create3, dest_create3)

    print(f"[INFO] Copied turtlebot4 meshes from {src_turtlebot4} to {dest_turtlebot4}")
    print(f"[INFO] Copied create3 meshes from {src_create3} to {dest_create3}")

    return []

def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_turtlebot4')
    OpaqueFunction(function=copy_meshes_to_shared),

    # Launch arguments
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Start Webots simulation
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    # Nodes for each robot
    robot_nodes = []
    for i, robot_name in enumerate(ROBOT_NAMES):
        spawn_position = SPAWN_POSITIONS[i]
        namespace = robot_name

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>'
            }],
        )

        # Wheel drop transforms
        tf_wheel_drop_left = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'wheel_drop_left_to_base_link',
            output='screen',
            namespace=namespace,
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'wheel_drop_left'],
        )

        tf_wheel_drop_right = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'wheel_drop_right_to_base_link',
            output='screen',
            namespace=namespace,
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'wheel_drop_right'],
        )

        # ROS control spawners
        ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
        controller_manager_timeout = ['--controller-manager-timeout', '50']
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            namespace=namespace,
            arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        )
        diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            namespace=namespace,
            arguments=['diffdrive_controller'] + controller_manager_timeout,
        )
        ros_control_spawners = [joint_state_broadcaster_spawner, diffdrive_controller_spawner]
        mappings = [
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), 
            ('/diffdrive_controller/odom', '/odom'),
            ('/Turtlebot4/rplidar', '/scan'),
            ('/Turtlebot4/oakd_stereo_camera/point_cloud', '/depth_camera')
            ]    

        # Webots controller
        robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot4.urdf')
        robot_driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True,
            },
            ros2_control_params  # Load all controller parameters from YAML
        ],
        remappings=mappings,
        namespace=namespace
        )

        # Wait for controller connection
        waiting_nodes = WaitForControllerConnection(
            target_driver=robot_driver,
            nodes_to_start=ros_control_spawners
        )

        # Add nodes to the list
        robot_nodes.extend([
            robot_state_publisher,
            tf_wheel_drop_left,
            tf_wheel_drop_right,
            robot_driver,
            waiting_nodes,
        ])

    # Return the LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='octagon_turt_world.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        OpaqueFunction(function=copy_meshes_to_shared),
        webots,
        webots._supervisor,
        *robot_nodes,
        # Shutdown Webots when simulation exits
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])