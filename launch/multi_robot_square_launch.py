import os
import launch
import shutil
import yaml
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

ROBOT_NAMES = ["Turtlebot4_0", "Turtlebot4_1", "Turtlebot4_2",  "Turtlebot4_3"]


# copy the meshes manually to the shared directory, so the webots on macos can find them (outside of vm)
def copy_meshes_to_shared(context, *args, **kwargs):
    package_dir = get_package_share_directory('webots_ros2_turtlebot4')
    src_meshes_dir = os.path.join(package_dir, 'meshes')

    shared_env = os.environ.get('WEBOTS_SHARED_FOLDER')
    shared_dir = shared_env.split(':')[-1]  # pick the VM path

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
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    # Start a Webots simulation instance
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    robot_nodes = []
    
    for robot_name in ROBOT_NAMES:
        namespace = robot_name  # Use robot_name as namespace
        

        
        # Create the robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>'  # Dummy URDF
            }],
        )

        # wheel_drop_left and right
        tf_wheel_drop_left = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='wheel_drop_left_to_base_link',
            output='screen',
            namespace=namespace,
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'wheel_drop_left'],
        )

        tf_wheel_drop_right = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='wheel_drop_right_to_base_link',
            output='screen',
            namespace=namespace,
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'wheel_drop_right'],
        )

        # Add the bridge nodes
        cliff_intensity_bridge = Node(
            package='webots_ros2_turtlebot4',
            executable='cliff_intensity_bridge',
            name='cliff_intensity_bridge',
            namespace=namespace,  # Namespace per robot
        )

        ir_intensity_bridge = Node(
            package='webots_ros2_turtlebot4',
            executable='ir_intensity_bridge',
            name='ir_intensity_bridge',
            namespace=namespace,  # Namespace per robot
        )


        # ROS control spawners
        ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control_multi.yaml')
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
            # Controller topics
            (f'/{namespace}/diffdrive_controller/cmd_vel_unstamped', f'/{namespace}/cmd_vel'), 
            (f'/{namespace}/diffdrive_controller/odom', f'/{namespace}/odom'),
            # Sensor topics: Remap double-namespaced to single-namespaced
            (f'/{namespace}/{namespace}/rplidar', f'/{namespace}/scan'),
            (f'/{namespace}/{namespace}/rplidar/point_cloud', f'/{namespace}/rplidar/point_cloud'),
            (f'/{namespace}/{namespace}/oakd_stereo_camera/point_cloud', f'/{namespace}/depth_camera'),
            (f'/{namespace}/{namespace}/cliff_front_left', f'/{namespace}/cliff_front_left'),
            (f'/{namespace}/{namespace}/cliff_front_right', f'/{namespace}/cliff_front_right'),
            (f'/{namespace}/{namespace}/cliff_side_left', f'/{namespace}/cliff_side_left'),
            (f'/{namespace}/{namespace}/cliff_side_right', f'/{namespace}/cliff_side_right'),
            (f'/{namespace}/{namespace}/ir_intensity', f'/{namespace}/ir_intensity'),
            (f'/{namespace}/{namespace}/ir_intensity_front_center_left/point_cloud', f'/{namespace}/ir_intensity_front_center_left/point_cloud'),
            (f'/{namespace}/{namespace}/ir_intensity_front_center_right/point_cloud', f'/{namespace}/ir_intensity_front_center_right/point_cloud'),
            (f'/{namespace}/{namespace}/ir_intensity_front_left/point_cloud', f'/{namespace}/ir_intensity_front_left/point_cloud'),
            (f'/{namespace}/{namespace}/ir_intensity_front_right/point_cloud', f'/{namespace}/ir_intensity_front_right/point_cloud'),
            (f'/{namespace}/{namespace}/ir_intensity_left/point_cloud', f'/{namespace}/ir_intensity_left/point_cloud'),
            (f'/{namespace}/{namespace}/ir_intensity_right/point_cloud', f'/{namespace}/ir_intensity_right/point_cloud'),
            (f'/{namespace}/{namespace}/ir_intensity_side_left/point_cloud', f'/{namespace}/ir_intensity_side_left/point_cloud'),
            (f'/{namespace}/{namespace}/cliff_front_left/point_cloud', f'/{namespace}/cliff_front_left/point_cloud'),
            (f'/{namespace}/{namespace}/cliff_front_right/point_cloud', f'/{namespace}/cliff_front_right/point_cloud'),
            (f'/{namespace}/{namespace}/cliff_side_left/point_cloud', f'/{namespace}/cliff_side_left/point_cloud'),
            (f'/{namespace}/{namespace}/cliff_side_right/point_cloud', f'/{namespace}/cliff_side_right/point_cloud'),
            (f'/{namespace}/{namespace}/oakd_rgb_camera/camera_info', f'/{namespace}/oakd_rgb_camera/camera_info'),
            (f'/{namespace}/{namespace}/oakd_rgb_camera/image_color', f'/{namespace}/oakd_rgb_camera/image_color'),
            (f'/{namespace}/{namespace}/oakd_stereo_camera/camera_info', f'/{namespace}/oakd_stereo_camera/camera_info'),
            (f'/{namespace}/{namespace}/oakd_stereo_camera/image', f'/{namespace}/oakd_stereo_camera/image'),
            (f'/{namespace}/{namespace}/p3d_gps', f'/{namespace}/p3d_gps'),
            (f'/{namespace}/{namespace}/p3d_gps/speed', f'/{namespace}/p3d_gps/speed'),
            (f'/{namespace}/{namespace}/p3d_gps/speed_vector', f'/{namespace}/p3d_gps/speed_vector'),
            (f'/{namespace}/{namespace}/light_front_left', f'/{namespace}/light_front_left'),
            (f'/{namespace}/{namespace}/light_front_right', f'/{namespace}/light_front_right'),
        ]     

        # Create a ROS node interacting with the simulated robot
        robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot4.urdf')
        robot_driver = WebotsController(
            robot_name=robot_name,
            parameters=[
                PathJoinSubstitution([package_dir, 'resource', 'ros2control_multi.yaml']),
                {'robot_description': robot_description_path,
                 'use_sim_time': use_sim_time,
                 'set_robot_state_publisher': True,
                 },
                ros2_control_params
            ],
            remappings=mappings,
            namespace=namespace
        )

        # Wait for the simulation to be ready to start navigation nodes
        waiting_nodes = WaitForControllerConnection(
            target_driver=robot_driver,
            nodes_to_start=ros_control_spawners
        )    

        robot_nodes.extend([
            robot_state_publisher,
            tf_wheel_drop_left,
            tf_wheel_drop_right,
            cliff_intensity_bridge,
            ir_intensity_bridge,
            robot_driver,
            waiting_nodes,
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='square_turt_world_multi.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),        
        OpaqueFunction(function=copy_meshes_to_shared),   
        webots,
        webots._supervisor,
        *robot_nodes,
        # The following action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
