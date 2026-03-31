import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess

from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    pkg_prj_bringup = get_package_share_directory('drone_bringup')
    pkg_prj_description = get_package_share_directory('drone_description')
    pkg_prj_gazebo = get_package_share_directory('drone_gazebo')
    pkg_ros_motor_model = get_package_share_directory('ros_motor_model')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    sdf_file = os.path.join(pkg_prj_description, 'models', 'S550', 'model.sdf')
    mavros_config = os.path.join(pkg_prj_bringup, 'config', 'px4_mavros.yaml')
    ros_second_order_motor_model_config = os.path.join(
        pkg_ros_motor_model, 'config', 'second_order_motor.yaml')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # --- Declare launch arguments ---
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=os.path.expanduser('~/Downloads/PX4-Autopilot'),
        description='Path to PX4-Autopilot directory'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz2.'
    )

    # --- Gazebo Sim ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_prj_gazebo,
            'worlds',
            'S550_empty_world.sdf'
        ])}.items()
    )

    # --- ros_gz_bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'use_sim_time': True},
            {'config_file': os.path.join(
                pkg_prj_bringup, 'config',
                'ros_gz_bridge_s550.yaml')},
            {'qos_overrides./tf_static.publisher.history.durability':
                'transient_local'},
        ],
        output='screen'
    )

    # --- Camera bridges ---
    infra1_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera/infra1/image_rect_raw'],
        parameters=[{'use_sim_time': True}, {'qos': 'sensor_data'}],
        output='screen'
    )

    infra2_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera/infra2/image_rect_raw'],
        parameters=[{'use_sim_time': True}, {'qos': 'sensor_data'}],
        output='screen'
    )

    depth_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera/depth/image_rect_raw'],
        parameters=[{'use_sim_time': True}, {'qos': 'sensor_data'}],
        output='screen'
    )

    # --- Robot state publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # --- ros_motor_model (second order) ---
    # /uav/cmd_raw → motor model → /uav/actual_rpm + Gazebo motors
    # Motor topic: /S550/gazebo/command/motor_speed (PX4 writes to
    # /model/S550/command/motor_speed which is a dead topic → no conflict)
    ros_second_order_motor_model = Node(
        package='ros_motor_model',
        executable='ros_second_order_motor_model',
        output='screen',
        parameters=[{'use_sim_time': False},
                    ros_second_order_motor_model_config
                    ]
    )

    # --- PX4 SITL ---
    # PX4 gz_bridge reads Gazebo IMU directly for EKF2.
    # EKF2 fuses simulated IMU → proper attitude + position estimation.
    # RC input via QGC joystick → /mavros/rc/in
    gz_resource_path = ':'.join([
        os.path.join(pkg_prj_description, 'models'),
        os.path.join(pkg_prj_gazebo, 'worlds'),
    ])

    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                'cd ', LaunchConfiguration('px4_dir'),
                ' && PX4_SYS_AUTOSTART=4100',
                ' PX4_GZ_MODEL_NAME=S550',
                ' PX4_GZ_WORLD=S550_world',
                ' GZ_SIM_RESOURCE_PATH=', gz_resource_path,
                ' ./build/px4_sitl_default/bin/px4 -d',
            ]
        ],
        output='screen',
    )

    # --- MAVROS ---
    # /mavros/local_position/odom  ← PX4 EKF2 출력
    # /mavros/rc/in                ← QGC joystick RC 입력
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            mavros_config,
            {'use_sim_time': False},
        ]
    )

    # --- RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_prj_bringup,
                                      'config',
                                      's550.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        px4_dir_arg,
        rviz_arg,
        gz_sim,
        bridge,
        infra1_bridge,
        infra2_bridge,
        depth_bridge,
        robot_state_publisher,
        rviz,
        ros_second_order_motor_model,
        px4_sitl,
        mavros_node,
    ])
