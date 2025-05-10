from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm1_ur_type = LaunchConfiguration("arm1_ur_type")
    arm2_ur_type = LaunchConfiguration("arm2_ur_type")

    arm1_robot_ip = LaunchConfiguration("arm1_robot_ip")
    arm2_robot_ip = LaunchConfiguration("arm2_robot_ip")

    arm1_use_fake_hardware = LaunchConfiguration("arm1_use_fake_hardware")
    arm1_mock_sensor_commands = LaunchConfiguration("arm1_mock_sensor_commands")
    arm2_use_fake_hardware = LaunchConfiguration("arm2_use_fake_hardware")
    arm2_mock_sensor_commands = LaunchConfiguration("arm2_mock_sensor_commands")

    headless_mode = LaunchConfiguration("headless_mode")

    arm1_kinematics_parameters_file = LaunchConfiguration(
        "arm1_kinematics_parameters_file"
    )
    arm2_kinematics_parameters_file = LaunchConfiguration(
        "arm2_kinematics_parameters_file"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("my_robot_cell_control"),
                    "urdf",
                    "dual_setup_controlled.urdf.xacro",
                ]
            ),
            " ",
            "arm1_robot_ip:=",
            arm1_robot_ip,
            " ",
            "arm2_robot_ip:=",
            arm2_robot_ip,
            " ",
            "arm1_ur_type:=",
            arm1_ur_type,
            " ",
            "arm2_ur_type:=",
            arm2_ur_type,
            " ",
            "arm1_use_fake_hardware:=",
            arm1_use_fake_hardware,
            " ",
            "arm2_use_fake_hardware:=",
            arm2_use_fake_hardware,
            " ",
            "arm1_kinematics_parameters_file:=",
            arm1_kinematics_parameters_file,
            " ",
            "arm2_kinematics_parameters_file:=",
            arm2_kinematics_parameters_file,
            " ",
            "arm1_mock_sensor_commands:=",
            arm1_mock_sensor_commands,
            " ",
            "arm2_mock_sensor_commands:=",
            arm2_mock_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm1_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur10",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm2_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur10",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm1_robot_ip",
            default_value="192.168.0.101",
            description="IP address by which arm1 can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm2_robot_ip",
            default_value="192.168.0.100",
            description="IP address by which arm2 can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm1_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_robot_cell_control"),
                    "config",
                    "arm1_calibration.yaml",
                ]
            ),
            description="The calibration configuration of arm1.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm2_kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_robot_cell_control"),
                    "config",
                    "arm2_calibration.yaml",
                ]
            ),
            description="The calibration configuration of arm2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm1_use_fake_hardware",
            default_value="false",
            description="Start arm1 with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm2_use_fake_hardware",
            default_value="false",
            description="Start arm2 with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm1_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for arm1's sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm2_mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for arm2's sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )