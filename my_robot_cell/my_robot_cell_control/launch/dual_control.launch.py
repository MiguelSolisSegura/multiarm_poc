"""Dual-UR10 launch file for UR ROS 2 driver (single controller manager, two arms).

*Two UR10s, prefixes **arm1_** and **arm2_***
------------------------------------------------
This launch script spins up **one** `ros2_control_node` that exposes both
hardware interfaces and loads every controller described in
`my_robot_cell_control/config/combined_controllers.yaml`.  Per-arm helper
nodes (dashboard client, state helper, URScript interface, etc.) are started
in their own ROS namespace (`arm1`, `arm2`).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

###############################################################################
# Helper – build the set of auxiliary nodes for one arm                      #
###############################################################################

def make_arm_side_nodes(ns: str, robot_ip, headless_mode, use_tool_comm):
    """Return dashboard, state helper, (optional) tool-comm & URScript nodes."""

    dashboard_client = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        namespace=ns,
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    robot_state_helper = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        namespace=ns,
        name="robot_state_helper",
        output="screen",
        parameters=[{"headless_mode": headless_mode}, {"robot_ip": robot_ip}],
    )

    tool_comm = Node(
        package="ur_robot_driver",
        executable="tool_communication.py",
        namespace=ns,
        name="tool_comm",
        condition=IfCondition(use_tool_comm),
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace=ns,
        name="urscript_interface",
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
    )

    # Return as a list so the caller can simply concatenate both arms’ nodes.
    return [dashboard_client, robot_state_helper, tool_comm, urscript_interface]

###############################################################################
# Main opaque setup                                                          #
###############################################################################

def launch_setup(context, *args, **kwargs):
    # Shared launch configurations ---------------------------------------------
    robot_ip1               = LaunchConfiguration("robot_ip1")
    robot_ip2               = LaunchConfiguration("robot_ip2")

    runtime_config_package  = LaunchConfiguration("runtime_config_package")
    controllers_file        = LaunchConfiguration("controllers_file")
    description_package     = LaunchConfiguration("description_package")
    combined_description    = LaunchConfiguration("combined_description_file")

    use_fake_hardware       = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands    = LaunchConfiguration("fake_sensor_commands")
    headless_mode           = LaunchConfiguration("headless_mode")
    controller_timeout      = LaunchConfiguration("controller_spawner_timeout")
    use_tool_comm           = LaunchConfiguration("use_tool_communication")

    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # -------------------------------------------------------------------------
    # Build combined robot_description (URDF produced by xacro)                 
    # -------------------------------------------------------------------------
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", combined_description]),
        " ",
        "robot_ip1:=", robot_ip1, " ",
        "robot_ip2:=", robot_ip2, " ",
        "use_fake_hardware:=", use_fake_hardware, " ",
        "fake_sensor_commands:=", fake_sensor_commands, " ",
        "headless_mode:=", headless_mode,
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # -------------------------------------------------------------------------
    # Controller manager (one instance for both arms)                           
    # -------------------------------------------------------------------------
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", controllers_file
    ])

    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            "ur10_update_rate.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    # -------------------------------------------------------------------------
    # Per-arm helper nodes                                                      
    # -------------------------------------------------------------------------
    arm1_nodes = make_arm_side_nodes(
        ns="arm1",
        robot_ip=robot_ip1,
        headless_mode=headless_mode,
        use_tool_comm=use_tool_comm,
    )

    arm2_nodes = make_arm_side_nodes(
        ns="arm2",
        robot_ip=robot_ip2,
        headless_mode=headless_mode,
        use_tool_comm=use_tool_comm,
    )

    # -------------------------------------------------------------------------
    # Robot-state publisher for the combined model                              
    # -------------------------------------------------------------------------
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # -------------------------------------------------------------------------
    # RViz                           
    # -------------------------------------------------------------------------
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        #condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "10", # timeout (seconds)
            ]
            + inactive_flags
            + controllers,
        )
    
    controllers_active = [
        "arm1_scaled_joint_trajectory_controller",
        "arm1_joint_state_broadcaster",
        "arm1_io_and_status_controller",
        "arm1_speed_scaling_state_broadcaster",
        "arm1_force_torque_sensor_broadcaster",
        "arm1_tcp_pose_broadcaster",
        "arm1_ur_configuration_controller",
        "arm2_scaled_joint_trajectory_controller",
        "arm2_joint_state_broadcaster",
        "arm2_io_and_status_controller",
        "arm2_speed_scaling_state_broadcaster",
        "arm2_force_torque_sensor_broadcaster",
        "arm2_tcp_pose_broadcaster",
        "arm2_ur_configuration_controller",
    ]
    controllers_inactive = [
        "arm1_joint_trajectory_controller",
        "arm1_forward_velocity_controller",
        "arm1_forward_position_controller",
        "arm1_force_mode_controller",
        "arm1_passthrough_trajectory_controller",
        "arm1_freedrive_mode_controller",
        "arm2_joint_trajectory_controller",
        "arm2_forward_velocity_controller",
        "arm2_forward_position_controller",
        "arm2_force_mode_controller",
        "arm2_passthrough_trajectory_controller",
        "arm2_freedrive_mode_controller",
    ]

    #if activate_joint_controller.perform(context) == "true":
    #    controllers_active.append(initial_joint_controller.perform(context))
    #    controllers_inactive.remove(initial_joint_controller.perform(context))

    if use_fake_hardware.perform(context) == "true":
        controllers_active.remove("arm1_tcp_pose_broadcaster")
        controllers_active.remove("arm2_tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    def make_controller_stopper(ns: str):
        return Node(
            package   = "ur_robot_driver",
            executable= "controller_stopper_node",
            namespace = ns,
            name      = "controller_stopper",
            output    = "screen",
            emulate_tty = True,
            condition = UnlessCondition(use_fake_hardware),
            parameters = [
                {"headless_mode": headless_mode},
                {"joint_controller_active": activate_joint_controller},
                {"consistent_controllers": [
                    f"{ns}_io_and_status_controller",
                    f"{ns}_force_torque_sensor_broadcaster",
                    f"{ns}_joint_state_broadcaster",
                    f"{ns}_speed_scaling_state_broadcaster",
                    f"{ns}_tcp_pose_broadcaster",
                    f"{ns}_ur_configuration_controller",
                ]},
            ],
        )
    
    controller_stopper_arm1 = make_controller_stopper("arm1")
    controller_stopper_arm2 = make_controller_stopper("arm2")
    controller_stoppers = [controller_stopper_arm1, controller_stopper_arm2]

    # Aggregate and return all launch entities                                  
    return [control_node, ur_control_node, rsp_node, rviz_node] + arm1_nodes + arm2_nodes + controller_spawners + controller_stoppers

###############################################################################
# Launch description                                                         #
###############################################################################

def generate_launch_description():
    args = [
        # IPs                                                                   
        DeclareLaunchArgument("robot_ip1", description="IP of *arm1* UR10"),
        DeclareLaunchArgument("robot_ip2", description="IP of *arm2* UR10"),

        # Packages & files                                                      
        DeclareLaunchArgument(
            "runtime_config_package", default_value="my_robot_cell_control",
            description="Package that contains combined_controllers.yaml",
        ),
        DeclareLaunchArgument(
            "controllers_file", default_value="combined_controllers.yaml",
            description="YAML file with all controllers for both arms.",
        ),
        DeclareLaunchArgument(
            "description_package", default_value="my_robot_cell_control",
            description="Package that holds the combined URDF/Xacro.",
        ),
        DeclareLaunchArgument(
            "combined_description_file", default_value="dual_setup_controlled.urdf.xacro",
            description="URDF/Xacro describing the cell (arm1_, arm2_ prefixes).",
        ),

        # Driver-wide toggles                                                   
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="20"),
        DeclareLaunchArgument("use_tool_communication", default_value="false"),

        # Additional options
        DeclareLaunchArgument(
            "activate_joint_controller", default_value="true",
            description="Activate loaded joint controller."
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="arm1_scaled_joint_trajectory_controller",
            choices=[
                "arm1_scaled_joint_trajectory_controller",
                "arm1_joint_trajectory_controller",
                "arm1_forward_velocity_controller",
                "arm1_forward_position_controller",
                "arm1_freedrive_mode_controller",
                "arm1_passthrough_trajectory_controller",
            ],
            description="Initially loaded robot controller.",
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])

###############################################################################
# Xacro hint (adapted to arm1_/arm2_)                                         
###############################################################################
# <robot name="dual_ur10" xmlns:xacro="http://www.ros.org/wiki/xacro">
#   <xacro:include filename="$(find ur_description)/urdf/ur.urdf.xacro"/>
#   <xacro:ur type="ur10" prefix="arm1_" name="arm1_ur10" robot_ip="${robot_ip1}"/>
#   <xacro:ur type="ur10" prefix="arm2_" name="arm2_ur10" robot_ip="${robot_ip2}"/>
# </robot>
# ---------------------------------------------------------------------------
