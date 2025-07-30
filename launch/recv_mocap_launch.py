from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessIO, OnProcessExit

def generate_launch_description():
    rigid_body_name = LaunchConfiguration("rb_name")

    rigid_body_arg = DeclareLaunchArgument(
            "rb_name",
            default_value = "starling_2",
            description = "Name of Rigid body to be streamed"
        )


    vrpn_node = Node(
            package = "optitrack",
            name = "publish_mocap_odometry_node",
            executable = "publish_mocap_odometry_node",
            output = "screen",
            parameters = [{
                "rigid_body_name" : rigid_body_name,
                "server_address" : "192.168.1.42:3883"
                }],
            remappings = [("/starling_2", rigid_body_name)]
        )

    reset_estimator_node = Node(
            package = "optitrack",
            name = "reset_estimator_node",
            executable = "reset_estimator_node",
            output = "screen"
       )

    wait_vrpn_stream = RegisterEventHandler(
            OnProcessIO(
                target_action = vrpn_node,
                on_stderr = lambda event: [
                    ExecuteProcess(
                        #cmd = ["echo", f"Launch File Received: {event.text}"],
                        #output = "screen"
                            cmd = ["ros2", "run", "optitrack", "reset_estimator_node"],
                            output = "screen"
                            )
                    ] if b"Successfully connected to VRPN server." in event.text else[]
                
             )
        )

    shutdown_on_vrpn_exit = RegisterEventHandler(
            OnProcessExit(
                target_action = vrpn_node,
                on_exit = [Shutdown()]
                )
        )

    return LaunchDescription([
        rigid_body_arg,
        vrpn_node,
        wait_vrpn_stream,
        shutdown_on_vrpn_exit,
        ])

