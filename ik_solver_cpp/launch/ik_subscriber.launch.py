from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    with open('/home/kkm/ws_moveit/src/ggul_bot_v6_config/config/ggul_bot_v6.urdf', 'r') as urdf_file:
        robot_description = urdf_file.read()
    with open('/home/kkm/ws_moveit/src/ggul_bot_v6_config/config/kinematics.yaml', 'r') as kin_file:
        import yaml
        robot_description_kinematics = yaml.safe_load(kin_file)

    return LaunchDescription([
        Node(
            package='ik_solver_cpp',
            executable='ik_subscriber_node',
            name='ik_subscriber_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_kinematics': robot_description_kinematics}
            ]
        )
    ])

