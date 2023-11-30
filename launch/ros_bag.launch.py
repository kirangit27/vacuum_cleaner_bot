from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    
    all_topics = LaunchConfiguration('all_topics')

    return LaunchDescription([

        DeclareLaunchArgument(
            'all_topics',
            default_value='False'
        ),

        Node(
            package='vacuum_cleaner_bot',
            executable='vc_bot',
            name='WalkerBot',
            parameters=[{
                "all_topics": LaunchConfiguration('all_topics'),
            }]
        ),

        ExecuteProcess(
        condition=IfCondition(all_topics),
        cmd=[
            'ros2', 'bag', 'record', '-o results/bag_file', '-a', '-x', '/scan.*'
        ],
        shell=True
        )

    ])
