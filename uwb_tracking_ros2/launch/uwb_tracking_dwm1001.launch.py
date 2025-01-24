#<launch>
#    <arg name="use_static_tf" default="true" />
#    <node pkg="tf" type="static_transform_publisher" name="uwb_static_tf"
#      args="2.5 2.5 0 3.14159 0 0 map uwb_map 30"
#      if="$(arg use_static_tf)" />
#
#    <!--Exctract info from dwm1001 dev boards, filter them and publish them into topics-->
#    <node pkg="uwb_tracking_ros2" type="uwb_tracking_dwm1001.py" name="uwb_tracking_dwm1001" output="screen">
#        <rosparam file="$(find uwb_tracking_ros2)/cfg/params_dwm1001.yaml" command="load"/>
#    </node> 
#
#    <!-- Display topics of the tags into Rviz for visualization -->
#    <node pkg="uwb_tracking_ros2" type="viz_dwm1001.py" name="visualize_dwm1001" />
#    <!-- Run rviz with the saved rviz file-->
#    <node type="rviz" name="rviz" pkg="rviz" args="-d $(find uwb_tracking_ros2)/rviz/dwm1001_rviz_config.rviz" />
# 
#</launch>


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_static_tf',
            default_value='true',
            description='Whether to use static TF'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='uwb_static_tf',
            arguments=['2.5', '2.5', '0', '1.0', '0', '0', '0', 'map', 'uwb_map'],
            condition=IfCondition(LaunchConfiguration('use_static_tf'))
        ),

        Node(
            package='uwb_tracking_ros2',
            executable='uwb_tracking_dwm1001',
            name='uwb_tracking_dwm1001',
            output='screen',
            parameters=[{
                'rosparam': 'file://$(find uwb_tracking_ros2)/cfg/params_dwm1001.yaml',
                'command': 'load'
            }]
        ),

        Node(
            package='uwb_tracking_ros2',
            executable='viz_dwm1001',
            name='visualize_dwm1001'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', '$(find uwb_tracking_ros2)/rviz/dwm1001_rviz_config.rviz']
        ),
    ])



# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.conditions import IfCondition
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_static_tf',
#             default_value='true',
#             description='Whether to use static TF'
#         ),

#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='uwb_static_tf',
#             arguments=['2.5', '2.5', '0', '3.14159', '0', '0', 'map', 'uwb_map', '30'],
#             condition=IfCondition(LaunchConfiguration('use_static_tf'))
#         ),

#         Node(
#             package='uwb_tracking_ros2',
#             executable='uwb_tracking_dwm1001',
#             name='uwb_tracking_dwm1001',
#             output='screen',
#             parameters=[{
#                 'rosparam': 'file://$(find uwb_tracking_ros2)/cfg/params_dwm1001.yaml',
#                 'command': 'load'
#             }]
#         ),

#         Node(
#             package='uwb_tracking_ros2',
#             executable='viz_dwm1001',
#             name='visualize_dwm1001'
#         ),

#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz',
#             arguments=['-d', '$(find uwb_tracking_ros2)/rviz/dwm1001_rviz_config.rviz']
#         ),
#     ])