"""Run BlueBoat in a self-contained still-water Gazebo world."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    share=get_package_share_directory('blueboat_description')
    return LaunchDescription([
        DeclareLaunchArgument('headless',default_value='true'),
        DeclareLaunchArgument('controller',default_value='true'),
        DeclareLaunchArgument('world',default_value=os.path.join(share,'worlds','blueboat_water.sdf')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',os.path.join(share,'models')+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH','')),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',os.path.join(get_package_prefix('blueboat_gazebo'),'lib')+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')),
        SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH',os.path.join(get_package_prefix('blueboat_gazebo'),'lib')+os.pathsep+os.environ.get('GZ_GUI_PLUGIN_PATH','')),
        ExecuteProcess(cmd=['gz','sim','-s','-r','-v','2',LaunchConfiguration('world')],output='screen'),
        ExecuteProcess(cmd=['gz','sim','-g'],condition=UnlessCondition(LaunchConfiguration('headless')),output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='blueboat_bridge',arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/blueboat/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/blueboat/propulsion/left/applied_thrust@std_msgs/msg/Float64[gz.msgs.Double',
            '/blueboat/propulsion/right/applied_thrust@std_msgs/msg/Float64[gz.msgs.Double',
            '/blueboat/propulsion/left/rpm@std_msgs/msg/Float64[gz.msgs.Double',
            '/blueboat/propulsion/right/rpm@std_msgs/msg/Float64[gz.msgs.Double',
            '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double'],output='screen'),
        Node(package='usv_control',executable='blueboat_velocity_controller.py',
             condition=IfCondition(LaunchConfiguration('controller')),
             parameters=[{'use_sim_time':True}],output='screen'),
    ])
