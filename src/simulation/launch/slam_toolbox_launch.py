import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions, launch_description_sources
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get references to the configuration files 
    #       This configuration is what you need to do. Be aware that the 'mapping.yaml' is your own configuration 
    mapping_config_file = os.path.join(get_package_share_directory('simulation'), 'slam', 'mapping.yaml')

    # Declare parameters for launch file
    # Simulation time is set to true when we work with gazebo
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description="Use Gazebo time")

    # Declare here that you want to resume mapping and give and additional path to the map file and the initial pose    
    use_map_resume = LaunchConfiguration('use_map_file')
    use_map_resume_arg = DeclareLaunchArgument('use_map_file', default_value='False', description="Resume from existig map?")
    param_map_path = LaunchConfiguration('map_file')
    param_map_path_arg = DeclareLaunchArgument('map_file', default_value='', description="Map file path for continuation")
    param_map_pose = LaunchConfiguration('map_pose')
    param_map_pose_arg = DeclareLaunchArgument('map_pose', default_value="[0.0, 0.0, 0.0]", description="Map start pose")

    # Great!, but how do you save the map that you will use later to continue? Just type the following line in your terminal: NOTE: Remember that <> indicates that it must be changed by you
    #     ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename : '<map_name: maze_vxx>'}"

    # For loading your slam_toolbox with your saved map, then, in your terminal please type: NOTE: Remember that <> indicates that it must be changed by you
    #    ros2 launch green_turtle slam_toolbox.launch.py use_map_file:='True' map_file:='/home/<user_name: student>/dev_ws/<map_name: maze_vxx>' map_pose:="[-10.5, -4.5, 0.0]"

    # Main SLAM node - conditional execution
    # Nodes don't accept conditions, wrap the node around a Group that does accept conditionlas
    # The conditionals are used to spawn the same node but with different parameters: default only sim time; resume: with sim time, map file and map pose
    slam_node_default = GroupAction([ 
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                mapping_config_file,
                {'use_sim_time': use_sim_time},
            ],
            output='screen')
        ],
        condition=IfCondition(PythonExpression(['not ', use_map_resume])),
        scoped=False)

    # This is the node that will execute when there is a given map for continuing with the mapping task. NOTE: Remember that it uses .posegraph and .data files for the map.
    slam_node_resume = GroupAction([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                mapping_config_file,
                {'use_sim_time': use_sim_time},
                {'map_file_name': param_map_path},
                {'map_start_pose': param_map_pose}
            ],
            output='screen')
        ],
        condition=IfCondition(use_map_resume),
        scoped=False)

    # Start rviz
    rviz_config_dir = os.path.join(get_package_share_directory('simulation'),
                                   'rviz', 'simulation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    launch_navigation = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                get_package_share_directory('turtlebot3_navigation2') + '/launch/navigation2.launch.py'), launch_arguments=[("use_sim_time", "True"), ("map", param_map_path)])

    ld = LaunchDescription()
    ld.add_action(use_map_resume_arg)
    ld.add_action(param_map_pose_arg)
    ld.add_action(param_map_path_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_node_default)
    ld.add_action(slam_node_resume)
    ld.add_entity(rviz_node)
    ld.add_entity(launch_navigation)
    
    return ld