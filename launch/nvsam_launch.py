# Copyright 2023 Hossamelden
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    ExecuteProcess
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.conditions import IfCondition


# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('nvsam'),
    'urdf',
    'zed_descr.urdf.xacro'
)

def launch_setup(context, *args, **kwargs):
    svo_path = LaunchConfiguration('svo_path')
    max_dist = LaunchConfiguration('max_distance')
    camera_name = LaunchConfiguration('camera_name')
    camera_id = LaunchConfiguration('camera_id')
    camera_model = LaunchConfiguration('camera_model')
    node_name = LaunchConfiguration('node_name')
    xacro_path = LaunchConfiguration('xacro_path')
    publish_urdf = LaunchConfiguration('publish_urdf')
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    
    
    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=camera_name_val,
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name_val, ' ',
                    'camera_model:=', camera_model_val, ' '
                ])
        }]
    )
    
    # nvsam node
    nvsam_node = Node(
        package='nvsam',
        namespace=camera_name_val,
        executable='nvsam',
        name=node_name,
        output='screen',
        parameters=[{
            'svo_path': svo_path,
            'camera_id': camera_id,
            'max_distance': max_dist
        }]
    )

    # Launch RViz with the specified configuration file
    rviz_config_file = os.path.join(get_package_share_directory('nvsam'), 'rviz', 'nvsam.rviz')
    rviz_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_config_file],
        output='screen',
        name='rviz_node'
    )

    return [rsp_node, nvsam_node, rviz_node]

def generate_launch_description():
    return LaunchDescription(
        [
            # Set environment variable for colorized output
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'camera_id',
                default_value=TextSubstitution(text='0'),
                description='The ID of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'max_distance',
                default_value=TextSubstitution(text='20.0'),
                description='The maximum distance to process'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                default_value='zed2i',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm']),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and starts Robot State Published to propagate static TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='true',
                description='Enable publication of the `odom -> camera_link` TF.',
                choices=['true', 'false']),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text='live'),
                description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.'),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF file as a xacro file.'),
            DeclareLaunchArgument(
                'node_name',
                default_value='nvsam_node',
                description='The name of the nvsam node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`'),

            # Execute the nodes
            OpaqueFunction(function=launch_setup),
        ]
    )

