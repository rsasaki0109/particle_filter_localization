import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pf_param_dir = launch.substitutions.LaunchConfiguration(
        'pf_param_dir',
        default=os.path.join(
            get_package_share_directory('particle_filter_localization'),
            'param',
            'pf.yaml'))

    pf = launch_ros.actions.Node(
        package='particle_filter_localization',
        node_executable='pf_localization_node',
        parameters=[pf_param_dir],
        remappings=[('/pf_localization/gnss_pose','/gnss_pose'),
                    ('/pf_localization/imu','/imu/data'),
                    ('/pf_localization/map','/mapcloud'),
                    ('/pf_localization/cloud','/cloud')
                    ],
        output='screen'
        )
    
    tf = launch_ros.actions.Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','imu_link']
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'pf_param_dir',
            default_value=pf_param_dir,
            description='Full path to pf parameter file to load'),
        pf,
        tf,
            ])
