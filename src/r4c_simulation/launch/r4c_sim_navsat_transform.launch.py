import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Configuration file paths
    navsat_transform_file_path = os.path.join(get_package_share_directory("r4c_simulation"),
                                              'config/navsat_transform.yaml')

    # # IMU topic filter (discard measurements from OAK-D camera)
    # start_agc_imu_filter_cmd = Node(
    #   package='r4c_localization',
    #   executable='agcbox_imu_filter_node',
    #   name='agcbox_imu_filter'
    #   remappings=[('/input_imu', '/r4c_tractor/agcbox/imu'), ('/output_imu', '/r4c_tractor/agcbox/heading_imu')]
    # )

    # # ENU to NavSatFix converter
    # start_enu_to_navsatfix_cmd = Node(
    #   package='r4c_localization',
    #   executable='enu_pos_to_navsatfix_node',
    #   name='enu_pos_to_navsatfix'
    #   remappings=[('/input_enu_pos', '/r4c_tractor/agcbox/gnss/enu/pos'), ('/output_fix', '/r4c_tractor/agcbox/gnss/wgs84_fix')]
    # )

    # Publish a constant datum fix to visualize satellite map in rviz
    start_pub_datum_fix_cmd = Node(
        package='r4c_localization',
        executable='pub_datum_fix_node',
        name='pub_datum_fix_node',
        parameters=[{'datum': [37.94375462117764, 22.772031169116122]}]
    )

    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_transform_file_path],
        remappings=[
            ('/imu/data', '/rover/navheading'), #('/imu/data', '/r4c_tractor/agcbox/heading_imu'),
            ('/odometry/filtered', '/r4c_tractor/wheel_odom'),
            ('/gps/fix', '/r4c_tractor/gnss/fix'), # ('/gps/fix', '/r4c_tractor/agcbox/gnss/wgs84_fix'),
            ('/gps/filtered', '/r4c_tractor/nobu/gnss/wgs84_fix')
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_pub_datum_fix_cmd)
    ld.add_action(start_navsat_transform_cmd)

    return ld
