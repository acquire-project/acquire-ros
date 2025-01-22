from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    image_width = LaunchConfiguration('image_width')
    image_width_launch_arg = DeclareLaunchArgument('image_width', default_value='2448', description='Image width')
    
    image_height = LaunchConfiguration('image_height')
    image_height_launch_arg = DeclareLaunchArgument('image_height', default_value='2048', description='Image height')
    
    zarr_out_path = LaunchConfiguration('zarr_out_path')
    zarr_out_path_launch_arg = DeclareLaunchArgument('zarr_out_path', default_value='/tmp/out.zarr', description='zarr output path')
    
    blackfly_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('spinnaker_camera_driver'),
                'launch',
                'driver_node.launch.py'
            ])
        ]),
        launch_arguments={
            'serial': "'21278186'",
            'camera_type': 'blackfly_s',
            'camera_name': 'blackfly'
        }.items()
    )
    
    zarr_writer_node = Node(
        package='acquire_zarr', 
        executable='image_zarr_writer_node', 
        parameters=[{
            'zarr_out_path': zarr_out_path,
            'dimension_sizes': [0, image_height, image_width],
            'chunk_sizes': [1, image_height, image_width],
        }],
        remappings=[
            ('image_raw', '/blackfly/image_raw'),
        ],
        output='screen')
    
    return LaunchDescription([
        image_width_launch_arg,
        image_height_launch_arg,
        zarr_out_path_launch_arg,
        blackfly_driver_node,
        zarr_writer_node,
    ])
    
