import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():

    video_device = LaunchConfiguration('video_device')
    video_device_launch_arg = DeclareLaunchArgument('video_device', default_value='/dev/video0', description='Video device path')

    image_width = LaunchConfiguration('image_width')
    image_width_launch_arg = DeclareLaunchArgument('image_width', default_value='640', description='Image width')
    
    image_height = LaunchConfiguration('image_height')
    image_height_launch_arg = DeclareLaunchArgument('image_height', default_value='480', description='Image height')
    
    zarr_out_path = LaunchConfiguration('zarr_out_path')
    zarr_out_path_launch_arg = DeclareLaunchArgument('zarr_out_path', default_value='/tmp/out.zarr', description='zarr output path')
    
    usb_camera = Node(
        package='usb_cam', 
        executable='usb_cam_node_exe', 
        parameters=[{
            'video_device': video_device, 
            'image_width': image_width, 
            'image_height': image_height,
        }],
        output='screen')

    zarr_writer_node = Node(
        package='acquire_zarr', 
        executable='image_zarr_writer_node', 
        parameters=[{
            'zarr_out_path': zarr_out_path,
            'dimension_sizes': [0, image_height, image_width],
            'dimension_names': ['time', 'height', 'width'],
            'dimension_types': [2, 0, 0],
            'dimension_chunk_px': [1, image_height, image_width],
            'dimension_shard_chunks': [1, 1, 1],
        }],
        output='screen',
        remappings=[('/image_data', '/image_raw')])

    # List of nodes, arguements, etc to add to the launch description.
    return launch.LaunchDescription([
        video_device_launch_arg,
        image_width_launch_arg,
        image_height_launch_arg,
        zarr_out_path_launch_arg,
        usb_camera,
        zarr_writer_node])

