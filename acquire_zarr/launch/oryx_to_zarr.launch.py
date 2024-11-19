from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument as LaunchArg
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration as LaunchConfig

oryx_parameters = {
    'debug': False,
    'compute_brightness': False,
    'adjust_timestamp': True,
    'dump_node_map': False,
    'frame_rate_auto': 'Off',
    'buffer_queue_size': 10,
    # Stack acquisition parameters
    'acquisition_mode': 'MultiFrame',  # For finite stack capture
    'frame_count': LaunchConfig('stack_size'),  # Number of images in stack
    # GPIO configuration
    'line_selector': LaunchConfig('gpio_line'),  # Select GPIO line
    'line_linemode': 'Output',  # Configure as output
    'line_source': 'ExposureActive',  # Signal during exposure
    # Timing configuration
    'frame_rate_enable': True,
    'frame_rate': LaunchConfig('frame_rate'),
    'exposure_mode': 'Timed',
    'exposure_auto': 'Off',
    'exposure_time': LaunchConfig('exposure_time'),
}

def launch_setup(context, *args, **kwargs):
    """Launch camera driver node."""
    parameter_file = PathJoinSubstitution(
            [FindPackageShare('spinnaker_camera_driver'), 'config', 'oryx.yaml']
        )

    camera_node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        output='screen',
        name='oryx',
        parameters=[
            oryx_parameters,
            {
                'ffmpeg_image_transport.encoding': 'hevc_nvenc',
                'parameter_file': parameter_file,
                'serial_number': [LaunchConfig('serial')],
            },
        ],
    )

    zarr_writer_node = Node(
        package='acquire_zarr', 
        executable='zarr_writer_node_exe', 
        parameters=[{
            'zarr_out_path': LaunchConfig('zarr_out_path'),
            'dimension_sizes': [LaunchConfig('stack_size'), LaunchConfig('image_height'), LaunchConfig('image_width')],
            'chunk_sizes': [LaunchConfig('stack_size'), LaunchConfig('image_height'), LaunchConfig('image_width')],
        }],
        remappings=[
            ('image_raw', '/oryx/image_raw'),
        ],
        output='screen'
    )

    return [camera_node, zarr_writer_node]

def generate_launch_description():
    return LaunchDescription([
        LaunchArg('serial', default_value="'23011970'", description='Camera serial number'),
        LaunchArg('image_width', default_value='2448', description='Image width'),
        LaunchArg('image_height', default_value='2048', description='Image height'),
        LaunchArg('zarr_out_path', default_value='/tmp/out.zarr', description='zarr output path'),
        LaunchArg('stack_size', default_value='100', description='Number of images in stack'),
        LaunchArg('gpio_line', default_value='Line2', description='GPIO line for timing signals'),
        LaunchArg('frame_rate', default_value='10.0', description='Frame rate in Hz'),
        LaunchArg('exposure_time', default_value='10000.0', description='Exposure time in microseconds'),
        OpaqueFunction(function=launch_setup)
    ])
