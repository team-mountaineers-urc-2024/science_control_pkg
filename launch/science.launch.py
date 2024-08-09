import launch
import launch_ros.actions

def generate_launch_description():

    launch_description = launch.LaunchDescription()

    launch_description.add_action(
            launch_ros.actions.Node(
            package='science_control_pkg',
            executable='ocean_optics_spectrometer_node',
            name='ocean_optics_spectrometer_node')
        )
    
    launch_description.add_action(
            launch_ros.actions.Node(
            package='science_control_pkg',
            executable='dynamixel_control_node',
            name='dynamixel_control_node')
        )

    launch_description.add_action(
            launch_ros.actions.Node(
            package='science_control_pkg',
            executable='subsurface_motor_node',
            name='subsurface_motor_node')
        )

    # Maybe some USB Cameras, other sensors, etc.
    
    return launch_description
