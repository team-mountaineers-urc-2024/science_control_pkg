# science_control_pkg

Message [Nate Adkins](mailto:npa00003@mix.wvu.edu) on Slack with any questions or suggestions

## Overview

This ROS 2 package provides nodes for controlling various parts of the science payload. The `science.launch.py` launch file contains code to start all of the required parts of the science payload. Newly created nodes necessary for the science payload should be added to the launch file as needed.

## Nodes

### Node: ocean_optics_spectrometer_node

This node exposes a service for collecting data from the Ocean Optics spectrometer used in the science payload.

#### Parameters

- **serial_number**: Serial Number of Ocean Optics Spectrometer (default: 'S14413').
- **service_name**: Name of the service the node exposes to recieve spectrometer data (default: '/spectrometer').

#### Services

- **/spectrometer**: Service for collecting data from the spectrometer


### Node: science_motor_control_node

This node provides a service-based interface for controlling the drivebase. It uses a service client to send CAN commands to the motors.

#### Parameters

- **dynamixel_device_path**: Device path used by the U2D2 for the science payload (default: '/dev/u2d2').
- **dynamixel_baudrate**: Baudrate used by the U2D2 for the science payload (default: 3000000).
- **centrifuge_dynamixel_id**: ID for the dynamixel responsible for turning the centrifuge (default: 10).
- **centrifuge_action_name**: Name of action used to control and track progress of the centrifuge.
- **collector1_dynamixel_id**: ID for the first dynamixel responsible for collecting material (default: 20).
- **collector2_dynamixel_id**: ID for the second dynamixel responsible for collecting material (default: 21).
- **collector3_dynamixel_id**: ID for the third dynamixel responsible for collecting material (default: 22).

## Building and Running

1. Clone this package into your ROS 2 workspace.

    ```bash
    cd <path_to_your_workspace>
    cd src
    git clone git@github.com:wvu-urc/science_control_pkg.git
    ```

2. Build the ROS 2 workspace.

    ```bash
    cd ..
    colcon build
    ```

3. Source the ROS 2 workspace.

    ```bash
    source setup.bash
    ```

4. Run the `ocean_optics_spectrometer_node`.

    ```bash
    ros2 run science_control_pkg ocean_optics_spectrometer_node
    ```
    
    ### **OR**
    
    Run the `science_motor_control_node`.

    ```bash
    ros2 run science_control_pkg science_motor_control_node
    ```

6. Ensure that the required actions/services are being exposed/sent with appropriate message types.

