# ROS node for handling MyActuator Sub-surface motor
# utilize myactuator library
# method for speed and resetting motor
# GUI-side may have a slider for speed of motor >> this node needs to take that input

import myactuator_lib, can
import can_interface_pkg
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import SubsurfaceMotor
from robot_interfaces.msg import CanCommand

# motor ID == 0x14A
class MyActuatorMotorNode(Node):

    def __init__(self):
        super().__init__('subsurface_motor_node')
        
        self.declare_parameter('motor_id', 0x14A)
        self.declare_parameter('can_topic_name', 'outgoing_can_commands') 
        self.declare_parameter('motor_service_name', "SubsurfaceMotor")
        self.motor = myactuator_lib.Motor(
            self.get_parameter('motor_id').get_parameter_value().integer_value
        )
        self.subsurface_motor_service = self.create_service(
            srv_type = SubsurfaceMotor,
            srv_name = self.get_parameter('motor_service_name').get_parameter_value().string_value,
            callback = self.subsurface_motor_callback
        )

        # Create the CAN publisher
        self.can_command_publisher = self.create_publisher(
            msg_type=CanCommand,
            topic= self.get_parameter('can_topic_name').get_parameter_value().string_value,
            qos_profile=10
            )
        
    # 2 operations: turning on (speed) and off motor
    def subsurface_motor_callback(self, request, response):
        if request.state == True:
            # TODO: Turn on motor
            can_message = self.motor.Speed_Closed_loop_Control_Command(request.speed)
            self.send_can_message(can_message)
            response.is_successful = True
            pass
        elif request.state == False:
            # TODO: Turn off motor
            can_message = self.motor.Motor_stop_command()
            self.send_can_message(can_message)
            response.is_successful = True
            pass
        else:
            response.is_successful = False
            pass

        #TODO: take care of response
        return response
    
    def send_can_message(self, can_command: can.Message):

            can_outgoing_ros_message = CanCommand()
            can_outgoing_ros_message.arbitration_id = can_command.arbitration_id
            can_outgoing_ros_message.is_extended_id = can_command.is_extended_id
            can_outgoing_ros_message.byte_0 = can_command.data[0]
            can_outgoing_ros_message.byte_1 = can_command.data[1]
            can_outgoing_ros_message.byte_2 = can_command.data[2]
            can_outgoing_ros_message.byte_3 = can_command.data[3]
            can_outgoing_ros_message.byte_4 = can_command.data[4]
            can_outgoing_ros_message.byte_5 = can_command.data[5]
            can_outgoing_ros_message.byte_6 = can_command.data[6]
            can_outgoing_ros_message.byte_7 = can_command.data[7]

            self.can_command_publisher.publish(can_outgoing_ros_message)



def main(args=None):
    rclpy.init(args=args)

    motor_node = MyActuatorMotorNode()

    rclpy.spin(motor_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()