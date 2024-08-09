from dynamixel_lib import Dynamixel, U2D2, XL430
import rclpy, time, traceback
from rclpy.node import Node
from robot_interfaces.srv import SpinCentrifuge, CollectSample, SendScoopHome, RebootScoop

class DynamixelControlNode(Node):

    def __init__(self):
        super().__init__('dynamixel_control_node')

        self.declare_parameter('u2d2_device_path', '/dev/urc/mtc/sci_u2d2')
        self.declare_parameter('dynamixel_baudrate', 9600)


        self.declare_parameter('centrifuge_dynamixel_id', 4)
        self.declare_parameter('spin_centrifuge_service_name', "/spin_centrifuge")

        self.declare_parameter('collector1_dynamixel_id', 1)
        self.declare_parameter('collector2_dynamixel_id', 2)
        self.declare_parameter('collector3_dynamixel_id', 3)
        self.declare_parameter('collect_sample_service_name', "/collect_sample")
        self.declare_parameter('send_collector_home_service_name', "/send_collector_home")
        self.declare_parameter('reboot_scoop_service_name', '/reboot_scoop')

        self.u2d2 = U2D2(
            self.get_parameter('u2d2_device_path').get_parameter_value().string_value,
            self.get_parameter('dynamixel_baudrate').get_parameter_value().integer_value,
        )

        self.centrifuge_motor = Dynamixel(
            XL430,
            self.get_parameter('centrifuge_dynamixel_id').get_parameter_value().integer_value,
            self.u2d2
        )

        self.collector1_motor = Dynamixel(
            XL430,
            self.get_parameter('collector1_dynamixel_id').get_parameter_value().integer_value,
            self.u2d2
        )

        self.collector2_motor = Dynamixel(
            XL430,
            self.get_parameter('collector2_dynamixel_id').get_parameter_value().integer_value,
            self.u2d2
        )

        self.collector3_motor = Dynamixel(
            XL430,
            self.get_parameter('collector3_dynamixel_id').get_parameter_value().integer_value,
            self.u2d2
        )

        self.spin_centrifuge_service = self.create_service(
            srv_type = SpinCentrifuge,
            srv_name = self.get_parameter('spin_centrifuge_service_name').get_parameter_value().string_value,
            callback = self.spin_centrifuge_callback
        )

        self.collect_sample_service = self.create_service(
            srv_type = CollectSample,
            srv_name = self.get_parameter('collect_sample_service_name').get_parameter_value().string_value,
            callback = self.collect_sample_callback
        )

        self.send_collector_home_service = self.create_service(
            srv_type = SendScoopHome,
            srv_name = self.get_parameter('send_collector_home_service_name').get_parameter_value().string_value,
            callback = self.send_collector_home_callback
        )

        self.reboot_scoop_service = self.create_service(
            srv_type = RebootScoop,
            srv_name = self.get_parameter('reboot_scoop_service_name').get_parameter_value().string_value,
            callback = self.reboot_scoop_callback
        )
    

    def spin_centrifuge_callback(self, request, response):
        try:
            goal_position = int((request.cuvette_id - 1) * 341.33333333)
            
            self.centrifuge_motor.write(XL430.TorqueEnable, 0)
            self.centrifuge_motor.write(XL430.OperatingMode, 3)

            self.centrifuge_motor.write(XL430.TorqueEnable, 1)
            self.centrifuge_motor.write(XL430.GoalPosition, goal_position)
            
            response.is_successful = True
            response.end_position = request.cuvette_id

            start = time.time()
            # Get Motor Position
            pos = self.centrifuge_motor.read(XL430.PresentPosition, 0)
            # Loop until Motor is within 10 positions from its intended position out of range 0-4095
            while(pos < goal_position - 10 or  pos > goal_position + 10):
                #If 5 seconds have passed, and motor has not reached any acceptable position, return nonsuccessfully
                if time.time() - start > 5:
                    response.is_successful = False
                    response.end_position = -1
                    break
                pos = self.centrifuge_motor.read(XL430.PresentPosition, 0)

            self.centrifuge_motor.write(XL430.TorqueEnable,0)
            return response
        except Exception as e:
            #figure out how to send exception to GUI
            #traceback.format_exc()
            response.is_successful = False
            return response



    def collect_sample_callback(self, request, response):
        try:
            if(request.start == True):
                # Get Requested Collector
                collector = -1
                if request.collector_id == 1:
                    collector = self.collector1_motor
                elif request.collector_id == 2:
                    collector = self.collector2_motor
                elif request.collector_id == 3:
                    collector = self.collector3_motor
                else:
                    self.get_logger().error(f'Provided collector ID "{request.collector_id}" is not in the valid range of 1-3.')
                    return
                
                # Put Motor in Velocity Control Mode
                collector.write(XL430.TorqueEnable, 0)
                collector.write(XL430.OperatingMode, 1)
                collector.write(XL430.TorqueEnable, 1)

                #Tell Motor to move at Velocity 200
                collector.write(XL430.GoalVelocity,140)

                response.is_successful = True

                # start = time.time()
                # # Get Motor Position
                # pos = collector.read(XL430.PresentPosition, 0)
                # # Loop until motor reaches position 1200+
                # while(pos % 4096 <= 1200 or pos % 4096 >= 4086):
                #     pos = collector.read(XL430.PresentPosition, 0)
                # # Loop from current position (should be 1200+) until it spins around and reaches 400+
                # while(pos % 4096 > 1200 or pos % 4096 < 400):
                #     #If 5 seconds have passed, and motor has not reached any acceptable position, return nonsuccessfully
                #     if time.time() - start > 5:
                #         response.is_successful = False
                #         break
                #     pos = collector.read(XL430.PresentPosition, 0)

                # collector.write(XL430.TorqueEnable, 0)

                return response
            else:
            
                collector = -1
                if request.collector_id == 1:
                    collector = self.collector1_motor
                elif request.collector_id == 2:
                    collector = self.collector2_motor
                elif request.collector_id == 3:
                    collector = self.collector3_motor
                else:
                    self.get_logger().error(f'Provided collector ID "{request.collector_id}" is not in the valid range of 1-3.')
                    return
                
                collector.write(XL430.TorqueEnable, 0)
                collector.write(XL430.OperatingMode, 1)

                response.is_successful = True

                return response
        except Exception as e:
            #figure out how to send exception to GUI
            #traceback.format_exc()
            response.is_successful = False
            return response

    

    def send_collector_home_callback(self, request, response):
        try:
            # Get Requested Collector
            collector = -1
            if request.collector_id == 1:
                collector = self.collector1_motor
            elif request.collector_id == 2:
                collector = self.collector2_motor
            elif request.collector_id == 3:
                collector = self.collector3_motor
            else:
                self.get_logger().error(f'Provided collector ID "{request.collector_id}" is not in the valid range of 1-3.')
                return
            
            # Put Motor in Position Control Mode
            collector.write(XL430.TorqueEnable, 0)
            collector.write(XL430.OperatingMode, 3)
            collector.write(XL430.TorqueEnable, 1)

            # Send Motor to Position 0
            collector.write(XL430.GoalPosition, 0)

            response.is_successful = True

            start = time.time()
            # Get Motor Position
            pos = collector.read(XL430.PresentPosition, 0)
            # Loop until Motor is in Position 0-9 out of range 0-4095
            while(pos > 10):
                #If 5 seconds have passed, and motor has not reached any acceptable position, return nonsuccessfully
                if time.time() - start > 5:
                    response.is_successful = False
                    break
                pos = collector.read(XL430.PresentPosition, 0)

            collector.write(XL430.TorqueEnable, 0)

            return response
        except Exception as e:
            #figure out how to send exception to GUI
            #traceback.format_exc()
            response.is_successful = False
            return response

    def reboot_scoop_callback(self, request, response):
        try:
            # Get Requested Collector
            collector = -1
            if request.collector_id == 1:
                collector = self.collector1_motor
            elif request.collector_id == 2:
                collector = self.collector2_motor
            elif request.collector_id == 3:
                collector = self.collector3_motor
            else:
                self.get_logger().error(f'Provided collector ID "{request.collector_id}" is not in the valid range of 1-3.')
                return
                
            collector.reboot(request.collector_id)
            response.is_successful = True
            return response
        except Exception as e:
            #figure out how to send exception to GUI
            #traceback.format_exc()
            response.is_successful = False
            return response

        

def main(args=None):

    rclpy.init(args=args)
    dynamixel_control_node = DynamixelControlNode()
    rclpy.spin(dynamixel_control_node)
    dynamixel_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
