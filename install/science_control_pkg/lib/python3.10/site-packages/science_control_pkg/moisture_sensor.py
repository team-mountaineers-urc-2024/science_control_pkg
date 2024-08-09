import rclpy
from rclpy.node import Node
from robot_interfaces.srv import MoistureSensor

# ROS service node that calls a Pico function to read moisture sensor data from analog pins
class MoistureSensorNode(Node):

    def __init__ (self):
        super().__init__('moisture_sensor_node')
        self.declare_parameter('service_name', "MoistureSensor")

        self.moisture_sensor_service = self.create_service(
            srv_type=MoistureSensor, 
            srv_name = self.get_parameter('service_name').get_parameter_value().string_value,
            callback = moisture_sensor_callback
        )

    def moisture_sensor_callback(self, request, response):
        if request.collect == True:
            # call Pico function here!
            # ....
            response.is_successful = True
            return response
        else:
            response.is_successful = False
            
        return response
        

def main(args=None):
    rclpy.init(args=None)
    moisture_node = MoistureSensorNode()
    rclpy.spin(moisture_node)
    rclpy.shutdown(moisture_node)


if __name__ == '__main__':
    main()
