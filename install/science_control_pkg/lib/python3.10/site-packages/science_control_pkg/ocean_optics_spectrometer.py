import seabreeze
seabreeze.use('pyseabreeze')
from seabreeze.spectrometers import Spectrometer
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import CollectSpectrometerData

# Install the following two commands: 
# pip install seabreeze[pyseabreeze]
# seabreeze_os_setup

# Addtionally, add user to dialout and tty groups using:
# sudo usermod -a -G tty username_here
# sudo usermod -a -G dialout username_here

class SpectrometerNode(Node):

    def __init__(self):
        super().__init__('science_spectrometer_node')
        self.declare_parameter('serial_number', "S14413")
        self.declare_parameter('service_name', '/spectrometer') 

        self.spectrometer_service = self.create_service(
            srv_type= CollectSpectrometerData,
            srv_name= self.get_parameter('service_name').get_parameter_value().string_value,
            callback= self.collect_data
        )


    def collect_data(self, request, response):

        sn = self.get_parameter('serial_number').get_parameter_value().string_value
        
        try:
            spec = Spectrometer.from_serial_number(sn)
        except Spectrometer._backend.SeaBreezeError:
            try:
                self.get_logger().error(f'Failed to open spectrometer by the provided serial number {sn}. Trying the first available.')
                spec = Spectrometer.from_first_available()
            except Spectrometer._backend.SeaBreezeError:
                self.get_logger().error(f'No spectrometers were available')
                raise Exception("Could not find a spectrometer connected.")

        try:
            spec.integration_time_micros(request.integration_time)
            response.wavelengths = list(spec.wavelengths())
            response.intensities = list(spec.intensities())
            response.is_successful = True
        except Exception:
            self.get_logger().info(f'Integration Time "{request.integration_time}" was invalid. Must be between 10-85,000,000')
            response.wavelengths = []
            response.intensities = []
            response.is_successful = False

        return response

def main(args=None):

    rclpy.init(args=args)
    spectrometer_node = SpectrometerNode()
    rclpy.spin(spectrometer_node)
    spectrometer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()