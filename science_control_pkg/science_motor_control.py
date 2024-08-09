'''
Have the dynamixel control code read through the motor ids and 
validate if the ids given in the parameters match the ones read.
If not, raise Exception or give error of some sort.

----------
Alternative solution (Something like this):
----------

    collected_ids = [] 

    for index, id in enumerate(collected_ids):
        send baurdate command to change to unused baudrate
        send message to change id to index using new baudrate
        send message to change baudrate to user-desired baudrate (whichever is default in dynamixel library)

    now know all motors are id 0 through number of motors found 

    still have issue of mapping motors to physcial places. May not 
    be an error if control each scoop (or whatever it ends up being)
    sequentially
'''
from dynamixel_lib import Dynamixel, U2D2, MX106
import rclpy, time
from rclpy.node import Node
from rclpy.action import ActionServer
from science_control_pkg.action import LowerCollector, SpinCentrifuge


class ScienceMotorNode(Node):

    def __init__(self):
        super().__init__('science_motor_control_node')
        self.declare_parameter('dynamixel_device_path', '/dev/u2d2')
        self.declare_parameter('dynamixel_baudrate', 3000000)
        self.declare_parameter('centrifuge_dynamixel_id', 10)
        self.declare_parameter('centrifuge_action_name', "spin_carousel")

        self.declare_parameter('collector1_dynamixel_id', 20)
        self.declare_parameter('collector2_dynamixel_id', 21)
        self.declare_parameter('collector3_dynamixel_id', 22)

        def make_action_server(motor_id: int):
            return ActionServer(
                node = self,
                action_type = LowerCollector,
                action_name = (f"lower_collector_{motor_id}"),
                execute_callback = lambda goal_handle: self.lower_collector_callback(
                    goal_handle, 
                    motor_id
                    )
                )
        
        self.spin_carousel_action_server = ActionServer(
            self,
            SpinCentrifuge,
            self.get_parameter('centrifuge_action_name').get_parameter_value().string_value,
            self.spin_centrifuge_callback
        )
        
        self.collector_1_action_server = make_action_server(self.get_parameter('collector1_dynamixel_id').get_parameter_value().string_value)
        self.collector_2_action_server = make_action_server(self.get_parameter('collector2_dynamixel_id').get_parameter_value().string_value)
        self.collector_3_action_server = make_action_server(self.get_parameter('collector3_dynamixel_id').get_parameter_value().string_value)

        self.u2d2 = U2D2(
            make_action_server(self.get_parameter('dynamixel_device_path').get_parameter_value().string_value),
            make_action_server(self.get_parameter('dynamixel_baudrate').get_parameter_value().integer_value),
        ) # Note from Shawn: Parameter should not be passed into function??


    def lower_collector_callback(self, goal_handle, motor_id):
        feedback_msg = LowerCollector.Feedback()

        def curr_time_secs():
            secs, nsecs = self.get_clock().now().seconds_nanoseconds
            return secs + (nsecs/1e9)
        
        curr_lowered_meters = 0
        lower_distance_goal = goal_handle.request.lower_collector_meters
        previous_time = curr_time_secs()
        # Feedback loop
        while curr_lowered_meters <= lower_distance_goal:

            self.get_logger().info(f'Collector is being lowered to {lower_distance_goal}. (Currently at {curr_lowered_meters} meters)')

            lin_vel = 1 # THIS NEEDS TO BE READ FROM THE MOTOR, OR SENT TO MOTOR. SETTING TO 1 m/s NOW. 
            new_time = curr_time_secs()
            curr_lowered_meters = lin_vel * ( new_time - previous_time)
            time.sleep(0.001) # wait a millisecond so sampling rate on integration is not too high 

            feedback_msg.curr_meters_lowered = curr_lowered_meters
            goal_handle.publish_feedback(feedback_msg)
            previous_time = new_time


        goal_handle.succeed()
        result = LowerCollector.Result()
        result.final_meters_reached = True 
        result.final_meters_lowered = curr_lowered_meters
        return result
    

    def spin_centrifuge_callback(self, goal_handle):

        feedback_msg = SpinCentrifuge.Feedback()

        def curr_time_secs():
            secs, nsecs = self.get_clock().now().seconds_nanoseconds
            return secs + (nsecs/1e9)
        
        total_seconds_elapsed = 0
        # Feedback loop
        while (total_seconds_elapsed <= goal_handle.request.required_elapsed_seconds_spent_spinning):

            self.get_logger().info(f'Spinning centrifuge')

            motor = Dynamixel(MX106,self.get_parameter('centrifuge_dynamixel_id').get_parameter_value().string_value)
            output = motor.read(MX106.PresentVelocity)
            time.sleep(0.001) # wait a millisecond so sampling rate is not too high
            total_seconds_elapsed += curr_time_secs() 

            feedback_msg.curr_rpm = 420420 # READING FROM MOTOR NEEDS PARSED, WAITING ON THAT FUNCTIONALITY TO COME FROM DYNAMIXEL LIBRARY
            feedback_msg.current_elapsed_seconds_spent_spinning = total_seconds_elapsed
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = SpinCentrifuge.Result()
        result.final_rpm = 0 ; output = motor.read(MX106.PresentVelocity) # READING FROM MOTOR NEEDS PARSED, WAITING ON THAT FUNCTIONALITY TO COME FROM DYNAMIXEL LIBRARY
        result.total_seconds_spent_spinning = total_seconds_elapsed
        return result


def main(args=None):

    rclpy.init(args=args)
    science_motor_mode = ScienceMotorNode()
    rclpy.spin(science_motor_mode)
    science_motor_mode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()