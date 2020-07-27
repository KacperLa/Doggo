import time
import math
from example_interfaces.action import Fibonacci
import rclpy

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy

from sensor_msgs.msg import Joy

RunVirtualRail = Fibonacci

class FSM(Node):
  
    def __init__(self):
        super().__init__('fsm')
        self.mode = "idle"
        self._goal_handle = None
        
        self._action_server = ActionServer(
            self,
            RunVirtualRail,
            'run_virtual_rail',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        #self.request_mode_service = self.create_service(self.request_mode)

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.manual_mode,
            10)
        self.subscription
        
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()
        
    def request_mode(self,request, response):
        self.mode = request.mode
        response.success = True
        return response

    def goal_callback(self, goal_request='idle'):
        """Accepts or rejects a client request to begin an action."""
        if self.mode == "virtual_rail":
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT


    def cancel_callback(self, goal_handle):
        """Accepts or rejects a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


    async def execute_callback(self, goal_handle):
        # this is where the actual virtual rail execution code goes

        # Start executing the action
        while True:
            start_time = current_time()

            # check if the goal has been cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.success = False
                return Fibonacci.Result()

            if path_complete():
                goal_handle.succeed()
                result = Fibonacci.Result()
                result.success = True
                return Fibonacci.Result()

            # Publish a pose to the position controller
            msg=Pose()
            pose.data = calc_position_setpoint(time, percent_complete)
            self.abs_pose_publisher.publish(msg)


            # Publish the feedback, could be current setpoint, percent complete, whatever we need to display in the GUI
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for loop time
            while (current_time() - start_time < loop_time):
                ros.spin_once() # this is te asynchronous bit, allows other call backs to run sunch as cancelling the goal, rejecting incoming goals, basically Python asyncio

    def manual_mode(self, msg):
        x = msg.axes[1]
        y = msg.axes[0]
        a = math.atan2(x, y)+math.pi
        #v = math.sqrt(x**2 + y**2)
        w = msg.axes[3]
        v = -(((msg.axes[5]+1)/2)-1)
        drive_base_cmd = [a,v,w]
        self.get_logger().info('I heard: "%s"' % drive_base_cmd)

    def idle_mode():
        pass
        # do nothing

    def check_vbus():
        if vbus < threshold:
            self.state = error

def main(args=None):
    rclpy.init(args=args)

    fsm = FSM()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(fsm, executor=executor)

    fsm.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
