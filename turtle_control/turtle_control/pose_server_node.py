import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_interface.action import GoToPose
from my_interface.srv import SetSpeedLimits
import math

class PoseServerNode(Node):
    def __init__(self):
        super().__init__('pose_server_node')
        
        # Use a Reentrant Callback Group to handle position updates while tracking loops run
        self.callback_group = ReentrantCallbackGroup()
        
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Explicitly attach the subscriber and servers to the callback group
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10, callback_group=self.callback_group)
        
        self.speed_srv = self.create_service(
            SetSpeedLimits, 'set_speed_limits', self.set_speed_limits_callback, callback_group=self.callback_group)
        
        self.action_server = ActionServer(
            self, GoToPose, 'go_to_pose', self.execute_action_callback, callback_group=self.callback_group)
        
        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg

    def set_speed_limits_callback(self, request, response):
        if request.max_linear_speed <= 0 or request.max_angular_speed <= 0:
            response.success = False
            response.message = "Speed limits must be positive non-zero values!"
            return response
        
        self.set_parameters([
            rclpy.parameter.Parameter('max_linear_speed', rclpy.Parameter.Type.DOUBLE, request.max_linear_speed),
            rclpy.parameter.Parameter('max_angular_speed', rclpy.Parameter.Type.DOUBLE, request.max_angular_speed)
        ])
        response.success = True
        response.message = "Successfully updated speed limits."
        return response

    async def execute_action_callback(self, goal_handle):
        self.get_logger().info('Driving to target point...')
        feedback_msg = GoToPose.Feedback()
        target = goal_handle.request.target
        
        rate = self.create_rate(20) # 20Hz
        
        # Safe delay waiting for initial simulation data
        while self.current_pose is None:
            await rclpy.task.ValueReadyTask(None) # Safe asynchronous yield point
            
        while True:
            max_linear = self.get_parameter('max_linear_speed').value
            max_angular = self.get_parameter('max_angular_speed').value
            
            dx = target.x - self.current_pose.x
            dy = target.y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.1: # Safe linear tolerance
                break
                
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_pose.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            cmd = Twist()
            if abs(angle_error) > 0.2:
                cmd.linear.x = 0.0
                cmd.angular.z = max(min(2.0 * angle_error, max_angular), -max_angular)
            else:
                cmd.linear.x = max(min(1.5 * distance, max_linear), -max_linear)
                cmd.angular.z = max(min(2.0 * angle_error, max_angular), -max_angular)
                
            self.cmd_vel_pub.publish(cmd)
            
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)
            rate.sleep()

        # Final Heading Yaw Alignment Loop
        while True:
            max_angular = self.get_parameter('max_angular_speed').value
            angle_error = target.yaw - self.current_pose.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            if abs(angle_error) < 0.05: # Safe angle tolerance
                break
                
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = max(min(1.5 * angle_error, max_angular), -max_angular)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.cmd_vel_pub.publish(Twist())
        goal_handle.succeed()
        
        result = GoToPose.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PoseServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
