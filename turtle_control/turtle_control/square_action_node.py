import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from my_interface.action import GoToPose
from my_interface.srv import SetSpeedLimits

class SquareActionNode(Node):
    def __init__(self):
        super().__init__('square_action_node')
        
        # Instantiate a Reentrant Callback Group to allow concurrent thread operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Bind the client and servers explicitly to the multi-threaded callback group
        self.reset_client = self.create_client(
            SetSpeedLimits, 'trigger_clean_reset', callback_group=self.callback_group)
        self.action_client = ActionClient(
            self, GoToPose, 'go_to_pose', callback_group=self.callback_group)
            
        self.action_server = ActionServer(
            self, GoToPose, 'draw_square', 
            execute_callback=self.execute_square,
            callback_group=self.callback_group)

    async def execute_square(self, goal_handle):
        self.get_logger().info('Wiping canvas environment...')
        req = SetSpeedLimits.Request()
        
        # Call the reset node asynchronously using await syntax
        future_reset = self.reset_client.call_async(req)
        await future_reset
        
        # Bounding box paths safely inside the canvas limits (0 to 11)
        square_corners = [
            (8.54, 5.54, 1.57),  # Corner 1
            (8.54, 8.54, 3.14),  # Corner 2
            (5.54, 8.54, 4.71),  # Corner 3
            (5.54, 5.54, 0.0)    # Corner 4 (Return Home)
        ]
        
        self.action_client.wait_for_server()
        
        for idx, (x, y, yaw) in enumerate(square_corners):
            pose_goal = GoToPose.Goal()
            pose_goal.target.x = x
            pose_goal.target.y = y
            pose_goal.target.yaw = yaw
            
            self.get_logger().info(f'Sending corner target navigation {idx+1}/4...')
            
            # Send goals asynchronously and await transit completion safely without deadlocks
            send_goal_future = self.action_client.send_goal_async(pose_goal)
            goal_handle_client = await send_goal_future
            
            if not goal_handle_client.accepted:
                self.get_logger().error('Corner target dropped by server node!')
                goal_handle.abort()
                res = GoToPose.Result()
                res.success = False
                return res
                
            result_future = goal_handle_client.get_result_async()
            await result_future
            
        self.get_logger().info('Square trace complete!')
        goal_handle.succeed()
        res = GoToPose.Result()
        res.success = True
        return res

def main(args=None):
    rclpy.init(args=args)
    node = SquareActionNode()
    
    # Spin the node using a MultiThreadedExecutor instead of a single thread loop
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
