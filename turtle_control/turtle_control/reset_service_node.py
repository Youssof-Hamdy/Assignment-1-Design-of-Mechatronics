import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen
from my_interface.srv import SetSpeedLimits

class ResetServiceNode(Node):
    def __init__(self):
        super().__init__('reset_service_node')
        self.srv = self.create_service(SetSpeedLimits, 'trigger_clean_reset', self.handle_reset)
        
        self.clear_cli = self.create_client(Empty, '/clear')
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen')

    def handle_reset(self, request, response):
        pen_req = SetPen.Request(off=1)
        self.pen_cli.call_async(pen_req)
        
        tele_req = TeleportAbsolute.Request(x=5.54, y=5.54, theta=0.0)
        self.teleport_cli.call_async(tele_req)
        
        pen_req_on = SetPen.Request(r=255, g=255, b=255, width=3, off=0)
        self.pen_cli.call_async(pen_req_on)
        
        self.clear_cli.call_async(Empty.Request())
        
        response.success = True
        response.message = "Turtlesim canvas wiped and turtle teleported successfully."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetServiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
