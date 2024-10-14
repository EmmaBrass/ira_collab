import rclpy
from rclpy.node import Node
from ira_interfaces.srv import ReadyCheck

class ReadyCheckServer(Node):
    def __init__(self):
        super().__init__('ready_check_server')
        self.srv = self.create_service(ReadyCheck, 'check_ready', self.handle_ready_check)

    def handle_ready_check(self, request, response):
        response.acknowledged = input("Is the canvas ready? (y/n): ") == 'y'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ReadyCheckServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()