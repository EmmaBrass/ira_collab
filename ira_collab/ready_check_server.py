import rclpy
from rclpy.node import Node
from ira_interfaces.srv import ReadyCheck
from pynput import keyboard

class ReadyCheckServer(Node):
    def __init__(self):
        super().__init__('ready_check_server')
        self.srv = self.create_service(ReadyCheck, 'check_ready', self.handle_ready_check)
        self.get_logger().info("Ready check server is ready!")

    def handle_ready_check(self, request, response):

        # If asking the user whether canvas is ready / turn is complete
        if request.done_check == False:
            iterations = 0
            while iterations < 5:
                self.get_logger().info("Press the 'yes' key when ready.")
                with keyboard.Events() as events:
                    # Block for as long as possible
                    event = events.get(1e6)
                    if event.key == keyboard.KeyCode.from_char('z'):
                        response.acknowledged = True
                        self.get_logger().info("Yes has been pressed, sending response as True.")
                        return response
                    else:
                        iterations += 1
            self.get_logger().info("Yes was not pressed after 5 tries... sending response as False.")
            response.acknowledged = False
            return response
        
        # If asking the user if the painting is complete
        else:
            self.get_logger().info("Press the 'yes' key if the painting is complete, or the 'no' key to keep painting.")
            with keyboard.Events() as events:
                # Block for as long as possible
                event = events.get(1e6)
                if event.key == keyboard.KeyCode.from_char('z'):
                    self.get_logger().info("Yes has been pressed, sending response as True.")
                    response.acknowledged = True
                elif event.key == keyboard.KeyCode.from_char('x'):
                    self.get_logger().info("No has been pressed, sending response as False.")
                    response.acknowledged = False
                return response
        

def main(args=None):
    rclpy.init(args=args)
    node = ReadyCheckServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()