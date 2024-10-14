

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

from ira_common.general_gpt import GPT

from ira_interfaces.msg import GptComplete
from ira_interfaces.msg import SystemState
from ira_interfaces.msg import CanvasImage

import time, cv2, os

class GPTNode(Node):
        
    def __init__(self):
        super().__init__('gpt_node')
        self.declare_parameter('sim', False)
        self.sim_mode = self.get_parameter('sim').get_parameter_value().bool_value

        self.gpt = GPT(collab=True)
        self.state_seq = -1

        # To track whether still painting in my_turn
        self.still_painting = False
        
        self.canvas_image = None

        # Initialise publishers
        self.gpt_complete_publisher = self.create_publisher(GptComplete, 'gpt_complete', 10)

        # Initialise subscribers
        self.canvas_image_subscription = self.create_subscription(
            CanvasImage,
            'canvas_image',
            self.canvas_image_callback, 
            10
        )
        self.system_state_subscription = self.create_subscription(
            SystemState,
            'system_state',
            self.system_state_callback, 
            10
        )

        time.sleep(10)
        self.get_logger().info("GPT node initialised")
        self.get_logger().info(f"Simulation mode: {self.sim_mode}")

    def canvas_image_callback(self, msg):
        """
        Save the most recent cropped foi image.
        """
        self.get_logger().info("In canvas_image_callback")
        self.canvas_image = self.bridge.imgmsg_to_cv2(msg.image)

    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        if msg.seq > self.state_seq or self.still_painting == True:
            self.state_seq = msg.seq
            if msg.state == 'startup_ready':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <startup_ready>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'startup_pic':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <startup_pic>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'your_turn':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <your_turn>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'your_turn_pic':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <your_turn_pic>")
                self.gpt_complete(msg.seq)
            elif msg.state == 'comment':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <comment>")
                self.gpt_complete(msg.seq)
            elif msg.state == 'my_turn':
                if self.still_painting == False:
                    response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <my_turn>")
                    self.get_logger().info(response)
                    self.gpt_complete(msg.seq)
                    self.still_painting = True
                elif self.still_painting == True:
                    time.sleep(5)
                    response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <still_my_turn>") 
                    self.get_logger().info(response)
                    self.gpt_complete(msg.seq)
            elif msg.state == 'my_turn_pic':
                self.still_painting = False
                time.sleep(3) # give time for arm to look down so it seems like IRA has seen her work before she comments.
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <my_turn_pic>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'ask_done':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <ask_done>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'completed':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <completed>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            else:
                self.get_logger().warn("Unknown system state for gpt node.")
                self.gpt_complete(msg.seq)

    def gpt_complete(self, seq):
        self.get_logger().info("In gpt_complete")
        msg = GptComplete()
        msg.seq = seq
        msg.complete = True
        for i in range(5):
            self.gpt_complete_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    gpt_node = GPTNode()

    rclpy.spin(gpt_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gpt_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    