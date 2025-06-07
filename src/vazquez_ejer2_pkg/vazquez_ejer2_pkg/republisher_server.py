import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from vazquez_ejer2_interfaces.action import RepublishText

class RepublisherClient(Node):
    def __init__(self, text):
        super().__init__('republisher_client')
        self._action_client = ActionClient(self, RepublishText, 'republish_text')
        self._text = text

    def send_goal(self):
        self._action_client.wait_for_server()
        goal_msg = RepublishText.Goal()
        goal_msg.text = self._text

        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        print(f'nodo2: {feedback_msg.feedback.current_word}')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            print('nodo2: Texto republicado!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    text = ' '.join(sys.argv[1:])  # texto pasado por terminal o por launch
    node = RepublisherClient(text)
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
