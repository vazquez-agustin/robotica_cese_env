import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from vazquez_ejer2_interfaces.action import RepublishText

class RepublisherServer(Node):
    def __init__(self):
        super().__init__('republisher_server')
        self._action_server = ActionServer(
            self,
            RepublishText,
            'republish_text',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Republicando: {goal_handle.request.text}')
        words = goal_handle.request.text.split()

        for word in words:
            feedback_msg = RepublishText.Feedback()
            feedback_msg.current_word = word
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {word}')
            await rclpy.task.Future.sleep(1)

        goal_handle.succeed()
        result = RepublishText.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = RepublisherServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
