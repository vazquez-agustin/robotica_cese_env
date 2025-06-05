import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from rclpy.executors import ExternalShutdownException

class CounterSubscriber(Node):
    def __init__(self):
        super().__init__('counter_subscriber')

        # Se declara argumento de entrada
        self.declare_parameter('reset_counter', 50)
        self.reset_counter = self.get_parameter('reset_counter').get_parameter_value().integer_value
        self.get_logger().info(f'Counter will be reset at ({self.reset_counter})')


        # Se crea el servicio cliente para acceder
        self.reset_client = self.create_client(Trigger, '/reset_counter')

        # Se espera a que el servicio este creado por un servidor
        if not self.reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Server /reset_counter is not available.')

        self.subscription = self.create_subscription(
            Int32,
            'counter_topic',
            self.subscription_listener_callback,
            10)

    def subscription_listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        if  msg.data >= self.reset_counter:
            self.get_logger().info(f'Counter reached ({self.reset_counter}), calling service /reset_counter')
            self.request = Trigger.Request()
            self.future = self.reset_client.call_async(self.request)
            self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        response = self.future.result()  # Trigger.Response
        self.get_logger().info(f'Response received: success={response.success} message="{response.message}"')
            

def main(args=None):
    rclpy.init(args=args)
    node = CounterSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
