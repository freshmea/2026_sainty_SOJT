import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Simple_pub(Node):
    def __init__(self):
        super().__init__("simple_pub")
        self.create_timer(1.0/5, self.print_hello)
        self.pub = self.create_publisher(String, "message", 10)

    def print_hello(self):
        data = String()
        data.data = "Hello, ROS2 World!"
        self.pub.publish(data)
        # print(data.data)
        self.get_logger().info(data.data)


def main():
    rclpy.init()
    node = Simple_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Hello, ROS2 World!")
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
