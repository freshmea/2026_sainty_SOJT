import rclpy
from rclpy.node import Node


class Simple_pub(Node):
    def __init__(self):
        super().__init__("simple_pub")
        self.create_timer(1.0/5, self.print_hello)

    def print_hello(self):
        print("Hello, ROS2 World!")


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
