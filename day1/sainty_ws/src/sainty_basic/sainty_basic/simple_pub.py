import rclpy
from rclpy.node import Node


def print_hello():
    print("Hello, ROS2 World!")


def main():
    rclpy.init()
    node = Node("simple_pub")
    Hz = 5
    node.create_timer(1.0/Hz, print_hello)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Hello, ROS2 World!")
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
