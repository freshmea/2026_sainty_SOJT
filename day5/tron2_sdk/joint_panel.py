"""Send position targets from ten sliders and display measured motor angles."""
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from sensor_msgs.msg import JointState

from joint_control import COMMAND_TOPIC, limits


def main():
    rclpy.init()
    node = rclpy.create_node('joint_panel', namespace='/tron2')
    publisher = node.create_publisher(JointState, COMMAND_TOPIC, 1)
    root = tk.Tk()
    root.title('TRON2 WF | Joint control')
    panel = ttk.Frame(root, padding=12)
    panel.grid(sticky='nsew')
    for col, title in enumerate(('관절', '목표 각도 (°)', '실측 (°)')):
        ttk.Label(panel, text=title).grid(row=0, column=col)
    sliders, feedback = {}, {}
    status = tk.StringVar(value='모터 상태 수신 대기')

    def send(_=None):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(sliders)
        msg.position = [math.radians(s.get()) for s in sliders.values()]
        publisher.publish(msg)

    for row, (name, (lo, hi)) in enumerate(limits().items(), 1):
        ttk.Label(panel, text=name.removesuffix('_Joint')).grid(row=row, column=0, sticky='w')
        slider = tk.Scale(panel, from_=math.ceil(math.degrees(lo)), to=math.floor(math.degrees(hi)),
                          length=280, resolution=1, orient='horizontal', command=send)
        slider.set(0)
        slider.grid(row=row, column=1)
        sliders[name] = slider
        feedback[name] = ttk.Label(panel, text='—', width=9, anchor='e')
        feedback[name].grid(row=row, column=2)

    def receive(msg):
        for name, angle in zip(msg.name, msg.position):
            if name in feedback:
                feedback[name].configure(text=f'{math.degrees(angle):.1f}')
        status.set('실측 수신 중 · 슬라이더를 움직이면 목표 각도를 전송합니다')

    def zero():
        for slider in sliders.values():
            slider.set(0)
        send()

    ttk.Button(panel, text='모든 목표 0°', command=zero).grid(row=11, column=0)
    ttk.Label(panel, textvariable=status).grid(row=11, column=1, columnspan=2)
    node.create_subscription(JointState, '/tron2/joint_states', receive, 1)
    def poll():
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            root.after(30, poll)
        else:
            root.quit()
    root.after(30, poll)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
