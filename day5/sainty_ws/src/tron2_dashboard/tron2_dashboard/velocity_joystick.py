"""Spring-centred virtual stick controlling body forward speed and yaw rate."""
import math
import signal
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = rclpy.create_node('velocity_joystick')
    max_linear = node.declare_parameter('max_linear', .7).value
    max_yaw = node.declare_parameter('max_yaw', 1.).value
    publisher = node.create_publisher(Twist, 'cmd_vel', 1)
    root = tk.Tk()
    root.title('TRON2 WF · 속도 조이스틱')
    root.resizable(False, False)
    root.configure(bg='#f4f7f5')
    tk.Label(root, text='WF  |  속도 조이스틱', font=('Noto Sans CJK KR', 16, 'bold'),
             bg='#f4f7f5', fg='#173c43').pack(pady=(12, 2))
    tk.Label(root, text=f'놓으면 정지  |  ±{max_linear:g} m/s · ±{max_yaw:g} rad/s', bg='#f4f7f5').pack()
    canvas = tk.Canvas(root, width=330, height=330, bg='#f4f7f5', highlightthickness=0)
    canvas.pack()
    center, radius = 165, 111
    canvas.create_oval(54, 54, 276, 276, fill='#e1edea', outline='#527f77', width=2)
    canvas.create_line(54, 165, 276, 165, fill='#9ebbb4')
    canvas.create_line(165, 54, 165, 276, fill='#9ebbb4')
    for x, y, label in [(165, 35, '전진'), (165, 294, '후진'), (28, 165, '좌회전'), (302, 165, '우회전')]:
        canvas.create_text(x, y, text=label, fill='#173c43')
    knob = canvas.create_oval(0, 0, 0, 0, fill='#176a65', outline='white', width=2)
    status = tk.StringVar()
    tk.Label(root, textvariable=status, bg='#173c43', fg='white', padx=14, pady=9).pack(fill='x')
    cmd = Twist()

    def set_stick(x, y):
        scale = max(1., math.hypot(x, y))
        x, y = x / scale, y / scale
        cmd.linear.x, cmd.angular.z = -max_linear * y, -max_yaw * x
        px, py = center + radius*x, center + radius*y
        canvas.coords(knob, px-14, py-14, px+14, py+14)
        status.set(f'전후 {cmd.linear.x:+.2f} m/s     회전 {cmd.angular.z:+.2f} rad/s')
        if rclpy.ok():
            publisher.publish(cmd)

    def drag(event):
        set_stick((event.x-center)/radius, (event.y-center)/radius)

    def stop(event=None):
        set_stick(0., 0.)

    def tick():
        if rclpy.ok():
            publisher.publish(cmd)
            root.after(50, tick)
        else:
            root.quit()

    stop()
    canvas.bind('<ButtonPress-1>', drag)
    canvas.bind('<B1-Motion>', drag)
    root.bind('<ButtonRelease-1>', stop)
    root.bind('<FocusOut>', stop)
    root.bind('<Escape>', stop)
    root.protocol('WM_DELETE_WINDOW', root.quit)
    signal.signal(signal.SIGINT, lambda *_: root.quit())
    signal.signal(signal.SIGTERM, lambda *_: root.quit())
    tick()
    try:
        root.mainloop()
    finally:
        stop()
        root.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
