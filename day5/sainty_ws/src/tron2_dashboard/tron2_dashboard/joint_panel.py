"""Portable ROS-only console for the native WF motor modes."""
from xml.etree import ElementTree

import yaml
from ament_index_python.packages import get_package_share_path
import math
import tkinter as tk
from tkinter import ttk
from time import monotonic

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

TIMEOUT = 0.25


def model_limits():
    share = get_package_share_path('tron2_description')
    urdf = ElementTree.parse(share / 'urdf/wf_tron2a.urdf')
    joints = {j.get('name'): tuple(float(j.find('limit').get(k)) for k in ('lower', 'upper'))
              for j in urdf.findall("joint[@type='revolute']")}
    cylinder = urdf.find("link[@name='wheel_L_Link']/collision/geometry/cylinder")
    cfg = yaml.safe_load((share / 'config/wf_control.yaml').read_text())['PointfootCfg']
    return joints, round(cfg['commands']['max']['lin_vel_x'] / float(cylinder.get('radius')), 6)


def main():
    rclpy.init()
    node = rclpy.create_node('joint_panel', namespace='/tron2')
    sim = node.declare_parameter('simulation', False).value
    pub = node.create_publisher(JointState, 'wf_joint_controller/targets', 1)
    root = tk.Tk()
    root.title(('시뮬레이션' if sim else '실기체') + ' · TRON2 WF 관절 패널')
    panel = ttk.Frame(root, padding=12)
    panel.grid()
    for i, h in enumerate(('관절', '목표', '실측')):
        ttk.Label(panel, text=h).grid(row=0, column=i)
    sliders, labels = {}, {}
    status = tk.StringVar(value='피드백 수신 대기')
    held = False
    last_state = last_status = 0.
    allowed = False
    seeded = False
    limits, wheel_speed = model_limits()
    for row, (name, (low, high)) in enumerate(limits.items(), 1):
        wheel = name.startswith('wheel_')
        unit = 'rad/s' if wheel else '°'
        lo, hi = (-wheel_speed, wheel_speed) if wheel else (
            math.ceil(math.degrees(low)), math.floor(math.degrees(high)))
        ttk.Label(panel, text=f'{name.removesuffix("_Joint")} ({unit})').grid(row=row, column=0, sticky='w')
        slider = tk.Scale(panel, orient='horizontal', from_=lo, to=hi, resolution=.05 if wheel else 1,
                          length=240)
        slider.set(0)
        slider.grid(row=row, column=1)
        sliders[name] = slider
        labels[name] = ttk.Label(panel, width=9, anchor='e')
        labels[name].grid(row=row, column=2)

    def ready():
        return allowed and pub.get_subscription_count() > 0 and monotonic()-last_state <= TIMEOUT and monotonic()-last_status < 0.5

    def receive(msg):
        nonlocal last_state, seeded
        if set(msg.name) != set(sliders) or not len(msg.name) == len(msg.position) == len(msg.velocity):
            return
        if not all(math.isfinite(v) for v in (*msg.position, *msg.velocity)):
            return
        if monotonic()-last_state > TIMEOUT:
            seeded = False
        for name, p, v in zip(msg.name, msg.position, msg.velocity):
            wheel = name.startswith('wheel_')
            labels[name].configure(text=f'{v if wheel else math.degrees(p):.2f}')
            if not seeded:
                sliders[name].set(0 if wheel else math.degrees(p))
        seeded = True
        last_state = monotonic()

    def permission(msg):
        nonlocal allowed, last_status
        allowed, last_status = msg.data, monotonic()

    def release(event=None):
        nonlocal held
        if held:
            held = False
            pub.publish(JointState())

    def press(event):
        nonlocal held
        held = ready()

    button = ttk.Button(panel, text='명령 해제' if sim else '누르는 동안 적용')
    button.grid(row=11, column=0)
    if sim:
        button.configure(command=release)
        for slider in sliders.values():
            for event in ('<ButtonPress-1>', '<ButtonPress-2>', '<KeyPress>'):
                slider.bind(event, press)
    else:
        button.bind('<ButtonPress-1>', press)
        root.bind('<ButtonRelease-1>', release)
        button.bind('<Leave>', release)
        root.bind('<FocusOut>', release)
    ttk.Label(panel, textvariable=status).grid(row=11, column=1, columnspan=2)
    node.create_subscription(JointState, 'joint_state_broadcaster/joint_states', receive, 1)
    node.create_subscription(Bool, 'panel_motion_enabled', permission, 1)

    def poll():
        if not rclpy.ok():
            root.quit()
            return
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0)
        if not ready():
            release()
        if held:
            m = JointState()
            m.header.stamp = node.get_clock().now().to_msg()
            m.name = list(sliders)
            m.position = [0. if n.startswith('wheel_') else math.radians(sliders[n].get()) for n in m.name]
            m.velocity = [sliders[n].get() if n.startswith('wheel_') else 0. for n in m.name]
            pub.publish(m)
        text = '적용 중' if held else '목표 설정 후 버튼을 누르세요'
        if sim and not held:
            text = '슬라이더를 움직이면 즉시 적용합니다'
        if not ready():
            text = '관찰 모드' if monotonic()-last_state < TIMEOUT else '피드백 수신 대기'
        status.set(text)
        root.after(40, poll)
    try:
        root.after(40, poll)
        root.mainloop()
    finally:
        if rclpy.ok():
            release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
