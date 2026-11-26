"""Single-window mouse controls and a live MuJoCo view for the WF ONNX policy."""
import math
import time
import tkinter as tk

import mujoco
from PIL import Image, ImageTk


class DrivePanel:
    def __init__(self, root, env, actor, renderer, speed, yaw_rate):
        self.root = root
        self.env = env
        self.actor = actor
        self.input_name = actor.get_inputs()[0].name
        self.renderer = renderer
        self.commands = {
            "전진": (speed, 0.), "후진": (-speed, 0.),
            "좌회전": (0., yaw_rate), "우회전": (0., -yaw_rate), "정지": (0., 0.),
        }
        self.period = env.model.opt.timestep * env.substeps
        self.camera = mujoco.MjvCamera()
        self.camera.azimuth, self.camera.elevation, self.camera.distance = 112, -13, 2.35
        env.model.geom_group[env.floor] = 2
        self.options = mujoco.MjvOption()
        self.options.geomgroup[0] = 0
        self.options.sitegroup[:] = 0

        root.title("TRON2 WF | 마우스 조종")
        root.resizable(False, False)
        self.screen = tk.Label(root, borderwidth=0)
        self.screen.pack()
        self.status = tk.StringVar()
        tk.Label(root, textvariable=self.status, font=("Noto Sans CJK KR", 11),
                 anchor="w", padx=12).pack(fill="x", pady=(5, 2))
        pad = tk.Frame(root)
        pad.pack(pady=(2, 10))
        layout = [("전진", 0, 1), ("좌회전", 1, 0), ("정지", 1, 1),
                  ("우회전", 1, 2), ("후진", 2, 1)]
        self.buttons = {}
        for label, row, col in layout:
            button = tk.Button(pad, text=label, width=10, font=("Noto Sans CJK KR", 11),
                               command=lambda name=label: self.select(name))
            button.grid(row=row, column=col, padx=3, pady=1)
            self.buttons[label] = button
        tk.Button(pad, text="자세 초기화", command=self.reset).grid(row=2, column=2)
        self.reset()

    def select(self, name):
        if self.fallen and name != "정지":
            return
        self.selected = name
        for key, button in self.buttons.items():
            active = key == name
            button.configure(relief="sunken" if active else "raised",
                             bg="#a9d8f6" if active else "#ececec")

    def reset(self):
        self.obs, _ = self.env.reset(seed=101, options={"speed": 0.})
        self.fallen = False
        self.select("정지")
        self.next_step = time.monotonic()

    def update(self):
        now = time.monotonic()
        # Keep physics at 50 Hz and redraw separately. Bound catch-up after dragging the window.
        self.next_step = max(self.next_step, now - 4*self.period)
        vx, wz = self.commands[self.selected]
        while not self.fallen and self.next_step <= now:
            self.env.speed = vx
            self.obs[-3:] = [vx, 0., wz]
            action = self.actor.run(None, {self.input_name: self.obs})[0].ravel()
            # The eight-second evaluation horizon does not end a manual driving session.
            self.obs, _, self.fallen, _, _ = self.env.step(action)
            self.next_step += self.period
            if self.fallen:
                self.select("정지")

        d = self.env.data
        base = d.body("base_Link")
        self.camera.lookat[:] = base.xpos
        self.camera.lookat[2] = max(.48, base.xpos[2] * .66)
        self.renderer.update_scene(d, camera=self.camera, scene_option=self.options)
        photo = ImageTk.PhotoImage(Image.fromarray(self.renderer.render()), master=self.root)
        self.screen.configure(image=photo)
        self.screen.image = photo

        heading = math.atan2(base.xmat[3], base.xmat[0])
        forward = d.qvel[0]*math.cos(heading) + d.qvel[1]*math.sin(heading)
        state = "넘어짐 · 자세 초기화를 누르세요" if self.fallen else self.selected
        self.status.set(f"{state}    |    {d.time:.1f} s    |    실제 전후 속도 {forward:+.2f} m/s"
                        f"    |    진행 방향 {math.degrees(heading):+.0f}°")
        self.root.after(15, self.update)


def drive(env, actor, speed, yaw_rate):
    root = tk.Tk()
    try:
        width, height = 720, 420
        env.model.vis.global_.offwidth = max(width, env.model.vis.global_.offwidth)
        env.model.vis.global_.offheight = max(height, env.model.vis.global_.offheight)
        with mujoco.Renderer(env.model, width=width, height=height) as renderer:
            panel = DrivePanel(root, env, actor, renderer, speed, yaw_rate)
            root.protocol("WM_DELETE_WINDOW", root.quit)
            panel.update()
            root.mainloop()
    finally:
        root.destroy()
