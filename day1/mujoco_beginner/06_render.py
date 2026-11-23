"""GL 사용 가능 시 PNG 저장. 렌더 성공은 별도 검증한다."""
import mujoco
from PIL import Image
from common import load_model, RESULTS

model, data = load_model("arm.xml")
data.joint("hinge").qpos[0] = 0.7853981634
mujoco.mj_forward(model, data)
camera = mujoco.MjvCamera()
camera.lookat[:] = [0.17, 0, 0.5]
camera.distance = 1.4
camera.azimuth = 120
camera.elevation = -35
with mujoco.Renderer(model, height=640, width=960) as renderer:
    renderer.update_scene(data, camera=camera)
    pixels = renderer.render()
Image.fromarray(pixels).save(RESULTS / "06_arm.png")
print("Saved:", RESULTS / "06_arm.png")
