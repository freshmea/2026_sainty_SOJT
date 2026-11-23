"""첫 실행: 그래픽 없이 0.2초 동안 공을 떨어뜨린다."""
import mujoco

XML = """
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1"/>
    <body name="ball" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.08" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)
print(f"MuJoCo={mujoco.__version__}; nq={model.nq}, nv={model.nv}")
print(f"before: time={data.time:.3f}, z={data.qpos[2]:.6f}")
for _ in range(100):
    mujoco.mj_step(model, data)
print(f"after:  time={data.time:.3f}, z={data.qpos[2]:.6f}")
assert abs(data.time - 0.2) < 1e-10
assert 0.79 < data.qpos[2] < 0.81
print("PASS: the ball fell during 100 physics steps.")
