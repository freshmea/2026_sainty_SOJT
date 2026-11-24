
import mujoco

XML = """
<mujoco model="example">
    <worldbody>
        <body name="box" pos="0 0 0.5">
        <freejoint/>
            <geom type="box" size="0.1 0.1 0.1"/>
        </body>
    </worldbody>
</mujoco>
"""


def main():
    print("ok")
    # mujoco
    print(mujoco.__version__)

    # 변수
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    ## free-shape -> nq 7개(x,y,z, 쿼터니안(x,y,z,w)) nv x,y,z x,y,z 6개 nu 0

    print(f"nq = {model.nq}, nv = {model.nv}, nu = {model.nu}")

    print(f"{data.time:.3f}")
    print(f"{data.qpos[2]:.6f}")
    # 실행
    for _ in range(100):
        mujoco.mj_step(model, data)
        # time.delay(0.002) - (연산시간..)
    print(f"{data.qpos[2]:.6f}")

    print(f"{data.time:.3f}")


if __name__ == "__main__":
    main()
