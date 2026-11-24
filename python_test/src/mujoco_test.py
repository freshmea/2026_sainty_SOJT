import mujoco

XML = """
<mujoco model="example">
    <worldbody>
        <body name="box" pos="0 0 0.5">
            <geom type="box" size="0.1 0.1 0.1"/>
        </body>
    </worldbody>
</mujoco>
"""

def main():
    print("ok")
    # mujoco
    print(mujoco.__version__)
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    print(f"{data.time:.3f}")
    mujoco.mj_step(model, data)
    print(f"{data.time:.3f}")

if __name__ == "__main__":
    main()
