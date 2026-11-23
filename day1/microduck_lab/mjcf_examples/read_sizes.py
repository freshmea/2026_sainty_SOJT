"""Run with a project-relative XML path, e.g. mjcf_examples/native.xml."""
import sys
import mujoco

model = mujoco.MjModel.from_xml_path(sys.argv[1])
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
for i in range(model.ngeom):
    print(model.geom(i).name, "radius", model.geom_size[i, 0],
          "world xyz", data.geom_xpos[i])
