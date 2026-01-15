import mujoco
import mujoco.viewer
import time
import numpy as np
XML="""
<mujoco>
  <option timestep="0.002"/>

  <worldbody>
    <geom type="plane" size="0 0 0.1" rgba="1 1 1 1"/>

    <body name="walker" pos="0 0 1.0">
      ...
    </body>
  </worldbody>
</mujoco>
"""

class Environment:
    def __init__(self):
        # Load and merge plane + robot
        with open("/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/URDF_files/walker_assembly/urdf/walker_assembly.xml") as f:
            robot_xml = f.read()
        XML = robot_xml.replace(
            "<mujoco>",
            """<mujoco>
               <option timestep="0.002" gravity="0 0 -9.81"/>"""
        ).replace(
            "<worldbody>",
            """<worldbody>
               <geom name="ground" type="plane" pos="0 0 -0.2"
                     size="1 1 0.1"
                     rgba="0.8 0.8 0.8 1"
                     friction="1 0.8 0.01"/>"""
        )
        f=open("/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/URDF_files/walker_assembly/urdf/temp.xml","w")
        f.write(XML)
        f.close()
        self.model = mujoco.MjModel.from_xml_path("/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/URDF_files/walker_assembly/urdf/temp.xml")
        self.data = mujoco.MjData(self.model)
        for i in range(self.model.nu):
          print(i, mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i))
    def launch(self):
        viewer=mujoco.viewer.launch_passive(self.model, self.data)
        t = 0
        while viewer.is_running() and t<100:
            #self.data.ctrl[0] = 1.0 * np.sin(t)
            mujoco.mj_step(self.model, self.data)
            viewer.sync()
            t += 0.01
        viewer.close()
                


if __name__ == "__main__":
    env=Environment()
    env.launch()
    