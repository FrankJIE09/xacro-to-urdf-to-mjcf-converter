import mujoco
import mujoco.viewer
import time
import os


def run_simulation():
    """
    一个完整的函数，用于创建、运行并控制一个由肌腱驱动的MuJoCo小车模型。
    (已修正不稳定的gear值)
    """

    # =========================================================================
    # 1. 完整的MJCF模型定义 (作为Python字符串)
    # =========================================================================
    xml_string = """
<mujoco model="Tendon Driven Car">
  <compiler angle="degree" coordinate="local" inertiafromgeom="true"/>

  <option integrator="RK4" timestep="0.01"/>

  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>

    <body name="chassis" pos="0 0 0.5">
      <geom type="box" size="0.5 0.3 0.1" rgba="0 0.8 0.3 1"/>
      <joint type="free" name="root"/>

      <body name="left_wheel" pos="0 0.35 0">
        <joint name="left_wheel_hinge" type="hinge" axis="-1 1 0" pos="0 0 0"/>
        <geom type="cylinder" size="0.15 0.05" rgba="0.2 0.5 1 1" mass="0.1"/>
      </body>

      <body name="right_wheel" pos="0 -0.35 0">
        <joint name="right_wheel_hinge" type="hinge" axis="-1 1 0" pos="0 0 0"/>
        <geom type="cylinder" size="0.15 0.05" rgba="1 0.3 0.3 1" mass="0.1"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <fixed name="forward_tendon">
      <joint joint="left_wheel_hinge" coef="1"/>
      <joint joint="right_wheel_hinge" coef="1"/>
    </fixed>
    <fixed name="turn_tendon">
      <joint joint="left_wheel_hinge" coef="-1"/>
      <joint joint="right_wheel_hinge" coef="1"/>
    </fixed>
  </tendon>

  <actuator>
    <general name="forward_motor" tendon="forward_tendon" gear="15"/>
    <general name="turn_motor" tendon="turn_tendon" gear="10"/>
  </actuator>
</mujoco>
"""

    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    control_mode = 'turn'

    if control_mode == 'forward':
        data.ctrl[0] = 1.0
        data.ctrl[1] = 0.0
        print("控制模式: 前进。gear值已修正。")
    elif control_mode == 'turn':
        data.ctrl[0] = 0.0
        data.ctrl[1] = 1.0
        print("控制模式: 转向。gear值已修正。")

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("\n仿真启动... 按下 ESC 或关闭窗口退出。")
            log_timer = time.time()

            left_wheel_id = model.joint('left_wheel_hinge').id
            right_wheel_id = model.joint('right_wheel_hinge').id

            left_wheel_dof_addr = model.jnt_dofadr[left_wheel_id]
            right_wheel_dof_addr = model.jnt_dofadr[right_wheel_id]

            while viewer.is_running():
                step_start = time.time()
                mujoco.mj_step(model, data)

                if time.time() - log_timer > 0.2:
                    left_speed = data.qvel[left_wheel_dof_addr]
                    right_speed = data.qvel[right_wheel_dof_addr]
                    print(f"左轮(蓝)速度: {left_speed:.2f}, 右轮(红)速度: {right_speed:.2f}")
                    log_timer = time.time()

                viewer.sync()
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    except Exception as e:
        print(f"仿真过程中发生错误: {e}")


if __name__ == '__main__':
    run_simulation()