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
<mujoco>
  <worldbody>
    <body name="FL_1">
      <inertial pos="0 0 0" mass="1" diaginertia="1.66667e-05 1.66667e-05 1.66667e-05"/>
      <joint pos="0 0 0" axis="1 0 0" type="slide"/>
      <joint pos="0 0 0" axis="0 1 0" type="slide"/>
      <joint pos="0 0 0" axis="0 0 1" type="slide"/>
    </body>
    <body name="FL_2" pos="0.2 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="1.66667e-05 1.66667e-05 1.66667e-05"/>
      <joint pos="0 0 0" axis="1 0 0" type="slide"/>
      <joint pos="0 0 0" axis="0 1 0" type="slide"/>
      <joint pos="0 0 0" axis="0 0 1" type="slide"/>
    </body>
  </worldbody>
  <deformable>
    <flex name="FL" dim="1" body="world FL_1 FL_2" vertex="-0.2 0 0 0 0 0 0 0 0" element="0 1 1 2"/>
  </deformable>
  <equality>
    <flex flex="FL"/>
  </equality>
</mujoco>
"""

    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    control_mode = 'turn'

    # if control_mode == 'forward':
    #     data.ctrl[0] = 1.0
    #     data.ctrl[1] = 0.0
    #     print("控制模式: 前进。gear值已修正。")
    # elif control_mode == 'turn':
    #     data.ctrl[0] = 0.0
    #     data.ctrl[1] = 1.0
    #     print("控制模式: 转向。gear值已修正。")

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("\n仿真启动... 按下 ESC 或关闭窗口退出。")
            log_timer = time.time()

            # left_wheel_id = model.joint('left_wheel_hinge').id
            # right_wheel_id = model.joint('right_wheel_hinge').id

            # left_wheel_dof_addr = model.jnt_dofadr[left_wheel_id]
            # right_wheel_dof_addr = model.jnt_dofadr[right_wheel_id]

            while viewer.is_running():
                step_start = time.time()
                mujoco.mj_step(model, data)

                if time.time() - log_timer > 0.2:
                    # left_speed = data.qvel[left_wheel_dof_addr]
                    # right_speed = data.qvel[right_wheel_dof_addr]
                    # print(f"左轮(蓝)速度: {left_speed:.2f}, 右轮(红)速度: {right_speed:.2f}")
                    log_timer = time.time()

                viewer.sync()
                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    except Exception as e:
        print(f"仿真过程中发生错误: {e}")


if __name__ == '__main__':
    run_simulation()