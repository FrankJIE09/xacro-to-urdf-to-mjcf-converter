import mujoco
import mujoco.viewer
import os
import time

# 脚本假定从仓库的根目录运行
# This script assumes it's run from the root of the repository.
xml_path = 'mjcf_models/rohand_left/rohand_left.xml'

# 加载模型和数据
# Load the model and data
try:
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
except Exception as e:
    print(f"加载模型时出错: {e}")
    exit()

# 打印用户操作指南
# Print instructions for the user
print("\n=====================================================================")
print("  MuJoCo 仿真器已启动 - 位置控制模式 (持久化)")
print("  --------------------------------------------------")
print("  请在仿真器窗口中，按 'Ctrl+L' (或者在菜单中选择 'Control') 来打开控制面板。")
print("  您使用滑块设定的控制值将会被保持，直到您再次修改它。")
print("\n  模型中的执行器 (data.ctrl) 列表:")

# 列出所有执行器的名称
# List all actuators by name
actuator_names = [model.actuator(i).name for i in range(model.nu)]
for i, name in enumerate(actuator_names):
    print(f"    - 索引 {i}: {name}")
print("=====================================================================\n")


# 使用 launch_passive 启动一个“被动”的查看器，我们自己控制仿真循环
# Launch a "passive" viewer, giving us control over the simulation loop
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 仿真主循环
    # Main simulation loop
    while viewer.is_running():
        step_start = time.time()
        
        # 在这里，我们不修改 data.ctrl。
        # 当用户在GUI中操作滑块时，viewer会在后台为我们更新data.ctrl。
        # 因为我们的循环不会重置它，所以控制信号会一直持续。
        # Here, we do not modify data.ctrl.
        # The viewer updates it for us in the background when the user interacts with the sliders.
        # Since our loop doesn't reset it, the control signal persists.
        mujoco.mj_step(model, data)

        # 同步查看器画面
        # Sync the viewer to update the visualization
        viewer.sync()

        # 稳定仿真速度，使其接近实时
        # Stabilize the simulation speed to be close to real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step) 