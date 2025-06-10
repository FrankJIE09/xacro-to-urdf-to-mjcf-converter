# convert_elfin.py
import mujoco
import os

# 定义URDF文件的完整路径
# 假设你当前执行脚本的目录就是 urdf/ 目录，那么路径是相对的
# 如果你在其他目录执行，需要给出完整的路径
urdf_file_path = "elfin3.urdf"

# 定义MJCF文件的输出路径
mjcf_output_path = "elfin3.xml"

try:
    print(f"尝试从URDF文件加载模型: {urdf_file_path}")
    # MuJoCo会尝试解析 package:// 路径
    model = mujoco.MjModel.from_xml_path(urdf_file_path)
    print("模型加载成功！")

    # 将模型保存为MJCF格式
    mujoco.mj_saveLastXML(mjcf_output_path, model)
    print(f"URDF '{urdf_file_path}' 已成功转换为 MJCF '{mjcf_output_path}'.")

except Exception as e:
    print(f"转换URDF到MJCF时发生错误: {e}")
    print("请确保：")
    print("1. 你的ROS环境已正确source（尤其是ROS_PACKAGE_PATH环境变量）。")
    print("2. 'elfin_description' 包在你的ROS包路径中可以找到。")
    print(f"当前ROS_PACKAGE_PATH: {os.environ.get('ROS_PACKAGE_PATH', '未设置')}")