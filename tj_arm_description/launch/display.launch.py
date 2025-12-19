import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. 获取功能包路径
    pkg_share = get_package_share_directory('tj_arm_description')
    
    # 2. 拼接 URDF 路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'TJ_arm', 'tianji_dual_arms.urdf')
    
    # ==========================================
    # 【新增】打印路径到终端 (绿色高亮，方便查看)
    # ==========================================
    print("\n" + "="*50)
    print(f"\033[92m[DEBUG] URDF File Path: {urdf_file}\033[0m")
    print("="*50 + "\n")
    # ==========================================

    # 3. 读取 URDF 内容
    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)

    # 4. 定义节点

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 5. 返回 Launch 描述
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])