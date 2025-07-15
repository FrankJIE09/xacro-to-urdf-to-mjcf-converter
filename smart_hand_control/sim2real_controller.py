#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Sim2Real 控制器 - 将 MuJoCo 仿真中的手部控制映射到实际的 OHand 灵巧手

功能特点:
1. 实时读取 MuJoCo 仿真中的手部状态
2. 将仿真控制信号映射到实际 OHand 灵巧手
3. 支持位置控制、角度控制和腱索驱动
4. 提供状态监控和错误处理
5. 支持配置文件自定义映射参数

作者: AI Assistant
日期: 2024
"""

import time
import numpy as np
import threading
from typing import Dict, List, Optional, Tuple
import logging
import os

# 导入 MuJoCo Python 绑定
try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("警告: 未找到 mujoco 库，请安装: pip install mujoco")
    mujoco = None

# 导入 OHand Modbus 客户端
try:
    from ohand_modbus_client import OHandModbusClient
except ImportError:
    print("警告: 未找到 ohand_modbus_client.py，请确保文件在同一目录")
    OHandModbusClient = None

# 导入寄存器定义
try:
    from roh_registers_v1 import *
except ImportError:
    print("警告: 未找到 roh_registers_v1.py，请确保文件在同一目录")
    # 定义基本的寄存器地址作为备用
    ROH_FINGER_POS_TARGET0 = 1135
    ROH_FINGER_ANGLE_TARGET0 = 1155
    ROH_FINGER_POS0 = 1145
    ROH_FINGER_ANGLE0 = 1165
    ROH_FINGER_STATUS0 = 1085

class Sim2RealController:
    """
    Sim2Real 控制器类
    将 MuJoCo 仿真中的手部控制映射到实际的 OHand 灵巧手
    """
    
    def __init__(self, 
                 mujoco_model_path: str = "../mjcf_models/rohand_left/rohand_left.xml",
                 serial_port: str = "/dev/ttyUSB0",
                 slave_id: int = 2,
                 mapping_config: Dict = None):
        """
        初始化 Sim2Real 控制器
        
        Args:
            mujoco_model_path: MuJoCo 模型文件路径
            serial_port: OHand 串口端口
            slave_id: OHand Modbus 从站 ID
            mapping_config: 映射配置字典
        """
        self.mujoco_model_path = mujoco_model_path
        self.serial_port = serial_port
        self.slave_id = slave_id
        
        # 默认映射配置
        self.default_mapping = {
            # 手指索引映射 (MuJoCo -> OHand)
            'finger_mapping': {
                'thumb_flex': 0,      # 拇指弯曲
                'index_finger_flex': 1,  # 食指弯曲
                'middle_finger_flex': 2, # 中指弯曲
                'ring_finger_flex': 3,   # 无名指弯曲
                'little_finger_flex': 4, # 小指弯曲
                'thumb_rot': 5,       # 拇指旋转
            },
            
            # 角度范围映射 (MuJoCo角度 -> OHand位置)
            'position_scaling': {
                'thumb_flex': (90.0, 0.0, 0, 50000),      # 拇指弯曲角度范围 90-0度
                'index_finger_flex': (0.0, -90.0, 0, 65535),  # 食指弯曲角度范围 0到-90度
                'middle_finger_flex': (90.0, 0.0, 0, 65535), # 中指弯曲角度范围 90-0度
                'ring_finger_flex': (90.0, 0.0, 0, 65535),   # 无名指弯曲角度范围 90-0度
                'little_finger_flex': (90.0, 0.0, 0, 65535), # 小指弯曲角度范围 90-0度
                'thumb_rot': (92.0, -2.0, 0, 65535),  # 拇指旋转角度范围 92到-2度
            },
            
            # 控制模式
            'control_mode': 'position',  # 'position' 或 'angle'
            
            # 更新频率 (Hz)
            'update_rate': 50,
            
            # 安全限制
            'safety_limits': {
                'max_velocity': 30000,  # 最大速度
                'max_current': 1000,    # 最大电流 (mA)
                'max_force': 5000,      # 最大力量 (mN)
            }
        }
        
        # 合并用户配置
        if mapping_config:
            self.mapping_config = self._merge_config(self.default_mapping, mapping_config)
        else:
            self.mapping_config = self.default_mapping
        
        # 初始化组件
        self.mujoco_data = None
        self.mujoco_model = None
        self.ohand_client = None
        self.viewer = None
        
        # 控制状态
        self.is_running = False
        self.control_thread = None
        self.last_update_time = 0
        
        # pygame显示相关
        self.pygame_screen = None
        self.pygame_font = None
        self.display_thread = None
        self.current_angles = {}
        self.current_ohand_positions = {}
        self.use_terminal_display = False  # 是否使用终端显示
        self.last_display_time = 0  # 上次显示时间
        
        # 日志配置
        self.setup_logging()
        
    def setup_logging(self):
        """配置日志系统"""
        # 禁用日志输出
        logging.getLogger().setLevel(logging.ERROR)
        self.logger = logging.getLogger('Sim2RealController')
        self.logger.setLevel(logging.ERROR)
    
    def initialize_pygame_display(self):
        """初始化pygame显示窗口"""
        # 移除此方法，因为不再使用pygame显示
        pass
    
    def clear_terminal(self):
        """清屏（跨平台）"""
        os.system('cls' if os.name == 'nt' else 'clear')
    
    def print_angles_display(self):
        """在终端打印角度信息（不滚动）"""
        # 使用简单的回车符实现原地更新
        print('\r', end='', flush=True)
        
        # 构建单行显示内容
        lines = []
        lines.append("=" * 60)
        lines.append("           Sim2Real 角度监控")
        lines.append("=" * 60)
        lines.append(f"更新时间: {time.strftime('%H:%M:%S')}")
        lines.append("-" * 60)
        
        # 手指名称映射
        finger_names = {
            'thumb_flex': '拇指弯曲',
            'index_finger_flex': '食指弯曲',
            'middle_finger_flex': '中指弯曲',
            'ring_finger_flex': '无名指弯曲',
            'little_finger_flex': '小指弯曲',
            'thumb_rot': '拇指旋转'
        }
        
        # 添加每个手指的信息
        for finger_key, finger_name in finger_names.items():
            sim_angle = self.current_angles.get(finger_key, 0.0)
            ohand_pos = self.current_ohand_positions.get(finger_key, 0)
            
            lines.append(f"{finger_name:12} | 仿真角度: {sim_angle:6.1f}° | OHand位置: {ohand_pos:6}")
        
        lines.append("-" * 60)
        lines.append("按 Ctrl+C 退出程序")
        lines.append("=" * 60)
        
        # 一次性打印所有内容
        print('\n'.join(lines))
    
    def update_display(self):
        """更新显示"""
        # 移除此方法，因为不再使用pygame显示
        pass
    
    def display_loop(self):
        """显示循环"""
        # 移除此方法，因为不再使用pygame显示
        pass
    
    def _merge_config(self, default: Dict, user: Dict) -> Dict:
        """合并默认配置和用户配置"""
        merged = default.copy()
        for key, value in user.items():
            if isinstance(value, dict) and key in merged:
                merged[key] = self._merge_config(merged[key], value)
            else:
                merged[key] = value
        return merged
    
    def initialize_mujoco(self) -> bool:
        """初始化 MuJoCo 仿真"""
        try:
            if mujoco is None:
                self.logger.error("MuJoCo 库未安装")
                return False

            # 加载模型
            self.mujoco_model = mujoco.MjModel.from_xml_path(self.mujoco_model_path)
            self.mujoco_data = mujoco.MjData(self.mujoco_model)

            self.logger.info("MuJoCo 仿真初始化成功")
            return True

        except Exception as e:
            self.logger.error(f"MuJoCo 初始化失败: {e}")
            return False
    
    def initialize_ohand(self) -> bool:
        """初始化 OHand 灵巧手"""
        try:
            if OHandModbusClient is None:
                self.logger.error("OHand Modbus 客户端未找到")
                return False
            
            self.ohand_client = OHandModbusClient(
                port=self.serial_port,
                slave_id=self.slave_id
            )
            
            if not self.ohand_client.connect():
                self.logger.error("无法连接到 OHand 灵巧手")
                return False
            
            # 设置安全参数
            self._set_safety_parameters()
            
            self.logger.info("OHand 灵巧手初始化成功")
            return True
            
        except Exception as e:
            self.logger.error(f"OHand 初始化失败: {e}")
            return False
    
    def _set_safety_parameters(self):
        """设置安全参数"""
        try:
            safety_limits = self.mapping_config['safety_limits']
            
            # 设置所有手指的速度限制
            for finger_idx in range(6):
                self.ohand_client.set_finger_speed(finger_idx, safety_limits['max_velocity'])
                
                # 设置电流限制 (仅弯曲手指)
                if finger_idx < 5:
                    self.ohand_client.set_finger_current_limit(finger_idx, safety_limits['max_current'])
                    self.ohand_client.set_finger_force_limit(finger_idx, safety_limits['max_force'])
            
            self.logger.info("安全参数设置完成")
            
        except Exception as e:
            self.logger.warning(f"设置安全参数失败: {e}")
    
    def get_mujoco_joint_angles(self) -> Dict[str, float]:
        """从 MuJoCo 获取各个手指proximal_link关节的角度"""
        if self.mujoco_data is None:
            return {}
        
        angles = {}
        
        # 定义关节名称映射 (MuJoCo关节名 -> 手指名)
        joint_mapping = {
            'if_proximal_link': 'index_finger_flex',      # 食指近端关节
            'mf_proximal_link': 'middle_finger_flex',     # 中指近端关节
            'rf_proximal_link': 'ring_finger_flex',       # 无名指近端关节
            'lf_proximal_link': 'little_finger_flex',     # 小指近端关节
            'th_proximal_link': 'thumb_flex',             # 拇指近端关节
            'th_root_link': 'thumb_rot',                   # 拇指根部旋转关节
        }
        
        # 遍历所有关节，找到对应的proximal_link关节
        for i in range(self.mujoco_model.njnt):
            joint_name = mujoco.mj_id2name(self.mujoco_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name and joint_name in joint_mapping:
                finger_name = joint_mapping[joint_name]
                # 获取关节角度 (弧度)
                if i < len(self.mujoco_data.qpos):
                    angle_rad = float(self.mujoco_data.qpos[i])
                    # 转换为角度
                    angle_deg = np.degrees(angle_rad)
                    angles[finger_name] = angle_deg
        
        return angles
    
    def map_angle_to_ohand(self, finger_name: str, sim_angle_deg: float) -> int:
        """将 MuJoCo 角度映射到 OHand 位置"""
        if finger_name not in self.mapping_config['position_scaling']:
            return 0
        
        sim_min, sim_max, real_min, real_max = self.mapping_config['position_scaling'][finger_name]
        
        # 限制输入范围
        sim_angle_deg = np.clip(sim_angle_deg, sim_min, sim_max)
        
        # 线性映射
        normalized = (sim_angle_deg - sim_min) / (sim_max - sim_min)
        real_position = int(real_min + normalized * (real_max - real_min))
        
        return real_position
    
    def update_ohand_positions(self, mujoco_angles: Dict[str, float]):
        """更新 OHand 位置"""
        if self.ohand_client is None:
            return
        
        try:
            finger_mapping = self.mapping_config['finger_mapping']
            
            for finger_name, sim_angle_deg in mujoco_angles.items():
                if finger_name in finger_mapping:
                    finger_idx = finger_mapping[finger_name]
                    real_position = self.map_angle_to_ohand(finger_name, sim_angle_deg)
                    
                    # 设置目标位置
                    if self.mapping_config['control_mode'] == 'position':
                        self.ohand_client.set_finger_target_pos(finger_idx, real_position)
                    else:  # angle mode
                        # 直接使用角度
                        self.ohand_client.set_finger_target_angle(finger_idx, sim_angle_deg)
            
        except Exception as e:
            self.logger.error(f"更新 OHand 位置失败: {e}")
    
    def get_ohand_status(self) -> Dict:
        """获取 OHand 状态信息"""
        if self.ohand_client is None:
            return {}
        
        try:
            status = {}
            
            # 获取所有手指状态
            for finger_idx in range(6):
                status_code, status_desc = self.ohand_client.get_finger_status(finger_idx)
                current_pos = self.ohand_client.get_finger_current_pos(finger_idx)
                current_angle = self.ohand_client.get_finger_current_angle(finger_idx)
                current_force = self.ohand_client.get_finger_force(finger_idx) if finger_idx < 5 else None
                
                status[finger_idx] = {
                    'status_code': status_code,
                    'status_desc': status_desc,
                    'position': current_pos,
                    'angle': current_angle,
                    'force': current_force
                }
            
            return status
            
        except Exception as e:
            self.logger.error(f"获取 OHand 状态失败: {e}")
            return {}
    
    def control_loop(self):
        """主控制循环"""
        update_interval = 1.0 / self.mapping_config['update_rate']
        
        while self.is_running:
            try:
                current_time = time.time()
                
                # 检查更新频率
                if current_time - self.last_update_time >= update_interval:
                    # 获取 MuJoCo 关节角度
                    mujoco_angles = self.get_mujoco_joint_angles()
                    
                    # 更新当前角度数据（用于显示）
                    self.current_angles = mujoco_angles.copy()
                    
                    # 更新 OHand 位置
                    self.update_ohand_positions(mujoco_angles)
                    
                    # 获取 OHand 状态
                    ohand_status = self.get_ohand_status()
                    
                    # 更新OHand位置数据（用于显示）
                    for finger_name in mujoco_angles.keys():
                        if finger_name in self.mapping_config['finger_mapping']:
                            finger_idx = self.mapping_config['finger_mapping'][finger_name]
                            real_position = self.map_angle_to_ohand(finger_name, mujoco_angles[finger_name])
                            self.current_ohand_positions[finger_name] = real_position
                    
                    # 移除调试日志输出
                    pass
                    
                    self.last_update_time = current_time
                
                # 显示逻辑已移至主线程，此处移除
                pass
                
                time.sleep(0.001)  # 小延迟避免CPU过载
                
            except Exception as e:
                self.logger.error(f"控制循环错误: {e}")
                time.sleep(0.1)  # 错误时稍长延迟
    
    def start(self):
        """启动 Sim2Real 控制器"""
        if self.is_running:
            self.logger.warning("控制器已在运行")
            return False
        
        # 初始化 MuJoCo
        if not self.initialize_mujoco():
            return False
        
        # 初始化 OHand
        if not self.initialize_ohand():
            return False
        
        # 启动控制线程
        self.is_running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        self.logger.info("Sim2Real 控制器已启动")
        return True
    
    def stop(self):
        """停止 Sim2Real 控制器"""
        self.is_running = False
        
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
        
        if self.ohand_client:
            self.ohand_client.disconnect()
        
        if self.viewer:
            self.viewer.close()
        
        self.logger.info("Sim2Real 控制器已停止")
    
    def run_interactive(self):
        """运行交互式模式"""
        if not self.start():
            return

        try:
            print("Sim2Real 控制器已启动")
            print("关闭窗口或Ctrl+C退出")

            # 启动官方viewer
            with mujoco.viewer.launch_passive(self.mujoco_model, self.mujoco_data) as viewer:
                # 设置默认相机视角
                viewer.cam.azimuth = -1.17
                viewer.cam.elevation = 89.00
                viewer.cam.distance = 0.30
                viewer.cam.lookat[:] = [-0.09, -0.00, 0.01]
                
                last_display_time = 0
                display_interval = 0.5  # 显示更新间隔（秒）
                
                while self.is_running and viewer.is_running():
                    current_time = time.time()
                    
                    # MuJoCo仿真步进
                    mujoco.mj_step(self.mujoco_model, self.mujoco_data)
                    viewer.sync()
                    
                    # 定期更新显示
                    if current_time - last_display_time >= display_interval:
                        self.print_angles_display()
                        last_display_time = current_time
                    
                    # time.sleep(0.01)  # 控制仿真速度
                    
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            self.stop()

def create_custom_mapping():
    """创建自定义映射配置"""
    return {
        'position_scaling': {
            # 针对特定手指的精细调整 - 基于角度范围
            'thumb_flex': (-24.8, 0, 65535, 0),      # 拇指弯曲角度范围较小 0-80度
            'index_finger_flex': (-70.2,0.0 , 65535, 0),  # 食指弯曲角度范围 0-90度
            'middle_finger_flex': ( -40.6, 0.0, 65535, 0),  # 食指弯曲角度范围 0-90度
            'ring_finger_flex': (-64.4, 0.0, 65535, 0),  # 食指弯曲角度范围 0-90度
            'little_finger_flex': (-64.0, 0.0, 65535, 0),  # 食指弯曲角度范围 0-90度
            'thumb_rot': (-2.0, 92.0, 0, 65535),  # 拇指旋转角度范围 -2到92度
        },
        'control_mode': 'position',
        'update_rate': 100,  # 更高的更新频率
        'safety_limits': {
            'max_velocity': 20000,  # 更保守的速度限制
            'max_current': 800,
            'max_force': 4000,
        }
    }

def main():
    """主函数"""
    print("=== Sim2Real 控制器 ===")
    print("将 MuJoCo 仿真控制映射到实际的 OHand 灵巧手")
    
    # 配置参数
    config = {
        'mujoco_model_path': '../mjcf_models/rohand_left/rohand_left.xml',
        'serial_port': '/dev/ttyUSB0',  # 根据实际情况调整
        'slave_id': 2,
        'mapping_config': create_custom_mapping()
    }
    
    # 创建控制器
    controller = Sim2RealController(**config)
    
    try:
        # 运行交互式模式
        controller.run_interactive()
        
    except Exception as e:
        print(f"运行错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        controller.stop()

if __name__ == "__main__":
    main() 