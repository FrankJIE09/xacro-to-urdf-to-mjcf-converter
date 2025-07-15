#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
from pymodbus.client import ModbusSerialClient
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from pymodbus.constants import Endian
from pymodbus.exceptions import ModbusException

# 导入寄存器定义 (需要 roh_registers_v1.py 在同一目录或 Python 路径中)
try:
    from roh_registers_v1 import *
except ImportError:
    print("错误：未找到 roh_registers_v1.py 文件。请确保它在正确的目录中。")
    exit(1)

# --- 全局常量 ---
# 手指索引映射 (方便使用)
FINGER_THUMB_BEND = 0
FINGER_INDEX = 1
FINGER_MIDDLE = 2
FINGER_RING = 3
FINGER_PINKY = 4
FINGER_THUMB_ROT = 5
# 可以添加更多手指索引，如果寄存器列表中包含 6-9

# 状态码映射 (来自文档 4.3 节)
STATUS_MAP = {
    0: "STATUS_OPENING (正在展开)",
    1: "STATUS_CLOSING (正在抓取)",
    2: "STATUS_POS_REACHED (位置到位停止)",
    3: "STATUS_OVER_CURRENT (电流保护停止)",
    4: "STATUS_FORCE_REACHED (力控到位停止)",
    5: "STATUS_STUCK (电机堵转停止)",
}

# 错误子代码映射 (来自文档 3.4 节)
SUB_EXCEPTION_MAP = {
    1: "ERR_STATUS_INIT (等待初始化或正在初始化)",
    2: "ERR_STATUS_CALI (等待校正)",
    3: "ERR_INVALID_DATA (无效的寄存器值)",
    4: "ERR_STATUS_STUCK (电机堵转)",
    5: "ERR_OP_FAILED (操作失败)",
    6: "ERR_SAVE_FAILED (保存失败)",
}

# --- OHand Modbus 客户端类 ---

class OHandModbusClient:
    """
    用于通过 Modbus RTU 协议与 OHand 灵巧手交互的客户端类。
    使用 pymodbus 库进行串口通信。
    """
    def __init__(self, port, slave_id=2, baudrate=115200, parity='N', stopbits=1, bytesize=8, timeout=1):
        """
        初始化 Modbus 客户端。

        :param port: 串口端口号 (例如 '/dev/ttyUSB0' 或 'COM3').
        :param slave_id: OHand 的 Modbus 从站 ID (默认: 17).
        :param baudrate: 波特率 (默认: 115200).
        :param parity: 校验位 ('N', 'E', 'O') (默认: 'N').
        :param stopbits: 停止位 (1, 1.5, 2) (默认: 1).
        :param bytesize: 数据位 (5, 6, 7, 8) (默认: 8).
        :param timeout: 通信超时时间 (秒) (默认: 1).
        """
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout
        )
        self.client.port =port
        self.slave_id = slave_id
        self.is_connected = False
        # Modbus 标准通常使用大端字节序 (Big Endian)
        self.byte_order = Endian.BIG
        self.word_order = Endian.BIG

        # 配置日志记录器 (可选, 用于调试)
        logging.basicConfig()
        self.log = logging.getLogger(self.__class__.__name__)
        self.log.setLevel(logging.INFO) # 设置为 DEBUG 获取更详细信息

    def connect(self):
        """连接到 OHand 设备。"""
        # self.log.info(f"尝试连接到 {self.client.port} (Slave ID: {self.slave_id})...")
        print(f"尝试连接到 {self.client.port} (Slave ID: {self.slave_id})...")
        if self.client.connect():
            self.is_connected = True
            # self.log.info("连接成功.")
            print("连接成功.")
            return True
        else:
            self.is_connected = False
            # self.log.error("连接失败.")
            print("连接失败.")
            return False

    def disconnect(self):
        """断开与 OHand 设备的连接。"""
        if self.is_connected:
            # self.log.info("正在断开连接...")
            print("正在断开连接...")
            self.client.close()
            self.is_connected = False
            # self.log.info("连接已断开.")
            print("连接已断开.")

    def _read_registers(self, address, count=1):
        """
        内部辅助函数：读取保持寄存器 (功能码 0x03)。

        :param address: 起始寄存器地址。
        :param count: 要读取的寄存器数量。
        :return: pymodbus 读取响应对象，如果失败则返回 None。
        :raises ModbusException: 如果发生 Modbus 通信错误。
        """
        if not self.is_connected:
            # self.log.error("读取寄存器错误：未连接。")
            print("读取寄存器错误：未连接。")
            raise ModbusException("客户端未连接")
        try:
            # self.log.debug(f"读取寄存器: 地址={address}, 数量={count}, 从站ID={self.slave_id}")
            response = self.client.read_holding_registers(address, count, slave=self.slave_id)
            if response.isError():
                # self.log.error(f"读取寄存器 {address} 时发生 Modbus 错误: {response}")
                print(f"读取寄存器 {address} 时发生 Modbus 错误: {response}")
                raise ModbusException(f"读取寄存器错误: {response}")
            # self.log.debug(f"读取成功: {response.registers}")
            return response
        except ModbusException as e:
            # self.log.error(f"读取寄存器 {address} 时发生 Modbus 异常: {e}")
            print(f"读取寄存器 {address} 时发生 Modbus 异常: {e}")
            raise e
        except Exception as e:
            # self.log.error(f"读取寄存器 {address} 时发生意外错误: {e}")
            print(f"读取寄存器 {address} 时发生意外错误: {e}")
            raise ModbusException(f"意外读取错误: {e}")


    def _write_single_register(self, address, value):
        """
        内部辅助函数：写入单个保持寄存器 (功能码 0x06)。

        :param address: 要写入的寄存器地址。
        :param value: 要写入的 16 位值。
        :return: True 如果写入成功。
        :raises ModbusException: 如果发生 Modbus 通信错误。
        """
        if not self.is_connected:
            # self.log.error("写入寄存器错误：未连接。")
            print("写入寄存器错误：未连接。")
            raise ModbusException("客户端未连接")
        try:
            # self.log.debug(f"写入单个寄存器: 地址={address}, 值={value}, 从站ID={self.slave_id}")
            response = self.client.write_register(address, value, slave=self.slave_id)
            if response.isError():
                # self.log.error(f"写入寄存器 {address} 时发生 Modbus 错误: {response}")
                print(f"写入寄存器 {address} 时发生 Modbus 错误: {response}")
                raise ModbusException(f"写入寄存器错误: {response}")
            # self.log.debug(f"写入成功。")
            return True
        except ModbusException as e:
            # self.log.error(f"写入寄存器 {address} 时发生 Modbus 异常: {e}")
            print(f"写入寄存器 {address} 时发生 Modbus 异常: {e}")
            raise e
        except Exception as e:
            # self.log.error(f"写入寄存器 {address} 时发生意外错误: {e}")
            print(f"写入寄存器 {address} 时发生意外错误: {e}")
            raise ModbusException(f"意外写入错误: {e}")

    def _write_multiple_registers(self, address, values):
        """
        内部辅助函数：写入多个保持寄存器 (功能码 0x10)。

        :param address: 起始寄存器地址。
        :param values: 要写入的 16 位值列表。
        :return: True 如果写入成功。
        :raises ModbusException: 如果发生 Modbus 通信错误。
        """
        if not self.is_connected:
            # self.log.error("写入多个寄存器错误：未连接。")
            print("写入多个寄存器错误：未连接。")
            raise ModbusException("客户端未连接")
        try:
            # self.log.debug(f"写入多个寄存器: 地址={address}, 值={values}, 从站ID={self.slave_id}")
            response = self.client.write_registers(address, values, slave=self.slave_id)
            if response.isError():
                # self.log.error(f"写入多个寄存器 {address} 时发生 Modbus 错误: {response}")
                print(f"写入多个寄存器 {address} 时发生 Modbus 错误: {response}")
                raise ModbusException(f"写入多个寄存器错误: {response}")
            # self.log.debug(f"写入成功。")
            return True
        except ModbusException as e:
            # self.log.error(f"写入多个寄存器 {address} 时发生 Modbus 异常: {e}")
            print(f"写入多个寄存器 {address} 时发生 Modbus 异常: {e}")
            raise e
        except Exception as e:
            # self.log.error(f"写入多个寄存器 {address} 时发生意外错误: {e}")
            print(f"写入多个寄存器 {address} 时发生意外错误: {e}")
            raise ModbusException(f"意外写入错误: {e}")

    # --- 实现所有寄存器访问方法 ---

    # - 基本信息 (R) -
    def get_protocol_version(self):
        """读取协议版本号 (ROH_PROTOCOL_VERSION, Addr: 1000, R)"""
        response = self._read_registers(ROH_PROTOCOL_VERSION, 1)
        if response:
            raw_value = response.registers[0]
            major = (raw_value >> 8) & 0xFF
            minor = raw_value & 0xFF
            return major, minor
        return None

    def get_firmware_version(self):
        """读取固件版本号 (ROH_FW_VERSION, Addr: 1001, R)"""
        response = self._read_registers(ROH_FW_VERSION, 1)
        if response:
            raw_value = response.registers[0]
            major = (raw_value >> 8) & 0xFF
            minor = raw_value & 0xFF
            return major, minor
        return None

    def get_firmware_revision(self):
        """读取固件修订版本号 (ROH_FW_REVISION, Addr: 1002, R)"""
        response = self._read_registers(ROH_FW_REVISION, 1)
        return response.registers[0] if response else None

    def get_hardware_version(self):
        """读取硬件版本号 (ROH_HW_VERSION, Addr: 1003, R)"""
        response = self._read_registers(ROH_HW_VERSION, 1)
        if response:
            raw_value = response.registers[0]
            hw_type = (raw_value >> 8) & 0xFF
            hw_ver = raw_value & 0xFF
            return hw_type, hw_ver
        return None

    def get_bootloader_version(self):
        """读取 Bootloader 版本号 (ROH_BOOT_VERSION, Addr: 1004, R)"""
        response = self._read_registers(ROH_BOOT_VERSION, 1)
        if response:
            raw_value = response.registers[0]
            major = (raw_value >> 8) & 0xFF
            minor = raw_value & 0xFF
            return major, minor
        return None

    # - 基本配置 (R/W 或 W) -
    def get_node_id(self):
        """读取灵巧手节点 ID (ROH_NODE_ID, Addr: 1005, R)"""
        response = self._read_registers(ROH_NODE_ID, 1)
        # 文档说明仅低8位有效
        return (response.registers[0] & 0xFF) if response else None

    def set_node_id(self, node_id):
        """设置灵巧手节点 ID (ROH_NODE_ID, Addr: 1005, W)。写入成功后 ROH 会保存并重启。"""
        if 0 <= node_id <= 255:
            return self._write_single_register(ROH_NODE_ID, node_id)
        else:
            print("错误: Node ID 必须在 0-255 之间。")
            # self.log.error("设置 Node ID 错误: Node ID 必须在 0-255 之间。")
            return False

    def get_sub_exception_code(self):
        """读取错误子代码 (ROH_SUB_EXCEPTION, Addr: 1006, R)。用于获取 EC04 设备故障的具体原因。"""
        response = self._read_registers(ROH_SUB_EXCEPTION, 1)
        if response:
            code = response.registers[0]
            description = SUB_EXCEPTION_MAP.get(code, f"未知子错误代码: {code}")
            return code, description
        return None, "读取失败"

    def get_battery_voltage(self):
        """读取电池电压值 (ROH_BATTERY_VOLTAGE, Addr: 1007, R)。单位 mV。(文档注记：暂时不可用)"""
        # self.log.warning("读取电池电压：根据文档，此功能暂时不可用。")
        print("警告: 读取电池电压：根据文档，此功能暂时不可用。")
        response = self._read_registers(ROH_BATTERY_VOLTAGE, 1)
        return response.registers[0] if response else None # 单位 mV

    def get_self_test_level(self):
        """读取开机自检开关设置 (ROH_SELF_TEST_LEVEL, Addr: 1008, R)"""
        response = self._read_registers(ROH_SELF_TEST_LEVEL, 1)
        return response.registers[0] if response else None

    def set_self_test_level(self, level):
        """
        设置开机自检开关 (ROH_SELF_TEST_LEVEL, Addr: 1008, W)。设置时保存到非易失存储器。
        0: 等待 ROH_START_INIT 写 1 自检
        1: 允许开机归零 (默认)
        2: 允许开机完整自检
        """
        if level in [0, 1, 2]:
            return self._write_single_register(ROH_SELF_TEST_LEVEL, level)
        else:
            # self.log.error(f"设置自检级别错误: 无效的级别 {level}。必须是 0, 1, 或 2。")
            print(f"错误: 设置自检级别错误: 无效的级别 {level}。必须是 0, 1, 或 2。")
            return False

    def get_beep_switch(self):
        """读取蜂鸣器开关状态 (ROH_BEEP_SWITCH, Addr: 1009, R)。1: 允许发声, 0: 静音。"""
        response = self._read_registers(ROH_BEEP_SWITCH, 1)
        return response.registers[0] if response else None

    def set_beep_switch(self, enable):
        """设置蜂鸣器开关状态 (ROH_BEEP_SWITCH, Addr: 1009, W)。设置时保存到非易失存储器。"""
        value = 1 if enable else 0
        return self._write_single_register(ROH_BEEP_SWITCH, value)

    def set_beep_period(self, duration_ms):
        """使蜂鸣器发声指定时间 (ROH_BEEP_PERIOD, Addr: 1010, W)。单位毫秒。"""
        if 0 <= duration_ms <= 65535:
             return self._write_single_register(ROH_BEEP_PERIOD, duration_ms)
        else:
            # self.log.error(f"设置蜂鸣器周期错误: 时长 {duration_ms} 超出范围 (0-65535ms)。")
            print(f"错误: 设置蜂鸣器周期错误: 时长 {duration_ms} 超出范围 (0-65535ms)。")
            return False

    def get_button_press_count(self):
        """读取按键按下次数 (ROH_BUTTON_PRESS_CNT, Addr: 1011, R)。主要用于校正时确认。"""
        response = self._read_registers(ROH_BUTTON_PRESS_CNT, 1)
        return response.registers[0] if response else None

    def set_button_press_count(self, count):
        """设置按键按下次数 (ROH_BUTTON_PRESS_CNT, Addr: 1011, W)。主要用于校正时确认。"""
        if 0 <= count <= 65535:
            return self._write_single_register(ROH_BUTTON_PRESS_CNT, count)
        else:
            # self.log.error(f"设置按键次数错误: 次数 {count} 超出范围 (0-65535)。")
            print(f"错误: 设置按键次数错误: 次数 {count} 超出范围 (0-65535)。")
            return False

    # - 控制指令 (W) -
    def set_recalibrate(self, key_value):
        """
        请求重新校正 (ROH_RECALIBRATE, Addr: 1012, W)。
        需要写入特定值（非公开）让 ROH 灵巧手进入校正状态。
        """
        # self.log.warning("调用重新校正：需要特定的非公开值。")
        print("警告: 调用重新校正：需要特定的非公开值。")
        if 0 <= key_value <= 65535:
            return self._write_single_register(ROH_RECALIBRATE, key_value)
        else:
            # self.log.error("重新校正错误：值超出范围 (0-65535)。")
            print("错误: 重新校正错误：值超出范围 (0-65535)。")
            return False

    def start_initialization(self):
        """开始自检 (ROH_START_INIT, Addr: 1013, W)。仅当自检级别设为 0 时有效。"""
        # 通常写入 1 来触发
        return self._write_single_register(ROH_START_INIT, 1)

    def reset_device(self, dfu_mode=False):
        """
        复位设备 (ROH_RESET, Addr: 1014, W)。
        :param dfu_mode: 如果为 True，则写入 0 以外的值使设备重启进入 DFU 模式。
                         如果为 False (默认)，则写入 0 使设备重启到工作模式。
        """
        value = 1 if dfu_mode else 0 # 文档描述写入非0进入DFU，写入0重启到工作模式
        return self._write_single_register(ROH_RESET, value)

    def power_off_device(self):
        """关机 (ROH_POWER_OFF, Addr: 1015, W)。(文档注记：暂时不可用)"""
        # self.log.warning("关机：根据文档，此功能暂时不可用。")
        print("警告: 关机：根据文档，此功能暂时不可用。")
        # 通常写入 1 来触发，但由于不可用，可能无效
        return self._write_single_register(ROH_POWER_OFF, 1)

    # - 校准数据 (R/W) -
    # 注意：文档说明用户无需设置这些值，它们是出厂校准值。只提供读取方法。
    #      如果确实需要写入，可以取消注释 set_... 方法。

    def get_cali_end(self, finger_index):
        """读取指定手指运行区间上限（绝对位置）(ROH_CALI_END[0-5], Addr: 1020-1025, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_CALI_END0 + finger_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取校准上限错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取校准上限错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    # def set_cali_end(self, finger_index, value):
    #     """设置指定手指运行区间上限（绝对位置）(ROH_CALI_END[0-5], Addr: 1020-1025, W)。"""
    #     if 0 <= finger_index <= 5:
    #         if 0 <= value <= 65535:
    #             address = ROH_CALI_END0 + finger_index
    #             return self._write_single_register(address, value)
    #         else:
    #             self.log.error(f"设置校准上限错误：值 {value} 超出范围 (0-65535)。")
    #             return False
    #     else:
    #         self.log.error(f"设置校准上限错误：无效的手指索引 {finger_index} (应为 0-5)。")
    #         return False

    def get_cali_start(self, finger_index):
        """读取指定手指运行区间下限（绝对位置）(ROH_CALI_START[0-5], Addr: 1030-1035, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_CALI_START0 + finger_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取校准下限错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取校准下限错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    # def set_cali_start(self, finger_index, value):
    #     """设置指定手指运行区间下限（绝对位置）(ROH_CALI_START[0-5], Addr: 1030-1035, W)。"""
    #     # ... (实现同 set_cali_end) ...

    def get_cali_thumb_preset_pos(self, preset_index):
        """读取大拇指旋转预设位置（绝对位置）(ROH_CALI_THUMB_POS[0-2], Addr: 1040-1042, R)。"""
        if 0 <= preset_index <= 2:
            address = ROH_CALI_THUMB_POS0 + preset_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取拇指预设位置错误：无效的预设索引 {preset_index} (应为 0-2)。")
            print(f"错误: 获取拇指预设位置错误：无效的预设索引 {preset_index} (应为 0-2)。")
            return None

    # def set_cali_thumb_preset_pos(self, preset_index, value):
    #     """设置大拇指旋转预设位置（绝对位置）(ROH_CALI_THUMB_POS[0-2], Addr: 1040-1042, W)。"""
    #     # ... (实现同 set_cali_end) ...

    # - PID 参数 (R/W) -
    #   文档说明这些值乘以了 100 存储为 uint16。方法接受 float，进行转换。

    def _get_pid_param(self, base_address, finger_index):
        """内部辅助函数：读取单个 PID 参数（乘以 100 存储）。"""
        if 0 <= finger_index <= 5:
            address = base_address + finger_index
            response = self._read_registers(address, 1)
            return (response.registers[0] / 100.0) if response else None
        else:
            # self.log.error(f"获取 PID 参数错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取 PID 参数错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def _set_pid_param(self, base_address, finger_index, value_float):
        """内部辅助函数：写入单个 PID 参数（乘以 100 存储）。"""
        if 0 <= finger_index <= 5:
            # 将浮点值乘以 100 并转换为 uint16
            value_int = int(round(value_float * 100.0))
            if 0 <= value_int <= 65535:
                address = base_address + finger_index
                return self._write_single_register(address, value_int)
            else:
                # self.log.error(f"设置 PID 参数错误：转换后的值 {value_int} 超出范围 (0-65535)。原始值: {value_float}")
                print(f"错误: 设置 PID 参数错误：转换后的值 {value_int} 超出范围 (0-65535)。原始值: {value_float}")
                return False
        else:
            # self.log.error(f"设置 PID 参数错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 设置 PID 参数错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_p(self, finger_index):
        """读取指定手指的 P 值 (ROH_FINGER_P[0-5], Addr: 1045-1050, R)。"""
        return self._get_pid_param(ROH_FINGER_P0, finger_index)

    def set_finger_p(self, finger_index, value_float):
        """设置指定手指的 P 值 (ROH_FINGER_P[0-5], Addr: 1045-1050, W)。会保存到非易失存储。"""
        return self._set_pid_param(ROH_FINGER_P0, finger_index, value_float)

    def get_finger_i(self, finger_index):
        """读取指定手指的 I 值 (ROH_FINGER_I[0-5], Addr: 1055-1060, R)。"""
        return self._get_pid_param(ROH_FINGER_I0, finger_index)

    def set_finger_i(self, finger_index, value_float):
        """设置指定手指的 I 值 (ROH_FINGER_I[0-5], Addr: 1055-1060, W)。会保存到非易失存储。"""
        return self._set_pid_param(ROH_FINGER_I0, finger_index, value_float)

    def get_finger_d(self, finger_index):
        """读取指定手指的 D 值 (ROH_FINGER_D[0-5], Addr: 1065-1070, R)。"""
        return self._get_pid_param(ROH_FINGER_D0, finger_index)

    def set_finger_d(self, finger_index, value_float):
        """设置指定手指的 D 值 (ROH_FINGER_D[0-5], Addr: 1065-1070, W)。会保存到非易失存储。"""
        return self._set_pid_param(ROH_FINGER_D0, finger_index, value_float)

    def get_finger_g(self, finger_index):
        """读取指定手指的 G 值 (抗重力?) (ROH_FINGER_G[0-5], Addr: 1075-1080, R)。"""
        return self._get_pid_param(ROH_FINGER_G0, finger_index)

    def set_finger_g(self, finger_index, value_float):
        """设置指定手指的 G 值 (ROH_FINGER_G[0-5], Addr: 1075-1080, W)。会保存到非易失存储。"""
        return self._set_pid_param(ROH_FINGER_G0, finger_index, value_float)

    # - 状态读取 (R) -
    def get_finger_status(self, finger_index):
        """读取指定手指的状态码 (ROH_FINGER_STATUS[0-5], Addr: 1085-1090, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_STATUS0 + finger_index
            response = self._read_registers(address, 1)
            if response:
                code = response.registers[0]
                description = STATUS_MAP.get(code, f"未知状态码: {code}")
                return code, description
            return None, "读取失败"
        else:
            # self.log.error(f"获取手指状态错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取手指状态错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None, "无效索引"

    def get_finger_current_limit(self, finger_index):
        """读取指定手指的电机电流限制值 (ROH_FINGER_CURRENT_LIMIT[0-5], Addr: 1095-1100, R)。单位 mA。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_CURRENT_LIMIT0 + finger_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取电流限制错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取电流限制错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def set_finger_current_limit(self, finger_index, limit_ma):
        """设置指定手指的电机电流限制值 (ROH_FINGER_CURRENT_LIMIT[0-5], Addr: 1095-1100, W)。单位 mA。开机时恢复为默认值。"""
        if 0 <= finger_index <= 5:
            if 0 <= limit_ma <= 65535:
                address = ROH_FINGER_CURRENT_LIMIT0 + finger_index
                return self._write_single_register(address, limit_ma)
            else:
                # self.log.error(f"设置电流限制错误：值 {limit_ma} 超出范围 (0-65535 mA)。")
                print(f"错误: 设置电流限制错误：值 {limit_ma} 超出范围 (0-65535 mA)。")
                return False
        else:
            # self.log.error(f"设置电流限制错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 设置电流限制错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_current(self, finger_index):
        """读取指定手指的当前电机电流值 (ROH_FINGER_CURRENT[0-5], Addr: 1105-1110, R)。单位 mA。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_CURRENT0 + finger_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取电流错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取电流错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def get_finger_force_limit(self, finger_index):
        """读取指定手指的力量限制值 (ROH_FINGER_FORCE_LIMIT[0-4], Addr: 1115-1119, R)。单位 mN。"""
        # 注意：大拇指旋转 (索引 5) 没有力量限制寄存器
        if 0 <= finger_index <= 4:
            address = ROH_FINGER_FORCE_LIMIT0 + finger_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取力量限制错误：无效的手指索引 {finger_index} (应为 0-4)。")
            print(f"错误: 获取力量限制错误：无效的手指索引 {finger_index} (应为 0-4)。")
            return None

    def set_finger_force_limit(self, finger_index, limit_mn):
        """设置指定手指的力量限制值 (ROH_FINGER_FORCE_LIMIT[0-4], Addr: 1115-1119, W)。单位 mN。开机时恢复为默认值。"""
        # 注意：大拇指旋转 (索引 5) 没有力量限制寄存器
        if 0 <= finger_index <= 4:
            if 0 <= limit_mn <= 65535:
                address = ROH_FINGER_FORCE_LIMIT0 + finger_index
                return self._write_single_register(address, limit_mn)
            else:
                # self.log.error(f"设置力量限制错误：值 {limit_mn} 超出范围 (0-65535 mN)。")
                print(f"错误: 设置力量限制错误：值 {limit_mn} 超出范围 (0-65535 mN)。")
                return False
        else:
            # self.log.error(f"设置力量限制错误：无效的手指索引 {finger_index} (应为 0-4)。")
            print(f"错误: 设置力量限制错误：无效的手指索引 {finger_index} (应为 0-4)。")
            return False

    def get_finger_force(self, finger_index):
        """读取指定手指的当前力量值 (ROH_FINGER_FORCE[0-4], Addr: 1120-1124, R)。单位 mN。"""
        # 注意：大拇指旋转 (索引 5) 没有力量寄存器
        if 0 <= finger_index <= 4:
            address = ROH_FINGER_FORCE0 + finger_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取力量错误：无效的手指索引 {finger_index} (应为 0-4)。")
            print(f"错误: 获取力量错误：无效的手指索引 {finger_index} (应为 0-4)。")
            return None

    # - 运动控制 (R/W) -
    def get_finger_speed(self, finger_index):
        """读取指定手指的逻辑速度 (ROH_FINGER_SPEED[0-5], Addr: 1125-1130, R)。单位: 逻辑位置/秒。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_SPEED0 + finger_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取速度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取速度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def set_finger_speed(self, finger_index, speed):
        """设置指定手指的逻辑速度 (ROH_FINGER_SPEED[0-5], Addr: 1125-1130, W)。单位: 逻辑位置/秒。开机时恢复为默认值 (65535)。"""
        if 0 <= finger_index <= 5:
            if 0 <= speed <= 65535:
                address = ROH_FINGER_SPEED0 + finger_index
                return self._write_single_register(address, speed)
            else:
                # self.log.error(f"设置速度错误：值 {speed} 超出范围 (0-65535)。")
                print(f"错误: 设置速度错误：值 {speed} 超出范围 (0-65535)。")
                return False
        else:
            # self.log.error(f"设置速度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 设置速度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_target_pos(self, finger_index):
        """读取指定手指的逻辑目标位置 (ROH_FINGER_POS_TARGET[0-5], Addr: 1135-1140, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_POS_TARGET0 + finger_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取目标位置错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取目标位置错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def set_finger_target_pos(self, finger_index, position):
        """设置指定手指的逻辑目标位置 (ROH_FINGER_POS_TARGET[0-5], Addr: 1135-1140, W)。写入后手指会开始移动。"""
        if 0 <= finger_index <= 5:
            if 0 <= position <= 65535:
                address = ROH_FINGER_POS_TARGET0 + finger_index
                return self._write_single_register(address, position)
            else:
                # self.log.error(f"设置目标位置错误：值 {position} 超出范围 (0-65535)。")
                print(f"错误: 设置目标位置错误：值 {position} 超出范围 (0-65535)。")
                return False
        else:
            # self.log.error(f"设置目标位置错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 设置目标位置错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_current_pos(self, finger_index):
        """读取指定手指的当前逻辑位置 (ROH_FINGER_POS[0-5], Addr: 1145-1150, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_POS0 + finger_index
            response = self._read_registers(address, 1)
            return response.registers[0] if response else None
        else:
            # self.log.error(f"获取当前位置错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取当前位置错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def get_finger_target_angle(self, finger_index):
        """读取指定手指的目标角度 (ROH_FINGER_ANGLE_TARGET[0-5], Addr: 1155-1160, R)。单位：度。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_ANGLE_TARGET0 + finger_index
            response = self._read_registers(address, 1)
            if response:
                # 寄存器存储的是 int16，值为角度 * 100
                decoder = BinaryPayloadDecoder.fromRegisters(response.registers, byteorder=self.byte_order, wordorder=self.word_order)
                scaled_value = decoder.decode_16bit_int()
                return scaled_value / 100.0
            return None
        else:
            # self.log.error(f"获取目标角度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取目标角度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def set_finger_target_angle(self, finger_index, angle_deg):
        """设置指定手指的目标角度 (ROH_FINGER_ANGLE_TARGET[0-5], Addr: 1155-1160, W)。单位：度。写入后手指会开始移动。"""
        if 0 <= finger_index <= 5:
            # 将角度乘以 100 并转换为 int16
            scaled_value = int(round(angle_deg * 100.0))
            if -32768 <= scaled_value <= 32767:
                address = ROH_FINGER_ANGLE_TARGET0 + finger_index
                builder = BinaryPayloadBuilder(byteorder=self.byte_order, wordorder=self.word_order)
                builder.add_16bit_int(scaled_value)
                payload = builder.to_registers()
                return self._write_single_register(address, payload[0])
            else:
                # self.log.error(f"设置目标角度错误：转换后的值 {scaled_value} 超出 int16 范围。原始角度: {angle_deg}")
                print(f"错误: 设置目标角度错误：转换后的值 {scaled_value} 超出 int16 范围。原始角度: {angle_deg}")
                return False
        else:
            # self.log.error(f"设置目标角度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 设置目标角度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_current_angle(self, finger_index):
        """读取指定手指的当前角度 (ROH_FINGER_ANGLE[0-5], Addr: 1165-1170, R)。单位：度。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_ANGLE0 + finger_index
            response = self._read_registers(address, 1)
            if response:
                # 寄存器存储的是 int16，值为角度 * 100
                decoder = BinaryPayloadDecoder.fromRegisters(response.registers, byteorder=self.byte_order, wordorder=self.word_order)
                scaled_value = decoder.decode_16bit_int()
                return scaled_value / 100.0
            return None
        else:
            # self.log.error(f"获取当前角度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            print(f"错误: 获取当前角度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    # --- 便利方法 (示例) ---
    def get_all_finger_status(self):
        """获取所有手指的状态。"""
        status_dict = {}
        for i in range(6): # 0 到 5
            try:
                code, desc = self.get_finger_status(i)
                status_dict[i] = {"code": code, "description": desc}
            except ModbusException:
                status_dict[i] = {"code": None, "description": "读取失败"}
        return status_dict

    def get_all_finger_current_positions(self):
        """获取所有手指的当前逻辑位置。"""
        pos_dict = {}
        for i in range(6):
            try:
                pos = self.get_finger_current_pos(i)
                pos_dict[i] = pos
            except ModbusException:
                pos_dict[i] = None
        return pos_dict

    def get_all_finger_current_angles(self):
        """获取所有手指的当前角度。"""
        angle_dict = {}
        for i in range(6):
            try:
                angle = self.get_finger_current_angle(i)
                angle_dict[i] = angle
            except ModbusException:
                angle_dict[i] = None
        return angle_dict

    def set_all_fingers_target_pos(self, positions):
        """
        使用一次 Modbus 写多个寄存器 (0x10) 指令设置所有手指的目标逻辑位置。

        :param positions: 包含 6 个目标位置值的列表或元组 (对应手指 0-5)。
                          如果列表长度不足 6，则只设置前面的手指。
                          列表中的值应在 0-65535 之间。
        :return: True 如果写入成功。
        :raises ModbusException: 如果发生 Modbus 通信错误。
        :raises ValueError: 如果输入列表无效。
        """
        if not isinstance(positions, (list, tuple)):
            raise ValueError("输入必须是列表或元组")
        if not (0 < len(positions) <= 6):
             raise ValueError("列表长度必须在 1 到 6 之间")

        values_to_write = []
        for pos in positions:
            if not (0 <= pos <= 65535):
                raise ValueError(f"位置值 {pos} 超出范围 (0-65535)")
            values_to_write.append(pos)

        # 补齐不足6个手指的部分 (可选，取决于是否总是想写6个)
        # while len(values_to_write) < 6:
        #     # 可以使用当前位置填充，或者引发错误，或者用特定值填充
        #     # 这里我们只写入用户提供的位置
        #     pass

        if not values_to_write:
            return True # 没有要写入的值

        start_address = ROH_FINGER_POS_TARGET0
        return self._write_multiple_registers(start_address, values_to_write)


# --- 主程序示例 ---
if __name__ == "__main__":
    # --- 配置 ---
    SERIAL_PORT = '/dev/ttyUSB0'  # 在 Linux 上可能是这个，Windows 上可能是 'COMx'
    # SERIAL_PORT = 'COM3'       # Windows 示例
    SLAVE_ID = 3              # OHand 的 Modbus 从站地址

    # --- 创建客户端实例 ---
    hand = OHandModbusClient(port=SERIAL_PORT, slave_id=SLAVE_ID)

    try:
        # --- 连接 ---
        if not hand.connect():
            exit(1) # 连接失败则退出

        # --- 读取信息示例 ---
        print("\n--- 读取设备信息 ---")
        proto_ver = hand.get_protocol_version()
        fw_ver = hand.get_firmware_version()
        fw_rev = hand.get_firmware_revision()
        hw_ver = hand.get_hardware_version()
        boot_ver = hand.get_bootloader_version()
        node_id = hand.get_node_id()
        print(f"协议版本: {proto_ver}")
        print(f"固件版本: {fw_ver}, 修订: {fw_rev}")
        print(f"硬件版本: 类型={hw_ver[0] if hw_ver else 'N/A'}, 版本={hw_ver[1] if hw_ver else 'N/A'}")
        print(f"Bootloader 版本: {boot_ver}")
        print(f"节点 ID: {node_id}")

        # --- 读取状态示例 ---
        print("\n--- 读取手指状态 ---")
        # 读取单个手指状态
        index_finger_status_code, index_finger_status_desc = hand.get_finger_status(FINGER_INDEX)
        print(f"食指状态: Code={index_finger_status_code}, Desc='{index_finger_status_desc}'")

        # 读取所有手指当前位置和角度
        current_positions = hand.get_all_finger_current_positions()
        current_angles = hand.get_all_finger_current_angles()
        print("所有手指当前逻辑位置:", current_positions)
        print("所有手指当前角度 (度):", {k: f"{v:.2f}" if v is not None else 'N/A' for k, v in current_angles.items()})

        # 读取拇指 P, I, D 值
        thumb_p = hand.get_finger_p(FINGER_THUMB_BEND)
        thumb_i = hand.get_finger_i(FINGER_THUMB_BEND)
        thumb_d = hand.get_finger_d(FINGER_THUMB_BEND)
        print(f"拇指弯曲 PID: P={thumb_p:.2f}, I={thumb_i:.2f}, D={thumb_d:.2f}")


        # --- 写入控制示例 ---
        print("\n--- 控制示例 ---")
        # 示例：设置食指速度为 10000
        print("设置食指速度为 10000...")
        if hand.set_finger_speed(FINGER_INDEX, 65535):
            print("设置成功。")
        else:
            print("设置失败。")

        # 示例：设置食指目标位置为 30000 (0-65535 范围内的逻辑值)
        target_pos = 0
        print(f"设置食指目标逻辑位置为 {target_pos}...")
        if hand.set_finger_target_pos(FINGER_INDEX, target_pos):
            print("设置成功，手指应开始移动。")
        else:
            print("设置失败。")

        # 等待几秒钟让手指移动 (仅为示例，实际应用需要更好的状态监控)
        import time
        time.sleep(3)

        # 读取移动后的位置
        new_pos = hand.get_finger_current_pos(FINGER_INDEX)
        print(f"食指移动后当前逻辑位置: {new_pos}")

        # 示例：设置拇指旋转目标角度为 45 度
        target_angle = 45.0
        print(f"设置拇指旋转目标角度为 {target_angle}°...")
        if hand.set_finger_target_angle(FINGER_THUMB_ROT, target_angle):
             print("设置成功，手指应开始移动。")
        else:
             print("设置失败。")

        time.sleep(2)
        new_angle = hand.get_finger_current_angle(FINGER_THUMB_ROT)
        print(f"拇指旋转后当前角度: {new_angle:.2f}°")

        # 示例：使用写多个寄存器指令让食指和中指同时移动到位置 10000
        print("\n同时设置食指和中指目标位置为 10000...")
        try:
            if hand.set_all_fingers_target_pos([10000, 10000]): # 只设置前两个手指
                 print("多位置设置成功。")
            else:
                 print("多位置设置失败。")
        except ValueError as e:
            print(f"多位置设置值错误: {e}")
        except ModbusException as e:
            print(f"多位置设置 Modbus 错误: {e}")


    except ModbusException as e:
        print(f"发生 Modbus 错误: {e}")
    except KeyboardInterrupt:
        print("\n用户中断。")
    except Exception as e:
        print(f"发生未知错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # --- 断开连接 ---
        hand.disconnect()