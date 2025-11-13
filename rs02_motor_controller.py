# rs02_motor_controller.py
"""
RS02 电机控制器 - 位置模式(PP)控制
基于 ZLGCAN 和 RS02 通信协议
"""

from zlgcan import *
import threading
import time
import struct
from enum import Enum
import ctypes

class MotorMode(Enum):
    """电机运行模式"""
    DISABLED = 0
    PP_MODE = 1      # 位置模式(PP)
    VELOCITY_MODE = 2
    CURRENT_MODE = 3
    CSP_MODE = 4     # 循环同步位置模式

class MotorController:
    def __init__(self):
        self._zcan = ZCAN()
        self._dev_handle = INVALID_DEVICE_HANDLE
        self._can_handle = INVALID_CHANNEL_HANDLE
        
        self._is_open = False
        self._is_chn_open = False
        self._terminated = False
        
        # 电机参数
        self.motor_id = 0x7F
        self.master_id = 0xFD
        self.current_mode = MotorMode.DISABLED
        
        # 状态监控
        self._position = 0.0
        self._velocity = 0.0
        self._torque = 0.0
        self._voltage = 0.0
        self._temperature = 0.0
        self._fault_status = 0
        
        # 线程控制
        self._read_thread = None
        self._lock = threading.RLock()
        
        # 回调函数
        self._status_callback = None
        
    def connect(self, dev_type=41, dev_index=0, chn_index=0, 
                baudrate=1000000, data_baudrate=5000000):
        """连接设备和打开通道"""
        try:
            print(f"正在连接设备 - 类型: {dev_type}, 索引: {dev_index}, 通道: {chn_index}")
            print(f"波特率设置 - 仲裁域: {baudrate}, 数据域: {data_baudrate}")
            
            # 打开设备
            self._dev_handle = self._zcan.OpenDevice(dev_type, dev_index, 0)
            if self._dev_handle == INVALID_DEVICE_HANDLE:
                raise Exception("打开设备失败")
            self._is_open = True
            print("设备打开成功")
            
            # 配置通道
            chn_cfg = ZCAN_CHANNEL_INIT_CONFIG()
            chn_cfg.can_type = ZCAN_TYPE_CAN
            
            # 设置波特率
            print("设置波特率...")
            self._zcan.ZCAN_SetValue(self._dev_handle, f"{chn_index}/can_baud_rate", str(baudrate))
            
            self._can_handle = self._zcan.InitCAN(self._dev_handle, chn_index, chn_cfg)
            if self._can_handle == INVALID_CHANNEL_HANDLE:
                raise Exception("初始化通道失败")
            print("通道初始化成功")
                
            # 启动通道
            ret = self._zcan.StartCAN(self._can_handle)
            if ret != ZCAN_STATUS_OK:
                raise Exception("启动通道失败")
            print("通道启动成功")
                
            self._is_chn_open = True
            
            # 启动接收线程
            self._terminated = False
            self._read_thread = threading.Thread(target=self._read_worker)
            self._read_thread.daemon = True
            self._read_thread.start()
            
            print(f"电机控制器连接成功 - 设备: {dev_type}, 通道: {chn_index}")
            return True
            
        except Exception as e:
            print(f"连接失败: {e}")
            self.disconnect()
            return False
    
    def disconnect(self):
        """断开连接"""
        print("正在断开连接...")
        self._terminated = True
        
        if self._read_thread:
            self._read_thread.join(timeout=1)
            
        if self._is_chn_open:
            try:
                self.stop()  # 先停止电机
            except:
                pass
            self._zcan.ResetCAN(self._can_handle)
            self._is_chn_open = False
            
        if self._is_open:
            self._zcan.CloseDevice(self._dev_handle)
            self._is_open = False
            
        print("电机控制器已断开连接")
    
    def set_status_callback(self, callback):
        """设置状态回调函数"""
        self._status_callback = callback
    
    def _read_worker(self):
        """接收数据工作线程"""
        print("启动接收线程...")
        while not self._terminated and self._is_chn_open:
            try:
                # 检查CAN消息
                can_num = self._zcan.GetReceiveNum(self._can_handle, ZCAN_TYPE_CAN)
                if can_num > 0:
                    can_msgs, act_num = self._zcan.Receive(self._can_handle, can_num, 10)
                    if act_num:
                        self._process_received_messages(can_msgs, act_num)
                
                time.sleep(0.001)
                
            except Exception as e:
                print(f"接收数据错误: {e}")
                time.sleep(0.01)
    
    def _process_received_messages(self, msgs, count):
        """处理接收到的消息"""
        for i in range(count):
            msg = msgs[i].frame
            self._parse_message(msg)
    
    def _parse_message(self, msg):
        """解析CAN消息"""
        try:
            # 解析扩展帧ID (私有协议格式)
            mode = (msg.can_id >> 24) & 0x1F
            target_id = msg.can_id & 0xFF
            
            if target_id != self.motor_id:
                return
                
            if mode == 0x02:  # 类型2: 电机反馈数据
                self._parse_feedback_message(msg)
            elif mode == 0x12:  # 类型18: 参数读取响应
                self._parse_parameter_response(msg)
        except Exception as e:
            print(f"解析消息错误: {e}")
    
    def _parse_feedback_message(self, msg):
        """解析反馈消息"""
        try:
            # 将数据转换为字节
            data_bytes = bytes([msg.data[i] for i in range(msg.can_dlc)])
            
            # 解析故障信息
            fault_info = (msg.can_id >> 16) & 0xFF
            mode_status = (msg.can_id >> 22) & 0x03
            
            # 解析位置、速度、力矩
            if len(data_bytes) >= 6:
                position_raw = struct.unpack_from('>H', data_bytes, 0)[0]
                velocity_raw = struct.unpack_from('>H', data_bytes, 2)[0]
                torque_raw = struct.unpack_from('>H', data_bytes, 4)[0]
                
                # 转换为实际值
                self._position = self._uint_to_float(position_raw, -12.57, 12.57, 16)
                self._velocity = self._uint_to_float(velocity_raw, -44.0, 44.0, 16)
                self._torque = self._uint_to_float(torque_raw, -17.0, 17.0, 16)
                
                # 调用状态回调
                if self._status_callback:
                    self._status_callback({
                        'position': self._position,
                        'velocity': self._velocity,
                        'torque': self._torque,
                        'fault_status': fault_info,
                        'mode_status': mode_status
                    })
                
        except Exception as e:
            print(f"解析反馈消息错误: {e}")
    
    def _parse_parameter_response(self, msg):
        """解析参数响应消息"""
        pass
    
    def _create_can_message(self, mode, data_bytes):
        """创建CAN消息结构体"""
        msg = ZCAN_Transmit_Data()
        msg.transmit_type = 0  # 正常发送
        msg.frame.eff = 1  # 扩展帧
        msg.frame.rtr = 0  # 数据帧
        msg.frame.can_id = self._build_can_id(mode)
        msg.frame.can_dlc = len(data_bytes)
        
        # 正确设置数据 - 使用 c_ubyte_Array_8
        for i in range(8):
            if i < len(data_bytes):
                msg.frame.data[i] = data_bytes[i]
            else:
                msg.frame.data[i] = 0
                
        return msg
    
    def _build_can_id(self, mode):
        """构建CAN ID (私有协议格式)"""
        can_id = (mode & 0x1F) << 24
        can_id |= (self.master_id & 0xFFFF) << 8
        can_id |= (self.motor_id & 0xFF)
        return can_id
    
    def _send_can_message(self, msg):
        """发送CAN消息"""
        try:
            # 创建消息数组
            msgs_array = (ZCAN_Transmit_Data * 1)()
            msgs_array[0] = msg
            
            # 发送消息
            ret = self._zcan.Transmit(self._can_handle, msgs_array, 1)
            if ret == 1:
                return True
            else:
                print(f"发送失败，返回值: {ret}")
                return False
                
        except Exception as e:
            print(f"发送消息异常: {e}")
            return False
    
    def _send_message(self, mode, data_bytes):
        """发送CAN消息"""
        if not self._is_chn_open:
            print("通道未打开")
            return False
            
        try:
            # 确保数据字节长度为8
            if len(data_bytes) < 8:
                data_bytes = data_bytes + [0] * (8 - len(data_bytes))
            elif len(data_bytes) > 8:
                data_bytes = data_bytes[:8]
            
            # 创建消息结构体
            msg = self._create_can_message(mode, data_bytes)
            
            # 打印调试信息
            print(f"发送消息 - 模式: 0x{mode:02X}, ID: 0x{msg.frame.can_id:08X}")
            print(f"数据: {[f'0x{b:02X}' for b in data_bytes]}")
            
            # 发送消息
            success = self._send_can_message(msg)
            if success:
                print("消息发送成功")
            else:
                print("消息发送失败")
            return success
            
        except Exception as e:
            print(f"发送消息错误: {e}")
            return False
    
    def _float_to_uint(self, x, x_min, x_max, bits):
        """浮点数转无符号整数"""
        span = x_max - x_min
        offset = x_min
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min
        return int((x - offset) * ((1 << bits) - 1) / span)
    
    def _uint_to_float(self, x, x_min, x_max, bits):
        """无符号整数转浮点数"""
        span = x_max - x_min
        offset = x_min
        return (x * span) / ((1 << bits) - 1) + offset

class PositionModeController(MotorController):
    """位置模式(PP)控制器"""
    
    def __init__(self):
        super().__init__()
        # 位置模式参数
        self.profile_velocity = 0.1      # rad/s
        self.profile_acceleration = 1.0 # rad/s²
        self.target_torque = 0.1        # N.m
        self.position_window = 0.1       # rad
        self.position_window_time = 50  # ms
    
    def enable(self):
        """使能电机 - 通信类型3"""
        try:
            print("发送电机使能命令...")
            # 通信类型3: 电机使能运行
            # 数据区全0
            data_bytes = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            
            if self._send_message(0x03, data_bytes):
                print("电机使能命令发送成功")
                self.current_mode = MotorMode.PP_MODE
                time.sleep(2)  # 等待电机响应
                return True
            else:
                print("电机使能命令发送失败")
                return False
        except Exception as e:
            print(f"使能电机错误: {e}")
            return False
    
    def disable(self):
        """失能电机 - 通信类型4"""
        try:
            print("发送电机停止命令...")
            # 通信类型4: 电机停止运行
            # 数据区全0
            data_bytes = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            
            if self._send_message(0x04, data_bytes):
                print("电机停止命令发送成功")
                self.current_mode = MotorMode.DISABLED
                return True
            else:
                print("电机停止命令发送失败")
                return False
        except Exception as e:
            print(f"停止电机错误: {e}")
            return False
    
    def stop(self):
        """停止电机 (同失能)"""
        return self.disable()
    
    def set_position_mode_params(self, velocity=None, acceleration=None, 
                               torque=None, window=None, window_time=None):
        """设置位置模式参数"""
        if velocity is not None:
            self.profile_velocity = max(0.1, min(44.0, velocity))
        if acceleration is not None:
            self.profile_acceleration = max(1.0, min(100.0, acceleration))
        if torque is not None:
            self.target_torque = max(0.1, min(17.0, torque))
        if window is not None:
            self.position_window = max(0.01, min(1.0, window))
        if window_time is not None:
            self.position_window_time = max(10, min(1000, window_time))
        
        print(f"位置模式参数更新: 速度={self.profile_velocity}rad/s, "
              f"加速度={self.profile_acceleration}rad/s², "
              f"力矩={self.target_torque}N.m")
    
    def move_to_position(self, target_position, velocity=None, blocking=False, timeout=10.0):
        """移动到指定位置 - 通信类型1"""
        try:
            if not self._is_chn_open:
                raise Exception("通道未打开")
                
            if self.current_mode != MotorMode.PP_MODE:
                print("电机未使能，正在使能...")
                if not self.enable():
                    raise Exception("电机使能失败")
                time.sleep(0.2)  # 等待电机使能
            
            # 使用指定速度或默认速度
            move_velocity = velocity if velocity is not None else self.profile_velocity
            
            print(f"移动到位置: {target_position}rad, 速度: {move_velocity}rad/s")
            
            # 通信类型1: 运控模式电机控制指令
            # 数据格式: 
            # Byte0-1: 目标角度(-4π~4π)
            # Byte2-3: 目标角速度(-44~44rad/s)
            # Byte4-5: 目标力矩(-17~17Nm)
            # Byte6-7: Kp, Kd (设为0使用默认值)
            
            position_uint = self._float_to_uint(target_position, -12.57, 12.57, 16)
            velocity_uint = self._float_to_uint(move_velocity, 0, 44.0, 16)
            torque_uint = self._float_to_uint(self.target_torque, 0, 17.0, 16)
            
            data_bytes = [
                (position_uint >> 8) & 0xFF,  # 位置高字节
                position_uint & 0xFF,         # 位置低字节
                (velocity_uint >> 8) & 0xFF,  # 速度高字节
                velocity_uint & 0xFF,         # 速度低字节
                (torque_uint >> 8) & 0xFF,    # 力矩高字节
                torque_uint & 0xFF,           # 力矩低字节
                0x00,  # Kp高字节
                0x00   # Kp低字节
            ]
            
            if self._send_message(0x01, data_bytes):
                print(f"位置指令发送成功")
                
                if blocking:
                    return self._wait_for_position(target_position, timeout)
                else:
                    return True
            else:
                print("位置指令发送失败")
                return False
                
        except Exception as e:
            print(f"移动到位错误: {e}")
            return False
    
    def _wait_for_position(self, target_position, timeout):
        """等待到达目标位置"""
        print(f"等待到达目标位置: {target_position}rad, 超时: {timeout}秒")
        start_time = time.time()
        position_tolerance = 0.05  # 位置容差
        
        while time.time() - start_time < timeout:
            current_pos = self._position
            error = abs(current_pos - target_position)
            
            if error <= position_tolerance:
                print(f"到达目标位置: {target_position}rad, 实际位置: {current_pos:.3f}rad")
                return True
                
            # 每0.5秒打印一次进度
            if int((time.time() - start_time) * 2) % 2 == 0:
                print(f"当前位置: {current_pos:.3f}rad, 误差: {error:.3f}rad")
                
            time.sleep(0.1)
        
        print(f"位置移动超时: 当前位置={self._position:.3f}rad, 目标位置={target_position}rad")
        return False
    
    def jog_positive(self, speed=1.0):
        """正向点动"""
        print(f"正向点动, 速度: {speed}rad/s")
        return self.move_to_position(1.57, speed)  # 正向移动90度
    
    def jog_negative(self, speed=1.0):
        """负向点动"""
        print(f"负向点动, 速度: {speed}rad/s")
        return self.move_to_position(-1.57, speed)  # 负向移动90度
    
    def get_status(self):
        """获取电机状态"""
        return {
            'position': round(self._position, 3),
            'velocity': round(self._velocity, 3),
            'torque': round(self._torque, 3),
            'mode': self.current_mode.name,
            'connected': self._is_chn_open
        }


# 诊断函数
def diagnose_communication():
    """诊断通信问题"""
    from zlgcan import ZCAN, ZCAN_Transmit_Data
    
    zcan = ZCAN()
    
    try:
        print("=== 通信诊断开始 ===")
        
        # 打开设备
        dev_handle = zcan.OpenDevice(41, 0, 0)
        if dev_handle == INVALID_DEVICE_HANDLE:
            print("打开设备失败")
            return
        
        print("设备打开成功")
        
        # 配置通道
        chn_cfg = ZCAN_CHANNEL_INIT_CONFIG()
        chn_cfg.can_type = ZCAN_TYPE_CAN
        
        can_handle = zcan.InitCAN(dev_handle, 0, chn_cfg)
        if can_handle == INVALID_CHANNEL_HANDLE:
            print("初始化通道失败")
            return
        
        print("通道初始化成功")
        
        # 启动通道
        ret = zcan.StartCAN(can_handle)
        if ret != ZCAN_STATUS_OK:
            print("启动通道失败")
            return
        
        print("通道启动成功")
        
        # 测试发送简单消息
        print("\n测试发送简单消息...")
        msg = ZCAN_Transmit_Data()
        msg.transmit_type = 0
        msg.frame.eff = 1  # 扩展帧
        msg.frame.rtr = 0
        msg.frame.can_id = 0x0000FD7F  # 简单测试ID
        msg.frame.can_dlc = 8
        
        # 正确设置数据
        test_data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        for i in range(8):
            msg.frame.data[i] = test_data[i]
        
        msgs = (ZCAN_Transmit_Data * 1)()
        msgs[0] = msg
        
        ret = zcan.Transmit(can_handle, msgs, 1)
        print(f"发送测试消息结果: {ret}")
        
        # 清理
        zcan.ResetCAN(can_handle)
        zcan.CloseDevice(dev_handle)
        print("诊断完成")
        
    except Exception as e:
        print(f"诊断错误: {e}")

# 简化测试函数
def simple_test():
    """简化测试"""
    controller = PositionModeController()
    
    try:
        print("开始简化测试...")
        
        if controller.connect():
            print("连接成功，测试基本通信...")
            
            # 测试使能命令
            print("\n1. 测试使能命令")
            if controller.enable():
                print("使能成功")
                time.sleep(1)
                
                # 测试位置命令
                print("\n2. 测试位置命令")
                if controller.move_to_position(2):
                    print("位置命令发送成功")
                    time.sleep(2)
                
                # 停止
                print("\n3. 停止电机")
                controller.stop()
            else:
                print("使能失败")
                
        else:
            print("连接失败")
            
    except Exception as e:
        print(f"测试错误: {e}")
    finally:
        controller.disconnect()

if __name__ == "__main__":
    print("选择测试模式:")
    print("1. 简化电机测试")
    print("2. 通信诊断")
    
    choice = input("请输入选择 (1 或 2): ").strip()
    
    if choice == "2":
        diagnose_communication()
    else:
        simple_test()