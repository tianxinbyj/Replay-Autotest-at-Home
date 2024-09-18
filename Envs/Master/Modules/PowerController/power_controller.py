import glob
import json
import os
import sys
import threading
from typing import Optional, Callable

import serial.tools.list_ports

from Envs.Master.Modules.PowerController.powerControlModule.itech_pyvisa import *
from Envs.Master.Modules.PowerController.powerControlModule.itech_scpi import *
from Envs.Master.Modules.PowerController.powerControlModule.itech_serial import *
from Envs.Master.Modules.PowerController.powerControlModule.tdk_genesys import *

# 创建一个线程锁，用于同步访问串行资源
serial_lock = threading.Lock()

# 加入了一个缓存文件
CACHE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'serial_ports_cache.json')


class PowerSupplyObjectManager:
    def __init__(self):
        self.instruments = {}
        self.serial_lock = threading.Lock()  # 创建一个线程锁，用于同步访问串行资源
        self.display_names = []  # 用于给gui显示
        self.serial_numbers = []  # 用于在gui执行操作
        self.initialize_instruments()

    def initialize_instruments(self):
        """
        初始化设备并连接
        """
        for instrument in get_power_supplies():
            try:
                serial_number = instrument['serial_number']
                protocol = instrument['protocol']
                port = instrument['resource']
                self.instruments[serial_number] = get_instrument_class(protocol)(port)

                # 使用型号作为设备名，如果已经存在了，就按照下划线累加表示。
                name = instrument['product_version']
                if name not in self.display_names:
                    self.display_names.append(name)
                else:
                    count = 1
                    while f"{name}_{count}" in self.display_names:
                        count += 1
                    self.display_names.append(f"{name}_{count}")

                self.serial_numbers.append(serial_number)
            except Exception as e:
                print(f"Error initializing instrument on {instrument}: {e}")

    def get_instrument(self, serial_number):
        """
        获取设备
        """
        return self.instruments.get(serial_number)

    def close_all_instruments(self):
        """
        断开所有设备的连接
        """
        for instrument in self.instruments.values():
            try:
                instrument.close()  # 断开所有设备的连接
            except Exception as e:
                print(f"Error closing instrument: {e}")

    def perform_instrument_action(self, serial_number: str, action: str, *args, **kwargs):
        """
        通用的仪器操作执行函数。
        :param serial_number: 电源序列号, 唯一。
        :param action: 要执行的操作名称，如 'power_on', 'set_voltage' 等
        :param args: 传递给操作的位置参数
        :param kwargs: 传递给操作的关键字参数
        :return: 操作结果
        """
        with self.serial_lock:
            instrument = self.get_instrument(serial_number)
            if instrument:
                try:
                    method = getattr(instrument, action)
                    return method(*args, **kwargs)
                except Exception as e:
                    print(f"Error performing action {action} on instrument {serial_number}: {e}")
                    return None


def determine_protocol(port) -> Optional[dict]:
    """
    识别给定端口上连接的设备使用的通信协议。
    :param port: 要检查的端口名称。
    :return: 识别到的协议列表或None（如果无法识别协议）。
    """
    print('正在匹配串口设备和协议...')
    try:
        with ItechScpiBase(port) as ps:
            result = ps.get_serial_number()
            if result and result['serial_number']:
                return result
    except Exception as e:
        print(f"Itech SCPI protocol failed on {port}: {e}")

    try:
        with ItechSerialBase(port) as ps:
            result = ps.get_serial_number()
            if result and result['serial_number']:
                return result
    except Exception as e:
        print(f"Itech Frame Format protocol failed on {port}: {e}")

    try:
        with GenesysBase(port) as g:
            result = g.get_serial_number()
            if result and result['serial_number']:
                return result
    except Exception as e:
        print(f"Genesys protocol failed on {port}: {e}")

    return None


def load_cache() -> Optional[List[dict]]:
    if os.path.exists(CACHE_FILE):
        try:
            with open(CACHE_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading cache: {e}")
    return None


def save_cache(data: List[dict]):
    try:
        with open(CACHE_FILE, 'w') as f:
            json.dump(data, f)
    except Exception as e:
        print(f"Error saving cache: {e}")


def match_protocol(port: str, protocol: str) -> Optional[dict]:
    """
    使用指定的协议匹配端口上的设备。
    :param port: 要检查的端口名称。
    :param protocol: 要使用的协议列表。
    :return: 识别到的协议列表或None（如果无法识别协议）。
    """
    try:
        if protocol == "itech_serial":
            with ItechSerialBase(port) as ps:
                power_info = ps.get_serial_number()
        elif protocol == "itech_scpi":
            with ItechScpiBase(port) as ps:
                power_info = ps.get_serial_number()
        elif protocol == "tdk_genesys":
            with GenesysBase(port) as g:
                power_info = g.get_serial_number()
        else:
            return None

        if power_info and power_info.get('serial_number'):
            return power_info

    except Exception as e:
        print(f"{protocol} protocol failed on {port}: {e}")

    return None


def check_ports_with_caches(ports, caches) -> List[dict]:
    """
    检查端口并使用指定的协议进行匹配。
    :param ports: 要检查的端口列表。
    :param caches: 要使用的协议列表。
    :return: 包含端口和对应协议的列表。
    """
    power_infos = []

    for cache in caches:
        port = cache['resource']
        protocol = cache['protocol']
        if port in ports:
            try:
                # 检查port是否可以打开
                s = serial.Serial(port)
                s.close()

                power_info = match_protocol(port, protocol)
                if power_info:
                    power_infos.append(power_info)
            except (OSError, serial.SerialException):
                pass

    return power_infos


def list_serial_ports() -> Optional[List]:
    """
    列出当前系统上可用的串行端口，并尝试识别连接设备的协议。
    :return: 包含端口和对应协议的列表，如果无法识别端口则返回None。
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(10)]  # 假设最多支持10个COM口，可以修改。
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    # 使用缓存加速
    cached_protocols = load_cache()
    if cached_protocols:
        print("尝试使用缓存的串口协议")
        result = check_ports_with_caches(ports, cached_protocols)
        if result and len(cached_protocols) == len(result):
            print("使用缓存成功")
            return result

    print("缓存已过期，更新...")
    result = []
    for port in ports:
        try:
            # 检查port是否可以打开
            s = serial.Serial(port)
            s.close()

            # 判断该port连接设备的协议
            device_info = determine_protocol(port)
            if device_info:
                result.append(device_info)
        except (OSError, serial.SerialException):
            pass

    # 更新缓存内容
    if result:
        save_cache(result)

    return result


def list_visa_devices():
    """
    列出系统上使用VISA接口的设备。
    :return: 使用VISA接口的设备列表。
    """
    rm = pyvisa.ResourceManager()
    resources = rm.list_resources()
    result = []
    if resources:
        for idx, resource in enumerate(resources, start=1):
            # print(f"{idx}. {res}")
            if 'USB' in resource:
                # print(handle_usb_pattern(res))
                with ItechPyvisaBase(resource) as instrument:
                    device_info = instrument.query_idn()
                    if device_info:
                        result.append(device_info)

    return result


def get_power_supplies() -> Optional[List]:
    """
    获取并加载所有已连接的电源设备。
    :return: 包含所有已连接设备信息的列表。
    """
    power_supplies = []

    power_supplies.extend(list_serial_ports() or [])
    # power_supplies.extend(list_visa_devices() or [])

    return power_supplies


def get_instrument_class(protocol: str) -> Callable:
    """
    根据指定的协议名获取对应的仪器类。
    :param protocol: 协议名称（如 'tdk_genesys', 'itech_pyvisa', 'itech_serial'）。
    :return: 对应协议的仪器类。
    :raises ValueError: 如果协议不受支持。
    """
    if protocol == 'tdk_genesys':
        return GenesysBase
    elif protocol == 'itech_pyvisa':
        return ItechPyvisaBase
    elif protocol == 'itech_serial':
        return ItechSerialBase
    elif protocol == 'itech_scpi':
        return ItechScpiBase
    else:
        raise ValueError(f"Unsupported protocol: {protocol}")


def set_current(power_inter, serial_number, value):
    try:
        power_inter.perform_instrument_action(serial_number, 'set_current', value)
        return True
    except Exception as e:
        print("HTTPException caught with :", e)
        return False


def set_voltage(power_inter, serial_number, value):
    try:
        power_inter.perform_instrument_action(serial_number, 'set_max_voltage', 20)
        power_inter.perform_instrument_action(serial_number, 'set_voltage',
                                              value)
        return True
    except Exception as e:
        print("HTTPException caught with :", e)
        return False


def power_on(power_inter, serial_number):
    try:
        power_inter.perform_instrument_action(serial_number, 'power_on')
        return True
    except Exception as e:
        print("HTTPException caught with :", e)
        return False


def power_off(power_inter, serial_number):
    try:
        power_inter.perform_instrument_action(serial_number, 'power_off')
        return True
    except Exception as e:
        print("HTTPException caught with :", e)
        return False


def get_status(power_inter, serial_number):
    try:
        return power_inter.perform_instrument_action(serial_number, 'get_status')
    except Exception as e:
        print("HTTPException caught with :", e)
        return False


def switch_to_panel_mode(power_inter, serial_number):
    try:
        power_inter.perform_instrument_action(serial_number, 'switch_to_panel_mode')
        return True
    except Exception as e:
        print("HTTPException caught with :", e)
        return False


def switch_to_remote_mode(power_inter, serial_number):
    try:
        power_inter.perform_instrument_action(serial_number, 'switch_to_remote_mode')
        return True
    except Exception as e:
        print("HTTPException caught with :", e)
        return False


if __name__ == "__main__":
    power_ctrl = PowerSupplyObjectManager()
    status = get_status(power_inter=power_ctrl, serial_number=power_ctrl.serial_numbers[0])
    # set_current_flag = set_current(power_ctrl, power_ctrl.serial_numbers[0], 10)
    # set_voltage_flag = set_voltage(power_ctrl, power_ctrl.serial_numbers[0], 15)
    # power_on_flag = power_on(power_ctrl, power_ctrl.serial_numbers[0])
    power_off_flag = power_off(power_ctrl, power_ctrl.serial_numbers[0])
    # switch_to_panel_mode_flag = switch_to_panel_mode(power_ctrl, power_ctrl.serial_numbers[0])
    # switch_to_remote_mode_flag = switch_to_remote_mode(power_ctrl, power_ctrl.serial_numbers[0])