import inspect
import time
from typing import Tuple, List

import serial


# === Frame format processing ================================================
def _calculate_checksum(data_list: List[str]) -> str:
    """
    计算校验码
    """
    checksum = sum(int(hex_string, 16) for hex_string in data_list) & 0xFF

    return format(checksum, '02X')


def float_to_hex_le(value: float) -> List[str]:
    """
    Convert float value to hexadecimal.
    :param value: Any float number
    :return: A list of string of the hexadecimal presentation.
    """

    # Multiply the float by 1000 and convert to integer
    int_value = int(value * 1000)

    # Convert to little-endian hexadecimal byte representation
    hex_bytes = int_value.to_bytes(4, byteorder='little')

    # Convert each byte to formatted hexadecimal string
    hex_strings = [f'{byte:02X}' for byte in hex_bytes]

    return hex_strings


def hex_to_decimal_reversed(hex_string: str) -> int:
    reversed_hex_string = ''.join(reversed([hex_string[i:i + 2] for i in range(0, len(hex_string), 2)]))
    return int(reversed_hex_string, 16)


def gen_frame_bytes(*args, **kwargs) -> bytes:
    data_list = ['00' for _ in range(26)]
    for index, value in enumerate(args):
        if 0 <= index < 26:
            data_list[index] = value
    for key, value in kwargs.items():
        if key.startswith('index_'):
            try:
                index = int(key.split('_')[1])
                if 0 <= index < 26:
                    data_list[index] = value
            except ValueError:
                pass  # Ignore if the index cannot be converted to an integer

    data_list[-1] = _calculate_checksum(data_list[:-1])
    return bytes.fromhex(" ".join(data_list))


def translate_hex_data(hex_data: str) -> Tuple[bool, str]:
    if len(hex_data) > 6:
        error_codes = {
            '8': 'Success',
            '9': 'Checksum error',
            'a': 'Parameter error or overflow',
            'b': 'Command cannot be executed',
            'c': 'Invalid command'
        }
        return hex_data[6] in error_codes, error_codes.get(hex_data[6], 'Unknown')


# ============================================================================
# Itech serial base class for RS232
# ============================================================================
class ItechSerialBase(object):
    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port=self.port, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=1)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.ser.isOpen():
            self.ser.close()

    def close(self):
        if self.ser.isOpen():
            self.ser.close()

    # === Power control commands =============================================
    def send_to_power_supply(self, frame_bytes: bytes, max_attempts=3) -> dict:
        for attempt in range(max_attempts):
            try:
                self.ser.write(frame_bytes)
                response_data = self.ser.read(26)
                if len(response_data.hex()) > 6:
                    success, response_detail = translate_hex_data(response_data.hex())
                    if not success:
                        caller_name = inspect.stack()[1][3]
                        if caller_name in ["get_status",
                                           "get_serial_number"]:  # 特殊情况，它的hex不能被translate_hex_data判断为False
                            return {"success": True, "detail": 'Success', "data_hex": response_data.hex()}
                        print(f"Operation Failed: {caller_name}")

                    return {"success": success, "detail": response_detail, "data_hex": response_data.hex()}

            except serial.SerialException as e:
                time.sleep(0.5)

            # If all attempts fail, return a meaningless result
            return {"success": False, "detail": 'COM port error', "data_hex": "0" * 52}

    def get_serial_number(self):
        # 得到通过COM连接的电源产品信息。
        def _asic_filter(origin: str):
            result = ""
            for i in range(0, len(origin), 2):
                result += origin[i + 1]
            return result

        frame_bytes = gen_frame_bytes(index_0='AA', index_2='31')
        response_dict = self.send_to_power_supply(frame_bytes)
        success = response_dict['success']
        if not success:
            raise Exception("未连接或协议不匹配")

        data_hex = response_dict['data_hex']
        product_version = _asic_filter(data_hex[6:16])
        if product_version.endswith('0'):
            product_version = product_version[:-1]
        product_version = "IT" + product_version

        software_version = str(float(data_hex[18:20]) + float(data_hex[16:18]) / 100)
        serial_number = _asic_filter(data_hex[20:40])

        result_dict = {
            'conn_type': 'COM',
            'protocol': 'itech_serial',
            'product_version': product_version,
            'serial_number': serial_number,
            'resource': self.port
        }
        return result_dict

    def switch_to_remote_mode(self):
        """
        切换到远程模式，并会禁用面板上的所有按键。
        """
        frame_bytes = gen_frame_bytes(index_0='AA', index_2='20', index_3='01')
        response = self.send_to_power_supply(frame_bytes)
        success, detail, data_hex = response.values()
        if not success:
            raise Exception("操作失败，详细信息：{}".format(detail))

        return response

    def switch_to_panel_mode(self):
        """
        切换到面板模式。
        """
        frame_bytes = gen_frame_bytes(index_0='AA', index_2='20', index_3='00')
        response = self.send_to_power_supply(frame_bytes)
        success, detail, data_hex = response.values()
        if not success:
            raise Exception("操作失败，详细信息：{}".format(detail))

        return response

    def power_on(self):
        """
        控制电源输出状态，通电。
        """
        frame_bytes = gen_frame_bytes(index_0='AA', index_2='21', index_3='01')
        response = self.send_to_power_supply(frame_bytes)
        success, detail, data_hex = response.values()
        if not success:
            raise Exception("操作失败，详细信息：{}".format(detail))

        return response

    def power_off(self):
        """
        控制电源输出状态，下电。
        """
        frame_bytes = gen_frame_bytes(index_0='AA', index_2='21', index_3='00')
        response = self.send_to_power_supply(frame_bytes)
        success, detail, data_hex = response.values()
        if not success:
            raise Exception("操作失败，详细信息：{}".format(detail))

        return response

    def set_max_voltage(self, value: float):
        """
        设置最大电压。
        :param value: 期望的最大电压
        """
        hex_values = float_to_hex_le(value)
        voltage_kwargs = {f"index_{i + 3}": hex_value for i, hex_value in enumerate(hex_values)}

        frame_bytes = gen_frame_bytes(index_0='AA', index_2='22', **voltage_kwargs)
        response = self.send_to_power_supply(frame_bytes)
        success, detail, data_hex = response.values()
        if not success:
            raise Exception("操作失败，详细信息：{}".format(detail))

        return response

    def set_voltage(self, value: float):
        """
        设置当前电压。
        """
        hex_values = float_to_hex_le(value)
        voltage_kwargs = {f"index_{i + 3}": hex_value for i, hex_value in enumerate(hex_values)}

        frame_bytes = gen_frame_bytes(index_0='AA', index_2='23', **voltage_kwargs)
        response = self.send_to_power_supply(frame_bytes)
        success, detail, data_hex = response.values()
        if not success:
            raise Exception("操作失败，详细信息：{}".format(detail))

        return response

    def set_current(self, value: float):
        """
        设置最大电流。
        """
        hex_values = float_to_hex_le(value)[:2]
        current_kwargs = {f"index_{i + 3}": hex_value for i, hex_value in enumerate(hex_values)}

        frame_bytes = gen_frame_bytes(index_0='AA', index_2='24', **current_kwargs)
        response = self.send_to_power_supply(frame_bytes)
        success, detail, data_hex = response.values()
        if not success:
            raise Exception("操作失败，详细信息：{}".format(detail))

        return response

    def get_status(self, verbose: bool = False):
        """
        读取电源的信息状态。
        :param verbose: 日志显示，False:忽略, True:显示
        :return: 包含电源状态信息的字典，具体包括以下键值对：
                 - 'current_value': 当前电流值（单位：安培）
                 - 'voltage_value': 当前电压值（单位：伏特）
                 - 'current_set_value': 设定电流值（单位：安培）
                 - 'voltage_set_max_value': 设定最大电压值（单位：伏特）
                 - 'voltage_set_value': 设定电压值（单位：伏特）
                 - 'power_is_on': 电源是否开启（True 或 False）
                 - 'power_is_overheat': 电源是否过热（True 或 False）
                 - 'power_output_mode': 电源输出模式（'CV'、'CC' 或 'Unreg'）
                 - 'power_operation_status': 电源操作状态（'Remote' 或 'Panel'）
        """

        def extract_power_status(status_byte: int) -> Tuple[int, int, int, int, bool, int]:
            power_output_status = (status_byte & 0b00000001) >> 0
            overheat_status = (status_byte & 0b00000010) >> 1
            output_mode = (status_byte & 0b00001100) >> 2  # bits 2 and 3
            fan_speed_bits = (status_byte & 0b00111000) >> 3  # bits 4, 5, and 6
            operation_status = (status_byte & 0b10000000) >> 7

            is_max_fan_speed = (fan_speed_bits == 0b101)

            return power_output_status, overheat_status, output_mode, fan_speed_bits, is_max_fan_speed, operation_status

        frame_bytes = gen_frame_bytes(index_0='AA', index_2='26')
        response = self.send_to_power_supply(frame_bytes)
        success, detail, data_hex = response.values()
        if not success:
            raise Exception("操作失败，详细信息：{}".format(detail))

        current_value = hex_to_decimal_reversed(data_hex[6:10]) / 1000
        voltage_value = hex_to_decimal_reversed(data_hex[10:18]) / 1000
        current_set_value = hex_to_decimal_reversed(data_hex[20:24]) / 1000
        voltage_set_max_value = hex_to_decimal_reversed(data_hex[24:32]) / 1000
        voltage_set_value = hex_to_decimal_reversed(data_hex[32:40]) / 1000

        power_status = int(data_hex[18:20], 16)
        power_output_status, overheat_status, output_mode, fan_speed_bits, is_max_fan_speed, operation_status = extract_power_status(
            power_status)
        power_is_on = True if power_output_status == 1 else False
        power_is_overheat = True if overheat_status == 1 else False
        power_output_mode = 'Undefined'
        if output_mode == 0b01:
            power_output_mode = 'CV'
        elif output_mode == 0b10:
            power_output_mode = 'CC'
        elif output_mode == 0b11:
            power_output_mode = 'Unreg'
        power_operation_status = 'Remote' if operation_status == 1 else 'Panel'

        result_dict = {
            'current_set_value': current_set_value,
            'voltage_set_max_value': voltage_set_max_value,
            'voltage_set_value': voltage_set_value,
            'current_value': current_value,
            'voltage_value': voltage_value,
            'power_is_on': power_is_on,
            'power_is_overheat': power_is_overheat,
            'power_output_mode': power_output_mode,
            'power_operation_status': power_operation_status
        }

        if verbose:
            for key, value in result_dict.items():
                print(f'{key}: {value}')

        return result_dict

    def get_current(self):
        """
        Get the current value from the power supply.
        """
        status = self.get_status()
        return status['current_value']


if __name__ == '__main__':
    with ItechSerialBase('COM3') as ps:
        try:
            print(ps.get_status())
        except Exception as e:
            print(f"An error occurred: {e}")
