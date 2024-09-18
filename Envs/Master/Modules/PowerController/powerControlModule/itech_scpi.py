import serial


# ============================================================================
# Itech SCPI base class
# ============================================================================
class ItechScpiBase(object):
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

    def send_command(self, command):
        self.ser.write(f"{command}\n".encode())
        return self.ser.readline().decode().strip()

    def switch_to_remote_mode(self):
        """
        切换到远程模式，并会禁用面板上的所有按键。
        """
        self.send_command('SYST:REM')

    def switch_to_panel_mode(self):
        """
        切换到面板模式。
        """
        self.send_command('SYST:LOC')

    def power_on(self):
        self.send_command('OUTP ON')

    def power_off(self):
        self.send_command('OUTP OFF')

    def set_max_voltage(self, voltage):
        self.send_command(f'VOLT:PROT {voltage}')

    def set_voltage(self, voltage):
        self.send_command(f'VOLT {voltage}')

    def set_current(self, current):
        self.send_command(f'CURR {current}')

    def get_voltage(self):
        return self.send_command('MEAS:VOLT?')

    def get_current(self):
        return self.send_command('MEAS:CURR?')

    def get_remote_mode_status(self):
        return self.send_command('SYST:REM?')

    def get_max_voltage(self):
        return self.send_command('VOLT:PROT?')

    def get_max_current(self):
        return self.send_command('CURR:PROT?')

    def get_output_mode(self):
        return self.send_command('OUTP?')

    def get_set_current(self):
        return self.send_command('CURR?')

    def get_set_voltage(self):
        return self.send_command('VOLT?')

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
        parts = self.send_command('MEAS:ALL?').split(',')
        voltage_value = '{:.2f}'.format(float(parts[0]))
        if voltage_value == '-0.00':  # 当电压为0的时候会显示为-0.00
            voltage_value = '0.00'

        result_dict = {
            'current_set_value': '{:.2f}'.format(float(self.get_set_current())),
            'voltage_set_max_value': '{:.2f}'.format(float(self.get_max_voltage())),
            'voltage_set_value': '{:.2f}'.format(float(self.get_set_voltage())),
            'current_value': '{:.2f}'.format(float(parts[1])),
            'voltage_value': voltage_value,
            'power_is_on': True if self.get_output_mode() == '1' else False,
            # 'power_is_overheat': power_is_overheat,
            'power_output_mode': 'CV',
            'power_operation_status': 'Panel' if self.get_remote_mode_status() == '0' else 'Remote'
        }

        if verbose:
            for key, value in result_dict.items():
                print(f'{key}: {value}')

        return result_dict

    def get_serial_number(self):
        """
        查询设备信息。
        """
        response = self.send_command('*IDN?')
        # 解析响应以获取序列号
        parts = response.split(',')
        if len(parts) == 4:
            product_version = parts[1]
            serial_number = parts[2]
        else:
            raise Exception("未连接或协议不匹配")

        result_dict = {
            'conn_type': 'COM',
            'protocol': 'itech_scpi',
            'product_version': product_version,
            'serial_number': serial_number,
            'resource': self.port
        }
        return result_dict


if __name__ == '__main__':
    with ItechScpiBase('COM3') as ps:
        try:
            ps.get_serial_number()
        except Exception as e:
            print(f"An error occurred: {e}")
