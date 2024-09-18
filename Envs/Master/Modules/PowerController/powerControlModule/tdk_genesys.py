import serial
import time

# ============================================================================
# Commands
# ============================================================================

_INIT_CMD = {
    'address': 'ADR',
    'clear_status': 'CLS',
    'reset': 'RST',
    'remote_mode': 'RMT',
    'multi-drop_option': 'MDAV',
    'master-slave': 'MS'
}

_ID_CMD = {
    'identification': 'IDN',
    'software_version': 'REV',
    'serial_number': 'SN',
    'date_of_last_test': 'DATE'
}

_OUTPUT_CMD = {
    'set_voltage': 'PV',
    'voltage': 'MV',
    'set_current': 'PC',
    'current': 'MC',
    'operation_mode': 'MODE',
    'display_V_n_C': 'DVC',
    'power_supply_status': 'STT',
    'adc_filter': 'FILTER',
    'output_mode': 'OUT',
    'foldback_protection': 'FLD',
    'foldback_delay': 'FLB',
    'reset_foldback_delay': 'RSTFLB',
    'over_voltage_protection': 'OVP',
    'ovp_maximum': 'OVM',
    'under_voltage_limit': 'UVL',
    'auto_restart_mode': 'AST',
    'save_settings': 'SAV',
    'recall_last_settings': 'RCL'
}

OUTPUT_MODE = ['0', '1']


# ============================================================================
# Exception class
# ============================================================================
class GenesysBaseException(Exception):
    """ """
    pass


# ============================================================================
# Genesys base class
# ============================================================================
class GenesysBase(object):
    """ """

    PORT_SETTINGS = {
        'baudrate': 9600,
        'bytesize': serial.EIGHTBITS,
        'parity': serial.PARITY_NONE,
        'stopbits': serial.STOPBITS_ONE,
        'timeout': 1,
        'rtscts': False,
        'dsrdtr': False
    }

    PROTOCOL_SETTINGS = {
        'cr': "\r",
        'rtn_byte': b'\r',
        'default_device_address': 6,
        'write_resp_time': 0.050,  # [s]
        'read_resp_time': 0.050  # [s]
    }

    # WRITE_RESPONSE_TIME   = 0.005     # [s]
    # READ_RESPONSE_TIME    = 0.0015    # [s]

    def __init__(self, port=None, device_address=None):

        self.ser = self._init_ser()

        for key, value in self.PROTOCOL_SETTINGS.items():
            setattr(self, key, value)

        self.cmd_table = self._compile_cmd_table()

        if device_address is None:
            device_address = self.default_device_address
        self.device_address = device_address

        if port is not None:
            self.ser.port = port
            self._open()

        self.device_is_initialized = False
        if self.is_open() and not self.device_is_initialized:
            self._init_device()

    def __enter__(self):
        """ """
        if not self.device_is_initialized:
            self._init_device()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """ """
        if self.ser.isOpen():
            self.close()

        if exc_type is not None:
            return False

        return True

    def _init_ser(self):
        """initializes pyserial.Serial object with PORT_SETTINGS"""
        ser = serial.Serial()
        for key, value in self.PORT_SETTINGS.items():
            setattr(ser, key, value)
        return ser

    def _init_device(self):
        """ """
        self.send_address()
        self.device_is_initialized = True
        return

    def _compile_cmd_table(self):
        """defines which commands are part of returned command table"""
        return {**_INIT_CMD, **_ID_CMD, **_OUTPUT_CMD}

    # === serial communication ====================================================

    def _open(self):
        if not self.ser.isOpen():
            self.ser.open()
            if not self.ser.isOpen():
                raise GenesysBaseException('Failed to open port')
        else:
            raise GenesysBaseException('Port already open')

    def close(self):
        if self.ser.isOpen():
            self.ser.close()
        else:
            raise GenesysBaseException('Port already closed')

    def is_open(self):
        """returns True if serial port is open"""
        return self.ser.isOpen()

    def _write(self, cmd, verbose=False):
        """sends cmd to hardware

        cmd needs to be in a shape which can be understand by the hardware.

        """
        if cmd[-len(self.cr):] is not self.cr:
            cmd = ''.join([cmd, self.cr])

        if verbose:
            print('write...\n' + repr('    cmd: {}'.format(cmd)))
        rtn = self.ser.write(cmd.encode('utf-8'))
        time.sleep(self.write_resp_time)
        return rtn

    def _read(self, verbose=False):
        """
        Reads back string from device, returns immediately if no data is received.
        """
        if verbose:
            print('read...')
        msg = []

        while True:
            rtn = self.ser.read(1)
            if not rtn:  # No data received, timeout occurred
                print("Timeout or no data received from device")
                return None
            msg.append(rtn.decode('utf-8'))
            if rtn == self.rtn_byte:
                break  # End of message

        if verbose:
            print(repr('    msg: {}'.format(''.join(msg))))
        time.sleep(self.read_resp_time)
        return ''.join(msg[:-1])

    def _query(self, cmd, verbose=False):
        """ """
        # cmd = self._get_short_cmd(cmd)
        self._write(cmd, verbose=verbose)
        return self._read(verbose=verbose)

    def set_value(self, cmd_str, val, **kwargs):
        """specifies syntax for set commands"""
        cmd = ''.join([cmd_str, ' ', str(val)])
        self._query(cmd, **kwargs)

    def get_value(self, cmd_str, **kwargs):
        """specifies syntax for get commands"""
        cmd = ''.join([cmd_str, '?'])
        return self._query(cmd, **kwargs)

    # === high-level commands ====================================================

    def send_address(self):
        self.set_value('ADR', self.device_address)

    def get_remote_mode(self):
        return self.get_value('RMT')

    def get_set_point_voltage(self):
        return self.get_value('PV')

    def get_set_point_current(self):
        return self.get_value('PC')

    def get_current(self):
        return self.get_value('MC')

    def get_voltage(self):
        return self.get_value('MV')

    def set_voltage(self, voltage):
        self.set_value('PV', voltage)

    def set_current(self, current):
        self.set_value('CV', current)

    def set_output_mode(self, output_mode):
        self.set_value('OUT', output_mode)

    def power_on(self):
        self.set_output_mode(1)

    def power_off(self):
        self.set_output_mode(0)

    def get_output_mode(self):
        return self.get_value('OUT')

    def set_max_voltage(self, voltage):
        self.set_value('OVP', voltage)

    def get_over_voltage_protection(self):
        return self.get_value('OVP')

    def get_model_id(self):
        return self.get_value('IDN')

    def get_software_version(self):
        return self.get_value('REV')

    def get_serial_number(self):
        error_codes = ['C01', 'C02', 'C03', 'C04', 'C05']
        serial_number = self.get_value('SN')
        if not serial_number or serial_number in error_codes:
            raise Exception("未连接或协议不匹配")

        result_dict = {
            'conn_type': 'COM',
            'protocol': 'tdk_genesys',
            'product_version': 'tdk_genesys',
            'serial_number': serial_number,
            'resource': self.ser.port
        }
        return result_dict

    def get_last_test_date(self):
        return self.get_value('DATE')

    def get_operation_mode(self):
        return self.get_value('MODE')

    def set_remote_mode(self, remote_mode):
        self.set_value('RMT', remote_mode)

    def switch_to_panel_mode(self):
        self.set_remote_mode(0)

    def switch_to_remote_mode(self):
        self.set_remote_mode(1)

    def get_display_V_n_C(self):
        return self.get_value('DVC')

    def get_status(self, verbose: bool = False):
        """
        读取电源的信息状态。
        """

        # 查询输出状态
        output_state = self.get_output_mode()
        output_state = True if output_state == 'ON' else False

        voltage_value, voltage_set_value, current_value, current_set_value, voltage_set_max_value, _ = tuple(
            self.get_display_V_n_C().split(','))

        voltage_value = '{:.2f}'.format(float(voltage_value))
        current_value = '{:.2f}'.format(float(current_value))

        # 获取电源输出模式
        power_output_mode = self.get_operation_mode()

        # 获取远程模式状态
        remote_mode = 'Panel' if self.get_remote_mode() == 'LOC' else 'Remote'

        result_dict = {
            'current_set_value': current_set_value,
            'voltage_set_max_value': voltage_set_max_value,
            'voltage_set_value': voltage_set_value,
            'current_value': current_value,
            'voltage_value': voltage_value,
            'power_is_on': output_state,
            # 'power_is_overheat': power_is_overheat,
            'power_output_mode': power_output_mode,
            'power_operation_status': remote_mode
        }

        if verbose:
            for key, value in result_dict.items():
                print(f'{key}: {value}')
        return result_dict


# ============================================================================
# Genesys Models
# ============================================================================
class Genesys_20_75(GenesysBase):
    """ """
    MAX_VOLTAGE = 20  # [V]
    MAX_CURRENT = 75  # [A]


if __name__ == '__main__':
    with GenesysBase('COM3') as g:
        print(g.get_status())
