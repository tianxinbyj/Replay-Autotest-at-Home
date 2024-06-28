import pyvisa


# ============================================================================
# Itech pyvisa base class
# ============================================================================
class ItechPyvisaBase(object):
    def __init__(self, resource):
        self.resource = resource
        self.rm = None
        self.instrument = None

    def __enter__(self):
        # Initialize the resource manager
        self.rm = pyvisa.ResourceManager()
        # Open the instrument resource
        self.instrument = self.rm.open_resource(self.resource)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        # Close the instrument resource
        if self.instrument:
            self.instrument.close()
        # Close the resource manager
        if self.rm:
            self.rm.close()

    def close(self):
        # Close the instrument resource
        if self.instrument:
            self.instrument.close()
        # Close the resource manager
        if self.rm:
            self.rm.close()

    def query_idn(self):
        """
        查询设备信息。
        """
        try:
            # Query the instrument for its IDN string
            idn_response = self.instrument.query("*IDN?")

            # Split the IDN response to extract product version and serial number
            parts = idn_response.split(', ')

            # Extract product version and serial number
            product_version = parts[1] if len(parts) > 1 else None
            serial_number = parts[2] if len(parts) > 2 else None

            # Create the result dictionary
            return {
                'conn_type': 'USB',
                'protocol': 'itech_pyvisa',
                'product_version': product_version,
                'serial_number': serial_number,
                'resource': self.resource
            }

        except pyvisa.errors.VisaIOError as e:
            print(f"VISA I/O Error occurred: {e}")
        except IndexError:
            print("IndexError: Unable to extract product version or serial number from IDN response.")
        except Exception as e:
            print(f"An error occurred: {e}")

        return None

    def switch_to_remote_mode(self):
        """
        切换到远程模式，并会禁用面板上的所有按键。
        """
        self.instrument.write("SYST:REM")

    def switch_to_panel_mode(self):
        """
        切换到面板模式。
        """
        self.instrument.write("SYST:LOC")

    def power_on(self):
        """
        控制电源输出状态，通电。
        """
        self.instrument.write('OUTPut ON')

    def power_off(self):
        """
        控制电源输出状态，下电。
        """
        self.instrument.write('OUTPut OFF')

    def set_max_voltage(self, value: float):
        """
        设置最大电压。
        :param value: 期望的最大电压
        """
        self.instrument.write(f"VOLT:PROT:LEV {value}")

    def set_voltage(self, value: float):
        """
        设置当前电压。
        """
        self.instrument.write(f"VOLT {value}")

    def set_current(self, value: float):
        """
        设置最大电流。
        """
        self.instrument.write(f"CURR {value}")

    def get_status(self, verbose: bool = False):
        """
        读取电源的信息状态。
        """

        # 查询输出状态
        output_state = self.instrument.query('OUTPut?').strip()
        output_state = True if output_state == '1' else False

        # 获取当前电流
        current = self.instrument.query("MEAS:CURR?").strip()

        # 获取当前电压
        voltage = self.instrument.query("MEAS:VOLT?").strip()

        # 获取设定电流
        set_current = self.instrument.query("SOUR:CURR?").strip()

        # 获取设定电压
        set_voltage = self.instrument.query("SOUR:VOLT?").strip()

        # 获取最大电压
        voltage_set_max_value = self.instrument.query("VOLT:PROT:LEV?").strip()

        result_dict = {
            'current_set_value': set_current,
            'voltage_set_max_value': voltage_set_max_value,
            'voltage_set_value': set_voltage,
            'current_value': current,
            'voltage_value': voltage,
            'power_is_on': output_state,
            # 'power_is_overheat': power_is_overheat,
            # 'power_output_mode': power_output_mode,
            'power_operation_status': 'Remote'
        }

        if verbose:
            for key, value in result_dict.items():
                print(f'{key}: {value}')

        return result_dict


if __name__ == '__main__':
    resource_str = 'some resource address'
    with ItechPyvisaBase(resource_str) as instrument:
        result = instrument.query_idn()
        if result:
            print(result)
