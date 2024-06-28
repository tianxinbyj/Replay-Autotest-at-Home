import csv
import os
import string
import time
from datetime import datetime

import matplotlib.pyplot as plt
import paramiko
import pytz


class CameraMonitor:
    def __init__(self, host, username="root", password=""):
        if isinstance(host, str):
            self.host = [host]
        elif isinstance(host, list):
            self.host = host
        else:
            raise ValueError("host must be a string or a list of strings")
        self.username = username
        self.password = password
        self.clients = []

        self.desired_fps_values = {}
        self.command_outputs = {}
        # self.previous_values = {}
        self.cam_input_fps = {}
        self.fluctuating_input = {}
        self.titles_written = False

        self.csv_save_folder = '/home/vcar/Documents/replay_fps_record'
        self.csv_filename = 'ssh_command_outputs.csv'

        self.figures_folder = 'figures'

    def check_value(self, key, new_value):
        # Check for value changes and print alerts.
        # Set the threshold to minus or plus 1 of the desired value.
        if abs(int(new_value) - self.desired_fps_values[key]) > 1:
            print(f"Alert: {key} value has changed by more than 1")
            print(f"New value: {new_value}")
            self.fluctuating_input[key] = new_value
        else:
            # Remove from the dict.
            if key in self.fluctuating_input:
                del self.fluctuating_input[key]

    def connect(self):
        for host in self.host:
            try:
                client = paramiko.SSHClient()
                client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                client.connect(host, username=self.username, password=self.password)
                self.clients.append(client)
            except Exception as e:
                print(f"Failed to connect: {e}")

    def is_connected(self):
        """
        Check if the SSH client is connected.

        Returns:
            bool: True if connected, False otherwise.
        """
        for client in self.clients:
            if client.get_transport() is None or not client.get_transport().is_active():
                return False
        return True

    def disconnect(self):
        for client in self.clients:
            client.close()

    def save_fps_info(self):
        # Get the current time for the "time" column
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        utc_time = datetime.now(pytz.utc).timestamp()

        # Prepare a row to write to the CSV file with timestamp and values
        values_row = [utc_time] + [current_time] + [self.command_outputs.get(key, [''])[-1] for key in
                                                    self.command_outputs.keys()]

        csv_save_path = os.path.join(self.csv_save_folder, self.csv_filename)

        # If titles have not been written yet, write the header row with titles
        if not self.titles_written:
            with open(csv_save_path, mode='w', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)

                # Prepare the header row with both timestamp and keys
                header_row = ['UTC'] + ['time'] + list(self.command_outputs.keys())

                # Write the header row to the CSV file
                csv_writer.writerow(header_row)

            self.titles_written = True  # Set the flag to indicate titles have been written

        # Append the values row to the CSV file
        with open(csv_save_path, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(values_row)

        # print(f"Command outputs saved to {self.csv_filename}")

    def get_camera_fps(self, save_result=False):
        # List of commands to execute
        commands_to_execute = [
            'cat /sys/devices/platform/soc/47010000.cam_sys/47060000.cim/fps',
            'cat /sys/devices/platform/soc/47010000.cam_sys/47180000.cim_dma/fps'
        ]
        for index in range(len(self.clients)):
            for command in commands_to_execute:
                # Execute the command on the remote server
                stdin, stdout, stderr = self.clients[index].exec_command(command)

                # Capture the command output as a list of lines
                output_lines = stdout.readlines()

                # Process each line and accumulate values for each key
                for line in output_lines:
                    parts = line.strip().split()
                    translator = str.maketrans('', '', string.punctuation)
                    parts = [word.translate(translator) for word in parts]
                    if len(parts) == 9:
                        type = parts[0]
                        cam_id = parts[2]
                        in_fps = parts[5]
                        out_fps = parts[8]
                        pair = {}
                        name = 'J5A' if self.host[index] in ['172.31.131.35', '172.31.137.50'] else 'J5B'
                        key_in = 'pipe_' + cam_id + '_input' + f'_{name}'
                        value_in = in_fps

                        # 把input cam加入到desired_fps_values
                        if key_in not in self.desired_fps_values:
                            # 目前检测很简单，如果有两个IP就是双J5, 全20; 否则是单J5，全15.
                            self.desired_fps_values[key_in] = int(20) if len(self.host) == 2 else int(15)

                        # 显示所有相机的in fps, 默认不打印，只查看变化大的
                        # print(f'Cam ID: {key_in}, In fps: {in_fps}')

                        key_out = 'pipe_' + cam_id + '_output' + f'_{name}'
                        value_out = out_fps

                        pair[key_in] = value_in
                        pair[key_out] = value_out
                        for key, value in pair.items():
                            if key in self.command_outputs:
                                self.command_outputs[key].append(value)
                            else:
                                self.command_outputs[key] = [value]

        sorted_keys = sorted(self.command_outputs.keys(), key=lambda key: int(key.split('_')[1]))
        for key in sorted_keys:
            value = self.command_outputs[key][-1] if key in self.command_outputs else None
            if 'input' in key.split('_'):
                if value:
                    self.check_value(key, value)

                # print(key, value)
                self.cam_input_fps[key] = value

        # 保存相机帧率信息
        if save_result:
            self.save_fps_info()

    def draw_figures(self):
        if not os.path.exists(self.figures_folder):
            os.makedirs(self.figures_folder)
        # Remove existing image files before saving new ones
        for key in self.command_outputs.keys():
            image_filename = os.path.join(self.figures_folder, f'{key}.png')
            if os.path.exists(image_filename):
                os.remove(image_filename)

        # Create line graphs for each column and save as separate figures
        for key, values in self.command_outputs.items():
            plt.figure()
            plt.plot(values, marker='o')
            plt.title(key)
            plt.xlabel('Time')
            plt.ylabel(key)
            image_filename = os.path.join(self.figures_folder, f'{key}.png')
            plt.savefig(image_filename)
            plt.close()

        subplots_per_figure = 4
        key_groups = [sorted(list(self.command_outputs.keys()))[i:i + subplots_per_figure] for i in
                      range(0, len(self.command_outputs), subplots_per_figure)]
        for group_index, key_group in enumerate(key_groups):
            plt.figure(figsize=(16, 10))
            for i, key in enumerate(key_group, start=1):
                values = self.command_outputs[key]
                plt.subplot(len(key_group), 1, i)
                plt.plot(values, marker='o')
                plt.title(key)
                plt.xlabel('Time')
                plt.ylabel(key)
            all_graphs_filename = os.path.join(self.figures_folder, f'all_{group_index + 1}.png')
            plt.savefig(all_graphs_filename)
            plt.close()

    def get_camera_fps_once(self, save_result):
        try:
            self.get_camera_fps(save_result)
        except Exception as e:
            print(e)


if __name__ == '__main__':
    hostname = ['172.31.131.35', '172.31.131.36']
    username = 'root'
    password = ''
    cm = CameraMonitor(hostname, username, password)
    cm.connect()

    try:
        while True:
            cm.get_camera_fps()
            time.sleep(1)

    except KeyboardInterrupt:
        print("Program stopped.")

    except Exception as e:
        print(e)

    finally:
        cm.disconnect()
        # cm.draw_figures()
