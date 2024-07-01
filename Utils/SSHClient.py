"""  
Created on 2024/6/21.  
@author: Bu Yujun  
"""
import csv
import glob
import json
import os
import re
import shutil
import socket
import sys
import threading
import time
import dateparser

import numpy as np
import pandas as pd
import paramiko
import yaml


class SSHClient:

    def __init__(self, ip=None, username=None, password=None):
        self.interface_path = None
        self.ip = ip
        self.username = username
        self.password = password

    def load_info(self, ip, username, password):
        self.ip = ip
        self.username = username
        self.password = password

    def set_interface_path(self, project_path):
        self.interface_path = f'{project_path}/Envs/ReplayClient/Interfaces'

    def send_cmd(self, command):
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # 自动添加远程主机密钥
        res = None

        try:
            # 连接到远程主机
            ssh.connect(hostname=self.ip, username=self.username, password=self.password)
            stdin, stdout, stderr = ssh.exec_command(command)
            res = stdout.read().decode()
            err = stderr.read().decode()
            print(res)
            print(err)
            # print(stderr.read().decode())

        except Exception as e:
            print(f"Error: {e}")

        finally:
            ssh.close()
            return res

    def create_test_folder(self, test_folder, create_time=None):
        if not create_time:
            command = f'cd {self.interface_path} && python3 Api_MakeDirs.py -f {test_folder} -t 0'
        else:
            command = f'cd {self.interface_path} && python3 Api_MakeDirs.py -f {test_folder} -t "{(time.time())}"'
        res = self.send_cmd(command)
        print(command)
        return res

    def start_replay(self, scenario_id=None):
        if scenario_id:
            command = f'cd {self.interface_path} && python3 Api_StartReplay.py -s {scenario_id}'
        else:
            command = f'cd {self.interface_path} && python3 Api_StartReplay.py'

        print(command)
        res = self.send_cmd(command)

        command = 'DISPLAY=:0 gnome-terminal --command="tmux attach -t ses_replay"'
        self.send_cmd(command)
        try:
            r = eval(res.strip().split('\n')[-1])
            return r
        except:
            return 0

    def stop_replay(self):
        command = f'cd {self.interface_path} && python3 Api_StopReplay.py'
        self.send_cmd(command)
        print(command)

    def get_replay_process(self):
        command = f'cd {self.interface_path} && python3 Api_GetReplayProcess.py'
        res = self.send_cmd(command)
        try:
            r = eval(res.strip().split('\n')[-1])
            return r
        except:
            return 0

    def scp_file_local_to_remote(self, local_file, remote_file):
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            ssh.connect(self.ip, username=self.username, password=self.password)
            sftp = ssh.open_sftp()
            sftp.put(local_file, remote_file)
            sftp.close()
            ssh.close()
            return 'successful'

        except Exception as e:
            print(f"Error: {e}")
            ssh.close()
            return e

    def scp_folder_remote_to_local(self, local_folder, remote_folder):
        def download_folder_recursive(sftp, remote_dirpath, local_dirpath):
            """
            Recursively download a remote folder to a local path.
            """
            # 创建本地目录结构
            if not os.path.exists(local_dirpath):
                os.makedirs(local_dirpath)

                # 列出远程目录内容
            remote_files = sftp.listdir_attr(remote_dirpath)

            for file_attr in remote_files:
                remote_filename = file_attr.filename
                local_filepath = os.path.join(local_dirpath, remote_filename)

                # 如果是目录，则递归下载
                if file_attr.st_mode & 0o40000:  # 检查是否为目录
                    download_folder_recursive(sftp,
                                              os.path.join(remote_dirpath, remote_filename),
                                              local_filepath)
                else:  # 如果是文件，则下载
                    sftp.get(os.path.join(remote_dirpath, remote_filename), local_filepath)

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # 自动添加远程主机密钥

        try:
            ssh.connect(self.ip, username=self.username, password=self.password)
            sftp = ssh.open_sftp()
            download_folder_recursive(sftp, remote_folder, local_folder)
            print(f"Folder downloaded from {remote_folder} to {local_folder}")
            sftp.close()
            ssh.close()
            return 'successful'

        except Exception as e:
            print(f"Error: {e}")
            ssh.close()
            return e

    def scp_file_remote_to_local(self, local_file, remote_file):
        if os.path.exists(local_file):
            os.remove(local_file)

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            ssh.connect(self.ip, username=self.username, password=self.password)
            sftp = ssh.open_sftp()
            sftp.get(remote_file, local_file)
            sftp.close()
            ssh.close()
            return 'successful'

        except Exception as e:
            print(f"Error: {e}")
            ssh.close()
            return e


if __name__ == '__main__':
    ip = '172.31.131.222'
    username = 'vcar'
    password = '123456'
    project_path = '/home/vcar/work/pythonProject/Replay-Autotest-at-Home'
    ss = SSHClient(ip, username, password)
    ss.set_interface_path(project_path)

    # remote_file = f'/media/data/annotation/20240527_160340_n000002'
    # local_file = '/home/zhangliwei01/789'
    # ss.scp_folder_remote_to_local(local_file, remote_file)

    scenario_id = '20240528_150013_n000002'
    ss.start_replay(scenario_id)
