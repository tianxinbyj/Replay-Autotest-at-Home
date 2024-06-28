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

    def start_replay(self):
        command = f'cd {self.interface_path} && python3 Api_StartReplay.py'
        res = self.send_cmd(command)
        print(command)
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
        print(command)
        print(res)


if __name__ == '__main__':
    ip = '172.31.131.222'
    username = 'vcar'
    password = '123456'
    project_path = '/home/vcar/work/pythonProject/Replay-Autotest-at-Home'
    ss = SSHClient(ip, username, password)
    ss.set_interface_path(project_path)
    res = ss.start_replay()
    print('=======')
    print(res)

    time.sleep(5)
    print(ss.get_replay_process())
    ss.stop_replay()
