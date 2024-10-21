"""  
Created on 2024/6/21.  
@author: Bu Yujun  
"""
import os
import time

import paramiko
from scp import SCPClient

from Utils.Libs import bench_config


class SSHClient:

    def __init__(self, ip=None, username=None, password=None):
        self.temp_folder = None
        self.interface_path = None

        # 默认使用ReplayClient作为目标
        if not ip:
            self.ip = bench_config['ReplayClient']['ip']
            project_path = bench_config['ReplayClient']['py_path']
            self.interface_path = f'{project_path}/Envs/ReplayClient/Interfaces'
            self.temp_folder = f'{project_path}/Temp'
        else:
            self.ip = ip
        if not username:
            self.username = bench_config['ReplayClient']['username']
        else:
            self.username = username
        if not password:
            self.password = str(bench_config['ReplayClient']['password'])
        else:
            self.password = str(password)

        # self.clear_temp_folder()

    def load_info(self, ip, username, password):
        self.ip = ip
        self.username = username
        self.password = password

    def set_interface_path(self, project_path=None):
        self.interface_path = f'{project_path}/Envs/ReplayClient/Interfaces'
        self.temp_folder = f'{project_path}/Temp'
        self.clear_temp_folder()

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
            # print(res)
            # print('======================')
            # print(err)

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
        print(command)
        res = self.send_cmd(command)
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
        print(command)
        self.send_cmd(command)

    def get_replay_process(self):
        command = f'cd {self.interface_path} && python3 Api_GetReplayProcess.py'
        res = self.send_cmd(command)
        try:
            r = eval(res.strip().split('\n')[-1])
            return r
        except:
            return 0

    def cut_one_frame(self, scenario_id, frame_num, camera='CAM_FRONT_120', local_folder=None):
        command = (f'cd {self.interface_path} && python3 Api_CutOneFrame.py '
                   f'-s {scenario_id} -f {frame_num} -c {camera}')
        print(command)
        res = self.send_cmd(command)
        try:
            remote_pic_path = res.strip().split('\n')[-1]
            if local_folder:
                local_pic_path = os.path.join(local_folder, os.path.basename(remote_pic_path))
                self.scp_file_remote_to_local(local_pic_path, remote_pic_path)
                return local_pic_path
            else:
                return remote_pic_path
        except:
            return None

    def cut_frames(self, scenario_id, frame_index_list, camera='CAM_FRONT_120', local_folder=None):
        def split_list_into_groups(lst, group_size):
            return [lst[i:i + group_size] for i in range(0, len(lst), group_size)]

        for sub_frame_index_str in split_list_into_groups(frame_index_list, 500):

            frame_index_str = ' '.join([str(f) for f in sub_frame_index_str])
            command = (f'cd {self.interface_path} && python3 Api_CutFrames.py '
                       f'-s {scenario_id} -f {frame_index_str} -c {camera}')
            print(command)
            res = self.send_cmd(command)
            try:
                remote_pic_path = res.strip().split('\n')[-1]
                if local_folder:
                    self.scp_folder_remote_to_local(local_folder, remote_pic_path)
            except:
                return None

        return local_folder

    def get_scenario_info(self, scenario_id, info_type, local_folder=None):
        if info_type == 'VideoInfo':
            command = f'cd {self.interface_path} && python3 Api_GetVideoInfo.py -s {scenario_id}'
        else:
            command = f'cd {self.interface_path} && python3 Api_GetCalib.py -s {scenario_id}'
        print(command)
        res = self.send_cmd(command)
        try:
            remote_file_list = res.strip().split('\n')
            if local_folder:
                local_file_list = []
                for remote_file in remote_file_list:
                    if 'file' in remote_file:
                        remote_file = remote_file.split('file ')[-1]
                        local_file = os.path.join(local_folder, os.path.basename(remote_file))
                        self.scp_file_remote_to_local(local_file, remote_file)
                        local_file_list.append(local_file)
                    elif 'folder' in remote_file:
                        remote_file = remote_file.split('folder ')[-1]
                        local_file = os.path.join(local_folder, os.path.basename(remote_file))
                        self.scp_folder_remote_to_local(local_file, remote_file)
                        local_file_list.append(local_file)

                return local_file_list
            else:
                return remote_file_list
        except:
            return None

    def flash_camera_config(self, ecu_type):
        command = f'cd {self.interface_path} && python3 Api_ConfigFlash.py -e {ecu_type}'
        print(command)
        self.send_cmd(command)

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

    def scp_folder_local_to_remote(self, local_folder, remote_folder):
        self.clear_temp_folder()
        ssh = paramiko.SSHClient()
        ssh.load_system_host_keys()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            ssh.connect(self.ip, username=self.username, password=self.password)
            with SCPClient(ssh.get_transport()) as scp:
                # 递归复制整个文件夹
                scp.put(local_folder, remote_path=remote_folder, recursive=True)

            return 'successful'

        except Exception as e:
            print(f"Error: {e}")
            ssh.close()
            return e

    def clear_temp_folder(self):
        command = f'rm -rf {self.temp_folder}/*'
        print(command)
        self.send_cmd(command)


if __name__ == '__main__':
    ssh = SSHClient()
    local_folder = '/home/zhangliwei01/ZONE/TestProject/2J5/pilot/01_Prediction/20230602_144755_n000003/scenario_info/es37_calib'
    remote_folder = '/home/vcar/work'
    # ssh.scp_folder_local_to_remote(local_folder, remote_folder)

    ssh.flash_camera_config('ES37')

    scenario_id = '20230602_144755_n000003'
 
    local_folder = '/home/caobingqi/ZONE/Data/TestProject/1J5/Pilot/debug'
