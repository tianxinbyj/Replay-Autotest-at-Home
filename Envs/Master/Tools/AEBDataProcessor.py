"""
@Author: BU YUJUN
@Date: 2025/6/24 上午10:17
"""
import json
import os
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import boto3
from botocore.client import Config

import pandas as pd
import paramiko
import yaml

from Envs.Master.Modules.DataTransformer import DataTransformer, DataLoggerAnalysis
from Envs.Master.Tools.QCameraConfig import QCameraConfig
from Utils.FileDeliverer import calculate_local_folder_size, deliver_file, create_remote_folder, get_remote_disk_space
from Utils.Libs import ros_docker_path, kill_tmux_session_if_exists, create_folder, force_delete_folder, \
    generate_unique_id, ThreadManager, project_path, check_folder_space, check_connection


# Replay0Z上的
AEBRawDataPath = '/media/data/Q_DATA/AebRawData'
AEBReplayDataPath = '/media/data/Q_DATA/AebReplayData'

vehicle_id = {
    '005': 'Q3402',
    '006': 'Q3401',
    '001': 'Z0001',
    '194': 'Q0194'
}


def reindex(install_path, bag_folder):
    tmux_session = generate_unique_id(str(time.time())) + '_reindex_ses'
    tmux_window = generate_unique_id(str(time.time())) + '_reindex_win'
    os.system(f'tmux new-session -s {tmux_session} -n {tmux_window} -d')
    time.sleep(0.1)
    os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "bash {ros_docker_path}" C-m')
    time.sleep(3)
    os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
              f'"source {install_path}/setup.bash" C-m')
    time.sleep(1)
    os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "ros2 bag reindex -s sqlite3 {bag_folder}" C-m')

    while not os.path.exists(os.path.join(bag_folder, 'metadata.yaml')):
        time.sleep(1)
    kill_tmux_session_if_exists(tmux_session)


class AEBDataCloud:

    def __init__(self, endpoint_url, aws_access_key_id, aws_secret_access_key):
        # 配置S3客户端
        self.s3_client = boto3.client(
            's3',
            endpoint_url=endpoint_url,  # 你的S3 endpoint
            aws_access_key_id=aws_access_key_id,  # 替换为你的Access Key
            aws_secret_access_key=aws_secret_access_key,  # 替换为你的Secret Key
            config=Config(signature_version='s3v4'),
            region_name='cn-north-1'  # 区域名称, 可根据实际情况修改
        )

        self.download_record_path = Path(AEBRawDataPath) / 'AEBDownloadRecord.json'
        self.download_record = self.load_download_status()

    def list_objects(self, bucket_name, s3_path, version='', include=None, exclude=None, local_dir=AEBRawDataPath):
        if include is None:
            include = []
        if exclude is None:
            exclude = []

        try:
            object_list = {}
            total_size = 0
            total_objects = 0
            filtered_objects = 0
            continuation_token = None

            # 循环处理分页, 直到获取所有对象
            while True:
                # 准备请求参数
                params = {
                    'Bucket': bucket_name,
                    'Prefix': s3_path
                }
                # 如果有续传令牌, 添加到请求参数中
                if continuation_token:
                    params['ContinuationToken'] = continuation_token

                # 调用API获取对象列表
                response = self.s3_client.list_objects_v2(**params)

                # 处理当前页的对象
                if 'Contents' in response:
                    page_objects = len(response['Contents'])
                    total_objects += page_objects

                    for obj in response['Contents']:
                        s3_key = obj['Key']

                        if version in self.download_record and s3_key in self.download_record[version]:
                            print(f'{version}:{s3_key} 已下载, 排除')
                            continue

                        # 应用包含过滤条件
                        if len(include) and (not any([i in s3_key for i in include])):
                            continue

                        # 应用排除过滤条件
                        if len(exclude) and (any([e in s3_key for e in exclude])):
                            continue

                        # 显示符合条件的对象
                        total_size += obj['Size']
                        filtered_objects += 1
                        object_list[s3_key] = obj['Size']

                        print(f"- {s3_key} (大小 {round(obj['Size'] / 1024 ** 2, 2)} MB / {round(total_size / 1024 ** 2, 2)} MB) - {filtered_objects}个")

                # 检查是否还有更多对象
                if not response.get('IsTruncated', False):
                    break

                # 更新续传令牌, 用于获取下一页
                continuation_token = response.get('NextContinuationToken')

            # 对object list汇总处理, 成为下载单元
            package_totally = {}
            for s3_key in object_list:
                if '/install/' in s3_key:
                    unit = s3_key.split('/install/')[0]
                    if unit not in package_totally and any([f'{unit}/rosbag' in s3_key for s3_key in object_list]):
                        package_totally[unit] = 0

            for s3_key, s3_size in object_list.items():
                for unit in package_totally:
                    if unit in s3_key:
                        package_totally[unit] += s3_size
                        break

            sorted_list = sorted(package_totally.items(), key=lambda x: x[1], reverse=False)
            package_totally = dict(sorted_list)

            # 输出汇总信息
            print('=========================================================================')
            print(f"在路径 {s3_path} 下共找到 {total_objects} 个对象, "
                  f"符合条件的有 {filtered_objects} 个, "
                  f"总大小 {round(total_size / 1024 ** 3, 2)} GB")
            for unit, unit_size in package_totally.items():
                print(f'----> {unit}: 大小 >>>> {round(unit_size / 1024 ** 3, 2)} GB <<<<')

            if local_dir is not None:
                free, _, _ = check_folder_space(local_dir)
                print('=========================================================================')
                print(f'{AEBRawDataPath} 有空间 >>>> {round(free / 1024 ** 3, 2)} GB <<<<')
                s = 0
                ratio = 1.5
                package_for_download = {}
                for unit, unit_size in package_totally.items():
                    s += unit_size
                    if s * ratio + 20 * 1024 ** 3 < free:
                        package_for_download[unit] = unit_size
                        print(f'--------> {unit}: 大小 >>>> {round(unit_size / 1024 ** 3, 2)} GB <<<<, 累积大小 >>>> {round(s / 1024 ** 3, 2)} GB <<<<')
                    else:
                        break

                if not package_for_download:
                    print('空间不足, 无法下载任意package')

                return package_for_download

            return package_totally

        except Exception as e:
            print(f"列出对象时出错: {e}")
            return None

    def download_directory(self, bucket_name, s3_path, version='', include=None, exclude=None, target_num=None, target_size=None):
        if include is None:
            include = []
        if exclude is None:
            exclude = []

        t0 = time.time()
        local_dir = AEBRawDataPath
        os.makedirs(local_dir, exist_ok=True)

        """递归下载S3路径下的所有文件"""
        total_downloaded = 0
        total_size = 0
        break_flag = False

        # 使用分页器处理超过1000个对象的情况
        paginator = self.s3_client.get_paginator('list_objects_v2')
        for page in paginator.paginate(Bucket=bucket_name, Prefix=s3_path):
            if 'Contents' in page:
                for obj in page['Contents']:
                    s3_key = obj['Key']
                    # 跳过目录（S3中没有真正的目录, 只有以/结尾的键）
                    if s3_key.endswith('/'):
                        continue

                    if version not in self.download_record:
                        self.download_record[version] = {}

                    # 非install和parameter的会看是否已经下载过
                    if s3_key in self.download_record[version]:
                        print(f'{version}:{s3_key}已下载过, 跳过')
                        continue

                    # 如果需要包含字符, 则至少包含其中一个
                    if len(include) and (not any([i in s3_key for i in include])):
                        continue

                    # 如果需要排除字符, 则包含一个字符则排除
                    if len(exclude) and (any([e in s3_key for e in exclude])):
                        continue

                    # 构建本地文件路径
                    local_file = os.path.join(local_dir, os.path.relpath(s3_key, s3_path))
                    local_path = os.path.dirname(local_file)
                    os.makedirs(local_path, exist_ok=True)

                    # 下载文件
                    free, used, total = check_folder_space(local_dir)
                    ratio = 1.5
                    print('------------------------------------------------------------------')
                    print(f"{local_dir}可用 / 已用 / 总共: >>>> {round(free / 1024 ** 3, 2)} GB <<<< / {round(used / 1024 ** 3, 2)} GB / {round(total / 1024 ** 3, 2)} GB")
                    print(f"目标文件{s3_key}大小 {round(obj['Size'] / 1024 ** 2, 2)} MB")
                    aeb_raw_data_size = calculate_local_folder_size(local_dir)
                    print(f"AEBRawData占用空间 >>>> {round(aeb_raw_data_size / 1024 ** 3, 3)} GB <<<<, "
                          f"警戒水位 >>>> {round(aeb_raw_data_size * ratio / 1024 ** 3, 3)} GB <<<<, "
                          f"预计可下载 >>>> {round((free - aeb_raw_data_size) / (1 + ratio) / 1024 ** 3, 3)} GB <<<<")
                    if 'install' not in s3_key and 'params' not in s3_key and free < aeb_raw_data_size * ratio:
                        print(f"预留空间不足, 停止下载")
                        break_flag = True
                        break

                    self.s3_client.download_file(bucket_name, s3_key, local_file)
                    if 'install' not in s3_key and 'params' not in s3_key:
                        self.download_record[version][s3_key] = obj['Size'] / 1024 ** 2
                        self.save_download_status()

                    total_downloaded += 1
                    total_size += obj['Size']
                    print(f"下载: {s3_key} -> {local_file}, "
                          f"大小 {round(obj['Size'] / 1024 ** 2, 2)} MB")
                    if target_num and target_size:
                        print(f'个数{total_downloaded} / {target_num} 完成, {round(total_size / 1024 ** 3, 2)} GB / {round(target_size / 1024 ** 3, 2)} GB 完成, {total_size / target_size:.2%} 完成')
                    print(f"共计 {round(total_size / 1024 ** 2, 2)} MB, {round(time.time() - t0)} 秒, {total_downloaded} 个, "
                          f"平均速度 >>>> {round(total_size/(time.time() - t0) / 1024 ** 2)} MB/s <<<<")

            if break_flag:
                break

        # 输出更详细的下载统计信息
        print(f"下载完成, 共下载 {total_downloaded} 个文件, "
              f"总大小 {round(total_size / 1024 ** 2, 2)} MB, "
              f"文件保存在: {os.path.abspath(local_dir)}")

    def upload_directory(self, bucket_name, local_dir, s3_path):
        if not os.path.isdir(local_dir):
            print(f"错误：{local_dir} 不是一个有效的目录")
            return False

        total_uploaded = 0
        total_size = 0

        # 遍历本地目录
        for root, dirs, files in os.walk(local_dir):
            for file in files:
                local_file_path = os.path.join(root, file)

                # 获取相对路径, 用于保持目录结构
                relative_path = os.path.relpath(local_file_path, local_dir)
                s3_key = os.path.join(s3_path, relative_path).replace(os.sep, '/')

                # 上传文件
                file_size = os.path.getsize(local_file_path)
                try:
                    print(f"上传: {local_file_path} -> {s3_key} "
                          f"(大小: {round(file_size / 1024 ** 2, 2)} MB)")

                    self.s3_client.upload_file(local_file_path, bucket_name, s3_key)
                    total_uploaded += 1
                    total_size += file_size
                except Exception as e:
                    print(f"上传失败 {local_file_path}: {e}")

        # 输出上传统计信息
        print(f"上传完成, 共上传 {total_uploaded} 个文件, "
              f"总大小 {round(total_size / 1024 ** 2, 2)} MB")
        return True

    def save_download_status(self):
        with open(self.download_record_path, 'w', encoding='utf-8') as f:
            json.dump(self.download_record, f, ensure_ascii=False, indent=4)

    def load_download_status(self):
        if not self.download_record_path.exists():
            return {}

        with open(self.download_record_path, 'r', encoding='utf-8') as file:
            return json.load(file)


class AEBDataProcessor:

    def __init__(self):
        self.aeb_raw_data_path = Path(AEBRawDataPath)
        self.aeb_replay_data_path = Path(AEBReplayDataPath)
        aeb_raw_data_path_list = self.aeb_raw_data_path.rglob('*rosbag')
        for f in sorted(aeb_raw_data_path_list):
            raw_package_path = f.parent

            # 在AEBReplayDataPath生成新文件夹,确认是否生成过
            replay_package_path = self.aeb_replay_data_path / raw_package_path.relative_to(self.aeb_raw_data_path)
            if (replay_package_path / 'stats.txt').exists():
                print(f'{str(raw_package_path)}有转化记录, 跳过')
                continue

            # check 文件夹是否还在运动
            continue_flag = False
            f_size = calculate_local_folder_size(raw_package_path)
            print(f'正在检查{str(raw_package_path)}')
            for _ in range(3):
                time.sleep(5)
                if f_size != calculate_local_folder_size(raw_package_path):
                    continue_flag = True
                    break
            if continue_flag:
                print(f'{str(raw_package_path)}没有固定, 跳过')
                print(
                    f"{str(AEBRawDataPath)} 占用 >>>> {round(calculate_local_folder_size(AEBRawDataPath) / 1024 ** 3, 2)} <<<< GB, "
                    f"{str(AEBReplayDataPath)} 占用 >>>> {round(calculate_local_folder_size(AEBReplayDataPath) / 1024 ** 3, 2)} <<<< GB")
                continue

            install_gz = raw_package_path / 'install' / 'install.tar.gz'
            if not install_gz.exists():
                print(f'{str(raw_package_path)}未找到install文件, 跳过')
                continue

            parameter_folder = raw_package_path / 'params' / 'J6' / 'camera'
            if not parameter_folder.exists():
                print(f'{str(raw_package_path)}未找到camera参数文件, 跳过')
                continue

            print(f'{str(raw_package_path)}已经固定, 且没有转化记录, 开始转化')
            v2_folder = replay_package_path / 'params'
            if v2_folder.exists():
                shutil.rmtree(v2_folder)
            os.makedirs(v2_folder, exist_ok=True)

            # 生成install文件
            cmd = f'tar -xzvf {install_gz} -C {replay_package_path}'
            p = os.popen(cmd)
            p.read()

            # 生成相机车参文件
            camera_config = QCameraConfig()
            docker_path = '/home/vcar/Downloads/start_docker.sh'
            bin_tool_path = '/home/vcar/ZONE/Tools/v2_txt_to_bin_tools'
            camera_config.gen_v2(parameter_folder, v2_folder, docker_path, bin_tool_path, 'zone')

            # 组织数据
            data_group = {
                'Rosbag': {}, 'Qdata': {}
            }
            for dir_path in sorted(raw_package_path.rglob("*")):
                if dir_path.is_dir() and "rosbag2" in dir_path.name:
                    t0 = datetime.strptime(dir_path.name[8:], "%Y_%m_%d-%H_%M_%S").timestamp()
                    for file in sorted(dir_path.rglob("*.gz")):
                        t1 = datetime.strptime(file.name.split('.')[0], "%Y_%m_%d-%H_%M_%S").timestamp()
                        if t1 - t0 < 30:
                            continue

                        formatted_t0 = datetime.fromtimestamp(t0).strftime("%Y_%m_%d_%H_%M_%S")
                        formatted_t1 = datetime.fromtimestamp(t1).strftime("%Y_%m_%d_%H_%M_%S")
                        key = f'{formatted_t0}-{formatted_t1}'
                        data_group['Rosbag'][key] = {
                            'path': str(file.absolute()), 'start_time': t0, 'end_time': t1,
                        }
                        t0 = t1

            if data_group['Qdata']:
                print('数据需要处理Q图像数据')
                self.run_with_Q(data_group, replay_package_path)
            else:
                print('数据不需要处理Q图像数据')
                self.run_without_Q(data_group, replay_package_path)

    def run_with_Q(self, data_group, replay_package_path, update=False, render=False):
        pass

    def run_without_Q(self, data_group, replay_package_path, update=False, delete=True, render=False):
        task_manager = ThreadManager(max_threads=3)
        install_path = replay_package_path / 'install'
        data_transformer = DataTransformer(install_path)
        render_i = 0
        stats_file = replay_package_path / 'stats.txt'
        for rosbag_key, rosbag_info in data_group['Rosbag'].items():

            # 正在工作时, 删除stats.txt文件
            stats_file.unlink(missing_ok=True)

            # 确认磁盘空间
            free, used, total = check_folder_space(replay_package_path)
            left = 20
            print('------------------------------------------------------------------')
            print(f"可用 / 已用 / 总共: >>>> {round(free / 1024 ** 3, 2)} GB <<<< / {round(used / 1024 ** 3, 2)} GB / {round(total / 1024 ** 3, 2)} GB")
            print(f"{str(AEBRawDataPath)} 占用 >>>> {round(calculate_local_folder_size(AEBRawDataPath) / 1024 ** 3, 2)} <<<< GB, "
                  f"{str(AEBReplayDataPath)} 占用 >>>> {round(calculate_local_folder_size(AEBReplayDataPath) / 1024 ** 3, 2)} <<<< GB")

            if free < left * 1024 ** 3:
                print(f"预留空间不足{left} GB, 停止转化")
                sys.exit(1)

            ros2bag_path = replay_package_path / 'rosbag' / rosbag_key / 'ROSBAG' / 'COMBINE'
            meta_path = ros2bag_path / 'metadata.yaml'

            if not update and meta_path.exists():
                print(f'{rosbag_key} 包已存在')
                continue
            else:
                if ros2bag_path.exists():
                    shutil.rmtree(ros2bag_path)
                os.makedirs(ros2bag_path)

            cmd = f'tar -xzvf {rosbag_info["path"]} -C {ros2bag_path}'
            p = os.popen(cmd)
            p.read()
            reindex(install_path, ros2bag_path)
            print(f'{rosbag_key} 包生成完成')

            # 生成AVM图片,如果render不激活, 只会渲染前3个
            if render or render_i < 3:
                json_config_path = replay_package_path / 'params' / 'json'
                task_manager.add_task(data_transformer.gen_AVM_from_db3, ros2bag_path.parent.parent, json_config_path)
            if delete:
                os.remove(rosbag_info["path"])
                print(f'{rosbag_info["path"]} 已删除')

            render_i += 1

        print('等待所有线程都结束')
        task_manager.stop()
        with open(stats_file, 'w') as file:
            file.write('process_ok')
        raw_package_path = self.aeb_raw_data_path / replay_package_path.relative_to(self.aeb_replay_data_path)
        shutil.rmtree(raw_package_path)
        print(f"所有任务已完成, 程序退出, 删除{raw_package_path}")


class AEBDataTransfer:

    def __init__(self):
        self.aeb_replay_data_path = Path(AEBReplayDataPath)
        self.aeb_data_package_list = {}
        self.package_status_path = Path(AEBReplayDataPath) / 'AEBPackageStatus.json'
        self.package_status = self.load_package_status()
        self.list_packages()

    def list_packages(self):
        for rosbag_dir in self.aeb_replay_data_path.rglob('*rosbag'):
            replay_package_path = rosbag_dir.parent
            if (replay_package_path / 'stats.txt').exists():
                data_label = str(replay_package_path.relative_to(self.aeb_replay_data_path)).replace('/', '*')
                prepared_size, transferred_size = 0, 0
                prepared_num, transferred_num = 0, 0
                for rosbag_path in rosbag_dir.glob('*'):
                    if rosbag_path.is_dir() and str(rosbag_path.name) not in self.package_status:
                        self.package_status[str(rosbag_path.name)] = {
                            'path': str(rosbag_path),
                            'stats': 'prepared',
                            'size': calculate_local_folder_size(rosbag_path),
                        }
                        self.save_package_status()

                    package_status = self.package_status[str(rosbag_path.name)]
                    if package_status['stats'] == 'prepared':
                        prepared_size += package_status['size']
                        prepared_num += 1
                    elif package_status['stats'] == 'transferred':
                        transferred_size += package_status['size']
                        transferred_num += 1

                    self.aeb_data_package_list[data_label] = {
                        'path': str(replay_package_path),
                        'prepared_size': prepared_size,
                        'prepared_num': prepared_num,
                        'transferred_size': transferred_size,
                        'transferred_num': transferred_num,
                    }

        sorted_list = sorted(self.aeb_data_package_list.items(), key=lambda x: x[1]['prepared_size'], reverse=True)
        self.aeb_data_package_list = dict(sorted_list)

        for k, v in self.aeb_data_package_list.items():
            print(k, v['path'],
                  '>>>>', round(v['prepared_size'] / 1024 ** 3, 2), 'GB <<<<', v['prepared_num'],
                  round(v['transferred_size'] / 1024 ** 3, 2), v['transferred_num'])

        return self.aeb_data_package_list

    def transfer_data(self, host, username, password, data_label, remote_base_dir, check_flag=False):
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(
            host,
            port=22,
            username=username,
            password=password,
        )

        # 确认远程空间
        data_size = self.aeb_data_package_list[data_label]['prepared_size']
        remote_space = get_remote_disk_space(ssh, remote_base_dir)
        print(f"{data_label}数据大小 >>>> {round(data_size / 1024 ** 3, 2)} GB <<<<")
        print(f"{host} {remote_base_dir}远程空间大小 >>>> {round(remote_space / 1024 ** 3, 2)} GB <<<<")
        if remote_space < data_size + 20 * 1024 ** 3:
            print('远程空间大小可能不足!')
            if check_flag:
                print('是否继续传送数据 y:是, n:否')
                while True:
                    user_input = input("> ").strip().lower()
                    if user_input == "y":
                        print("继续传送数据")
                        break
                    elif user_input == "n":
                        print("停止传送数据")
                        return
                    else:
                        print("输入无效，请输入 'y' 或者 ‘n’ 以继续...")

        # 建立远程文件夹
        for dir in data_label.split('*'):
            remote_base_dir = create_remote_folder(ssh, remote_base_dir, dir)
            if not remote_base_dir:
                print(0)
                return

        package_path = Path(self.aeb_data_package_list[data_label]['path'])
        install_path = Path(package_path) / 'install'
        parameter_path = Path(package_path) / 'params'
        if not deliver_file(
                host=host, username=username, password=password,
                local_folder=install_path, remote_base_dir=remote_base_dir
        ):
            print(0)
            return

        if not deliver_file(
                host=host, username=username, password=password,
                local_folder=parameter_path, remote_base_dir=remote_base_dir
        ):
            print(0)
            return

        remote_rosbag_path = create_remote_folder(ssh, remote_base_dir, 'rosbag')
        if not remote_rosbag_path:
            print(0)
            return

        for rosbag_path in sorted((package_path / 'rosbag').glob('*')):
            if str(rosbag_path.name) not in self.package_status:
                print(f'{rosbag_path.name}是一个未知数据')
                continue
            elif self.package_status[str(rosbag_path.name)]['stats'] == 'prepared':
                self.package_status[str(rosbag_path.name)]['stats'] = 'busy'
                self.save_package_status()
                try:
                    if deliver_file(
                            host=host, username=username, password=password,
                            local_folder=rosbag_path, remote_base_dir=remote_rosbag_path,
                            backup_space=20,
                    ):
                        self.package_status[str(rosbag_path.name)]['stats'] = 'transferred'
                        self.save_package_status()
                    else:
                        self.package_status[str(rosbag_path.name)]['stats'] = 'prepared'
                        self.save_package_status()
                        print('停止传输数据')
                        break

                except Exception as e:
                    print(e)
                    self.package_status[str(rosbag_path.name)]['stats'] = 'prepared'
                    self.save_package_status()
                    print('停止传输数据')
                    break

    def save_package_status(self):
        with open(self.package_status_path, 'w', encoding='utf-8') as f:
            json.dump(self.package_status, f, ensure_ascii=False, indent=4)

    def load_package_status(self):
        if not self.package_status_path.exists():
            return {}

        with open(self.package_status_path, 'r', encoding='utf-8') as file:
            return json.load(file)


class AEBDataReplay:

    def __init__(self, bench_id):
        bench_config_yaml = os.path.join(project_path, 'Docs', 'Resources', 'bench_config.yaml')
        with open(bench_config_yaml, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
        if bench_id not in data:
            self.host = None
            self.username = None
            self.password = None
            self.replay_room = None
        else:
            bench_config = data[bench_id]
            self.interface_path = f'{bench_config["Master"]["py_path"]}/Envs/ReplayClient/Interfaces'
            self.host = bench_config['Master']['ip']
            self.username = bench_config['Master']['username']
            self.password = str(bench_config['Master']['password'])
            self.replay_room = f'/home/{self.username}/ZONE/AEBReplayRoom'

    def send_cmd(self, command):
        if self.host is None or (not check_connection(self.host)):
            return None

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # 自动添加远程主机密钥

        try:
            # 连接到远程主机
            ssh.connect(hostname=self.host, username=self.username, password=self.password)
            stdin, stdout, stderr = ssh.exec_command(command)
            res = stdout.read().decode()
            err = stderr.read().decode()
            ssh.close()
            return res

        except Exception as e:
            print(f"Error: {e}")
            ssh.close()
            return None

    def create_replay_workspace(self):
        command = f'cd {self.interface_path} && python3 Api_AEBReplayTask.py -a create -f {self.replay_room}'

        print(command)
        res = self.send_cmd(command)

        try:
            r = eval(res.strip().split('\n')[-1])
            return r
        except:
            return False


class AEBDataProcessor2:

    def __init__(self, folder):
        self.data_group = {}
        self.thread_list = []
        self.folder = folder

        '''
        # data group的架构
        20250614-005-AEB
        -- ZoneRos
        ---- rosbag2_2025_06_14-14_45_27
        ---- rosbag2_2025_06_14-15_19_47
        -- Qcraft
        ---- 20250614_105002_Q3402
        ---- 20250614_113118_Q3402
        ---- 20250614_122620_Q3402
        ---- 20250614_125956_Q3402
        ---- 20250614_133921_Q3402
        ---- 20250614_144736_Q3402
        '''

        for f in os.listdir(folder):
            if os.path.exists(os.path.join(folder, f, 'rosbag')):
                self.data_group[f] = {
                    'ZoneRos': os.path.join(folder, f),
                    'Qcraft': []
                }

        for k in self.data_group.keys():
            date = k.split('-')[0]
            zone_v = k.split('-')[1]
            qcraft_v = vehicle_id[zone_v]
            for f in os.listdir(folder):
                if (date in f and qcraft_v in f
                        and os.path.exists(os.path.join(folder, f, 'camera_jpg_img.stf'))):
                    self.data_group[k]['Qcraft'].append(os.path.join(folder, f))
                self.data_group[k]['Qcraft'].sort()

        self.invalid_q_package_path = os.path.join('/media/data/Q_DATA', 'invalid_q_package.json')
        if not os.path.exists(self.invalid_q_package_path):
            with open(self.invalid_q_package_path, "w") as f:
                json.dump([], f, indent=2)  # indent=2 使格式更美观
            self.invalid_q_package = []
        else:
            with open(self.invalid_q_package_path, 'r', encoding='utf-8') as file:
                self.invalid_q_package = json.load(file)

    def reindex(self, install_path, bag_folder):
        tmux_session = 'reindex_ses'
        tmux_window = 'reindex_win'
        os.system(f'tmux new-session -s {tmux_session} -n {tmux_window} -d')
        time.sleep(0.1)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "bash {ros_docker_path}" C-m')
        time.sleep(3)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"source {install_path}/setup.bash" C-m')
        time.sleep(1)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "ros2 bag reindex {bag_folder}" C-m')

        while not os.path.exists(os.path.join(bag_folder, 'metadata.yaml')):
            time.sleep(1)
        kill_tmux_session_if_exists(tmux_session)

    def run(self):
        for k, v in self.data_group.items():
            data_logger = DataLoggerAnalysis(v['ZoneRos'])
            install, ros2bag_info_path = data_logger.load_data()
            data_transformer = DataTransformer(install)

            with open(ros2bag_info_path, 'r', encoding='utf-8') as file:
                ros2bag_info = json.load(file)

            for qcraft_package_path in v['Qcraft']:
                qcraft_package_id = os.path.basename(qcraft_package_path)
                if qcraft_package_id in self.invalid_q_package:
                    print(f'========={qcraft_package_id}========== 为无效场景, 没有匹配的rosbag')
                    continue

                print(f'===========正在解析 {qcraft_package_id} ===========')
                qcraft_package_path = qcraft_package_path[len(data_transformer.q_docker_base_path):]
                h265_config_path, config_json_path, h265_temp_folder = data_transformer.qStf_to_h265(qcraft_package_path)
                with open(h265_config_path, 'r', encoding='utf-8') as file:
                    h265_config = yaml.safe_load(file)

                q_start_time = h265_config['start_time']
                q_end_time = h265_config['end_time']
                print(f'{os.path.basename(qcraft_package_path)}的时间区间为 {q_start_time} - {q_end_time} seconds, {q_end_time - q_start_time} seconds')

                # 删除标定文件夹
                config_path = os.path.join(self.folder, 'COMBINE_BAG', k, 'config')
                if os.path.exists(config_path):
                    shutil.rmtree(config_path)

                q_package_has_overlay = False
                for i, ros2bag in enumerate(ros2bag_info):
                    z_start_time = ros2bag['start_time']
                    z_end_time = ros2bag['end_time']
                    print(f'第{i + 1}个rosbag {os.path.basename(ros2bag["path"])} 时间区间为 {z_start_time} - {z_end_time} seconds, {z_end_time - z_start_time} seconds')
                    t0 = max(q_start_time, z_start_time)
                    t1 = min(q_end_time, z_end_time)
                    if t1 - t0 < 30:
                        print(f'==========> 与Q重叠没有区间段')
                        continue

                    q_package_has_overlay = True
                    print(f'==========> 与Q重叠区间段为{t0} - {t1} seconds, {t1 - t0} seconds')
                    d0 = datetime.fromtimestamp(int(t0)).strftime("%Y_%m_%d-%H_%M_%S")
                    d1 = datetime.fromtimestamp(int(t1)).strftime("%Y_%m_%d-%H_%M_%S")

                    # 放置install
                    ros2bag_dir = os.path.join(self.folder, 'COMBINE_BAG', k, f'{d0}={d1}')
                    os.makedirs(ros2bag_dir, exist_ok=True)
                    corr_install = os.path.join(self.folder, 'COMBINE_BAG', k, 'install')
                    if not os.path.exists(corr_install):
                        shutil.move(install, corr_install)

                    # 生成标定参数
                    if not os.path.exists(config_path):
                        create_folder(config_path)
                        camera_config = QCameraConfig()
                        docker_path = '/home/vcar/Downloads/start_docker.sh'
                        bin_tool_path = '/home/vcar/ZONE/Tools/v2_txt_to_bin_tools'
                        camera_config.gen_v2(config_json_path, config_path, docker_path, bin_tool_path,'qcraft')

                    # 筛选符合时间段的q h265文件
                    single_h265_config = {
                        'h265': {},
                        'start_time': t0,
                        'end_time': t1,
                    }
                    for topic in h265_config['h265']:
                        h265_list = h265_config['h265'][topic]['H265_path']
                        timestamp = pd.read_csv(h265_config['h265'][topic]['timestamp_path'], index_col=False)
                        timestamp = timestamp[(timestamp['time_stamp'] < t1) & (timestamp['time_stamp'] > t0)]

                        timestamp_path = os.path.join(ros2bag_dir, f"{topic.replace('/', '')}.csv")
                        timestamp.to_csv(timestamp_path, index=False)
                        h265_list = h265_list[timestamp.index.min() : timestamp.index.max()+1]
                        single_h265_config['h265'][topic] = {
                            'timestamp_path': timestamp_path,
                            'H265_path': h265_list
                        }

                    single_h265_config_path = os.path.join(ros2bag_dir, 'h265_config.yaml')
                    with open(single_h265_config_path, 'w', encoding='utf-8') as file:
                        yaml.dump(single_h265_config, file)

                    # 生成q_ros2bag
                    q_ros2bag_path = data_transformer.h265_to_db3(single_h265_config_path, ros2bag_dir)

                    # 生成z_ros2bag
                    z_ros2bag_zip_path = ros2bag['path']
                    z_ros2bag_path = os.path.join(ros2bag_dir, os.path.basename(z_ros2bag_zip_path).split('.tar')[0])
                    create_folder(z_ros2bag_path, update=True)
                    cmd = f'tar -xzvf {z_ros2bag_zip_path} -C {z_ros2bag_path}'
                    p = os.popen(cmd)
                    p.read()
                    self.reindex(corr_install, z_ros2bag_path)

                    # 合并q和z的rosbag
                    combined_ros2bag_folder = os.path.join(ros2bag_dir, 'ROSBAG', 'COMBINE')
                    create_folder(combined_ros2bag_folder, update=True)
                    data_transformer.combine_db3(
                        input_bags=[q_ros2bag_path, z_ros2bag_path],
                        output_bag=combined_ros2bag_folder,
                        start_time=t0,
                        stop_time=t1,
                    )

                    # 删除多余文件
                    for f in os.listdir(ros2bag_dir):
                        if f not in ['ROSBAG', 'PICTURE']:
                            f_path = os.path.join(ros2bag_dir, f)
                            if os.path.isdir(f_path):
                                shutil.rmtree(f_path)
                            else:
                                os.remove(f_path)

                    # 生成AVM图片
                    json_config_path = os.path.join(config_path, 'json')
                    if os.path.exists(json_config_path):
                        t = threading.Thread(target=data_transformer.gen_AVM_from_db3, args=(ros2bag_dir, json_config_path))
                        t.daemon = True
                        t.start()
                        self.thread_list.append(t)

                # 删除生成的临时H265文件
                force_delete_folder(h265_temp_folder)

                # 无重叠场景加入列表
                if not q_package_has_overlay and qcraft_package_id not in self.invalid_q_package:
                    self.invalid_q_package.append(qcraft_package_id)
                    with open(self.invalid_q_package_path, "w") as f:
                        json.dump(self.invalid_q_package, f, indent=2)  # indent=2 使格式更美观

        print('等待所有线程都结束')
        for t in self.thread_list:
            t.join()
        self.thread_list.clear()
