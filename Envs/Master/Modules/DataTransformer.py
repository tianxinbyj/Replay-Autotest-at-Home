"""
@Author: BU YUJUN, ZHENG HUIXIN
@Date: 2025/6/18 上午9:45
"""
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime

import pandas as pd
import psutil
import yaml

from Envs.Master.Modules.ConvertDataFolder import reverse_data_folder, get_all_folder_in_target_dir

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from Libs import get_project_path
sys.path.append(get_project_path())
from Envs.Master.Modules.Can2Ros.arxml_asc_parser import asc_parser
from Envs.Master.Modules.Ros2Bag2BirdView import Ros2Bag2BirdView
from Utils.Libs import ros_docker_path, variables, kill_tmux_session_if_exists, project_path, get_folder_size, \
    force_delete_folder
from Utils.Libs import topic2camera, bench_id, bench_config

camera2topic = {b:a for a,b in topic2camera.items()}
camera2camera = {
    'CAM_PBQ_FRONT_FISHEYE': 'CAM_FISHEYE_FRONT',
    'CAM_PBQ_REAR_FISHEYE': 'CAM_FISHEYE_BACK',
    'CAM_PBQ_LEFT_FISHEYE': 'CAM_FISHEYE_LEFT',
    'CAM_PBQ_RIGHT_FISHEYE': 'CAM_FISHEYE_RIGHT',
    'CAM_PBQ_FRONT_WIDE': 'CAM_FRONT_120',
    'CAM_PBQ_REAR': 'CAM_BACK',
}


class DataTransformer:

    def __init__(self, install_path=None, q_docker_base_path='/media/data'):
        self.parse_asc_data_file_name = 'ASCParseData'
        self.ros2bag_ins_name = os.path.join('ROSBAG', 'ROSBAG_INS')
        self.ros2bag_h265_name = os.path.join('ROSBAG', 'ROSBAG_H265')
        self.ros2bag_combine_name = os.path.join('ROSBAG', 'COMBINE')
        self.can_file_name = 'CAN_Trace'
        self.images_file_name = 'Images'
        self.csv = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', 'CAN_Signal.csv')
        self.mapping = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', 'CAN2ROS_ES37.csv')
        self.arxml = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', '20240315-cgy-ES37_IPD_V6.0.arxml')
        self.install_path = install_path
        self.q_docker_base_path = q_docker_base_path

    def kunyiMkv_to_h265(self, kunyi_package_path):
        image_folder = os.path.join(kunyi_package_path, 'Images')
        if not os.path.exists(image_folder):
            print(f'{kunyi_package_path} <UNK>')
            return None

        # 生成config
        video_config = {}
        video_config_path = os.path.join(image_folder, 'video_config.yaml')
        for topic, camera_name in topic2camera.items():
            video_list = glob.glob(os.path.join(image_folder, f'{camera_name}', f'*{camera_name}.mkv'))
            if not len(video_list):
                print(f'No video file found for {camera_name} in {kunyi_package_path}')
                return None
            if len(video_list) > 1:
                print(f'Multiple video files found for {camera_name} in {kunyi_package_path}')
                return None

            video_config[topic] = {
                'fps': 10,
                'path': video_list[0],
            }

        with open(video_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(video_config, file)

        # 生成H265
        h265_config = {
            'h265': {}, 'start_time': 0.0, 'end_time': 0.0,
        }
        min_ts, max_ts = 9999999999.0, 0.0
        h265_config_path = os.path.join(image_folder, 'h265_config.yaml')
        for topic, info in video_config.items():
            fps = info['fps']
            video_path = info['path']

            # 获得h265文件夹
            h265_folder = os.path.join(image_folder, topic.replace('/', ''))
            if os.path.exists(h265_folder):
                shutil.rmtree(h265_folder)
            os.makedirs(h265_folder)
            t0 = time.time()
            cmd = 'cd {:s}; python3 Api_Mkv2h265.py -i {:s} -o {:s} --crf 26 --ratio 3'.format(
                os.path.join(project_path, 'Envs', 'Master', 'Interfaces'),
                video_path, h265_folder,
            )
            print(cmd)
            p = os.popen(cmd)
            p.read()
            h265_file_list = sorted(glob.glob(os.path.join(h265_folder, '*.h265')))

            # 获得时间戳文件
            time_str = os.path.basename(video_path).split('_')[1]
            parts = time_str.rsplit('-', 1)
            datetime_part = parts[0]
            millisecond_part = parts[1]
            dt = datetime.strptime(datetime_part, "%Y-%m-%d-%H-%M-%S")
            dt = dt.replace(microsecond=int(millisecond_part) * 1000)
            start_time = dt.timestamp()
            raw_timestamp_path = os.path.join(h265_folder, 'frame_timestamps.txt')
            timestamp_data = pd.read_csv(raw_timestamp_path, header=None,
                             names=['frame_index', 'xxxx', 'time_stamp'],
                             sep=' ', engine='python')[['frame_index', 'time_stamp']]
            timestamp_data['time_stamp'] += start_time
            timestamp_path = os.path.join(image_folder, f"{topic.replace('/', '')}.csv")
            timestamp_data.to_csv(timestamp_path, index=False)
            min_ts = min(min_ts, timestamp_data['time_stamp'].min())
            max_ts = max(max_ts, timestamp_data['time_stamp'].max())

            h265_config['h265'][topic] = {
                'timestamp_path': timestamp_path,
                'H265_path': h265_file_list,
            }
            print(f'{topic} 转h265 用时 {time.time() - t0} s')

        h265_config['start_time'] = float(min_ts)
        h265_config['end_time'] = float(max_ts)
        with open(h265_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(h265_config, file)

        return h265_config_path

    def qStf_to_h265(self, qcraft_package_path, h265_folder_path='/Q_DATA/H265'):
        """
        qcraft_package_path和h265_folder_path都是docker内的地址
        """

        # 如果不是Replay0Z机,不能运行
        if bench_id != 'Replay0Z':
            print('本机不是Q机,无法执行函数')
            return None

        password = bench_config['Master']['password']

        # 解析q文件为h265
        tmux_session = variables['tmux_node']['q_parse'][0]
        tmux_window = variables['tmux_node']['q_parse'][1]
        kill_tmux_session_if_exists(tmux_session)

        os.system(f'tmux new-session -s {tmux_session} -n {tmux_window} -d')
        time.sleep(0.1)

        # 打开docker
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "sudo docker start qcraft_formpackage" C-m')
        time.sleep(1)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "{password}" C-m')
        time.sleep(2)

        # 进入docker
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "sudo docker exec -it qcraft_formpackage bash" C-m')
        time.sleep(2)

        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "cd qcraft_msg_tool/" C-m')
        time.sleep(0.1)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "rm -r {h265_folder_path}" C-m')
        time.sleep(3)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "mkdir {h265_folder_path}" C-m')
        time.sleep(0.1)

        cmd = f"./script/run_info_export.sh --run={qcraft_package_path} --output_dir={h265_folder_path}"
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "{cmd}" C-m')
        time.sleep(1)
        cmd = f"./script/run_export_image_only.sh --run={qcraft_package_path} --output_dir={h265_folder_path} --output_text=true"
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "{cmd}" C-m')

        # 监控文件夹大小
        full_folder_path = self.q_docker_base_path + h265_folder_path
        count = 3
        while count:
            time.sleep(2)
            folder_size_1 = get_folder_size(full_folder_path)
            time.sleep(2)
            folder_size_2 = get_folder_size(full_folder_path)
            print(f'{os.path.basename(qcraft_package_path)} data size = {round(folder_size_2 / 1024 / 1024, 2)} MB')
            if folder_size_2 == folder_size_1:
                count -= 1
        print(f'{os.path.basename(qcraft_package_path)} transfer stops')
        kill_tmux_session_if_exists(tmux_session)

        # 相机标定文件
        config_json_path = os.path.join(full_folder_path, 'run_info.json')
        if not os.path.exists(config_json_path):
            config_json_path = None

        h265_config = {
            'h265': {}, 'start_time': 0, 'end_time': 0, 'h265_temp': full_folder_path
        }
        min_ts, max_ts = 9999999999, 0
        data_storage_folder = os.path.join('/home', os.getlogin(), 'ZONE', 'temp')
        if os.path.exists(data_storage_folder):
            shutil.rmtree(data_storage_folder)
        os.makedirs(data_storage_folder)
        h265_config_path = os.path.join(data_storage_folder, 'h265_config.yaml')
        timestamp_col = ['frame_index', 'time_stamp', 'frame_type']

        for q_camera_name in os.listdir(full_folder_path):
            if not os.path.isdir(os.path.join(full_folder_path, q_camera_name)):
                continue

            z_camera_name = camera2camera[q_camera_name]
            topic = camera2topic[z_camera_name]
            timestamp_path = os.path.join(data_storage_folder, f"{topic.replace('/', '')}.csv")
            rows = []
            index = 0
            h265_file_list = sorted(glob.glob(os.path.join(full_folder_path, q_camera_name, '*.h265')))
            for h265_file in h265_file_list:
                time_stamp = float(h265_file.split('-')[-1][:-5])
                rows.append([index, time_stamp, 'I'])
                index += 1
                min_ts = min(time_stamp, min_ts)
                max_ts = max(time_stamp, max_ts)

            pd.DataFrame(rows, columns=timestamp_col).to_csv(timestamp_path, index=False)
            h265_config['h265'][topic] = {
                'timestamp_path': timestamp_path,
                'H265_path': h265_file_list,
            }

        h265_config['start_time'] = min_ts
        h265_config['end_time'] = max_ts
        with open(h265_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(h265_config, file)

        return h265_config_path, config_json_path, full_folder_path

    def h265_to_db3(self, h265_config_path, db3_dir, delete_raw_h265=False):
        tmux_session = variables['tmux_node']['h265_gen'][0]
        tmux_window = variables['tmux_node']['h265_gen'][1]
        kill_tmux_session_if_exists(tmux_session)

        with open(h265_config_path, 'r', encoding='utf-8') as file:
            h265_config = yaml.safe_load(file)

        min_ts = h265_config['start_time']
        max_ts = h265_config['end_time']

        if max_ts - min_ts < 15:
            print('H265没有足够的数据，不执行转化.db3')
            return

        print(f'开始转化.db3, 时间长度为{round(max_ts - min_ts, 3)} seconds')
        min_ts = datetime.fromtimestamp(int(min_ts)).strftime("%Y_%m_%d-%H_%M_%S")
        max_ts = datetime.fromtimestamp(int(max_ts)).strftime("%Y_%m_%d-%H_%M_%S")
        ros2bag_path = os.path.join(db3_dir, f'{min_ts}={max_ts}')
        os.system(f'tmux new-session -s {tmux_session} -n {tmux_window} -d')
        time.sleep(0.1)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "bash {ros_docker_path}" C-m')
        time.sleep(3)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"source {self.install_path}/setup.bash" C-m')

        h265_rosnode_path = os.path.join(project_path, 'Envs', 'Master', 'Modules', 'RosNode')
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "cd {h265_rosnode_path}" C-m')
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"python3 H265ToRosbagConverter.py -f {h265_config_path} -r {ros2bag_path}" C-m')

        # 监控文件夹大小，并删除中间数据
        count = 3
        t0 = time.time()
        while count:
            folder_size_1 = get_folder_size(db3_dir)
            time.sleep(8)
            folder_size_2 = get_folder_size(db3_dir)
            print(f'{round(time.time() - t0, 3)} {os.path.basename(ros2bag_path)} size = {round(folder_size_2 / 1024 / 1024, 2)} MB')
            if folder_size_2 == folder_size_1:
                count -= 1
        print(f'{os.path.basename(ros2bag_path)} gen stops')
        kill_tmux_session_if_exists(tmux_session)

        with open(h265_config_path, 'r', encoding='utf-8') as file:
            h265_config = yaml.safe_load(file)

        if delete_raw_h265:
            if 'h265_temp' in h265_config:
                force_delete_folder(h265_config['h265_temp'])
                del h265_config['h265_temp']
                for topic in h265_config['h265']:
                    os.remove(h265_config['h265'][topic]['timestamp_path'])
            else:
                for topic in h265_config['h265']:
                    shutil.rmtree(h265_config['h265'][topic]['H265_path'])
                    os.remove(h265_config['h265'][topic]['timestamp_path'])

        return ros2bag_path

    def kunyiCan_to_db3(self, kunyi_package_path, install_path=None):
        def check_process():
            return any(
                proc.info['name'] == 'python3' and 'can_to_ros_bag.py' in ' '.join(proc.info['cmdline'] or [])
                for proc in psutil.process_iter(['name', 'cmdline'])
            )

        def wait_for_parse_end():
            while check_process():
                time.sleep(10)
            return True

        def convert_can_to_ros(dirpath, csv_path, mapping_path, arxml_path, install_path, ros_bag_path):
            print(f"文件路径: {dirpath}")
            ros2_bag_name = ros_bag_path
            asc_data_path = asc_parser(dirpath, csv_path, arxml_path)
            script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Can2Ros')
            os.system(
                f'{ros_docker_path}; sleep 2; cd {script_path}; python3 can_to_ros_bag_api.py -i {install_path} -m {mapping_path} -d {asc_data_path} -n {ros2_bag_name} -r {ros_docker_path}')
            time.sleep(2)
        if not install_path:
            install_path = self.install_path
        if not install_path:
            return
        else:
            parse_asc_data_path = os.path.join(kunyi_package_path, self.parse_asc_data_file_name)
            ros2bag_ins_path = os.path.join(kunyi_package_path, self.ros2bag_ins_name)
            can_file_path = os.path.join(kunyi_package_path, self.can_file_name)
            if os.path.exists(parse_asc_data_path):
                shutil.rmtree(parse_asc_data_path)
            if os.path.exists(ros2bag_ins_path):
                shutil.rmtree(ros2bag_ins_path)
            os.makedirs(parse_asc_data_path)
            kill_tmux_session_if_exists('write_ros2_bag')
            convert_can_to_ros(can_file_path, self.csv, self.mapping, self.arxml, install_path, ros2bag_ins_path)
            time.sleep(5)
            wait_for_parse_end()
            kill_tmux_session_if_exists('write_ros2_bag')
            if os.path.exists(parse_asc_data_path):
                shutil.rmtree(parse_asc_data_path)

    def combine_db3(self, input_bags:list, output_bag:str, install_path=None, start_time=-1, stop_time=1e10):
        if not install_path:
            install_path = self.install_path
        if not install_path:
            return
        else:
            tmux_session = 'combine_session'
            tmux_window = 'combine_session_windows'
            kill_tmux_session_if_exists(tmux_session)
            os.system(f'tmux new-session -s {tmux_session} -n {tmux_window} -d')
            time.sleep(0.1)

            os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "bash {ros_docker_path}" C-m')
            os.system('sleep 3')
            os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                      f'"source {install_path}/setup.bash" C-m')

            py_path = os.path.join(project_path, 'Envs', 'Master', 'Modules', 'Can2Ros')
            os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                      f'"cd {py_path}" C-m')
            os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                      f'"python3 CombineRosBag.py -i {" ".join(input_bags)} -o {output_bag} -t {start_time} -p {stop_time}" C-m')
            # 判断合并是否完成
            while True:
                time.sleep(1)
                running = any(
                    proc.info['name'] == 'python3' and 'CombineRosBag.py' in ' '.join(proc.info['cmdline'] or [])
                    for proc in psutil.process_iter(['name', 'cmdline'])
                )
                if not running:
                    kill_tmux_session_if_exists(tmux_session)
                    break

    def combine_Kunyi_db3(self, kunyi_package_path, install_path=None):
        if not install_path:
            install_path = self.install_path
        if not install_path:
            return
        else:
            ros2bag_ins_path = os.path.join(kunyi_package_path, self.ros2bag_ins_name)
            ros2bag_h265_path = os.path.join(kunyi_package_path, self.ros2bag_h265_name)
            folder_names = get_all_folder_in_target_dir(ros2bag_h265_path)
            if len(folder_names) == 1:
                ros2bag_h265_path = folder_names[0]
            ros2bag_combine_path = os.path.join(kunyi_package_path, self.ros2bag_combine_name)
            input_bags = [ros2bag_h265_path, ros2bag_ins_path]
            output_bag = ros2bag_combine_path
            self.combine_db3(input_bags, output_bag, install_path)
            if os.path.exists(ros2bag_combine_path):
                # 删除两个rosbag包
                if os.path.exists(ros2bag_ins_path):
                    shutil.rmtree(ros2bag_ins_path)
                if len(folder_names) == 1:
                    if os.path.exists(ros2bag_h265_path):
                        shutil.rmtree(os.path.dirname(ros2bag_h265_path))
                else:
                    if os.path.exists(ros2bag_h265_path):
                        shutil.rmtree(ros2bag_h265_path)
            else:
                print("失败")

    def gen_AVM_from_db3(self, kunyi_package_path, config_path, install_path=None):
        if not install_path:
            install_path = self.install_path
        if not install_path:
            return
        else:
            bev_object = Ros2Bag2BirdView(install_path, kunyi_package_path, config_path)
            bev_object.extract_h265_raw_streams()
            bev_object.extract_frames_from_h265()

    def batch_generate_rosbags(self, kunyi_package_folder, install_path=None):
        """
        依次读取文件夹kunyi_package_folder内的kunyi_package文件夹，生成rosbag

        参数:
        kunyi_package_folder: 文件夹内包含多个kunyi_package数据包
        install_path： install路径

        """
        if not install_path:
            install_path = self.install_path
        if not install_path:
            return
        else:
            kunyi_packages = glob.glob(f"{kunyi_package_folder}/*")
            kunyi_packages.sort()
            for kunyi_package in kunyi_packages:
                folders_in_kunyi_package = glob.glob(f"{kunyi_package}/*")
                if os.path.join(kunyi_package, 'Config') in folders_in_kunyi_package and os.path.join(kunyi_package, self.can_file_name) in folders_in_kunyi_package and os.path.join(kunyi_package, self.images_file_name) in folders_in_kunyi_package:
                    h265_config_path = self.kunyiMkv_to_h265(kunyi_package)
                    self.h265_to_db3(h265_config_path, os.path.join(kunyi_package, self.ros2bag_h265_name))
                    self.kunyiCan_to_db3(kunyi_package, install_path)
                    self.combine_Kunyi_db3(kunyi_package)
                    self.gen_AVM_from_db3(kunyi_package, os.path.join(kunyi_package, 'Config'), install_path)
                    print(f"{kunyi_package}处理完成, 删除原始文件")
                    if os.path.exists(os.path.join(kunyi_package, self.can_file_name)):
                        shutil.rmtree(os.path.join(kunyi_package, self.can_file_name))
                    if os.path.exists(os.path.join(kunyi_package, self.images_file_name)):
                        shutil.rmtree(os.path.join(kunyi_package, self.images_file_name))
                else:
                    print(f"{kunyi_package}不包含指定文件夹")


class DataLoggerAnalysis:

    def __init__(self, folder):
        self.folder = os.path.join(folder, 'rosbag')

        # 解压缩
        install_zip = os.path.join(self.folder, 'install', 'install.tar.gz')
        if not os.path.exists(install_zip):
            print(f'未找到{install_zip}')
            return

        self.ros2bag_info = []
        for ros2bag_folder in glob.glob(os.path.join(self.folder, 'rosbag', 'rosbag2*')):
            t0 = datetime.strptime(os.path.basename(ros2bag_folder)[8:], "%Y_%m_%d-%H_%M_%S").timestamp()

            for ros2bag_zip in sorted(glob.glob(os.path.join(ros2bag_folder, '*.tar.gz'))):
                t1 = datetime.strptime(os.path.basename(ros2bag_zip).split('.')[0], "%Y_%m_%d-%H_%M_%S").timestamp()
                if t1 - t0 < 30:
                    continue

                self.ros2bag_info.append(
                    {
                        'start_time': t0, 'end_time': t1, 'path': ros2bag_zip
                    }
                )
                t0 = t1

    def load_data(self, target_folder='/media/data/Q_DATA'):
        cmd = 'cd {:s}; tar -xzvf install.tar.gz'.format(
            os.path.join(self.folder, 'install'),
        )
        p = os.popen(cmd)
        p.read()

        target_install = os.path.join(target_folder, 'install')
        if os.path.exists(target_install):
            shutil.rmtree(target_install)

        shutil.move(os.path.join(self.folder, 'install', 'install'), target_install)

        json_str = json.dumps(self.ros2bag_info, indent=2, sort_keys=False)
        ros2bag_info_path = os.path.join(target_folder, 'ros2bag_info.json')
        with open(ros2bag_info_path, 'w', encoding='utf-8') as f:
            f.write(json_str)

        return target_install, ros2bag_info_path


class DataDownloader:

    def __init__(self):
        pass

    def read_download_info(self, info_path):
        """
        从excel中读取所有文件在amazon云盘的路径为一个列表，并返回

        参数:
        info_path: excel的路径

        返回：
        download_info： 由多个字典组成的列表，字典为{'package_name': package_name,
                                  'vin': vin,
                                  'date': date,
                                  'video_serial': video_serial}
        """
        df = pd.read_excel(info_path)
        vin = '3D_data_LSJWK4095NS119733'
        download_info = []
        for index, row in df.iterrows():
            package_name = row['包名']
            date = row['批次']
            video_serial = row['视频序列']
            download_info.append({'package_name': package_name,
                                  'vin': vin,
                                  'date': date,
                                  'video_serial': video_serial})
        return download_info

    def batch_data_download_from_amazon(self, info_path, local_download_path='/home/hp/temp'):
        """
        从excel中获取所有文件在amazon云盘的路径，并批量下载至本地

        参数:
        info_path: excel的路径
        local_download_path: 下载至本地的路径
        """
        download_info = self.read_download_info(info_path)
        for info in download_info:
            s3_path = f"backup/data/collect/self/driving/{info['package_name']}/{info['vin']}/{info['date']}/"
            src_folder = os.path.join(local_download_path, 'download')
            dst_folder = os.path.join(local_download_path,  f"{info['date']}_{info['video_serial']}")
            result = self.data_download_from_amazon(s3_path, src_folder, info['video_serial'])
            if not result:
                return
            if os.path.exists(dst_folder):
                shutil.rmtree(dst_folder)
            os.makedirs(dst_folder)
            src_folders = glob.glob(os.path.join(src_folder, '*'))
            for folder in src_folders:
                shutil.move(folder, os.path.join(local_download_path, f"{info['date']}_{info['video_serial']}"))
            dst_folders = glob.glob(os.path.join(dst_folder, '*'))
            if len(src_folders) == len(dst_folders) and len(glob.glob(os.path.join(src_folder, '*'))) == 0:
                print(f"{s3_path}内的{info['video_serial']}已下载至{dst_folder}")
            else:
                print(f"文件移动至失败{dst_folder}")
                return

    def data_download_from_amazon(self, s3_path, local_download_path, serial_num=None):
        """
        从s3_path中下载文件至本地

        参数:
        s3_path: 云端路径
        local_download_path: 下载至本地的路径
        serial_num： 使用serial_num可以对路径中的文件做筛选，只下载对应序列号的文件
        """
        # 保证本地磁盘空间充足
        # 获取磁盘使用情况
        disk_usage = psutil.disk_usage('/home/hp/temp')

        # 转换为 GB
        total_gb = disk_usage.total / (1024 ** 3)
        free_gb = disk_usage.free / (1024 ** 3)
        used_gb = disk_usage.used / (1024 ** 3)
        if free_gb <= 30:
            print(f"剩余磁盘空间不足30G")
            return False
        if not os.path.exists(local_download_path):
            os.makedirs(local_download_path)
        endpoint_url = 'http://10.192.53.221:8080'  # 你的S3 endpoint
        aws_access_key_id = 'QB1YGVNUKJP2MRK8AK2R' # 替换为你的Access Key
        aws_secret_access_key = 'JxRde3bPdoxWaBBFwmmqH81ytiNIoTILh9CGCYJH'  # 替换为你的Secret Key
        bucket_name = 'prod-ac-dmp'
        tmux_session = 'download_session'
        tmux_window = 'download_session_windows'
        kill_tmux_session_if_exists(tmux_session)
        os.system(f'tmux new-session -s {tmux_session} -n {tmux_window} -d')
        time.sleep(0.1)

        py_path = os.path.join(project_path, 'Envs', 'Master', 'Interfaces')
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"cd {py_path}" C-m')
        if not serial_num:
            os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                      f'"python3 Api_DownloadS3.py -u {endpoint_url} -k {aws_access_key_id} -s {aws_secret_access_key} -n {bucket_name} -p {s3_path} -f {local_download_path} -x .pcap CAM_FRONT_30 CAM_BACK_LEFT CAM_BACK_RIGHT CAM_FRONT_LEFT CAM_FRONT_RIGHT" C-m')
        else:
            os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                      f'"python3 Api_DownloadS3.py -u {endpoint_url} -k {aws_access_key_id} -s {aws_secret_access_key} -n {bucket_name} -p {s3_path} -f {local_download_path} -i Config {serial_num} -x .pcap CAM_FRONT_30 CAM_BACK_LEFT CAM_BACK_RIGHT CAM_FRONT_LEFT CAM_FRONT_RIGHT" C-m')
        # 判断合并是否完成
        while True:
            time.sleep(1)
            running = any(
                proc.info['name'] == 'python3' and 'Api_DownloadS3.py' in ' '.join(proc.info['cmdline'] or [])
                for proc in psutil.process_iter(['name', 'cmdline'])
            )
            if not running:
                kill_tmux_session_if_exists(tmux_session)
                break
        return True

    def convert_folder_tree(self, src_dir, dst_dir):
        """
        转换下载的文件目录为处理需要的文件目录

        参数:
        src_dir -- 源文件目录
        dst_dir -- 转换后的文件目录保存的地址
        """
        reverse_data_folder(src_dir, dst_dir)


if __name__ == '__main__':
    t0 = time.time()
    ddd = DataDownloader()
    install_path = '/home/vcar/ZONE/manual_test_0709_5094/03_Workspace/install'
    qqq = DataTransformer(install_path=install_path)
    kunyi_package_path = '/media/data/kunyi_driving_data/20240118_133925_n000002'
    config_path = '/media/data/kunyi_driving_data/20240118_133925_n000002/Config'
    # h265_config_path = qqq.kunyiMkv_to_h265(kunyi_package_path)
    # qqq.h265_to_db3(h265_config_path, os.path.join(kunyi_package_path, qqq.ros2bag_h265_name))
    # qqq.kunyiCan_to_db3(kunyi_package_path)
    # qqq.combine_Kunyi_db3(kunyi_package_path)
    qqq.gen_AVM_from_db3(kunyi_package_path, config_path)
    print(time.time() - t0)

    # info_path = '/home/hp/temp/77w数据汇总.xlsx'
    # local_download_path = '/home/hp/temp'
    # kunyi_package_path = '/home/zhangliwei01/ZONE/20250529_102333_n000001'
    # qqq.combine_Kunyi_db3(kunyi_package_path)
    # h265_config_path = qqq.kunyiMkv_to_h265(kunyi_package_path)
    #
    # qqq.h265_to_db3(
    #     h265_config_path = '/home/zhangliwei01/ZONE/20250529_102333_n000001/Images/h265_config.yaml',
    #     db3_dir='/home/zhangliwei01/ZONE/20250529_102333_n000001/ROSBAG/ROSBAG_H265',
    # )
    # download_info = ddd.read_download_info('/home/hp/temp/77w数据汇总.xlsx')
    # for info in download_info:
    #     s3_path = f"backup/data/collect/self/driving/{info['package_name']}/{info['vin']}/{info['date']}/"
    #     src_folder = os.path.join(local_download_path, 'download')
    #     dst_folder = os.path.join(local_download_path, f"{info['date']}_{info['video_serial']}")
    #     result = ddd.data_download_from_amazon(s3_path, src_folder, info['video_serial'])
    #     if not result:
    #         break
    #     if os.path.exists(dst_folder):
    #         shutil.rmtree(dst_folder)
    #     os.makedirs(dst_folder)
    #     src_folders = glob.glob(os.path.join(src_folder, '*'))
    #     for folder in src_folders:
    #         shutil.move(folder, os.path.join(local_download_path, f"{info['date']}_{info['video_serial']}"))
    #     dst_folders = glob.glob(os.path.join(dst_folder, '*'))
    #     if len(src_folders) == len(dst_folders) and len(glob.glob(os.path.join(src_folder, '*'))) == 0:
    #         print(f"{s3_path}内的{info['video_serial']}已下载至{dst_folder}")
    #     else:
    #         print(f"文件移动至失败{dst_folder}")
    #         break
    #     folders_in_kunyi_package = glob.glob(f"{dst_folder}/*")
    #     if os.path.join(dst_folder, 'Config') in folders_in_kunyi_package and os.path.join(dst_folder,
    #                                                                                           qqq.can_file_name) in folders_in_kunyi_package and os.path.join(
    #             dst_folder, qqq.images_file_name) in folders_in_kunyi_package:
    #         h265_config_path = qqq.kunyiMkv_to_h265(dst_folder)
    #         # h265_config_path = '/home/hp/temp/20250529_102834_n000020/Images/h265_config.yaml'
    #         qqq.h265_to_db3(h265_config_path, os.path.join(dst_folder, qqq.ros2bag_h265_name))
    #         qqq.kunyiCan_to_db3(dst_folder, install_path)
    #         qqq.combine_Kunyi_db3(dst_folder)
    #         qqq.gen_AVM_from_db3(dst_folder, install_path)
    #         print(f"{dst_folder}处理完成, 删除原始文件")
    #         if os.path.exists(os.path.join(dst_folder, qqq.can_file_name)):
    #             shutil.rmtree(os.path.join(dst_folder, qqq.can_file_name))
    #         if os.path.exists(os.path.join(dst_folder, qqq.images_file_name)):
    #             shutil.rmtree(os.path.join(dst_folder, qqq.images_file_name))
    #     else:
    #         print(f"{dst_folder}不包含指定文件夹")


    # ddd.batch_data_download_from_amazon('/home/hp/temp/77w数据汇总.xlsx')
    # ddd.convert_folder_tree('/home/hp/temp/20250529', '/home/hp/temp')
    # install_path = '/home/hp/artifacts/ZPD_AH4EM/3.3.0_RC1/install'
    # q_docker_base_path = f'/media/data'
    #
    # qqq = DataTransformer(install_path=install_path)
    # kunyi_package_path = '/home/hp/temp'
    # qqq.batch_generate_rosbags(kunyi_package_path, install_path)
    # h265_config_path = '/home/zhangliwei01/ZONE/manual_scenario/20240119_145625_n000001/Images/h265_config.yaml'
    # qqq.h265_to_db3(h265_config_path, os.path.join(kunyi_package_path, qqq.ros2bag_h265_name))
    # qqq.kunyiCan_to_db3(install_path, kunyi_package_path)
    # qqq.combine_Kunyi_db3(kunyi_package_path)

    # qcraft_package_path = '/Q_DATA/debug_data/20250614_105002_Q3402'
    # h265_folder_path = '/Q_DATA/result'
    # h265_config_path = qqq.qStf_to_h265(qcraft_package_path, h265_folder_path)
    # h265_config_path = f'/home/{os.getlogin()}/ZONE/temp_data/h265_config.yaml'
    # qqq.h265_to_db3(h265_config_path, '/media/data/Q_DATA/ros2bag')

    # dla = DataLoggerAnalysis('/media/data/Q_DATA/debug_data/20250614-005-AEB')
    # target_install, ros2bag_info_path = dla.load_install()
    # print(target_install, ros2bag_info_path)