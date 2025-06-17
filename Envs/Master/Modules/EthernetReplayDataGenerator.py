"""  
@Author: BU YUJUN
@Date: 2025/5/26 16:21  
"""
import glob
import os
import shutil
import sys
import time

import psutil
import yaml
from datetime import datetime

from Envs.Master.Modules.Can2Ros.arxml_asc_parser import asc_parser

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from Libs import get_project_path
sys.path.append(get_project_path())
from Utils.VideoProcess import convert_video_h265, gen_h265_timestamp, normalize_h265_startcodes
from Utils.Libs import ros_docker_path, variables, kill_tmux_session_if_exists, project_path, get_folder_size
from Utils.Libs import topic2camera


class EthernetReplayDataGenerator:

    def __init__(self, file_path, install_path):
        self.install_path = install_path
        self.file_path = file_path
        self.can_file_path = os.path.join(self.file_path, 'CAN_Trace')
        self.ros2bag_path_h265 = os.path.join(self.file_path, 'ROSBAG', 'ROSBAG_H265')
        self.ros2bag_path_ins = os.path.join(self.file_path, 'ROSBAG', 'ROSBAG_INS')
        self.ros2bag_path_combine = os.path.join(self.file_path, 'ROSBAG', 'COMBINE')
        self.csv = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', 'CAN_Signal.csv')
        self.mapping = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', 'CAN2ROS_ES37.csv')
        self.arxml = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', '20240315-cgy-ES37_IPD_V6.0.arxml')
        self.config_path = os.path.join(self.file_path, 'Images', 'Config.yaml')
        self.video_config = self.gen_config_file()
        self.parse_asc_data_path = os.path.join(self.file_path, 'ASCParseData')
        self.h265_config_path = os.path.join(os.path.dirname(self.config_path), 'H265.yaml')
        self.h265_config = {
            topic: {
                'timestamp_path': '0',
                'H265_path': '0',
            } for topic in self.video_config.keys()
        }

        self.tmux_session = variables['tmux_node']['h265_gen'][0]
        self.tmux_window = variables['tmux_node']['h265_gen'][1]

    def gen_config_file(self):
        config = {}
        for topic, camera_name in topic2camera.items():
            video_list = glob.glob(os.path.join(self.file_path, 'Images', f'{camera_name}', f'*{camera_name}.mkv'))
            if not len(video_list):
                print(f'No video file found for {camera_name} in {self.file_path}')
                return None
            if len(video_list) > 1:
                print(f'Multiple video files found for {camera_name} in {self.file_path}')
                return None

            config[topic] = {
                'fps': 10,
                'path': video_list[0],
            }

        with open(self.config_path, 'w', encoding='utf-8') as file:
            yaml.dump(config, file)

        return config

    def transfer_h265(self):
        for topic, info in self.video_config.items():
            fps = info['fps']
            video_path = info['path']
            h265_path = os.path.join(os.path.dirname(video_path), f"{topic.replace('/', '')}.h265")
            timestamp_path = os.path.join(os.path.dirname(video_path), f"{topic.replace('/', '')}.csv")
            normalized_h265_path = os.path.join(os.path.dirname(video_path), f"{topic.replace('/', '')}_norm.h265")
            self.h265_config[topic]['timestamp_path'] = timestamp_path
            self.h265_config[topic]['H265_path'] = normalized_h265_path
            self.h265_config[topic]['video_path'] = video_path

            time_str = os.path.basename(video_path).split('_')[1]
            parts = time_str.rsplit('-', 1)
            datetime_part = parts[0]
            millisecond_part = parts[1]
            dt = datetime.strptime(datetime_part, "%Y-%m-%d-%H-%M-%S")
            dt = dt.replace(microsecond=int(millisecond_part) * 1000)
            start_time = dt.timestamp()

            convert_video_h265(video_path, fps, h265_path)
            gen_h265_timestamp(h265_path, start_time, timestamp_path)
            normalize_h265_startcodes(h265_path, normalized_h265_path)
            os.remove(h265_path)

        with open(self.h265_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(self.h265_config, file, sort_keys=False, indent=2, allow_unicode=True)

    def gen_h265_db3(self):
        kill_tmux_session_if_exists(self.tmux_session)
        os.system(f'tmux new-session -s {self.tmux_session} -n {self.tmux_window} -d')
        time.sleep(0.1)

        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "bash {ros_docker_path}" C-m')
        os.system('sleep 3')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"source {self.install_path}/setup.bash" C-m')

        h265_rosnode_path = os.path.join(project_path, 'Envs', 'Master', 'Modules', 'RosNode')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"cd {h265_rosnode_path}" C-m')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"python3 H265ToRosbagConverter.py -f {self.h265_config_path} -r {self.ros2bag_path_h265}" C-m')

    def stop(self):
        count = 3
        while count:
            time.sleep(3)
            folder_size_1 = get_folder_size(self.ros2bag_path_h265)
            time.sleep(3)
            folder_size_2 = get_folder_size(self.ros2bag_path_h265)
            print(f'ros2bag size == {round(folder_size_2 / 1024 / 1024, 2)} MB')
            if folder_size_2 == folder_size_1:
                count -= 1
        print(f'ros2bag gen stops')
        kill_tmux_session_if_exists(self.tmux_session)

        with open(self.h265_config_path, 'r', encoding='utf-8') as file:
            h265_config = yaml.safe_load(file)

        # for topic in h265_config:
        #     os.remove(h265_config[topic]['H265_path'])
        #     os.remove(h265_config[topic]['timestamp_path'])

        # os.remove(self.h265_config_path)

    def can_to_ros_bag(self):
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
            # asc_data_path = '/home/hp/ZHX/read_can_signal/parse_asc_data/CAN_Trace_1.csv'
            os.system(
                f'cd {script_path}; python3 can_to_ros_bag_api.py -i {install_path} -m {mapping_path} -d {asc_data_path} -n {ros2_bag_name}')
            time.sleep(2)
        if os.path.exists(self.parse_asc_data_path):
            shutil.rmtree(self.parse_asc_data_path)
        if os.path.exists(self.ros2bag_path_ins):
            shutil.rmtree(self.ros2bag_path_ins)
        os.makedirs(self.parse_asc_data_path)
        kill_tmux_session_if_exists('write_ros2_bag')
        convert_can_to_ros(self.can_file_path, self.csv, self.mapping, self.arxml, self.install_path, self.ros2bag_path_ins)
        time.sleep(5)
        wait_for_parse_end()
        kill_tmux_session_if_exists('write_ros2_bag')
        if os.path.exists(self.parse_asc_data_path):
            shutil.rmtree(self.parse_asc_data_path)
        # os.makedirs(self.parse_asc_data_path)


    def combine(self):
        input_bags = [self.ros2bag_path_h265, self.ros2bag_path_ins]
        output_bag = self.ros2bag_path_combine
        tmux_session = 'combine_session'
        tmux_window = 'combine_session_windows'
        kill_tmux_session_if_exists(tmux_session)
        os.system(f'tmux new-session -s {tmux_session} -n {tmux_window} -d')
        time.sleep(0.1)

        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "bash {ros_docker_path}" C-m')
        os.system('sleep 3')
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"source {self.install_path}/setup.bash" C-m')

        combine_path = os.path.join(project_path, 'Envs', 'Master', 'Modules', 'Can2Ros')
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"cd {combine_path}" C-m')
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"python3 CombineRosBag.py -i {" ".join(input_bags)} -o {output_bag}" C-m')
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
        # 删除两个rosbag包
        if os.path.exists(self.ros2bag_path_ins):
            shutil.rmtree(self.ros2bag_path_ins)
        if os.path.exists(self.ros2bag_path_h265):
            shutil.rmtree(self.ros2bag_path_h265)


if __name__ == '__main__':
    t0 = time.time()
    file_name = '/home/hp/temp/20250324_144918_n000003'
    install_path = '/home/hp/artifacts/ZPD_EP39/4.3.0_RC11/install'
    ee = EthernetReplayDataGenerator(file_name, install_path)
    ee.transfer_h265()
    ee.gen_h265_db3()
    ee.stop()
    ee.can_to_ros_bag()
    ee.combine()
    # print(time.time() - t0)