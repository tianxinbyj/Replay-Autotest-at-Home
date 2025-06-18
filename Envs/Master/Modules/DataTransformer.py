"""
@Author: BU YUJUN, ZHENG HUIXIN
@Date: 2025/6/18 上午9:45
"""
import glob
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

from Envs.Master.Modules.Can2Ros.arxml_asc_parser import asc_parser
from Envs.Master.Modules.Ros2Bag2BirdView import Ros2Bag2BirdView

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from Libs import get_project_path
sys.path.append(get_project_path())
from Utils.VideoProcess import normalize_h265_startcodes
from Utils.Libs import ros_docker_path, variables, kill_tmux_session_if_exists, project_path, get_folder_size
from Utils.Libs import topic2camera


class DataTransformer:

    def __init__(self, install_path=None):
        self.parse_asc_data_file_name = 'ASCParseData'
        self.ros2bag_ins_name = os.path.join('ROSBAG', 'ROSBAG_INS')
        self.ros2bag_h265_name = os.path.join('ROSBAG', 'ROSBAG_H265')
        self.ros2bag_combine_name = os.path.join('ROSBAG', 'COMBINE')
        self.can_file_name = 'CAN_Trace'
        self.csv = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', 'CAN_Signal.csv')
        self.mapping = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', 'CAN2ROS_ES37.csv')
        self.arxml = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', '20240315-cgy-ES37_IPD_V6.0.arxml')
        self.install_path = install_path
        self.tmux_session = variables['tmux_node']['h265_gen'][0]
        self.tmux_window = variables['tmux_node']['h265_gen'][1]

    def kunyiMkv_to_h265(self, kunyi_package_path):

        def convert_video_h265(video_path, target_fps, h265_path=None):
            if not h265_path:
                h265_path = os.path.join(project_path, 'Temp', f'{os.path.basename(video_path).split(".")[0]}.h265')

            video_convert_command = [
                'ffmpeg',
                '-i', video_path,  # 输入文件
                '-c:v', 'libx265',  # 使用H.265编码
                '-crf', '22',
                '-pix_fmt', 'yuv420p',  # 像素格式为yuv420p
                '-r', f'{target_fps}',  # 输出帧率15Hz
                '-x265-params', f'keyint={target_fps}:min-keyint={target_fps}:bframes=0',  # 每秒一个I帧，无B帧
                '-vcodec', 'hevc',  # 视频编码器
                '-an',  # 去除音频
                '-f', 'hevc',  # 强制输出格式为HEVC
                '-bsf:v', 'hevc_mp4toannexb',  # 比特流过滤器，确保NAL单元分隔符正确
                '-y',  # 覆盖输出文件
                h265_path
            ]

            try:
                subprocess.run(video_convert_command, check=True)
                print(f"{video_path} conversion completed successfully.")
                return h265_path
            except subprocess.CalledProcessError as e:
                print(f"{video_path} conversion failed: {e}")
                return None

        def gen_h265_timestamp(h265_path, start_time, timestamp_path=None):
            if not timestamp_path:
                timestamp_path = os.path.join(project_path, 'Temp', f'{os.path.basename(h265_path).split(".")[0]}.csv')

            cmd = [
                "ffmpeg",
                "-i", h265_path,
                "-vf", "showinfo",
                "-f", "null",
                "-"
            ]
            process = subprocess.Popen(
                cmd,
                stderr=subprocess.PIPE,  # FFmpeg 输出到 stderr
                universal_newlines=True  # 确保文本模式
            )

            # 优化后的正则表达式（精确匹配实际输出格式）
            pattern = re.compile(
                r"n:\s*(\d+).*?pts_time:([0-9.e+-]+).*?type:([IBP])\b"
            )

            col = ['frame_index', 'time_stamp', 'frame_type']
            rows = []
            for line in process.stderr:
                match = pattern.search(line)
                if match:
                    frame_index = int(match.group(1))
                    pts_time = float(match.group(2)) + float(start_time)
                    frame_type = match.group(3)
                    rows.append([frame_index, pts_time, frame_type])

            pd.DataFrame(rows, columns=col).to_csv(timestamp_path, index=False)
            return timestamp_path

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

        # 生成H265
        h265_config = {}
        h265_config_path = os.path.join(image_folder, 'h265_config.yaml')
        for topic, info in video_config.items():
            fps = info['fps']
            video_path = info['path']
            h265_path = os.path.join(image_folder, f"{topic.replace('/', '')}.h265")
            timestamp_path = os.path.join(image_folder, f"{topic.replace('/', '')}.csv")
            normalized_h265_path = os.path.join(image_folder, f"{topic.replace('/', '')}_norm.h265")
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

            h265_config[topic] = {
                'timestamp_path': timestamp_path,
                'H265_path': normalized_h265_path,
            }

        with open(video_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(video_config, file)
        with open(h265_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(h265_config, file)

        return h265_config_path

    def qStf_to_h265(self, qcaft_package_path):
        pass

    def h265_to_db3(self, h265_config_path, db3_path):
        kill_tmux_session_if_exists(self.tmux_session)
        os.system(f'tmux new-session -s {self.tmux_session} -n {self.tmux_window} -d')
        time.sleep(0.1)

        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "bash {ros_docker_path}" C-m')
        time.sleep(3)
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"source {self.install_path}/setup.bash" C-m')

        h265_rosnode_path = os.path.join(project_path, 'Envs', 'Master', 'Modules', 'RosNode')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"cd {h265_rosnode_path}" C-m')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"python3 H265ToRosbagConverter.py -f {h265_config_path} -r {db3_path}" C-m')

        # 监控文件夹大小，并删除中间数据
        count = 3
        while count:
            time.sleep(3)
            folder_size_1 = get_folder_size(db3_path)
            time.sleep(3)
            folder_size_2 = get_folder_size(db3_path)
            print(f'ros2bag size == {round(folder_size_2 / 1024 / 1024, 2)} MB')
            if folder_size_2 == folder_size_1:
                count -= 1
        print(f'ros2bag gen stops')
        kill_tmux_session_if_exists(self.tmux_session)

        with open(h265_config_path, 'r', encoding='utf-8') as file:
            h265_config = yaml.safe_load(file)

        for topic in h265_config:
            os.remove(h265_config[topic]['H265_path'])
            os.remove(h265_config[topic]['timestamp_path'])

    def kunyiCan_to_db3(self, install_path, kunyi_package_path):
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
                f'cd {script_path}; python3 can_to_ros_bag_api.py -i {install_path} -m {mapping_path} -d {asc_data_path} -n {ros2_bag_name}')
            time.sleep(2)
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

    def combine_db3(self, kunyi_package_path):
        ros2bag_ins_path = os.path.join(kunyi_package_path, self.ros2bag_ins_name)
        ros2bag_h265_path = os.path.join(kunyi_package_path, self.ros2bag_h265_name)
        ros2bag_combine_path = os.path.join(kunyi_package_path, self.ros2bag_combine_name)
        input_bags = [ros2bag_h265_path, ros2bag_ins_path]
        output_bag = ros2bag_combine_path
        tmux_session = 'combine_session'
        tmux_window = 'combine_session_windows'
        kill_tmux_session_if_exists(tmux_session)
        os.system(f'tmux new-session -s {tmux_session} -n {tmux_window} -d')
        time.sleep(0.1)

        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "bash {ros_docker_path}" C-m')
        os.system('sleep 3')
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"source {self.install_path}/setup.bash" C-m')

        py_path = os.path.join(project_path, 'Envs', 'Master', 'Modules', 'Can2Ros')
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"cd {py_path}" C-m')
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
        if os.path.exists(ros2bag_ins_path):
            shutil.rmtree(ros2bag_ins_path)
        if os.path.exists(ros2bag_h265_path):
            shutil.rmtree(ros2bag_h265_path)

    def gen_AVM_from_db3(self, install_path, kunyi_package_path):
        bev_object = Ros2Bag2BirdView(install_path, kunyi_package_path)
        bev_object.extract_h265_raw_streams()
        bev_object.extract_frames_from_h265()



if __name__ == '__main__':
    install_path = '/home/hp/artifacts/ZPD_EP39/4.3.0_RC11/install'
    qqq = DataTransformer()
    kunyi_package_path = '/home/hp/temp/20240123_145155_n000003'
    # h265_config_path = qqq.kunyiMkv_to_h265(kunyi_package_path)
    # db3_path = os.path.join(kunyi_package_path, qqq.ros2bag_h265_name)
    # qqq.h265_to_db3(h265_config_path, db3_path)
    # qqq.kunyiCan_to_db3(install_path, kunyi_package_path)
    # qqq.combine_db3(kunyi_package_path)
    qqq.gen_AVM_from_db3(install_path, kunyi_package_path)