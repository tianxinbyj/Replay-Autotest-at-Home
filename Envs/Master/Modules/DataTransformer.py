"""
@Author: BU YUJUN, ZHENG HUIXIN
@Date: 2025/6/18 上午9:45
"""
import glob
import os
import re
import subprocess
import sys
import time
from datetime import datetime

import pandas as pd
import yaml

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from Libs import get_project_path
sys.path.append(get_project_path())
from Utils.VideoProcess import normalize_h265_startcodes
from Utils.Libs import ros_docker_path, variables, kill_tmux_session_if_exists, project_path, get_folder_size
from Utils.Libs import topic2camera


class DataTransformer:

    def __init__(self, install_path=None):
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
                'H265_path': h265_path,
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

    def kunyiCan_to_db3(self, kunyi_package_path, db3_path):
        can_file_path = os.path.join(kunyi_package_path, 'CAN_Trace')
        if not os.path.exists(can_file_path):
            print(f'{kunyi_package_path} <UNK>')
            return None

    def combine_db3(self, db3_list, combine_db3_path):
        pass

    def gen_AVM_from_db3(self, db3_path, avm_folder):
        pass


if __name__ == '__main__':
    install_path = '/home/zhangliwei01/ZONE/TestProject/EP39/EP39_AD_4.3.0_RC24_ENG/03_Workspace/install'
    qqq = DataTransformer(install_path)
    kunyi_package_path = '/home/zhangliwei01/ZONE/manual_scenario/20240119_145625_n000001'
    h265_config_path = qqq.kunyiMkv_to_h265(kunyi_package_path)
    qqq.h265_to_db3(h265_config_path, '/home/zhangliwei01/ZONE/manual_scenario/temp')