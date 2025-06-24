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

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from Libs import get_project_path
sys.path.append(get_project_path())
from Envs.Master.Modules.Can2Ros.arxml_asc_parser import asc_parser
from Envs.Master.Modules.Ros2Bag2BirdView import Ros2Bag2BirdView
from Utils.VideoProcess import normalize_h265_startcodes
from Utils.Libs import ros_docker_path, variables, kill_tmux_session_if_exists, project_path, get_folder_size
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

    def __init__(self, install_path=None, q_docker_base_path=None):
        self.parse_asc_data_file_name = 'ASCParseData'
        self.ros2bag_ins_name = os.path.join('ROSBAG', 'ROSBAG_INS')
        self.ros2bag_h265_name = os.path.join('ROSBAG', 'ROSBAG_H265')
        self.ros2bag_combine_name = os.path.join('ROSBAG', 'COMBINE')
        self.can_file_name = 'CAN_Trace'
        self.csv = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', 'CAN_Signal.csv')
        self.mapping = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', 'CAN2ROS_ES37.csv')
        self.arxml = os.path.join(get_project_path(), 'Envs', 'Master', 'Modules', 'Can2Ros', 'config', '20240315-cgy-ES37_IPD_V6.0.arxml')
        self.install_path = install_path
        self.q_docker_base_path = q_docker_base_path

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

        with open(video_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(video_config, file)

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

        with open(h265_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(h265_config, file)

        return h265_config_path

    def qStf_to_h265(self, qcraft_package_path, h265_folder_path):
        """
        qcraft_package_path和h265_folder_path都是docker内的地址
        """

        # 如果不是Replay0Z机,不能运行
        if bench_id != 'Replay0Z':
            print('本机不是Q机,无法执行函数')
            return None

        password = bench_config['Master']['password']

        # 解析q文件为h265
        # '''
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
        time.sleep(1)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "mkdir {h265_folder_path}" C-m')
        time.sleep(0.1)

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
            print(f'camera data size == {round(folder_size_2 / 1024 / 1024, 2)} MB')
            if folder_size_2 == folder_size_1:
                count -= 1
        print(f'{qcraft_package_path} transfer stops')
        kill_tmux_session_if_exists(tmux_session)

        h265_config = {}
        data_storage_folder = os.path.join('/home', os.getlogin(), 'ZONE', 'temp_data')
        if os.path.exists(data_storage_folder):
            shutil.rmtree(data_storage_folder)
        os.makedirs(data_storage_folder)
        h265_config_path = os.path.join(data_storage_folder, 'h265_config.yaml')
        timestamp_col = ['frame_index', 'time_stamp', 'frame_type']
        for q_camera_name in os.listdir(full_folder_path):
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

            pd.DataFrame(rows, columns=timestamp_col).to_csv(timestamp_path, index=False)
            h265_config[topic] = {
                'timestamp_path': timestamp_path,
                'H265_path': h265_file_list,
            }

        h265_config['h265_temp'] = full_folder_path
        with open(h265_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(h265_config, file)

        return h265_config_path

    def h265_to_db3(self, h265_config_path, db3_path):
        tmux_session = variables['tmux_node']['h265_gen'][0]
        tmux_window = variables['tmux_node']['h265_gen'][1]
        kill_tmux_session_if_exists(tmux_session)

        with open(h265_config_path, 'r', encoding='utf-8') as file:
            h265_config = yaml.safe_load(file)

        if 'h265_temp' in h265_config:
            del h265_config['h265_temp']
        min_ts, max_ts = 9999999999, 0
        for topic in h265_config:
            timestamp = pd.read_csv(h265_config[topic]['timestamp_path'], index_col=False)['time_stamp']
            min_ts = min(float(timestamp.min()), min_ts)
            max_ts = max(float(timestamp.max()), max_ts)

        if max_ts - min_ts < 15:
            print('H265没有足够的数据，不执行转化.db3')
            return

        print(f'开始转化.db3, 时间长度为{round(max_ts - min_ts, 3)} seconds')
        min_ts = datetime.fromtimestamp(int(min_ts)).strftime("%Y_%m_%d-%H_%M_%S")
        max_ts = datetime.fromtimestamp(int(max_ts)).strftime("%Y_%m_%d-%H_%M_%S")
        ros2bag_path = os.path.join(db3_path, f'{min_ts}={max_ts}')
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
            folder_size_1 = get_folder_size(db3_path)
            time.sleep(8)
            folder_size_2 = get_folder_size(db3_path)
            print(f'{round(time.time() - t0)} ros2bag size == {round(folder_size_2 / 1024 / 1024, 2)} MB')
            if folder_size_2 == folder_size_1:
                count -= 1
        print(f'ros2bag gen stops')
        kill_tmux_session_if_exists(tmux_session)

        with open(h265_config_path, 'r', encoding='utf-8') as file:
            h265_config = yaml.safe_load(file)

        # if 'h265_temp' in h265_config:
        #     force_delete_folder(h265_config['h265_temp'])
        #     del h265_config['h265_temp']
        #     for topic in h265_config:
        #         os.remove(h265_config[topic]['timestamp_path'])
        # else:
        #     for topic in h265_config:
        #         os.remove(h265_config[topic]['H265_path'])
        #         os.remove(h265_config[topic]['timestamp_path'])

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

    def combine_db3(self, input_bags:list, output_bag:str):
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

    def combine_Kunyi_db3(self, kunyi_package_path):
        ros2bag_ins_path = os.path.join(kunyi_package_path, self.ros2bag_ins_name)
        ros2bag_h265_path = os.path.join(kunyi_package_path, self.ros2bag_h265_name)
        ros2bag_combine_path = os.path.join(kunyi_package_path, self.ros2bag_combine_name)
        input_bags = [ros2bag_h265_path, ros2bag_ins_path]
        output_bag = ros2bag_combine_path
        self.combine_db3(input_bags, output_bag)
        # 删除两个rosbag包
        if os.path.exists(ros2bag_ins_path):
            shutil.rmtree(ros2bag_ins_path)
        if os.path.exists(ros2bag_h265_path):
            shutil.rmtree(ros2bag_h265_path)

    def gen_AVM_from_db3(self, install_path, kunyi_package_path):
        bev_object = Ros2Bag2BirdView(install_path, kunyi_package_path)
        bev_object.extract_h265_raw_streams()
        bev_object.extract_frames_from_h265()


class DataLoggerAnalysis:

    def __init__(self, folder):
        self.folder = os.path.join(folder, 'rosbag')

        # 解压缩
        install_zip = os.path.join(self.folder, 'install', 'install.tar.gz')
        if not os.path.exists(install_zip):
            print(f'未找到{install_zip}')
            return

        cmd = 'cd {:s}; tar -xzvf install.tar.gz'.format(
            os.path.join(self.folder, 'install'),
        )
        p = os.popen(cmd)
        p.read()

        self.ros2bag_info = []
        for ros2bag_folder in glob.glob(os.path.join(self.folder, 'rosbag', 'rosbag2*')):
            t0 = datetime.strptime(os.path.basename(ros2bag_folder)[8:], "%Y_%m_%d-%H_%M_%S").timestamp()
            print(t0)

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

    def load_install(self, target_folder='/media/data/Q_DATA'):
        target_install = os.path.join(target_folder, 'install')
        if os.path.exists(target_install):
            shutil.rmtree(target_install)

        shutil.move(os.path.join(self.folder, 'install', 'install'), target_install)

        json_str = json.dumps(self.ros2bag_info, indent=2, sort_keys=False)
        ros2bag_info_path = os.path.join(target_folder, 'ros2bag_info.json')
        with open(ros2bag_info_path, 'w', encoding='utf-8') as f:
            f.write(json_str)

        return target_install, ros2bag_info_path


if __name__ == '__main__':
    install_path = '/home/vcar/ZONE/03_Workspace/install'
    q_docker_base_path = f'/media/data'

    qqq = DataTransformer(install_path=install_path, q_docker_base_path=q_docker_base_path)
    # kunyi_package_path = '/home/zhangliwei01/ZONE/manual_scenario/20240119_145625_n000001'
    # h265_config_path = qqq.kunyiMkv_to_h265(kunyi_package_path)
    # h265_config_path = '/home/zhangliwei01/ZONE/manual_scenario/20240119_145625_n000001/Images/h265_config.yaml'
    # qqq.h265_to_db3(h265_config_path, '/home/zhangliwei01/ZONE/manual_scenario/temp')

    # qcraft_package_path = '/Q_DATA/debug_data/20250614_105002_Q3402'
    # h265_folder_path = '/Q_DATA/result'
    # h265_config_path = qqq.qStf_to_h265(qcraft_package_path, h265_folder_path)
    # h265_config_path = f'/home/{os.getlogin()}/ZONE/temp_data/h265_config.yaml'
    # qqq.h265_to_db3(h265_config_path, '/media/data/Q_DATA/ros2bag')

    dla = DataLoggerAnalysis('/media/data/Q_DATA/debug_data/20250614-005-AEB')
    target_install, ros2bag_info_path = dla.load_install()
    print(target_install, ros2bag_info_path)