"""  
Created on 2024/6/28.  
@author: Bu Yujun  
"""

import copy
import glob
import os
import re
import shutil
import sys
import threading
import time
from datetime import datetime, timezone

import numpy as np
import pandas as pd
import yaml

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from Libs import get_project_path, draw_map
from Ros2BagParser import Ros2BagParser
from Ros2BagRecorder import Ros2BagRecorder
from Ros2BagPlayer import Ros2BagPlayer

sys.path.append(get_project_path())
from Utils.Libs import test_encyclopaedia, calculate_file_checksum, create_folder
from Utils.SSHClient import SSHClient
from Utils.Logger import send_log, bench_config


class ReplayController:

    def __init__(self, replay_config):
        # 变量初始化
        self.thread_list = []
        self.calib_file = {}
        self.scenario_calib_groups = {}
        self.scenario_replay_count = {}
        self.scenario_replay_datetime = {}

        # 定义异常累计分数，高了将重启板子
        self.abnormal_score = 0
        self.reboot_count = 0
        self.invalid_scenario_list = []

        # 读取参数
        self.replay_mode = replay_config['replay_mode'].lower()
        self.scenario_ids = replay_config['scenario_id']
        self.pred_raw_folder = replay_config['pred_folder']
        self.gt_raw_folder = replay_config['gt_folder']
        self.replay_action = replay_config['replay_action']
        self.bag_update = self.replay_action['bag_update']
        self.replay_end = self.replay_action['replay_end']
        self.truth_source = self.replay_action['truth_source']
        self.statistics_path = os.path.join(self.pred_raw_folder, 'topic_output_statistics.csv')
        if os.path.exists(self.statistics_path):
            self.origin_topic_statistics = pd.read_csv(self.statistics_path, index_col=0)
            if not len(self.origin_topic_statistics):
                self.origin_topic_statistics = None
        else:
            self.origin_topic_statistics = None

        self.product = replay_config['product']
        feature_group = replay_config['feature_group']
        self.record_topic = test_encyclopaedia[self.product]['record_topic'][feature_group]
        self.parse_topic = test_encyclopaedia[self.product]['parse_topic'][feature_group]

        # 建立文件夹
        create_folder(self.pred_raw_folder, False)
        create_folder(self.gt_raw_folder, False)

        # 实例化ssh_client用于控制ReplayClient的Api
        if not self.replay_action['video_info']:
            self.replay_client = None
        else:
            self.replay_client = SSHClient()
            if not self.replay_client.check_connection():
                self.replay_client = None

        # 实例化录包工具
        self.ros2bag_recorder = Ros2BagRecorder(
            workspace=replay_config['workspace']
        )

        # 实例化解包工具
        self.ros2bag_parser = Ros2BagParser(
            workspace=replay_config['workspace']
        )

        # 实例化播包工具
        self.ros2bag_player = Ros2BagPlayer(
            workspace=replay_config['workspace']
        )

        send_log(self, f'录制的topic: {self.record_topic}')
        send_log(self, f'解析的topic: {self.parse_topic}')

    def start_replay_and_record(self, scenario_id):
        send_log(self, '先录制一个空的包,用于清除缓存')
        null_folder, _ = self.ros2bag_recorder.start_record(
            scenario_id=scenario_id,
            folder=self.pred_raw_folder,
            topic_list=self.record_topic
        )
        time.sleep(5)
        self.ros2bag_recorder.stop_record()
        shutil.rmtree(null_folder)

        if self.replay_mode != 'ethernet':
            send_log(self, f'台架回灌模式,开始回灌{scenario_id}')
            self.replay_client.start_replay(
                scenario_id=scenario_id
            )
        else:
            send_log(self, f'网络回灌模式,开始回灌{scenario_id}')
            self.ros2bag_player.start_play(
                ros2bag_path=self.scenario_ids[scenario_id],
            )

        send_log(self, f'开始录包{scenario_id}')
        bag_folder, parser_folder = self.ros2bag_recorder.start_record(
            scenario_id=scenario_id,
            folder=self.pred_raw_folder,
            topic_list=self.record_topic
        )

        return bag_folder, parser_folder

    def stop_replay_and_record(self, scenario_id=None):
        if scenario_id:
            send_log(self, f'结束录包{scenario_id}')
        self.ros2bag_recorder.stop_record()

        if scenario_id:
            send_log(self, f'结束回灌{scenario_id}')

        if self.replay_mode != 'ethernet':
            self.replay_client.stop_replay()
        else:
            self.ros2bag_player.stop_play()

    def parse_bag(self, scenario_id):
        xz_l = glob.glob(os.path.join(self.pred_raw_folder,
                                      scenario_id, f'{scenario_id}*.tar.xz'))

        meta_l = glob.glob(
            os.path.join(self.pred_raw_folder,
                         scenario_id, f'{scenario_id}*', 'metadata.yaml'))

        if not len(meta_l) and len(xz_l):
            send_log(self, f'存在压缩包，先解压缩{scenario_id}')
            bag_xz_path = xz_l[0]
            cmd = 'cd {:s}; tar xvf {:s}'.format(
                os.path.join(self.pred_raw_folder, scenario_id),
                os.path.basename(bag_xz_path)
            )
            os.popen(cmd).read()
            send_log(self, f'{os.path.basename(bag_xz_path)}解压缩完成')
            bag_folder = bag_xz_path[:-7]

        elif len(meta_l):
            bag_folder = os.path.dirname(meta_l[0])

        else:
            bag_folder = None

        if not bag_folder and os.path.exists(bag_folder):
            send_log(self, f'无效的场景{scenario_id}')
            return

        parser_folder = os.path.join(self.pred_raw_folder, scenario_id, 'RawData')
        send_log(self, f'开始解析{scenario_id}')
        self.ros2bag_parser.getMsgInfo(
            bag_path=bag_folder,
            topic_list=self.parse_topic,
            folder=parser_folder,
            tag=scenario_id,
            ros2json_flag=self.replay_action.get('ros2json', False),
        )

    def compress_bag(self, scenario_id):
        # 压缩ros2bag
        xz_l = glob.glob(os.path.join(self.pred_raw_folder,
                                      scenario_id, f'{scenario_id}*.tar.xz'))
        if len(xz_l):
            os.remove(xz_l[0])

        meta_l = glob.glob(
            os.path.join(self.pred_raw_folder,
                         scenario_id, f'{scenario_id}*', 'metadata.yaml'))

        if len(meta_l):
            bag_folder = os.path.dirname(meta_l[0])
            send_log(self, '开始压缩 {:s}.tar.xz'.format(os.path.basename(bag_folder)))
            cmd = 'cd {:s}; tar -Jcvf {:s}.tar.xz {:s}'.format(
                os.path.dirname(bag_folder), os.path.basename(bag_folder), os.path.basename(bag_folder)
            )
            p = os.popen(cmd)
            p.read()
            send_log(self, '压缩完成 {:s}.tar.xz'.format(os.path.basename(bag_folder)))
            shutil.rmtree(bag_folder)

        # 压缩json文件夹
        parser_folder = os.path.join(self.pred_raw_folder, scenario_id, 'RawData')
        for f in os.listdir(parser_folder):
            json_folder = os.path.join(parser_folder, f)
            if os.path.isdir(json_folder):
                if self.replay_action.get('ros2json', False):
                    send_log(self, '开始压缩 {:s}'.format(json_folder))
                    cmd = 'cd {:s}; tar -Jcvf {:s}_json.tar.xz {:s}'.format(
                        os.path.dirname(json_folder), os.path.basename(json_folder), os.path.basename(json_folder)
                    )
                    p = os.popen(cmd)
                    p.read()
                    send_log(self, '压缩完成 {:s}.tar.xz'.format(os.path.basename(json_folder)))
                shutil.rmtree(json_folder)

    def group_scenarios_by_calib(self):
        scenario_groups = {}
        for scenario_id in self.scenario_ids:
            parser_folder = os.path.join(self.pred_raw_folder, scenario_id, 'RawData')
            test_topic_info = os.path.join(parser_folder, 'TestTopicInfo.yaml')
            if self.bag_update or (not os.path.exists(test_topic_info)):
                checksum_path = glob.glob(os.path.join(self.pred_raw_folder, scenario_id, 'scenario_info', f'calib-*'))[0]
                check_sum = os.path.basename(checksum_path).split('-')[-1]
                if check_sum not in scenario_groups:
                    scenario_groups[check_sum] = []
                scenario_groups[check_sum].append(scenario_id)
            else:
                send_log(self, f'{scenario_id}已经存在,不需录制')

        for key, value in scenario_groups.items():
            send_log(self, f'标定文件为{key}的场景为{value}')

        self.scenario_calib_groups = scenario_groups

    def get_calib(self, scenario_id):
        scenario_info_folder = os.path.join(self.pred_raw_folder, scenario_id, 'scenario_info')
        if os.path.exists(scenario_info_folder):
            shutil.rmtree(scenario_info_folder)
        os.makedirs(scenario_info_folder)

        if not self.replay_client:
            send_log(self, f'没有replay client连接，无法标定文件{scenario_id}')
            check_sum = calculate_file_checksum(os.path.join(get_project_path(), 'requirements.txt'))
        else:
            send_log(self, f'获取标定文件{scenario_id}')
            self.replay_client.get_scenario_info(
                scenario_id=scenario_id,
                info_type='Calib',
                local_folder=scenario_info_folder,
            )
            self.calib_file[scenario_id] = scenario_info_folder
            check_file_path = os.path.join(self.calib_file[scenario_id], 'yaml_calib', 'camera_0.yaml')
            check_sum = calculate_file_checksum(check_file_path)

        checksum_path = os.path.join(self.pred_raw_folder, scenario_id, 'scenario_info', f'calib-{check_sum}')
        with open(checksum_path, 'w') as file:
            file.write(check_sum)

    def get_video_info(self, scenario_id):
        if not self.replay_client:
            send_log(self, f'没有replay client连接，无法获得视频信息{scenario_id}')
            return

        # 如果之前没有获取过相机参数文件，这里补充获取一次
        if scenario_id not in self.calib_file:
            self.get_calib(scenario_id)

        send_log(self, f'获取场景信息{scenario_id}')
        scenario_info_folder = os.path.join(self.pred_raw_folder, scenario_id, 'scenario_info')
        if not os.path.exists(scenario_info_folder):
            os.makedirs(scenario_info_folder)
        self.replay_client.get_scenario_info(
            scenario_id=scenario_id,
            info_type='VideoInfo',
            local_folder=scenario_info_folder,
        )

        # 获取一个大致的场景时间与ECU时间的差,可能有用
        video_info_path = os.path.join(scenario_info_folder, 'video_info.yaml')
        with open(video_info_path) as f:
            video_info = yaml.load(f, Loader=yaml.FullLoader)
        parser_folder = os.path.join(self.pred_raw_folder, scenario_id, 'RawData')
        ego_csv_list = glob.glob(os.path.join(parser_folder, '*MotionIpd*hz.csv'))
        if len(ego_csv_list) and len(pd.read_csv(ego_csv_list[0])):
            ego_t0 = float(pd.read_csv(ego_csv_list[0])['time_stamp'][0])
            video_info['time_delta_estimated'] = video_info['start_time'] - ego_t0 - 1
            with open(video_info_path, 'w', encoding='utf-8') as f:
                yaml.dump(video_info, f, encoding='utf-8', allow_unicode=True)
        else:
            send_log(self, f'{scenario_id} VehicleMotion数据为空')

        sainspva_data_list = []
        sainspva_csv_list = glob.glob(os.path.join(parser_folder, f'SAINSPVA_{scenario_id}*data.csv'))
        if len(sainspva_csv_list):
            for sainspva_csv in sainspva_csv_list:
                sainspva_data = pd.read_csv(sainspva_csv, index_col=None)
                if len(sainspva_data):
                    sainspva_data_list.append(sainspva_data)

            if len(sainspva_data_list):
                sainspva_data = pd.concat(sainspva_data_list).sort_values(by=['time_stamp'])
                map_path = os.path.join(self.pred_raw_folder, scenario_id, 'scenario_info', f'{scenario_id}_map.png')
                try:
                    draw_map(sainspva_data, map_path)
                except Exception as e:
                    send_log(self, f'{e}, 绘制{scenario_id}地图失败')

    def get_annotation(self):
        if not os.path.isdir(self.gt_raw_folder):
            os.makedirs(self.gt_raw_folder)

        for scenario_id in self.scenario_ids:
            send_log(self, f'获取真值 {scenario_id}')
            if not len(self.truth_source):
                remote_folder = f'/media/data/annotation/{scenario_id}'
            else:
                remote_folder = f'/media/data/annotation/{scenario_id}_{self.truth_source}'
            local_folder = os.path.join(self.gt_raw_folder, scenario_id)
            self.replay_client.scp_folder_remote_to_local(local_folder, remote_folder)

    def copy_calib_file(self, scenario_id):
        if not self.replay_client:
            send_log(self, f'没有replay client连接，无法拷贝相机参数{scenario_id}')
            return

        send_log(self, f'拷贝相机参数{scenario_id}')
        # 拷贝参数到ReplayClient的Temp文件夹下
        local_folder = os.path.join(self.calib_file[scenario_id], 'es37_calib')
        remote_folder = f'{bench_config["ReplayClient"]["py_path"]}/Temp'
        self.replay_client.scp_folder_local_to_remote(local_folder, remote_folder)
        time.sleep(1)

        # 调用接口复制参数进ECU
        self.replay_client.flash_camera_config(ecu_type=self.product)

    def replay_one_scenario(self, calib_checksum, scenario_id):

        def parse_log_line(log_line):
            # 正则表达式提取时间和相机信息
            time_pattern = r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}.\d+)\]'
            camera_pattern = r'\[camera_(\d+)\]: (\d+\.\d+)'

            # 解析时间（强制按 UTC 处理）
            time_match = re.search(time_pattern, log_line)
            if not time_match:
                return None, None
            time_str = time_match.group(1)
            # 关键：指定 timezone=timezone.utc，避免本地时区影响
            dt = datetime.strptime(time_str, '%Y-%m-%d %H:%M:%S.%f').replace(tzinfo=timezone.utc)
            timestamp = dt.timestamp()  # 此时为 UTC 时间戳

            # 解析相机帧率
            camera_fps = {}
            for match in re.finditer(camera_pattern, log_line):
                camera_id = match.group(1)
                fps = float(match.group(2))
                camera_fps[f'camera_{camera_id}'] = fps

            camera_fps = dict(sorted(camera_fps.items(), key=lambda item: int(item[0].split('_')[1])))
            return timestamp, *camera_fps.values()

        # 有的时候部分topic的计数会小于目标数量，尝试3次
        try_count = 0
        while True:
            try_count += 1
            send_log(self, '===============================================')
            send_log(self, f'{scenario_id} 第{try_count}次场景录制开始')
            self.start_replay_and_record(scenario_id)
            start_time = time.time()

            while True:
                if self.replay_mode != 'ethernet':
                    replay_process = self.replay_client.get_replay_process()
                else:
                    replay_process = self.ros2bag_player.get_play_process(
                        start_time=start_time,
                        ros2bag_path=self.scenario_ids[scenario_id],
                    )

                send_log(self, '{:s}回灌进度{:.1%}'.format(scenario_id, replay_process))
                if float(replay_process) > self.replay_end / 100:
                    self.stop_replay_and_record(scenario_id)
                    break
                time.sleep(5)

            self.parse_bag(scenario_id)

            self.scenario_replay_count[scenario_id] = try_count
            self.scenario_replay_datetime[scenario_id] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            scenario_is_valid = self.analyze_raw_data(calib_checksum)

            if scenario_id in scenario_is_valid and scenario_is_valid[scenario_id] == 1:
                send_log(self, f'{scenario_id} 场景录制成功，尝试次数-{try_count}')
                break

            if try_count == self.replay_action['retest']:
                send_log(self, f'{scenario_id} {try_count}次场景录制全部失败，加入黑名单')
                break

        t = threading.Thread(target=self.compress_bag, args=(scenario_id,))
        t.daemon = True
        t.start()
        self.thread_list.append(t)

        # 网络回灌下，将logsim的时间戳对应关系保存到文件夹下
        time2time_path = glob.glob(os.path.join('/home', '*', '*H265.txt'))
        for f in time2time_path:
            shutil.copy2(f, os.path.join(self.pred_raw_folder, scenario_id))
            send_log(self, f'复制时间戳对照文件{time2time_path}')

        t_min, t_max = -1, 1e11
        logsim_txt_list = glob.glob(os.path.join(self.pred_raw_folder, 'CameraFrontWideH265.txt'))
        if len(logsim_txt_list):
            df = pd.read_csv(
                logsim_txt_list[0],
                header=None,  # 无表头行
                names=['t0', 't1'],  # 指定列名为t0和t1
                sep=',\s*',  # 分隔符为逗号+任意空格（处理数据中的空格）
                engine='python'  # 使用Python解析器处理复杂分隔符
            )
            t_min, t_max = df['t1'].min(), df['t1'].max()

        # 将最新的sensor_center日志文件复制到本地
        for f in glob.glob(os.path.join(self.pred_raw_folder, scenario_id, '*.log')):
            os.remove(f)
        cmd = 'cd {:s}; ./fetch_latest_logs.sh {:s} 5'.format(
            os.path.join(get_project_path(), 'Envs', 'Master', 'Interfaces'),
            os.path.join(self.pred_raw_folder, scenario_id)
        )
        p = os.popen(cmd)
        p.read()

        sensor_center_log_lines = []
        for f in glob.glob(os.path.join(self.pred_raw_folder, scenario_id, '*.log')):
            with open(f, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    if 'sensor send msg fps' in line:
                        sensor_center_log_lines.append(parse_log_line(line.strip()))

        if len(sensor_center_log_lines):
            sensor_center_log_csv_path = os.path.join(self.pred_raw_folder, scenario_id, 'sensor_center_log.csv')
            sensor_center_log = pd.DataFrame(sensor_center_log_lines, columns=['time_stamp', '4', '5', '6', '7', '8', '9']).sort_values('time_stamp')
            sensor_center_log = sensor_center_log[(sensor_center_log['time_stamp'] >= t_min) & (sensor_center_log['time_stamp'] <= t_max)]
            sensor_center_log.to_csv(sensor_center_log_csv_path, index=False)

        send_log(self, f'复制sensor_center日志文件{sensor_center_log_csv_path}')

    def start(self):

        # 1.获取真值，线程中执行
        if self.replay_action['truth']:
            t = threading.Thread(target=self.get_annotation)
            t.daemon = True
            t.start()
            self.thread_list.append(t)

        if self.replay_action['record']:
            # 2. 检查标定文件并分组
            self.calib_file = {}
            for scenario_id in self.scenario_ids:
                self.get_calib(scenario_id)

            # 3. 录制和解析
            self.group_scenarios_by_calib()
            for calib_checksum, scenario_group in self.scenario_calib_groups.items():
                if self.replay_action['calib']:
                    self.copy_calib_file(scenario_group[0])

                for scenario_id in scenario_group:
                    self.replay_one_scenario(calib_checksum, scenario_id)

                    # 积分触发重启控制器，并将invalid场景的数据删除重新测试
                    if self.replay_action['retest'] > 1 and self.abnormal_score > 5 and self.reboot_count <= 25:
                        if self.replay_client is not None:
                            send_log(self, f'积分={self.abnormal_score},尝试触发中间重测机制')
                            if not self.reboot_power():
                                send_log(self, f'重启失败,后续场景{scenario_id}不再录制')
                                break

                        self.wait_for_threading()
                        for invalid_scenario_id in self.invalid_scenario_list:
                            raw_folder = os.path.join(self.pred_raw_folder, invalid_scenario_id, 'RawData')
                            if os.path.exists(raw_folder):
                                shutil.rmtree(raw_folder)

                        invalid_scenario_list = copy.deepcopy(self.invalid_scenario_list)
                        send_log(self, f'reboot_count={self.reboot_count}, 重启后录制的场景为{invalid_scenario_list}')
                        for invalid_scenario_id in invalid_scenario_list:
                            self.replay_one_scenario(calib_checksum, invalid_scenario_id)

                invalid_scenario_list = copy.deepcopy(self.invalid_scenario_list)
                if self.replay_action['retest'] > 1 and len(invalid_scenario_list) > 0:
                    if self.replay_client is not None:
                        send_log(self, f'{calib_checksum},尝试触发最终重测机制')
                        if not self.reboot_power():
                            send_log(self, '重启失败,后续场景不再录制')
                            break

                    # 最后再将invalid的场景再测一次
                    self.wait_for_threading()
                    for invalid_scenario_id in invalid_scenario_list:
                        raw_folder = os.path.join(self.pred_raw_folder, invalid_scenario_id, 'RawData')
                        if os.path.exists(raw_folder):
                            shutil.rmtree(raw_folder)

                    invalid_scenario_list = copy.deepcopy(self.invalid_scenario_list)
                    send_log(self, f'reboot_count={self.reboot_count}, 重启后录制的场景为{invalid_scenario_list}')
                    for invalid_scenario_id in invalid_scenario_list:
                        self.replay_one_scenario(calib_checksum, invalid_scenario_id)

        if self.replay_action['video_info']:
            for scenario_id in self.scenario_ids:
                self.get_video_info(scenario_id)

        self.wait_for_threading()
        send_log(self, '清理临时文件夹')
        if self.replay_client:
            self.replay_client.clear_temp_folder()

        if self.origin_topic_statistics is None:
            self.analyze_raw_data()

    def analyze_raw_data(self, calib_checksum=None):
        rows = []
        index = []
        columns = []
        scenario_is_valid = {}

        for scenario_id in os.listdir(self.pred_raw_folder):
            # 获取calib_checksum
            checksum_paths = glob.glob(os.path.join(self.pred_raw_folder, scenario_id, 'scenario_info', 'calib-*'))
            if not len(checksum_paths):
                continue
            scenario_calib_checksum = os.path.basename(checksum_paths[0]).split('-')[-1]

            # 获得topic输出
            raw_folder = os.path.join(self.pred_raw_folder, scenario_id, 'RawData')
            if not os.path.exists(os.path.join(raw_folder, 'TestTopicInfo.yaml')):
                continue

            with open(os.path.join(raw_folder, 'TestTopicInfo.yaml')) as f:
                test_topic = yaml.load(f, Loader=yaml.FullLoader)

            row = []
            columns = []
            topic_duration = {}
            for topic in test_topic['topics_for_parser']:
                if topic in []:
                    continue

                columns.append(topic)
                topic_tag = topic.replace('/', '')
                hz_path_list = glob.glob(os.path.join(raw_folder, f'{topic_tag}*hz.csv'))
                if len(hz_path_list):
                    hz_data = pd.read_csv(hz_path_list[0], index_col=False)
                else:
                    hz_data = pd.DataFrame()

                if not len(hz_data):
                    row.append('0/0/0/0-0')
                    topic_duration[topic] = 0
                else:
                    topic_start_time = hz_data['time_stamp'].min()
                    topic_end_time = hz_data['time_stamp'].max()
                    topic_duration[topic] = topic_end_time - topic_start_time
                    row.append('{:d}/{:d}/{:.2f}/{:.2f}-{:.2f}'.format(
                    len(hz_data), hz_data['count'].sum(), topic_duration[topic],
                        hz_data['time_stamp'].min(), hz_data['time_stamp'].max()))

            if max(topic_duration.values()) - min(topic_duration.values()) <= 25:
                valid_flag = 1
            else:
                valid_flag = 0

            scenario_is_valid[scenario_id] = valid_flag
            replay_count = 1
            date_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            if scenario_id in self.scenario_replay_count:
                replay_count = self.scenario_replay_count[scenario_id]
                date_time = self.scenario_replay_datetime[scenario_id]
                if (isinstance(self.origin_topic_statistics, pd.DataFrame)
                        and scenario_id in self.origin_topic_statistics.index):
                    replay_count += self.origin_topic_statistics.at[scenario_id, 'replay_count']

            elif (isinstance(self.origin_topic_statistics, pd.DataFrame)
                  and scenario_id in self.origin_topic_statistics.index):
                replay_count = self.origin_topic_statistics.at[scenario_id, 'replay_count']
                date_time = self.origin_topic_statistics.at[scenario_id, 'record_time']

            row.extend([scenario_calib_checksum, date_time, replay_count, valid_flag])
            index.append(scenario_id)
            rows.append(row)

        self.invalid_scenario_list = []
        self.abnormal_score = 0
        if len(columns):
            output_statistics = pd.DataFrame(rows, columns=columns + ['calib_checksum', 'record_time', 'replay_count', 'isValid'], index=index)
            output_statistics.sort_values(by='record_time', inplace=True)
            output_statistics.to_csv(self.statistics_path)
            if calib_checksum:
                calib_output_statistics = output_statistics[output_statistics['calib_checksum'] == calib_checksum]
                # 计算失效积分
                # 每一个invalid场景增加1分，每次出现0/0/0/0-0增加1分
                # 超过5分，触发重启并重新测试invalid的场景
                self.invalid_scenario_list = calib_output_statistics[calib_output_statistics['isValid'] == 0].index.tolist()
                self.abnormal_score = len(self.invalid_scenario_list) + np.sum(calib_output_statistics.values == '0/0/0/0-0')

        send_log(self, f'重启积分为{self.abnormal_score}')
        send_log(self, f'失效场景为{self.invalid_scenario_list}')
        return scenario_is_valid

    def wait_for_threading(self):
        send_log(self, '等待所有线程都结束')
        for t in self.thread_list:
            t.join()
        self.thread_list.clear()

    def reboot_power(self):
        self.reboot_count += 1
        for _ in range(3):
            self.replay_client.control_power('off')
            time.sleep(15)
            res = self.replay_client.control_power('on_with_waiting')
            if res:
                return True
            time.sleep(5)

        return False


if __name__ == '__main__':

    test_project_path = '/home/zhangliwei01/ZONE/TestProject/EP39/AH4EM_DEBUG'
    workspace_folder = os.path.join(test_project_path, '03_Workspace')
    # 寻找所有TestConfig.yaml
    test_config_yaml_list = glob.glob(os.path.join(test_project_path, '04_TestData', '*', 'TestConfig.yaml'))
    pred_folder = os.path.join(test_project_path, '01_Prediction')
    gt_folder = os.path.join(test_project_path, '02_GroundTruth')

    feature_group_list = []
    task_folder_dict = {}
    test_config_dict = {}

    for test_config_yaml in test_config_yaml_list:
        with open(test_config_yaml) as f:
            test_config = yaml.load(f, Loader=yaml.FullLoader)
        test_topic = test_config['test_topic']
        test_config['pred_folder'] = pred_folder
        test_config['gt_folder'] = gt_folder
        feature_group_list.append(test_config['feature_group'])

        with open(test_config_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(test_config,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

        if test_topic not in task_folder_dict:
            task_folder_dict[test_topic] = []
        task_folder_dict[test_topic].append(os.path.dirname(test_config_yaml))

        if test_topic not in test_config_dict:
            test_config_dict[test_topic] = []
        test_config_dict[test_topic].append(test_config)

    scenario_dict = {}
    for test_topic in test_config_dict.keys():
        for test_config in test_config_dict[test_topic]:
            for scenario_tag in test_config['scenario_tag']:
                if isinstance(scenario_tag['scenario_id'], list):
                    scenario_id_dict = {s_id: None for s_id in scenario_tag['scenario_id']}
                    scenario_dict.update(scenario_id_dict)
                elif isinstance(scenario_tag['scenario_id'], dict):
                    scenario_dict.update(scenario_tag['scenario_id'])

    # 使用第一个test_config的值作为录包的test_action依据
    test_topic = list(test_config_dict.keys())[0]
    test_config = test_config_dict[test_topic][0]
    if (test_config['test_action']['ros2bag']['record']
            or test_config['test_action']['ros2bag']['truth']
            or test_config['test_action']['ros2bag']['video_info']):
        replay_config = {
            'product': test_config['product'],
            'feature_group': test_config['feature_group'],
            'replay_action': test_config['test_action']['ros2bag'],
            'pred_folder': test_config['pred_folder'],
            'gt_folder': test_config['gt_folder'],
            'workspace': os.path.join(test_project_path, '03_Workspace'),
            'scenario_id': scenario_dict,
            'replay_mode': test_config['replay_mode'],
        }

        print("test config", replay_config)
        ReplayController(replay_config).start()
        print('ReplayController 实例化成功')
    # ros2bag_path = '/home/zhangliwei01/ZONE/test_AEB/20231130_152434_n000001/20231130_152434_n000001merge_h265_can_ros2bag'
    #
    # RC.start_play_rosbag_and_record_Ethernet(ros2bag_path,'20231130_152434_n000001')
    # print("end end endl;")
