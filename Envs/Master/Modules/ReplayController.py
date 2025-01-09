"""  
Created on 2024/6/28.  
@author: Bu Yujun  
"""

import glob
import os
import shutil
import sys
import threading
import time

import pandas as pd
import yaml

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from Libs import get_project_path, draw_map
from Ros2BagParser import Ros2BagParser
from Ros2BagRecorder import Ros2BagRecorder

sys.path.append(get_project_path())

from Utils.Libs import test_encyclopaedia, calculate_file_checksum, create_folder
from Utils.SSHClient import SSHClient
from Utils.Logger import send_log, bench_config


class ReplayController:

    def __init__(self, replay_config):

        # 变量初始化
        self.thread_list = []
        self.calib_file = {}
        self.scenario_replay_count = {}
        self.scenario_replay_datetime = {}

        # 定义异常累计分数，高了将重启板子
        self.abnormal_score = 0
        self.reboot_count = 0

        # 读取参数
        self.scenario_ids = replay_config['scenario_id']
        self.pred_raw_folder = replay_config['pred_folder']
        self.gt_raw_folder = replay_config['gt_folder']
        self.replay_action = replay_config['replay_action']
        self.bag_update = self.replay_action['bag_update']
        self.replay_end = self.replay_action['replay_end']
        self.truth_source = self.replay_action['truth_source']

        self.product = replay_config['product']
        feature_group = replay_config['feature_group']
        self.record_topic = test_encyclopaedia[self.product]['record_topic'][feature_group]
        self.parse_topic = test_encyclopaedia[self.product]['parse_topic'][feature_group]

        # 建立文件夹
        create_folder(self.pred_raw_folder, False)
        create_folder(self.gt_raw_folder, False)

        # 实例化ssh_client用于控制ReplayClient的Api
        self.replay_client = SSHClient()

        # 实例化录包工具
        self.ros2bag_recorder = Ros2BagRecorder(
            workspace=replay_config['workspace']
        )

        # 实例化解包工具
        self.ros2bag_parser = Ros2BagParser(
            workspace=replay_config['workspace']
        )

        send_log(self, f'回灌的场景为: {self.scenario_ids}')
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

        send_log(self, f'开始回灌{scenario_id}')
        self.replay_client.start_replay(
            scenario_id=scenario_id)

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
        self.replay_client.stop_replay()

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
            tag=scenario_id
        )

    def compress_bag(self, scenario_id):
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

    def group_scenarios_by_calib(self):
        scenario_groups = {}
        for scenario_id in self.scenario_ids:
            parser_folder = os.path.join(self.pred_raw_folder, scenario_id, 'RawData')
            test_topic_info = os.path.join(parser_folder, 'TestTopicInfo.yaml')
            if self.bag_update or (not os.path.exists(test_topic_info)):
                check_file_path_1 = os.path.join(self.calib_file[scenario_id], 'json_calib', '100', 'front.json')
                check_file_path_2 = os.path.join(self.calib_file[scenario_id], 'json_calib', 'front.json')
                if os.path.exists(check_file_path_1):
                    check_sum = calculate_file_checksum(check_file_path_1)
                else:
                    check_sum = calculate_file_checksum(check_file_path_2)

                if check_sum not in scenario_groups:
                    scenario_groups[check_sum] = []
                scenario_groups[check_sum].append(scenario_id)
            else:
                send_log(self, f'{scenario_id}已经存在,不需录制')

        for key, value in scenario_groups.items():
            send_log(self, f'标定文件为{key}的场景为{value}')

        return list(scenario_groups.values())

    def get_calib(self, scenario_id):
        send_log(self, f'获取标定文件{scenario_id}')
        scenario_info_folder = os.path.join(self.pred_raw_folder, scenario_id, 'scenario_info')
        if os.path.exists(scenario_info_folder):
            shutil.rmtree(scenario_info_folder)
        os.makedirs(scenario_info_folder)
        self.replay_client.get_scenario_info(
            scenario_id=scenario_id,
            info_type='Calib',
            local_folder=scenario_info_folder,
        )
        self.calib_file[scenario_id] = scenario_info_folder

    def get_video_info(self, scenario_id):
        send_log(self, f'获取场景信息{scenario_id}')
        scenario_info_folder = os.path.join(self.pred_raw_folder, scenario_id, 'scenario_info')
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
        ego_csv = glob.glob(os.path.join(parser_folder, '*Ego*hz.csv'))[0]
        ego_data = pd.read_csv(ego_csv)
        if len(ego_data):
            ego_t0 = float(ego_data['time_stamp'][0])
            video_info['time_delta_estimated'] = video_info['start_time'] - ego_t0 - 1
            with open(video_info_path, 'w', encoding='utf-8') as f:
                yaml.dump(video_info, f, encoding='utf-8', allow_unicode=True)
        else:
            send_log(self, f'{scenario_id} /PI/EG/EgoMotionInfo 数据为空')

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
                    print(e, f'绘制{scenario_id}地图失败')

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
        send_log(self, f'拷贝相机参数{scenario_id}')

        # 拷贝参数到ReplayClient的Temp文件夹下
        local_folder = os.path.join(self.calib_file[scenario_id], 'es37_calib')
        remote_folder = f'{bench_config["ReplayClient"]["py_path"]}/Temp'
        self.replay_client.scp_folder_local_to_remote(local_folder, remote_folder)
        time.sleep(1)

        # 调用接口复制参数进ECU
        self.replay_client.flash_camera_config(ecu_type=self.product)

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
            for scenario_group in self.group_scenarios_by_calib():
                if self.replay_action['calib']:
                    self.copy_calib_file(scenario_group[0])

                for scenario_id in scenario_group:
                    # 有的时候部分topic的计数会小于目标数量，尝试3次
                    try_count = 0

                    while True:
                        try_count += 1
                        print(f'{scenario_id} 第{try_count}次场景录制开始')
                        send_log(self, f'{scenario_id} 第{try_count}次场景录制开始')
                        self.start_replay_and_record(scenario_id)

                        while True:
                            replay_process = self.replay_client.get_replay_process()
                            send_log(self, '{:s}回灌进度{:.1%}'.format(scenario_id, replay_process))
                            if float(replay_process) > self.replay_end / 100:
                                self.stop_replay_and_record(scenario_id)
                                break
                            time.sleep(8)

                        self.parse_bag(scenario_id)

                        self.scenario_replay_count[scenario_id] = try_count
                        self.scenario_replay_datetime[scenario_id] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                        scenario_is_valid = self.analyze_raw_data()

                        if scenario_id in scenario_is_valid and scenario_is_valid[scenario_id] == 1:
                            print(f'{scenario_id} 场景录制成功，尝试次数-{try_count}')
                            send_log(self, f'{scenario_id} 场景录制成功，尝试次数-{try_count}')
                            break

                        if try_count == self.replay_action['retest']:
                            print(f'{scenario_id} {try_count}次场景录制全部失败，加入黑名单')
                            send_log(self, f'{scenario_id} {try_count}次场景录制全部失败，加入黑名单')
                            break

                    t = threading.Thread(target=self.compress_bag, args=(scenario_id,))
                    t.daemon = True
                    t.start()
                    self.thread_list.append(t)

        if self.replay_action['video_info']:
            for scenario_id in self.scenario_ids:
                self.get_video_info(scenario_id)

        send_log(self, '等待所有线程都结束')
        print('等待所有线程都结束')
        for t in self.thread_list:
            t.join()
        self.thread_list.clear()

        send_log(self, '清理临时文件夹')
        self.replay_client.clear_temp_folder()

    def analyze_raw_data(self):
        statistics_path = os.path.join(self.pred_raw_folder, 'topic_output_statistics.csv')
        rows = []
        index = []
        columns = []
        scenario_is_valid = {}

        for scenario_id in os.listdir(self.pred_raw_folder):
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

            if max(topic_duration.values()) - min(topic_duration.values()) <= 10:
                valid_flag = 1
            else:
                valid_flag = 0

            scenario_is_valid[scenario_id] = valid_flag
            replay_count = 1
            date_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            if scenario_id in self.scenario_replay_count:
                replay_count = self.scenario_replay_count[scenario_id]
                date_time = self.scenario_replay_datetime[scenario_id]
            elif os.path.exists(statistics_path):
                topic_output_statistics = pd.read_csv(statistics_path, index_col=0)
                if scenario_id in topic_output_statistics.index:
                    replay_count = topic_output_statistics.at[scenario_id, 'replay_count']
                    date_time = topic_output_statistics.at[scenario_id, 'record_time']
            row.extend([date_time, replay_count, valid_flag])
            index.append(scenario_id)
            rows.append(row)

        if len(columns):
            res = pd.DataFrame(rows, columns=columns + ['record_time', 'replay_count', 'isValid'], index=index)
            res.sort_values(by='record_time', inplace=True)
            res.to_csv(statistics_path)

        return scenario_is_valid