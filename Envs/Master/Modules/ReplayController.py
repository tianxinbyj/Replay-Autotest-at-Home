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

from Libs import get_project_path
from Ros2BagParser import Ros2BagParser
from Ros2BagRecorder import Ros2BagRecorder

sys.path.append(get_project_path())

from Utils.Libs import bench_config, test_encyclopaedia, calculate_file_checksum
from Utils.SSHClient import SSHClient
from Utils.Logger import send_log


class ReplayController:

    def __init__(self, replay_config):

        # 变量初始化
        self.thread_list = []
        self.calib_file = {}

        # 读取参数
        self.replay_end = replay_config['replay_end']
        self.scenario_ids = replay_config['scenario_id']
        self.pred_raw_folder = replay_config['data_folder']['raw']['pred']
        self.gt_raw_folder = replay_config['data_folder']['raw']['gt']
        self.workspace = replay_config['data_folder']['workspace']
        self.replay_action = replay_config['replay_action']
        self.bag_update = self.replay_action['bag_update']

        product = replay_config['product']
        test_type = replay_config['test_type']
        self.record_topic = test_encyclopaedia[product]['record_topic'][test_type]
        self.parse_topic = test_encyclopaedia[product]['parse_topic'][test_type]

        # 建立文件夹
        if not os.path.isdir(self.pred_raw_folder):
            os.makedirs(self.pred_raw_folder)

        # 实例化ssh_client用于控制ReplayClient的Api
        self.replay_client = SSHClient(
            ip=bench_config['ReplayClient']['ip'],
            username=bench_config['ReplayClient']['username'],
            password=str(bench_config['ReplayClient']['password']),
        )
        self.replay_client.set_interface_path(bench_config['ReplayClient']['py_path'])

        # 实例化录包工具
        self.ros2bag_recorder = Ros2BagRecorder(
            workspace=self.workspace
        )

        # 实例化解包工具
        self.ros2bag_parser = Ros2BagParser(
            workspace=self.workspace
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

        self.get_video_info(scenario_id)

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
                check_file_path = os.path.join(self.calib_file[scenario_id], 'json_calib', '100', 'front.json')
                check_sum = calculate_file_checksum(check_file_path)
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
            ego_t0 = float(pd.read_csv(ego_csv)['time_stamp'][0])
            video_info['time_delta_estimated'] = video_info['start_time'] - ego_t0 - 1
            with open(video_info_path, 'w', encoding='utf-8') as f:
                yaml.dump(video_info, f, encoding='utf-8', allow_unicode=True)
        else:
            send_log(self, f'{scenario_id} /PI/EG/EgoMotionInfo 数据为空')

    def get_annotation(self):
        if not os.path.isdir(self.gt_raw_folder):
            os.makedirs(self.gt_raw_folder)

        for scenario_id in self.scenario_ids:
            send_log(self, f'获取真值 {scenario_id}')
            remote_folder = f'/media/data/annotation/{scenario_id}'
            local_folder = os.path.join(self.gt_raw_folder, scenario_id)
            self.replay_client.scp_folder_remote_to_local(local_folder, remote_folder)

    def copy_calib_file(self, scenario_id):
        # todo: 下电, 拷贝参数, 上电, 使用scenario_group[0]对应的参数
        send_log(self,  f'拷贝相机参数{scenario_id}')

    def start(self):

        # 1.获取真值，线程中执行
        if self.replay_action['get_gt']:
            t = threading.Thread(target=self.get_annotation)
            t.daemon = True
            t.start()
            self.thread_list.append(t)

        # 2. 检查标定文件并分组
        self.calib_file = {}
        for scenario_id in self.scenario_ids:
            self.get_calib(scenario_id)
        scenario_groups = self.group_scenarios_by_calib()

        # 3. 录制和解析
        if self.replay_action['record']:
            for scenario_group in scenario_groups:
                if self.replay_action['calib']:
                    self.copy_calib_file(scenario_group[0])

                for scenario_id in scenario_group:
                    self.start_replay_and_record(scenario_id)

                    while True:
                        replay_process = self.replay_client.get_replay_process()
                        send_log(self, '{:s}回灌进度{:.1%}'.format(scenario_id, replay_process))
                        if float(replay_process) > self.replay_end / 100:
                            self.stop_replay_and_record(scenario_id)
                            break
                        time.sleep(8)

                    self.parse_bag(scenario_id)
                    t = threading.Thread(target=self.compress_bag, args=(scenario_id,))
                    t.daemon = True
                    t.start()
                    self.thread_list.append(t)

                    self.analyze_raw_data()

        send_log(self, '等待所有线程都结束')
        for t in self.thread_list:
            t.join()
        self.thread_list.clear()

        send_log(self, '清理临时文件夹')
        self.replay_client.clear_temp_folder()

    def analyze_raw_data(self):
        rows = []
        index = []
        columns = []
        for scenario_id in os.listdir(self.pred_raw_folder):
            raw_folder = os.path.join(self.pred_raw_folder, scenario_id, 'RawData')
            if not os.path.exists(os.path.join(raw_folder, 'TestTopicInfo.yaml')):
                continue

            with open(os.path.join(raw_folder, 'TestTopicInfo.yaml')) as f:
                test_topic = yaml.load(f, Loader=yaml.FullLoader)

            row = []
            columns = []
            valid_count = 0
            i = 0
            for topic in test_topic['topics_for_parser']:
                topic_tag = topic.replace('/', '')
                hz_data = pd.read_csv(glob.glob(os.path.join(raw_folder, f'{topic_tag}*hz.csv'))[0],
                                      index_col=False)
                row.append('{:d}/{:d}/{:.3f}-{:.3f}'.format(
                    len(hz_data), hz_data['count'].sum(), hz_data['time_stamp'].min(), hz_data['time_stamp'].max()))

                if len(hz_data) > 500:
                    valid_count += 1
                columns.append(topic)
                i += 1

            if valid_count < i:
                valid_flag = 0
            else:
                valid_flag = 1

            row.append(valid_flag)
            index.append(scenario_id)
            rows.append(row)

        if columns:
            res = pd.DataFrame(rows, columns=columns + ['valid'], index=index)
            res.to_csv(os.path.join(self.pred_raw_folder, 'topic_output_statistics.csv'))


if __name__ == '__main__':
    replay_config = {
        'replay_end': 95,
        'scenario_id': [
            '20230602_144755_n000003',
            # '20230627_170934_n000001',
            # '20230703_103858_n000003',
            # '20230703_105701_n000001',
            # '20230706_160503_n000001',
            # '20230706_161116_n000001',
            # '20230706_162037_n000001',
            # '20230602_144755_n000005',
            # '20230614_135643_n000001',
            # '20230614_142204_n000004',
            # '20230627_173157_n000001',
            # '20230706_165109_n000002',
            # '20230706_184054_n000001',
        ],
        'data_folder': {
            'raw': {
                'pred': '/home/zhangliwei01/ZONE/TestProject/2J5/pilot/01_Prediction',
                'gt': '/home/zhangliwei01/ZONE/TestProject/2J5/pilot/02_GroundTruth',
            },
            'workspace': '/home/zhangliwei01/ZONE/TestProject/2J5/pilot/03_Workspace',
        },
        'replay_action': {
            'calib': False,
            'record': True,
            'get_gt': True,
            'bag_update': True,
        },
        'product': 'ES37',
        'test_type': 'pilot',
    }

    replay_controller = ReplayController(replay_config)
    replay_controller.start()
