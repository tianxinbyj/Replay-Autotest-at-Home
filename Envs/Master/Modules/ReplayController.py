"""  
Created on 2024/6/28.  
@author: Bu Yujun  
"""

import csv
import glob
import os
import shutil
import sys
import threading
import time

import numpy as np
import pandas as pd
import yaml

from Libs import get_project_path
from Ros2BagParser import Ros2BagParser
from Ros2BagRecorder import Ros2BagRecorder

sys.path.append(get_project_path())

from Utils.Libs import bench_config, test_encyclopaedia
from Utils.SSHClient import SSHClient


def write_log(csv_path, row, is_new=False):
    write_flag = 'w' if is_new else 'a+'
    with open(csv_path, write_flag, encoding='utf-8') as f:
        f_csv = csv.writer(f)
        f_csv.writerow(row)


class ReplayController:

    def __init__(self, replay_config):

        self.running_stage = None
        self.running_topic = None
        self.running_scenario_id = None
        self.write_log_flag = None
        self.thread_list = []

        # 读取参数
        self.replay_end = replay_config['replay_end']
        self.log_path = replay_config['log_path']
        self.bag_update = replay_config['bag_update']
        self.scenario_ids = replay_config['scenario_id']
        self.pred_raw_folder = replay_config['data_folder']['raw']['pred']
        self.gt_raw_folder = replay_config['data_folder']['raw']['gt']
        self.workspace = replay_config['data_folder']['workspace']
        self.bag_action = replay_config['bag_action']

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

        print(self.bag_action)
        print('录制的topic:', self.record_topic)
        print('解析的topic:', self.parse_topic)

    def start_replay_and_record(self, scenario_id):

        print('先录制一个空的包,用于清除缓存')
        null_folder, _ = self.ros2bag_recorder.start_record(
            scenario_id=scenario_id,
            folder=self.pred_raw_folder,
            topic_list=self.record_topic
        )
        time.sleep(5)
        self.ros2bag_recorder.stop_record()
        shutil.rmtree(null_folder)

        print(f'开始回灌{scenario_id}')
        self.replay_client.start_replay(
            scenario_id=scenario_id)

        print(f'开始录包{scenario_id}')
        bag_folder, parser_folder = self.ros2bag_recorder.start_record(
            scenario_id=scenario_id,
            folder=self.pred_raw_folder,
            topic_list=self.record_topic
        )

        return bag_folder, parser_folder

    def stop_replay_and_record(self, scenario_id=None):
        if scenario_id:
            print(f'结束录包{scenario_id}')
        self.ros2bag_recorder.stop_record()
        if scenario_id:
            print(f'结束回灌{scenario_id}')
        self.replay_client.stop_replay()

    def parse_bag(self, scenario_id):
        xz_l = glob.glob(os.path.join(self.pred_raw_folder,
                                     scenario_id, f'{scenario_id}*.tar.xz'))

        meta_l = glob.glob(
            os.path.join(self.pred_raw_folder,
                         scenario_id, f'{scenario_id}*', 'metadata.yaml'))

        if not len(meta_l) and len(xz_l):
            print(f'存在压缩包，先解压缩{scenario_id}')
            bag_xz_path = xz_l[0]
            cmd = 'cd {:s}; tar xvf {:s}'.format(
                os.path.join(self.pred_raw_folder, scenario_id),
                os.path.basename(bag_xz_path)
            )
            os.popen(cmd).read()
            print(f'{os.path.basename(bag_xz_path)}解压缩完成')
            bag_folder = bag_xz_path[:-7]

        elif len(meta_l):
            bag_folder = os.path.dirname(meta_l[0])

        else:
            bag_folder = None

        if not bag_folder and os.path.exists(bag_folder):
            print(f'无效的场景{scenario_id}')
            return

        parser_folder = os.path.join(self.pred_raw_folder, scenario_id, 'RawData')
        print(f'开始解析{scenario_id}')
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
            print('{:s}.tar.xz 开始压缩'.format(os.path.basename(bag_folder)))
            cmd = 'cd {:s}; tar -Jcvf {:s}.tar.xz {:s}'.format(
                os.path.dirname(bag_folder), os.path.basename(bag_folder), os.path.basename(bag_folder)
            )
            p = os.popen(cmd)
            p.read()
            print('{:s}.tar.xz 压缩完成'.format(os.path.basename(bag_folder)))
            shutil.rmtree(bag_folder)

        print(f'获取场景信息{scenario_id}')
        parser_folder = os.path.join(self.pred_raw_folder, scenario_id, 'RawData')
        scenario_info_folder = os.path.join(parser_folder, 'scenario_info')
        if os.path.exists(scenario_info_folder):
            shutil.rmtree(scenario_info_folder)
        os.makedirs(scenario_info_folder)

        self.replay_client.get_video_info(
            scenario_id=scenario_id,
            local_folder=scenario_info_folder,
        )

        # 获取一个大致的场景时间与ECU时间的差,可能有用
        video_info_path = os.path.join(scenario_info_folder, 'video_info.yaml')
        with open(video_info_path) as f:
            video_info = yaml.load(f, Loader=yaml.FullLoader)
        ego_csv = glob.glob(os.path.join(parser_folder, '*Ego*hz.csv'))[0]
        ego_data = pd.read_csv(ego_csv)
        if len(ego_data):
            ego_t0 = float(pd.read_csv(ego_csv)['time_stamp'][0])
            video_info['time_delta_estimated'] = video_info['start_time'] - ego_t0 - 1
            with open(video_info_path, 'w', encoding='utf-8') as f:
                yaml.dump(video_info, f, encoding='utf-8', allow_unicode=True)

    def get_annotation(self):
        if not os.path.isdir(self.gt_raw_folder):
            os.makedirs(self.gt_raw_folder)

        for scenario_id in self.scenario_ids:
            remote_folder = f'/media/data/annotation/{scenario_id}'
            local_folder = os.path.join(self.gt_raw_folder, scenario_id)
            self.replay_client.scp_folder_remote_to_local(local_folder, remote_folder)

    def run(self):
        # 0.记录log
        self.write_log_flag = 1
        self.auto_write_log()

        # 1.获取真值，线程中执行
        if self.bag_action['get_gt']:
            t = threading.Thread(target=self.get_annotation)
            t.daemon = True
            t.start()

        for scenario_id in self.scenario_ids:

            self.running_stage = f'{self.__class__.__name__}.bag_record'
            self.running_topic = 'All_Topic'
            self.running_scenario_id = scenario_id

            # 2.回灌和录包
            if self.bag_action['record']:
                scenario_folder = os.path.join(self.pred_raw_folder, scenario_id)
                if not self.bag_update:
                    if os.path.exists(scenario_folder):
                        print(scenario_id, '已存在, 不重新录制')
                        continue
                else:
                    if os.path.exists(scenario_folder):
                        shutil.rmtree(scenario_folder)
                        print(scenario_id, '已存在, 删除重新录制')

                self.start_replay_and_record(scenario_id)

                while True:
                    replay_process = self.replay_client.get_replay_process()
                    print(scenario_id, '回灌进度{:.2%}'.format(replay_process))
                    if float(replay_process) > self.replay_end / 100:
                        self.stop_replay_and_record(scenario_id)
                        break
                    time.sleep(8)

            # 3.解析和压缩
            if self.bag_action['parse']:
                self.parse_bag(scenario_id)

                if self.bag_action['compress']:
                    t = threading.Thread(target=self.compress_bag, args=(scenario_id, ))
                    t.daemon = True
                    t.start()
                    self.thread_list.append(t)

                self.analyze_raw_data()

        self.write_log_flag = 0
        print('等待所有线程都结束')
        for t in self.thread_list:
            print(t.ident)
            t.join()
        self.thread_list.clear()

    def write_log(self):
        while self.write_log_flag:
            row = [time.time(), self.running_scenario_id, self.running_topic, self.running_stage]
            write_log(self.log_path, row)
            time.sleep(np.pi / 2)

    def auto_write_log(self):
        t = threading.Thread(target=self.write_log)
        t.daemon = True
        t.start()

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
        'log_path': '123.csv',
        'bag_update': True,
        'scenario_id': ['20231130_184025_n000001'],
        'data_folder': {
            'raw': {
                'pred': '/home/zhangliwei01/ZONE/TestProject/temp/01_Rosbag',
                'gt': '/home/zhangliwei01/ZONE/TestProject/temp/02_Annotation',
            },
            'workspace': '/home/zhangliwei01/ZONE/TestProject/ES37_PP_Feature_20240611/03_Workspace',
        },
        'bag_action': {
            'record': True,
            'bag_update': True,
            'compress': True,
            'parse': True,
            'get_gt': False,
        },
        'product': 'ES37',
        'test_type': 'pilot',
    }

    replay_controller = ReplayController(replay_config)
    replay_controller.run()
