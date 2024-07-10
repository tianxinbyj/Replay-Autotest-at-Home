"""
@Author: BU YUJUN
@Date: 2024/7/8 上午10:00  
"""
import glob
import os
import sys
import time

import pandas as pd
import yaml

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Libs import get_project_path, replace_path_in_dict, copy_to_destination

sys.path.append(get_project_path())

from Utils.Libs import test_encyclopaedia, create_folder
from Utils.Logger import send_log

# 导入评测api
#1. 预处理Api
from Envs.Master.Modules.PerceptMetrics import PreProcess


def get_test_project_root_path(path):
    while True:
        if os.path.exists(os.path.join(path, '04_TestData')):
            return path
        path = os.path.dirname(path)


def get_raw_column(topic):
    if topic in [
        '/VA/Obstacles',
        '/VA/FrontWideObstacles2dDet',
        '/VA/BevObstaclesDet',
        '/PI/FS/ObjTracksHorizon'
    ]:
        raw_column = test_encyclopaedia['Information']['Obstacles']['raw_column']
    elif topic in [
        '/VA/Lines',
        'VA/FusLines',
        '/PI/FS/LaneMarkingsHorizonDebug',
        '/PI/FS/LaneMarkingsHorizon',
    ]:
        raw_column = test_encyclopaedia['Information']['Lines']['raw_column']
    elif topic in [
        '/VA/Objects'
    ]:
        raw_column = test_encyclopaedia['Information']['Objects']['raw_column']
    else:
        raw_column = None

    return raw_column


class DataGrinderPilotOneCase:

    def __init__(self, scenario_unit_path):
        # 变量初始化

        # 同时要考虑文件文件路径发生变化，需要修改yaml中的所有相关路径
        scenario_config_yaml = os.path.join(scenario_unit_path, 'TestConfig.yaml')
        with open(scenario_config_yaml, 'r', encoding='utf-8') as file:
            scenario_test_config = yaml.safe_load(file)
        test_project_root_path = get_test_project_root_path(scenario_unit_path)
        self.test_config = replace_path_in_dict(
            scenario_test_config,
            scenario_test_config['root_path'], test_project_root_path
        )

        # 加载测试相关的参数
        self.scenario_id = self.test_config['scenario_id']
        send_log(self, f'{self.scenario_id}开始数据处理')
        self.product = self.test_config['product']
        self.version = self.test_config['version']
        self.scenario_tag = self.test_config['scenario_tag']
        self.test_encyclopaedia = test_encyclopaedia[self.product]
        self.test_info = {
            'title': '{:s}-{:s}'.format(self.product, self.version),
            'date': time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime(time.time()))
        }
        self.test_item = self.test_config['test_item']
        self.topics_for_evaluation = self.test_item.keys()

        # 建立文件夹
        self.pred_raw_folder = self.test_config['data_folder']['raw']['pred']
        self.GT_raw_folder = self.test_config['data_folder']['raw']['GT']
        self.unit_folder = self.test_config['data_folder']['unit']
        self.workspace = self.test_config['workspace']
        self.DataFolder = os.path.join(self.unit_folder, '01_Data')

        # 初始化测试配置
        self.test_result_info_path = os.path.join(self.unit_folder, 'TestResultInfo.yaml')
        if not os.path.exists(self.test_result_info_path):
            self.test_result = {'General': {}, 'Topics': {topic: {
                'raw': {}, 'additional': {},
            } for topic in self.topics_for_evaluation}}

            self.save_test_result()

        test_encyclopaedia_yaml = os.path.join(self.unit_folder, 'TestEncyclopaedia.yaml')
        with open(test_encyclopaedia_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_encyclopaedia,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    def load_pred_data(self):
        test_topic_info_path = os.path.join(self.pred_raw_folder, 'TestTopicInfo.yaml')
        if not os.path.exists(test_topic_info_path):
            send_log(self, 'No TestTopicInfo.yaml is found. Please check')
            return

        self.load_test_result()
        with open(test_topic_info_path) as f:
            parsed_topic = yaml.load(f, Loader=yaml.FullLoader)['topics_for_parser']

        # 初始化感知数据
        for topic in parsed_topic:
            topic_tag = topic.replace('/', '')

            if topic in self.topics_for_evaluation:
                send_log(self, f'Prediction 正在读取{topic}')

                raw_column = get_raw_column(topic)
                if not raw_column:
                    continue

                # 读取原始数据
                csv_list = glob.glob(os.path.join(self.pred_raw_folder, f'{topic_tag}*.csv'))
                csv_data = []
                for csv_file in sorted(csv_list, reverse=False):
                    if 'hz' not in csv_file:
                        csv_data.append(pd.read_csv(csv_file, index_col=False))
                pred_data = pd.concat(csv_data).sort_values(by=['time_stamp'])[raw_column]

                # 读取时间辍，用于时间辍匹配
                pred_timestamp_csv = glob.glob(os.path.join(self.pred_raw_folder, f'{topic_tag}*hz.csv'))[0]
                pred_timestamp = pd.read_csv(pred_timestamp_csv, index_col=False).sort_values(
                    by=['time_stamp']).drop_duplicates(subset=['time_stamp'], keep='first')

                raw_folder = os.path.join(self.DataFolder, topic_tag, 'raw', 'pred')
                create_folder(raw_folder)

                path = os.path.join(raw_folder, 'pred_data.csv')
                pred_data.to_csv(path, index=False)
                self.test_result['Topics'][topic]['raw']['pred_data'] = path

                path = os.path.join(raw_folder, 'pred_timestamp.csv')
                pred_timestamp.to_csv(path, index=False)
                self.test_result['Topics'][topic]['raw']['pred_timestamp'] = path
                self.test_result['Topics'][topic]['raw']['pred_hz'] = float(os.path.basename(pred_timestamp_csv).split('_')[1])

            elif topic == '/PI/EG/EgoMotionInfo':
                send_log(self, f'Prediction 正在读取{topic}, 用于时间同步')

                # 读取原始数据
                csv_list = glob.glob(os.path.join(self.pred_raw_folder, f'{topic_tag}*.csv'))
                csv_data = []
                for csv_file in sorted(csv_list, reverse=False):
                    if 'hz' not in csv_file:
                        csv_data.append(pd.read_csv(csv_file, index_col=False))
                pred_data = pd.concat(csv_data).sort_values(by=['time_stamp'])
                create_folder(os.path.join(self.DataFolder, 'General'), update=False)
                path = os.path.join(self.DataFolder, 'General', 'pred_ego.csv')
                pred_data[['time_stamp', 'ego_vx']].to_csv(path, index=False)
                self.test_result['General']['pred_ego'] = path

        # 复制场景相关的信息
        scenario_info_folder = os.path.join(self.pred_raw_folder, 'scenario_info')
        self.test_result['General']['scenario_info'] = copy_to_destination(scenario_info_folder, self.unit_folder)

        self.save_test_result()

    def load_gt_data(self):
        gt_config_path = os.path.join(self.GT_raw_folder, 'yaml_management.yaml')
        if not os.path.exists(gt_config_path):
            print('No yaml_management.yaml is found. Please check')
            return

        self.load_test_result()
        with open(gt_config_path) as f:
            gt_config = yaml.load(f, Loader=yaml.FullLoader)

        gt_topic_mapping = {
            'od_csv': [
                '/VA/Obstacles',
                '/PI/FS/ObjTracksHorizon',
                '/VA/BevObstaclesDet',
                '/VA/FrontWideObstacles2dDet',
            ],
            'object_csv': ['/VA/Objects'],
            'lanebev_csv': [
                '/VA/Lines',
                'VA/FusLines',
                '/PI/FS/LaneMarkingsHorizonDebug',
                '/PI/FS/LaneMarkingsHorizon',
            ],
        }

        gt_topics = {}
        for topic in self.topics_for_evaluation:
            for gt_topic in gt_topic_mapping.keys():
                if topic in gt_topic_mapping[gt_topic]:
                    if gt_topic not in gt_topics:
                        gt_topics[gt_topic] = []
                    gt_topics[gt_topic].append(topic)

        for gt_topic, topic_list in gt_topics.items():
            flag = '{:s}_flag'.format(gt_topic)
            timestamp_csv_tag = '{:s}_timestamp'.format(gt_topic.split('_')[0])

            if flag in gt_config.keys() and gt_config[flag]:
                send_log(self, f'GroundTruth 正在读取{gt_topic}')

                # 读取原始数据
                csv_data = []
                for csv_file in gt_config['{:s}_list'.format(gt_topic)]:
                    path = os.path.join(self.GT_raw_folder, gt_topic, csv_file)
                    csv_data.append(pd.read_csv(path, index_col=False))
                gt_data = pd.concat(csv_data).rename(columns={
                    'timestamp': 'time_stamp',
                    'od_json_sequence': 'frame_id',
                    'subtype': 'sub_type',
                    'type_conf_3d': 'confidence',
                }).sort_values(by=['time_stamp'])

                if gt_topic == 'od_csv':
                    gt_data['vx_rel'] = gt_data['vx'] - gt_data['ego_v']
                    gt_data['vy_rel'] = gt_data['vy']

                # 读取时间辍，用于时间辍匹配
                gt_timestamp_csv = \
                    glob.glob(os.path.join(self.GT_raw_folder, f'{timestamp_csv_tag}*hz.csv'))[0]
                gt_timestamp = pd.read_csv(gt_timestamp_csv).sort_values(
                    by=['timestamp']).rename(columns={'timestamp': 'time_stamp'}) \
                    .drop_duplicates(subset=['time_stamp'], keep='first')
                gt_hz = float(os.path.basename(gt_timestamp_csv).split('_')[2])

                for topic in topic_list:
                    send_log(self, f'GroundTruth 保存数据于{topic}')
                    raw_column = get_raw_column(topic)

                    topic_tag = topic.replace('/', '')
                    raw_folder = os.path.join(self.DataFolder, topic_tag, 'raw', 'gt')
                    create_folder(raw_folder)

                    path = os.path.join(raw_folder, 'gt_data.csv')
                    gt_data[raw_column].to_csv(path, index=False)
                    self.test_result['Topics'][topic]['raw']['gt_data'] = path

                    path = os.path.join(raw_folder, 'gt_timestamp.csv')
                    gt_timestamp.to_csv(path, index=False)
                    self.test_result['Topics'][topic]['raw']['gt_timestamp'] = path
                    self.test_result['Topics'][topic]['raw']['gt_hz'] = gt_hz

        # 自车速度用于对齐
        ego_data_path = os.path.join(self.GT_raw_folder, 'od_ego.csv')
        if not os.path.exists(ego_data_path):
            send_log(self, 'GroundTruth未找到自车速度数据')
        else:
            send_log(self, f'GroundTruth 正在读取{os.path.basename(ego_data_path)}, 用于时间同步')
            ego_data = pd.read_csv(os.path.join(self.GT_raw_folder, 'od_ego.csv'))[
                ['ego_timestamp', 'INS_Speed']].rename(
                columns={'ego_timestamp': 'time_stamp', 'INS_Speed': 'ego_vx'})
            create_folder(os.path.join(self.DataFolder, 'General'), update=False)
            path = os.path.join(self.DataFolder, 'General', 'gt_ego.csv')
            ego_data.to_csv(path, index=False)
            self.test_result['General']['gt_ego'] = path

        self.save_test_result()

    def sync_timestamp(self):
        self.load_test_result()
        t0 = time.time()

        baseline_data = pd.read_csv(self.test_result['General']['gt_ego'], index_col=False)
        baseline_time_series = baseline_data['time_stamp'].to_list()
        baseline_velocity_series = baseline_data['ego_vx'].to_list()

        calibrated_data = pd.read_csv(self.test_result['General']['pred_ego'], index_col=False)
        calibrated_time_series = calibrated_data['time_stamp'].to_list()
        calibrated_velocity_series = calibrated_data['ego_vx'].to_list()

        t_delta, v_error = PreProcess.calculate_time_gap(
            baseline_time_series=baseline_time_series,
            baseline_velocity_series=baseline_velocity_series,
            calibrated_time_series=calibrated_time_series,
            calibrated_velocity_series=calibrated_velocity_series
        )
        send_log(self, f'时间辍同步耗时{round(time.time() - t0, 2)} sec, '
                       f'最佳时间间隔 = {t_delta}, 平均速度误差 = {v_error}')
        self.test_result['General']['time_gap'] = float(t_delta)

        self.save_test_result()

    def promote_rawdata(self):
        self.load_test_result()
        time_gap = self.test_result['General']['time_gap']

        for topic in self.test_result['Topics'].keys():
            raw = self.test_result['Topics'][topic]['raw']
            topic_tag = topic.replace('/', '')
            additional_folder = os.path.join(self.DataFolder, topic_tag, 'additional')
            create_folder(additional_folder)

            for k, v in raw.items():
                if isinstance(v, float):
                    self.test_result['Topics'][topic]['additional'][k] = v
                elif 'csv' in v:
                    data = pd.read_csv(v, index_col=False)

                    if 'pred' in k:
                        send_log(self, f'{topic} 时间辍同步')
                        data['time_stamp'] += time_gap

                    path = os.path.join(additional_folder, os.path.basename(v))
                    self.test_result['Topics'][topic]['additional'][k] = path
                    data.to_csv(path, index=False)

        self.save_test_result()

    def save_test_result(self):
        with open(self.test_result_info_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_result,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    def load_test_result(self):
        with open(self.test_result_info_path) as f:
            self.test_result = yaml.load(f, Loader=yaml.FullLoader)
