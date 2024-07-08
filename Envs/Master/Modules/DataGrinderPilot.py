"""
@Author: BU YUJUN
@Date: 2024/7/8 上午10:00  
"""
import glob
import os
import shutil
import sys
import time

import yaml
import pandas as pd

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Libs import get_project_path, replace_path_in_dict, copy_to_destination
sys.path.append(get_project_path())

from Utils.Libs import project_path, test_encyclopaedia, create_folder


class DataGrinderPilotOneCase:

    def __init__(self, scenario_unit_path):
        # 变量初始化


        # 同时要考虑文件文件路径发生变化，需要修改yaml中的所有相关路径
        scenario_config_yaml = os.path.join(scenario_unit_path, 'TestConfig.yaml')
        with open(scenario_config_yaml, 'r', encoding='utf-8') as file:
            scenario_test_config = yaml.safe_load(file)

        test_data_path = os.path.dirname(scenario_unit_path)
        while True:
            if os.path.exists(os.path.join(test_data_path, '04_TestData')):
                break
            test_data_path = os.path.dirname(test_data_path)

        self.test_config = replace_path_in_dict(
            scenario_test_config, scenario_test_config['root_path'], test_data_path
        )
        self.test_config['root_path'] = test_data_path

        # 加载测试相关的参数
        self.scenario_id = self.test_config['scenario_id']
        print(f'========== {self.scenario_id} ==========')
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
            self.test_result = {topic: {
                'raw': {}, 'clean': {},
            } for topic in self.topics_for_evaluation}
            self.test_result['General'] = {}
            self.save_test_result()

        test_encyclopaedia_yaml = os.path.join(self.unit_folder, 'TestEncyclopaedia.yaml')
        with open(test_encyclopaedia_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_encyclopaedia,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    def load_pred_data(self):
        test_topic_info_path = os.path.join(self.pred_raw_folder, 'TestTopicInfo.yaml')
        if not os.path.exists(test_topic_info_path):
            print('No TestTopicInfo.yaml is found. Please check')
            return

        self.load_test_result()
        with open(test_topic_info_path) as f:
            parsed_topic = yaml.load(f, Loader=yaml.FullLoader)['topics_for_parser']

        # 初始化感知数据
        for topic in parsed_topic:
            print('Prediction 正在读取 {:s}'.format(topic))
            topic_tag = topic.replace('/', '')

            if topic in self.topics_for_evaluation:

                # 读取原始数据
                csv_list = glob.glob(os.path.join(self.pred_raw_folder, f'{topic_tag}*.csv'))
                csv_data = []
                for csv_file in sorted(csv_list, reverse=False):
                    if 'hz' not in csv_file:
                        csv_data.append(pd.read_csv(csv_file, index_col=False))
                pred_data = pd.concat(csv_data).sort_values(by=['time_stamp'])

                # 读取时间辍，用于时间辍匹配
                pred_timestamp_csv = glob.glob(os.path.join(self.pred_raw_folder, f'{topic_tag}*hz.csv'))[0]
                pred_timestamp = pd.read_csv(pred_timestamp_csv, index_col=False).sort_values(
                    by=['time_stamp']).drop_duplicates(subset=['time_stamp'], keep='first')

                raw_folder = os.path.join(self.DataFolder, topic_tag, 'raw')
                create_folder(raw_folder)

                path = os.path.join(raw_folder, 'pred_data.csv')
                pred_data.to_csv(path, index=False)
                self.test_result[topic]['raw']['pred_data'] = path

                path = os.path.join(raw_folder, 'pred_timestamp.csv')
                pred_timestamp.to_csv(path, index=False)
                self.test_result[topic]['raw']['pred_timestamp'] = path
                self.test_result[topic]['raw']['pred_hz'] = float(os.path.basename(pred_timestamp_csv).split('_')[1])

            elif topic == '/PI/EG/EgoMotionInfo':

                # 读取原始数据
                csv_list = glob.glob(os.path.join(self.pred_raw_folder, f'{topic_tag}*.csv'))
                csv_data = []
                for csv_file in sorted(csv_list, reverse=False):
                    if 'hz' not in csv_file:
                        csv_data.append(pd.read_csv(csv_file, index_col=False))
                pred_data = pd.concat(csv_data).sort_values(by=['time_stamp'])
                path = os.path.join(self.unit_folder, 'pred_ego.csv')
                pred_data[['time_stamp', 'ego_vx']].to_csv(path, index=False)
                self.test_result['General']['pred_ego'] = path

        # 复制场景相关的信息
        scenario_info_folder = os.path.join(self.pred_raw_folder, 'scenario_info')
        self.test_result['General']['scenario_info'] = copy_to_destination(scenario_info_folder, self.unit_folder)

        self.save_test_result()

    def load_gt_data(self):
        annotation_topics = {
            'od_csv': '/VA/Obstacles',
            'object_csv': '/VA/Objects',
            'lanebev_csv': '/VA/Lines',
        }

        gt_config_path = os.path.join(self.GT_raw_folder, 'yaml_management.yaml')
        if not os.path.exists(gt_config_path):
            print('No yaml_management.yaml is found. Please check')
            return

        with open(gt_config_path) as f:
            gt_config = yaml.load(f, Loader=yaml.FullLoader)
        self.annotation_data = {}
        self.annotation_timestamp = {}
        self.annotation_hz = {}
        print('Annotation Topics is', annotation_topics.values())
        for annotation_topic, topic in annotation_topics.items():
            flag = '{:s}_flag'.format(annotation_topic)
            timestamp_csv_tag = '{:s}_timestamp'.format(annotation_topic.split('_')[0])
            if flag in annotation_value_config.keys() and annotation_value_config[flag]:
                print('Annotation 正在读取 {:s}'.format(topic))
                csv_data = []
                for csv_file in annotation_value_config['{:s}_list'.format(annotation_topic)]:
                    path = os.path.join(self.GT_raw_folder, annotation_topic, csv_file)
                    csv_data.append(pd.read_csv(path, index_col=False))

                self.annotation_data[topic] = pd.concat(csv_data).sort_values(by=['timestamp'])

                annotation_timestamp_csv = \
                    glob.glob(os.path.join(self.GT_raw_folder, f'{timestamp_csv_tag}*hz.csv'))[0]
                self.annotation_timestamp[topic] = pd.read_csv(annotation_timestamp_csv).sort_values(
                    by=['timestamp']).rename(columns={'timestamp': 'time_stamp'}) \
                    .drop_duplicates(subset=['time_stamp'], keep='first')['time_stamp'].values

                self.annotation_hz[topic] = float(os.path.basename(annotation_timestamp_csv).split('_')[2])

                if topic == '/VA/Obstacles':
                    type_text_list = self.product_relevant[self.sensor_label]['enum']['Obstacles']['type']
                    sub_type_text_list = self.product_relevant[self.sensor_label]['enum']['Obstacles']['sub_type']
                    self.annotation_data[topic].rename(columns={'timestamp': 'time_stamp',
                                                                'od_json_sequence': 'frame_id',
                                                                'subtype': 'sub_type'}, inplace=True)
                    # 尝试筛掉对向车道的车,因为真值来源于激光雷达
                    self.annotation_data[topic]['visibility'] = self.annotation_data[topic].apply(
                        lambda d: 1 - d['cutoff%'] if d['vx'] >= -8.3 else 0.1, axis=1)
                    self.annotation_data[topic]['type_text'] = self.annotation_data[topic].apply(
                        lambda d: type_text_list[d['type']] if d['type'] in [2, 18]
                        else sub_type_text_list[d['sub_type']] if d['sub_type'] in sub_type_text_list
                        else 'vehicle', axis=1)

                    # 为标注数据增加id, 根据类型重新编号
                    self.annotation_data[topic]['id_with_type'] = self.annotation_data[topic].apply(
                        lambda d: d['id'] + np.power(2, 16) * d['type'], axis=1)
                    id_with_type = sorted(list(set(self.annotation_data[topic]['id_with_type'].values)))
                    self.annotation_data[topic]['id_for_evaluation'] = self.annotation_data[topic].apply(
                        lambda d: id_with_type.index(d['id_with_type']) + 1, axis=1)

                elif topic == '/VA/Lines':
                    type_text_list = self.product_relevant[self.sensor_label]['enum']['Lines']['position']
                    self.annotation_data[topic] = self.annotation_data[topic][
                        self.annotation_data[topic]['position'] != -1]
                    self.annotation_data[topic]['position_text'] = self.annotation_data[topic].apply(
                        lambda d: type_text_list[d['position']], axis=1)
                    # 真值标注没有赋予车道线id
                    self.annotation_data[topic]['id_for_evaluation'] = range(1, len(self.annotation_data[topic]) + 1)
                    self.annotation_data[topic]['type'] = 2
                    self.annotation_data[topic].rename(columns={'timestamp': 'time_stamp',
                                                                'lane_json_sequence': 'frame_id',
                                                                'width_sigma': 'width',
                                                                'c0': 'c_y_0',
                                                                'c1': 'c_y_1',
                                                                'c2': 'c_y_2',
                                                                'c3': 'c_y_3'}, inplace=True)

                elif topic == '/VA/Objects':
                    pass

        # 自车速度用于对齐
        self.annotation_sync_data = pd.read_csv(os.path.join(self.GT_raw_folder, 'od_ego.csv'))[
            ['ego_timestamp', 'INS_Speed']].rename(columns={'ego_timestamp': 'time_stamp', 'INS_Speed': 'ego_vx'})


    def save_test_result(self):
        with open(self.test_result_info_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_result,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    def load_test_result(self):
        with open(self.test_result_info_path) as f:
            self.test_result = yaml.load(f, Loader=yaml.FullLoader)