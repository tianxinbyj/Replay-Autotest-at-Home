"""
@Author: BU YUJUN
@Date: 2024/7/8 上午10:00  
"""
import glob
import os
import sys
import time

import pandas as pd
import numpy as np
import yaml
from matplotlib import pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Libs import get_project_path, replace_path_in_dict, copy_to_destination

sys.path.append(get_project_path())

from Utils.Libs import test_encyclopaedia, create_folder
from Utils.Libs import font_size, title_font, axis_font, axis_font_white, text_font, legend_font, mpl_colors
from Utils.Logger import send_log

# 导入评测api
#1. 预处理Api
from Envs.Master.Modules.PerceptMetrics import PreProcess


def get_test_project_root_path(path):
    while True:
        if os.path.exists(os.path.join(path, '04_TestData')):
            return path
        path = os.path.dirname(path)


def get_topic_attribution(topic):
    for topic_belonging in test_encyclopaedia['Information']:
        if 'topics' in test_encyclopaedia['Information'][topic_belonging]:
            if topic in test_encyclopaedia['Information'][topic_belonging]['topics']:
                res = {
                    'topic_belonging': topic_belonging,
                    'raw_column': test_encyclopaedia['Information'][topic_belonging]['raw_column'],
                    'additional_column': test_encyclopaedia['Information'][topic_belonging]['raw_column'] +
                                         test_encyclopaedia['Information'][topic_belonging]['additional_column']
                }
                return res

    return None


class DataGrinderPilotOneCase:

    def __init__(self, scenario_unit_path):
        # 变量初始化
        send_log(self, '=' * 50)

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
        self.test_info = {
            'title': '{:s}-{:s}'.format(self.product, self.version),
            'date': time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime(time.time()))
        }

        self.scenario_tag = self.test_config['scenario_tag']
        self.test_encyclopaedia = test_encyclopaedia[self.product]
        self.test_item = self.test_config['test_item']
        self.topics_for_evaluation = self.test_item.keys()
        self.coverage_reference_point = self.test_config['coverage_reference_point']
        self.coverage_threshold = self.test_config['coverage_threshold']

        # 设置数据预处理类的变量池
        PreProcess.parameter_container['coverage_reference_point'] = self.coverage_reference_point
        PreProcess.parameter_container['coverage_threshold'] = self.coverage_threshold

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
                'frequency': {}, 'raw': {}, 'additional': {},
            } for topic in self.topics_for_evaluation}}

            self.save_test_result()

        test_encyclopaedia_yaml = os.path.join(self.unit_folder, 'TestEncyclopaedia.yaml')
        with open(test_encyclopaedia_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_encyclopaedia,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    def load_pred_data(self):
        test_topic_info_path = os.path.join(self.pred_raw_folder, 'RawData', 'TestTopicInfo.yaml')
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

                attribution = get_topic_attribution(topic)
                if not attribution:
                    send_log(self, f'Prediction 不存在{topic}对应的raw_column')
                    continue
                raw_column = attribution['raw_column']

                # 读取原始数据
                csv_list = glob.glob(os.path.join(self.pred_raw_folder, 'RawData', f'{topic_tag}*.csv'))
                csv_data = []
                for csv_file in sorted(csv_list, reverse=False):
                    if 'hz' not in csv_file:
                        csv_data.append(pd.read_csv(csv_file, index_col=False))
                pred_data = pd.concat(csv_data).sort_values(by=['time_stamp'])[raw_column]

                # 读取时间辍，用于时间辍匹配
                pred_timestamp_csv = glob.glob(os.path.join(self.pred_raw_folder, 'RawData', f'{topic_tag}*hz.csv'))[0]
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
                self.test_result['Topics'][topic]['frequency']['pred_hz'] = float(
                    os.path.basename(pred_timestamp_csv).split('_')[1])

            elif topic == '/PI/EG/EgoMotionInfo':
                send_log(self, f'Prediction 正在读取{topic}, 用于时间同步')

                # 读取原始数据
                csv_list = glob.glob(os.path.join(self.pred_raw_folder, 'RawData', f'{topic_tag}*.csv'))
                csv_data = []
                for csv_file in sorted(csv_list, reverse=False):
                    if 'hz' not in csv_file:
                        csv_data.append(pd.read_csv(csv_file, index_col=False))
                pred_data = pd.concat(csv_data).sort_values(by=['time_stamp']).iloc[50:]
                create_folder(os.path.join(self.DataFolder, 'General'), update=False)
                path = os.path.join(self.DataFolder, 'General', 'pred_ego.csv')
                pred_data[['time_stamp', 'ego_vx']].to_csv(path, index=False)
                self.test_result['General']['pred_ego'] = path

        # 复制场景相关的信息, 解析相机参数
        scenario_info_folder = os.path.join(self.pred_raw_folder, 'scenario_info')
        if copy_to_destination(scenario_info_folder, self.unit_folder):

            self.test_result['General']['camera_position'] = {}
            yaml_folder = os.path.join(scenario_info_folder, 'yaml_calib')
            cam_description_path = os.path.join(yaml_folder, 'cam_description.yaml')

            with open(cam_description_path, 'r', encoding='utf-8') as f:
                cam_description = yaml.safe_load(f)

            for i, cam_name in cam_description.items():
                cam_par_path = os.path.join(yaml_folder, f'camera_{i}.yaml')
                with open(cam_par_path, 'r', encoding='utf-8') as f:
                    cam_par = yaml.safe_load(f)
                    x, y, z = [
                        float(cam_par[f'cam_{i}_pos_x']),
                        float(cam_par[f'cam_{i}_pos_y']),
                        float(cam_par[f'cam_{i}_pos_z']),
                    ]
                self.test_result['General']['camera_position'][cam_name] = [x, y, z]
                send_log(self, f'{cam_name} 位于({x}, {y}, {z})')

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
            'od_csv': test_encyclopaedia['Information']['Obstacles']['topics'],
            'object_csv': test_encyclopaedia['Information']['Objects']['topics'],
            'lanebev_csv': test_encyclopaedia['Information']['Lines']['topics'],
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

                    attribution = get_topic_attribution(topic)
                    if not attribution:
                        send_log(self, f'GroundTruth 不存在{topic}对应的raw_column')
                        continue
                    raw_column = attribution['raw_column']

                    topic_tag = topic.replace('/', '')
                    raw_folder = os.path.join(self.DataFolder, topic_tag, 'raw', 'gt')
                    create_folder(raw_folder)

                    path = os.path.join(raw_folder, 'gt_data.csv')
                    gt_data[raw_column].to_csv(path, index=False)
                    self.test_result['Topics'][topic]['raw']['gt_data'] = path

                    path = os.path.join(raw_folder, 'gt_timestamp.csv')
                    gt_timestamp.to_csv(path, index=False)
                    self.test_result['Topics'][topic]['raw']['gt_timestamp'] = path
                    self.test_result['Topics'][topic]['frequency']['gt_hz'] = gt_hz

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

        start_time = max(min(calibrated_time_series) + t_delta, min(baseline_time_series)) + 1
        end_time = min(max(calibrated_time_series) + t_delta, max(baseline_time_series)) - 1
        send_log(self, f'测试指标的数据时间段为{start_time}-{end_time}')
        self.test_result['General']['start_time'] = float(start_time)
        self.test_result['General']['end_time'] = float(end_time)

        # 将同步后的自车速度可视化
        fig = plt.figure(figsize=(10, 5.625))
        fig.tight_layout()
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
        grid = plt.GridSpec(1, 1, wspace=0.2, hspace=0.25)
        ax = fig.add_subplot(grid[0, 0])

        ax.plot(calibrated_data['time_stamp'].values + t_delta,
                calibrated_data['ego_vx'].values * 3.6,
                color='red', linestyle='dashed', linewidth=2,
                label=f'ego_speed of {self.product}')
        ax.plot(baseline_data['time_stamp'].values,
                baseline_data['ego_vx'].values * 3.6,
                color='green', linestyle='dashed', linewidth=2,
                label=f'ego_speed of GroundTruth')
        ax.set_title(f'{self.scenario_id}  time_gap = {t_delta} sec', fontsize=font_size * 1.3)
        ax.set_xlabel('time[second]', fontdict=axis_font)
        ax.set_ylabel('ego_vx[km/h]', fontdict=axis_font)
        ax.set_xlim(start_time, end_time)
        ax.set_ylim(0, 140)
        ax.set_yticks(np.arange(0, 141, 20))
        ax.grid('--', color='gainsboro')
        ax.tick_params(direction='out', labelsize=font_size / 1.1, length=2)
        ax.legend(loc=0, prop=legend_font)

        path = os.path.join(self.DataFolder, 'General', 'SyncEgoVx.png')
        canvas = FigureCanvas(fig)
        canvas.print_figure(path, facecolor='white', dpi=100)
        self.test_result['General']['sync_ego_vx'] = path
        fig.clf()
        plt.close()
        send_log(self, f'{path} 保存完毕')

        self.save_test_result()

    def promote_rawdata(self):
        self.load_test_result()
        time_gap = self.test_result['General']['time_gap']

        for topic in self.test_result['Topics'].keys():
            raw = self.test_result['Topics'][topic]['raw']
            topic_tag = topic.replace('/', '')
            additional_folder = os.path.join(self.DataFolder, topic_tag, 'additional')
            create_folder(additional_folder)

            attribution = get_topic_attribution(topic)
            topic_belonging = attribution['topic_belonging']
            additional_column = attribution['additional_column']

            reprocess_cls = eval(f'PreProcess.{topic_belonging}Preprocess')
            send_log(self, f'{topic} 归属于 {topic_belonging}, 使用{topic_belonging}Preprocess')

            for k, v in raw.items():
                if 'csv' in v:
                    data = pd.read_csv(v, index_col=False)

                    if 'pred' in k:
                        send_log(self, f'{topic} {k} 时间辍同步')
                        data['time_stamp'] += time_gap

                    if 'timestamp' not in k:
                        ins = reprocess_cls(data)
                        send_log(self, f'{topic} {k} 预处理步骤 {ins.preprocess_types}')
                        data = ins.data[additional_column]

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
