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
from Envs.Master.Modules.PerceptMetrics import PreProcess, MatchTool


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


def sync_test_result(method):
    def wrapper(self, *args, **kwargs):
        method_start_time = time.time()
        self.load_test_result()
        result = method(self, *args, **kwargs)
        self.save_test_result()
        method_end_time = time.time()
        print(f'{self.__class__.__name__}.{method.__name__} '
              f'executed in {method_end_time - method_start_time:.2f} sec')
        return result

    return wrapper


class DataGrinderPilotOneCase:

    def __init__(self, scenario_unit_folder):
        # 变量初始化
        send_log(self, '=' * 50)

        # 同时要考虑文件文件路径发生变化, 需要修改yaml中的所有相关路径
        scenario_config_yaml = os.path.join(scenario_unit_folder, 'TestConfig.yaml')
        with open(scenario_config_yaml, 'r', encoding='utf-8') as file:
            scenario_test_config = yaml.safe_load(file)
        test_project_root_path = get_test_project_root_path(scenario_unit_folder)
        self.test_config = replace_path_in_dict(
            scenario_test_config,
            scenario_test_config['root_path'], test_project_root_path
        )
        self.scenario_unit_folder = scenario_unit_folder

        # 加载测试相关的参数
        self.scenario_id = self.test_config['scenario_id']
        send_log(self, f'{self.scenario_id}开始数据处理')
        print('=' * 25 + self.scenario_id + '=' * 25)
        self.product = self.test_config['product']
        self.version = self.test_config['version']
        self.test_info = {
            'title': '{:s}-{:s}'.format(self.product, self.version),
            'date': time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime(time.time()))
        }

        self.scenario_tag = self.test_config['scenario_tag']
        self.test_action = self.test_config['test_action']
        self.test_encyclopaedia = test_encyclopaedia[self.product]
        self.test_item = self.test_config['test_item']
        self.topics_for_evaluation = self.test_item.keys()

        # 建立文件夹
        self.pred_raw_folder = self.test_config['data_folder']['raw']['pred']
        self.GT_raw_folder = self.test_config['data_folder']['raw']['GT']
        self.DataFolder = os.path.join(self.scenario_unit_folder, '01_Data')

        # 初始化测试配置
        self.test_result_info_path = os.path.join(self.scenario_unit_folder, 'TestResultInfo.yaml')
        if not os.path.exists(self.test_result_info_path):
            self.test_result = {'General': {}}
            for topic in self.topics_for_evaluation:
                attribution = get_topic_attribution(topic)
                if not attribution:
                    send_log(self, f'不存在{topic}对应的分类')
                    continue

                topic_belonging = attribution['topic_belonging']
                self.test_result[topic_belonging] = {topic: {} for topic in self.topics_for_evaluation}
                self.test_result[topic_belonging]['GroundTruth'] = {}

            self.save_test_result()

        test_encyclopaedia_yaml = os.path.join(self.scenario_unit_folder, 'TestEncyclopaedia.yaml')
        with open(test_encyclopaedia_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_encyclopaedia,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    @sync_test_result
    def load_pred_data(self):
        test_topic_info_path = os.path.join(self.pred_raw_folder, 'RawData', 'TestTopicInfo.yaml')
        if not os.path.exists(test_topic_info_path):
            send_log(self, 'No TestTopicInfo.yaml is found. Please check')
            return

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
                topic_belonging = attribution['topic_belonging']

                # 读取原始数据
                csv_list = glob.glob(os.path.join(self.pred_raw_folder, 'RawData', f'{topic_tag}*.csv'))
                csv_data = []
                for csv_file in sorted(csv_list, reverse=False):
                    if 'hz' not in csv_file:
                        csv_data.append(pd.read_csv(csv_file, index_col=False))
                pred_data = pd.concat(csv_data).sort_values(by=['time_stamp'])[raw_column]

                # 读取时间辍, 用于时间辍匹配
                pred_timestamp_csv = glob.glob(os.path.join(self.pred_raw_folder, 'RawData', f'{topic_tag}*hz.csv'))[0]
                pred_timestamp = pd.read_csv(pred_timestamp_csv, index_col=False).sort_values(
                    by=['time_stamp']).drop_duplicates(subset=['time_stamp'], keep='first')
                pred_hz = float(os.path.basename(pred_timestamp_csv).split('_')[1])

                raw_folder = os.path.join(self.DataFolder, topic_belonging, topic_tag, 'raw')
                create_folder(raw_folder, False)
                self.test_result[topic_belonging][topic] = {
                    'frequency': pred_hz, 'raw': {},
                }

                path = os.path.join(raw_folder, 'pred_data.csv')
                pred_data.to_csv(path, index=False)
                self.test_result[topic_belonging][topic]['raw']['pred_data'] = self.get_relpath(path)

                path = os.path.join(raw_folder, 'pred_timestamp.csv')
                pred_timestamp.to_csv(path, index=False)
                self.test_result[topic_belonging][topic]['raw']['pred_timestamp'] = self.get_relpath(path)

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
                self.test_result['General']['pred_ego'] = self.get_relpath(path)

        # 复制场景相关的信息, 解析相机参数
        scenario_info_folder = os.path.join(self.pred_raw_folder, 'scenario_info')
        if copy_to_destination(scenario_info_folder, self.scenario_unit_folder):

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

    @sync_test_result
    def load_gt_data(self):
        gt_config_path = os.path.join(self.GT_raw_folder, 'yaml_management.yaml')
        if not os.path.exists(gt_config_path):
            print('No yaml_management.yaml is found. Please check')
            return

        with open(gt_config_path) as f:
            gt_config = yaml.load(f, Loader=yaml.FullLoader)

        gt_topic_mapping = {
            'od_csv': test_encyclopaedia['Information']['Obstacles']['topics'],
            'object_csv': test_encyclopaedia['Information']['Objects']['topics'],
            'lanebev_csv': test_encyclopaedia['Information']['Lines']['topics'],
        }

        gt_topic_belongings = {
            'od_csv': 'Obstacles',
            'object_csv': 'Objects',
            'lanebev_csv': 'Lines',
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

            if not (flag in gt_config.keys() and gt_config[flag]):
                continue

            send_log(self, f'GroundTruth 正在读取{gt_topic}')

            # 读取原始数据
            csv_data = []
            for csv_file in gt_config['{:s}_list'.format(gt_topic)]:
                csv_path = os.path.join(self.GT_raw_folder, gt_topic, csv_file)
                csv_data.append(pd.read_csv(csv_path, index_col=False))
            gt_data = pd.concat(csv_data).rename(columns={
                'timestamp': 'time_stamp',
                'od_json_sequence': 'frame_id',
                'subtype': 'sub_type',
                'type_conf_3d': 'confidence',
            }).sort_values(by=['time_stamp'])

            if gt_topic == 'od_csv':
                gt_data['vx_rel'] = gt_data['vx'] - gt_data['ego_v']
                gt_data['vy_rel'] = gt_data['vy']

            # 读取时间辍, 用于时间辍匹配
            gt_timestamp_csv = \
                glob.glob(os.path.join(self.GT_raw_folder, f'{timestamp_csv_tag}*hz.csv'))[0]
            gt_timestamp = pd.read_csv(gt_timestamp_csv).sort_values(
                by=['timestamp']).rename(columns={'timestamp': 'time_stamp'}) \
                .drop_duplicates(subset=['time_stamp'], keep='first')
            gt_hz = float(os.path.basename(gt_timestamp_csv).split('_')[2])

            gt_topic_belonging = gt_topic_belongings[gt_topic]
            send_log(self, f'GroundTruth 保存数据于{gt_topic_belonging}')

            attribution = get_topic_attribution(topic_list[0])
            raw_column = attribution['raw_column']

            raw_folder = os.path.join(self.DataFolder, gt_topic_belonging, 'GroundTruth', 'raw')
            create_folder(raw_folder, False)
            self.test_result[gt_topic_belonging]['GroundTruth'] = {
                'frequency': gt_hz, 'raw': {},
            }

            path = os.path.join(raw_folder, 'gt_data.csv')
            gt_data[raw_column].to_csv(path, index=False)
            self.test_result[gt_topic_belonging]['GroundTruth']['raw']['gt_data'] = self.get_relpath(path)

            path = os.path.join(raw_folder, 'gt_timestamp.csv')
            gt_timestamp.to_csv(path, index=False)
            self.test_result[gt_topic_belonging]['GroundTruth']['raw']['gt_timestamp'] = self.get_relpath(path)

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
            self.test_result['General']['gt_ego'] = self.get_relpath(path)

    @sync_test_result
    def sync_timestamp(self):

        baseline_data = pd.read_csv(
            self.get_abspath(self.test_result['General']['gt_ego']), index_col=False)
        baseline_time_series = baseline_data['time_stamp'].to_list()
        baseline_velocity_series = baseline_data['ego_vx'].to_list()

        calibrated_data = pd.read_csv(
            self.get_abspath(self.test_result['General']['pred_ego']), index_col=False)
        calibrated_time_series = calibrated_data['time_stamp'].to_list()
        calibrated_velocity_series = calibrated_data['ego_vx'].to_list()

        t_delta, v_error = PreProcess.calculate_time_gap(
            baseline_time_series=baseline_time_series,
            baseline_velocity_series=baseline_velocity_series,
            calibrated_time_series=calibrated_time_series,
            calibrated_velocity_series=calibrated_velocity_series
        )
        send_log(self, f'最佳时间间隔 = {t_delta}, 平均速度误差 = {v_error}')
        self.test_result['General']['time_gap'] = float(t_delta)

        time_start = max(min(calibrated_time_series) + t_delta, min(baseline_time_series)) + 1
        time_end = min(max(calibrated_time_series) + t_delta, max(baseline_time_series)) - 1
        send_log(self, f'测试指标的数据时间段为{time_start}-{time_end}')
        self.test_result['General']['time_start'] = float(time_start)
        self.test_result['General']['time_end'] = float(time_end)

        # 将同步后的自车速度保存, 后续插值车速需要
        calibrated_data['time_stamp'] += t_delta
        sync_ego_data = calibrated_data[(calibrated_data['time_stamp'] <= time_end + 1)
                                        & (calibrated_data['time_stamp'] >= time_start - 1)]
        path = os.path.join(self.DataFolder, 'General', 'SyncEgoVx.csv')
        sync_ego_data.to_csv(path, index=False)
        self.test_result['General']['sync_ego_data'] = self.get_relpath(path)

        # 将同步后的自车速度可视化
        fig = plt.figure(figsize=(10, 5.625))
        fig.tight_layout()
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
        grid = plt.GridSpec(1, 1, wspace=0.2, hspace=0.25)
        ax = fig.add_subplot(grid[0, 0])

        ax.plot(calibrated_data['time_stamp'],
                calibrated_data['ego_vx'] * 3.6,
                color='red', linestyle='dashed', linewidth=2,
                label=f'ego_speed of {self.product}')
        ax.plot(baseline_data['time_stamp'],
                baseline_data['ego_vx'] * 3.6,
                color='green', linestyle='dashed', linewidth=2,
                label=f'ego_speed of GroundTruth')
        ax.set_title(f'{self.scenario_id}  time_gap = {t_delta} sec', fontsize=font_size * 1.3)
        ax.set_xlabel('time[second]', fontdict=axis_font)
        ax.set_ylabel('ego_vx[km/h]', fontdict=axis_font)
        ax.set_xlim(time_start, time_end)
        ax.set_ylim(0, 140)
        ax.set_yticks(np.arange(0, 141, 20))
        ax.grid('--', color='gainsboro')
        ax.tick_params(direction='out', labelsize=font_size / 1.1, length=2)
        ax.legend(loc=0, prop=legend_font)

        path = os.path.join(self.DataFolder, 'General', 'SyncEgoVx.png')
        canvas = FigureCanvas(fig)
        canvas.print_figure(path, facecolor='white', dpi=100)
        self.test_result['General']['sync_ego_figure'] = self.get_relpath(path)
        fig.clf()
        plt.close()

    @sync_test_result
    def promote_additional(self):
        time_gap = self.test_result['General']['time_gap']
        time_start = self.test_result['General']['time_start']
        time_end = self.test_result['General']['time_end']

        for topic_belonging in self.test_result.keys():
            if topic_belonging == 'General':
                continue

            additional_column = (test_encyclopaedia['Information'][topic_belonging]['raw_column']
                                 + test_encyclopaedia['Information'][topic_belonging]['additional_column'])
            send_log(self, f'{topic_belonging}, 使用{topic_belonging}Preprocess')
            preprocess_ins = eval(f'PreProcess.{topic_belonging}Preprocess()')

            for topic in self.test_result[topic_belonging].keys():
                if topic != 'GroundTruth':
                    raw = self.test_result[topic_belonging][topic]['raw']
                    topic_tag = topic.replace('/', '')
                    additional_folder = os.path.join(self.DataFolder, topic_belonging, topic_tag, 'additional')
                    create_folder(additional_folder)
                    if 'additional' not in self.test_result[topic_belonging][topic]:
                        self.test_result[topic_belonging][topic]['additional'] = {}

                    # 时间辍补齐
                    send_log(self, f'{topic_belonging} {topic} 时间辍同步')
                    data = pd.read_csv(self.get_abspath(raw['pred_timestamp']), index_col=False)
                    data['time_stamp'] += time_gap
                    data = data[(data['time_stamp'] <= time_end) & (data['time_stamp'] >= time_start)]
                    path = os.path.join(additional_folder, 'pred_timestamp.csv')
                    self.test_result[topic_belonging][topic]['additional']['pred_timestamp'] = self.get_relpath(path)
                    data.to_csv(path, index=False)

                    # 预处理原始数据, 增加列
                    send_log(self, f'{topic_belonging} {topic} 预处理步骤 {preprocess_ins.preprocess_types}')
                    data = pd.read_csv(self.get_abspath(raw['pred_data']), index_col=False)
                    data['time_stamp'] += time_gap
                    data = data[(data['time_stamp'] <= time_end) & (data['time_stamp'] >= time_start)]

                    input_parameter_container = {
                        'coverage_reference_point': self.test_config['coverage_reference_point'],
                        'coverage_threshold': self.test_config['coverage_threshold'],
                        'ROI': self.test_config['pred_ROI'][topic],
                        'lane_width': 3.6,
                        'moving_threshold': 2,
                        'key_coverage_threshold': 0.1,
                    }
                    data = preprocess_ins.run(data, input_parameter_container)[additional_column]
                    path = os.path.join(additional_folder, 'pred_data.csv')
                    self.test_result[topic_belonging][topic]['additional']['pred_data'] = self.get_relpath(path)
                    data.to_csv(path, index=False)

                else:
                    raw = self.test_result[topic_belonging]['GroundTruth']['raw']
                    additional_folder = os.path.join(self.DataFolder, topic_belonging, 'GroundTruth', 'additional')
                    create_folder(additional_folder)
                    if 'additional' not in self.test_result[topic_belonging]['GroundTruth']:
                        self.test_result[topic_belonging]['GroundTruth']['additional'] = {}

                    # 时间辍补齐
                    send_log(self, f'{topic_belonging} GroundTruth 时间辍同步')
                    data = pd.read_csv(self.get_abspath(raw['gt_timestamp']), index_col=False)
                    data = data[(data['time_stamp'] <= time_end) & (data['time_stamp'] >= time_start)]
                    path = os.path.join(additional_folder, 'gt_timestamp.csv')
                    self.test_result[topic_belonging]['GroundTruth']['additional']['gt_timestamp'] = self.get_relpath(path)
                    data.to_csv(path, index=False)

                    # 预处理原始数据, 增加列
                    send_log(self, f'{topic_belonging} GroundTruth 预处理步骤 {preprocess_ins.preprocess_types}')
                    data = pd.read_csv(self.get_abspath(raw['gt_data']), index_col=False)
                    data = data[(data['time_stamp'] <= time_end) & (data['time_stamp'] >= time_start)]

                    input_parameter_container = {
                        'coverage_reference_point': self.test_config['coverage_reference_point'],
                        'coverage_threshold': self.test_config['coverage_threshold'],
                        'ROI': self.test_config['gt_ROI'],
                        'lane_width': 3.6,
                        'moving_threshold': 2,
                        'key_coverage_threshold': 0.1,
                    }
                    data = preprocess_ins.run(data, input_parameter_container)[additional_column]

                    path = os.path.join(additional_folder, 'gt_data.csv')
                    self.test_result[topic_belonging]['GroundTruth']['additional']['gt_data'] = self.get_relpath(path)
                    data.to_csv(path, index=False)

    @sync_test_result
    def match_timestamp(self):
        for topic_belonging in self.test_result.keys():
            if topic_belonging == 'General':
                continue

            gt_timestamp_path = self.get_abspath(self.test_result[topic_belonging]['GroundTruth']['additional']['gt_timestamp'])
            gt_timestamp = pd.read_csv(gt_timestamp_path, index_col=False)['time_stamp'].to_list()
            gt_hz = self.test_result[topic_belonging]['GroundTruth']['frequency']

            for topic in self.test_result[topic_belonging].keys():
                if topic == 'GroundTruth':
                    continue

                additional = self.test_result[topic_belonging][topic]['additional']
                topic_tag = topic.replace('/', '')
                match_folder = os.path.join(self.DataFolder, topic_belonging, topic_tag, 'match')
                create_folder(match_folder)
                if 'match' not in self.test_result[topic_belonging][topic]:
                    self.test_result[topic_belonging][topic]['match'] = {}

                pred_timestamp_path = self.get_abspath(additional['pred_timestamp'])
                pred_timestamp = pd.read_csv(pred_timestamp_path, index_col=False)['time_stamp'].to_list()
                pred_hz = self.test_result[topic_belonging][topic]['frequency']

                match_tolerance = self.test_config['timestamp_matching_tolerance'] / max(pred_hz, gt_hz)
                send_log(self, f'{topic_belonging} {topic} 时间差低于{match_tolerance} sec的尝试匹配, 进一步选择局部最优')

                match_pred_timestamp, match_gt_timestamp = MatchTool.match_timestamp(
                    pred_timestamp, gt_timestamp, match_tolerance)
                send_log(self, f'{topic_belonging} {topic} Prediction 时间戳总计{len(pred_timestamp)}个, 对齐{len(match_pred_timestamp)}')
                send_log(self, f'{topic_belonging} {topic} GroundTruth 时间戳总计{len(gt_timestamp)}个, 对齐{len(match_gt_timestamp)}')

                # 保存时间辍匹配数据
                match_timestamp_data = pd.DataFrame(columns=['gt_timestamp', 'pred_timestamp', 'match_gap'])
                if len(match_pred_timestamp):
                    match_timestamp_data['gt_timestamp'] = match_gt_timestamp
                    match_timestamp_data['pred_timestamp'] = match_pred_timestamp
                    match_timestamp_data['match_gap'] = match_timestamp_data['pred_timestamp'] - match_timestamp_data[
                        'gt_timestamp']

                path = os.path.join(match_folder, 'match_timestamp.csv')
                match_timestamp_data.to_csv(path, index=False)
                self.test_result[topic_belonging][topic]['match']['match_timestamp'] = self.get_relpath(path)

    @sync_test_result
    def match_object(self):
        for topic in self.test_result['Topics'].keys():
            additional = self.test_result['Topics'][topic]['additional']
            topic_tag = topic.replace('/', '')
            match_timestamp_path = self.test_result['Topics'][topic]['match']['match_timestamp']
            match_timestamp = pd.read_csv(self.get_abspath(match_timestamp_path), index_col=False)

            if not len(match_timestamp):
                send_log(self, f'{topic} 不存在匹配的时间戳, 请检查')
                continue

            attribution = get_topic_attribution(topic)
            topic_belonging = attribution['topic_belonging']
            additional_column = attribution['additional_column']

    def start(self):

        if self.test_action['preprocess']:
            self.load_pred_data()
            self.load_gt_data()
            self.sync_timestamp()
            self.promote_additional()

        if self.test_action['match']:
            self.match_timestamp()
        #     self.match_object()

    def get_relpath(self, path: str) -> str:
        return os.path.relpath(path, self.scenario_unit_folder)

    def get_abspath(self, path: str) -> str:
        return os.path.join(self.scenario_unit_folder, path)

    def save_test_result(self):
        with open(self.test_result_info_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_result,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    def load_test_result(self):
        with open(self.test_result_info_path) as f:
            self.test_result = yaml.load(f, Loader=yaml.FullLoader)
