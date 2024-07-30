"""
@Author: BU YUJUN
@Date: 2024/7/8 上午10:00  
"""
import glob
import json
import os
import shutil
import sys
import time

import pandas as pd
import numpy as np
import yaml
from matplotlib import pyplot as plt
from matplotlib import patches as pc
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Libs import get_project_path, copy_to_destination

sys.path.append(get_project_path())

from Utils.Libs import test_encyclopaedia, create_folder, contains_chinese, get_string_display_length
from Utils.Libs import font_size, title_font, axis_font, axis_font_white, text_font, legend_font, mpl_colors
from Utils.Logger import send_log

# 导入评测api
from Envs.Master.Modules.PerceptMetrics.PerceptMetrics import PreProcess, MatchTool, MetricEvaluator, MetricStatistics


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
              f'-> {method_end_time - method_start_time:.2f} sec')
        return result

    return wrapper


class DataGrinderPilotOneCase:

    def __init__(self, scenario_unit_folder):
        print('=' * 25 + self.__class__.__name__ + '=' * 25)
        scenario_config_yaml = os.path.join(scenario_unit_folder, 'TestConfig.yaml')
        with open(scenario_config_yaml, 'r', encoding='utf-8') as file:
            self.test_config = yaml.safe_load(file)
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
        self.pred_raw_folder = self.test_config['pred_folder']
        self.gt_raw_folder = self.test_config['gt_folder']
        self.DataFolder = os.path.join(self.scenario_unit_folder, '01_Data')

        # 初始化测试配置
        self.test_result_yaml = os.path.join(self.scenario_unit_folder, 'TestResult.yaml')
        if not os.path.exists(self.test_result_yaml):
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
                pred_data.to_csv(path, index=False, encoding='utf_8_sig')
                self.test_result[topic_belonging][topic]['raw']['pred_data'] = self.get_relpath(path)

                path = os.path.join(raw_folder, 'pred_timestamp.csv')
                pred_timestamp.to_csv(path, index=False, encoding='utf_8_sig')
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
                pred_data[['time_stamp', 'ego_vx']].to_csv(path, index=False, encoding='utf_8_sig')
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

        new_scenario_info_folder = os.path.join(self.scenario_unit_folder, '00_ScenarioInfo')
        if os.path.exists(new_scenario_info_folder):
            shutil.rmtree(new_scenario_info_folder)
        os.rename(os.path.join(self.scenario_unit_folder, 'scenario_info'), new_scenario_info_folder)

    @sync_test_result
    def load_gt_data(self):
        gt_config_path = os.path.join(self.gt_raw_folder, 'yaml_management.yaml')
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
                csv_path = os.path.join(self.gt_raw_folder, gt_topic, csv_file)
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
                glob.glob(os.path.join(self.gt_raw_folder, f'{timestamp_csv_tag}*hz.csv'))[0]
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
            gt_data[raw_column].to_csv(path, index=False, encoding='utf_8_sig')
            self.test_result[gt_topic_belonging]['GroundTruth']['raw']['gt_data'] = self.get_relpath(path)

            path = os.path.join(raw_folder, 'gt_timestamp.csv')
            gt_timestamp.to_csv(path, index=False, encoding='utf_8_sig')
            self.test_result[gt_topic_belonging]['GroundTruth']['raw']['gt_timestamp'] = self.get_relpath(path)

        # 自车速度用于对齐
        ego_data_path = os.path.join(self.gt_raw_folder, 'od_ego.csv')
        if not os.path.exists(ego_data_path):
            send_log(self, 'GroundTruth未找到自车速度数据')
        else:
            send_log(self, f'GroundTruth 正在读取{os.path.basename(ego_data_path)}, 用于时间同步')
            ego_data = pd.read_csv(os.path.join(self.gt_raw_folder, 'od_ego.csv'))[
                ['ego_timestamp', 'INS_Speed']].rename(
                columns={'ego_timestamp': 'time_stamp', 'INS_Speed': 'ego_vx'})
            create_folder(os.path.join(self.DataFolder, 'General'), update=False)
            path = os.path.join(self.DataFolder, 'General', 'gt_ego.csv')
            ego_data.to_csv(path, index=False, encoding='utf_8_sig')
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
        sync_ego_data.to_csv(path, index=False, encoding='utf_8_sig')
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
            preprocess_instance = eval(f'PreProcess.{topic_belonging}Preprocess()')

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
                    data.to_csv(path, index=False, encoding='utf_8_sig')

                    # 预处理原始数据, 增加列
                    send_log(self, f'{topic_belonging} {topic} 预处理步骤 {preprocess_instance.preprocess_types}')
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
                    data = preprocess_instance.run(data, input_parameter_container)[additional_column]
                    path = os.path.join(additional_folder, 'pred_data.csv')
                    self.test_result[topic_belonging][topic]['additional']['pred_data'] = self.get_relpath(path)
                    data.to_csv(path, index=False, encoding='utf_8_sig')

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
                    self.test_result[topic_belonging]['GroundTruth']['additional']['gt_timestamp'] = self.get_relpath(
                        path)
                    data.to_csv(path, index=False, encoding='utf_8_sig')

                    # 预处理原始数据, 增加列
                    send_log(self, f'{topic_belonging} GroundTruth 预处理步骤 {preprocess_instance.preprocess_types}')
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
                    data = preprocess_instance.run(data, input_parameter_container)[additional_column]

                    path = os.path.join(additional_folder, 'gt_data.csv')
                    self.test_result[topic_belonging]['GroundTruth']['additional']['gt_data'] = self.get_relpath(path)
                    data.to_csv(path, index=False, encoding='utf_8_sig')

    @sync_test_result
    def match_timestamp(self):
        for topic_belonging in self.test_result.keys():
            if topic_belonging == 'General':
                continue

            gt_timestamp_path = self.get_abspath(
                self.test_result[topic_belonging]['GroundTruth']['additional']['gt_timestamp'])
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
                send_log(self, f'{topic_belonging} {topic} 时间差低于{match_tolerance} sec的尝试匹配, '
                               f'进一步选择局部最优')

                match_pred_timestamp, match_gt_timestamp = MatchTool.match_timestamp(
                    pred_timestamp, gt_timestamp, match_tolerance)
                send_log(self, f'{topic_belonging} {topic} GroundTruth 时间戳总计{len(gt_timestamp)}个, '
                               f'对齐{len(match_gt_timestamp)} '
                               f'比例{len(match_gt_timestamp) / len(gt_timestamp):.2%}')
                send_log(self, f'{topic_belonging} {topic} Prediction 时间戳总计{len(pred_timestamp)}个, '
                               f'对齐{len(match_pred_timestamp)} '
                               f'比例{len(match_pred_timestamp) / len(pred_timestamp):.2%}')

                # 保存时间辍匹配数据
                match_timestamp_data = pd.DataFrame(columns=['gt_timestamp', 'pred_timestamp', 'match_gap'])
                if len(match_pred_timestamp):
                    match_timestamp_data['gt_timestamp'] = match_gt_timestamp
                    match_timestamp_data['pred_timestamp'] = match_pred_timestamp
                    match_timestamp_data['match_time_gap'] = (match_timestamp_data['pred_timestamp']
                                                              - match_timestamp_data['gt_timestamp'])

                path = os.path.join(match_folder, 'match_timestamp.csv')
                match_timestamp_data.to_csv(path, index=False, encoding='utf_8_sig')
                self.test_result[topic_belonging][topic]['match']['match_timestamp'] = self.get_relpath(path)
                self.test_result[topic_belonging][topic]['match_frequency'] \
                    = round(self.test_result[topic_belonging][topic]['frequency']
                            * len(match_pred_timestamp) / len(pred_timestamp), 2)

    @sync_test_result
    def match_object(self):
        for topic_belonging in self.test_result.keys():
            if topic_belonging == 'General':
                continue

            additional_column = (test_encyclopaedia['Information'][topic_belonging]['raw_column']
                                 + test_encyclopaedia['Information'][topic_belonging]['additional_column'])
            send_log(self, f'{topic_belonging}, 使用{topic_belonging}MatchTool')
            match_tool = eval(f'MatchTool.{topic_belonging}MatchTool()')
            match_column = ['corresponding_index', 'gt.flag', 'pred.flag']
            for col in additional_column:
                for kind in ['gt', 'pred']:
                    match_column.append(f'{kind}.{col}')

            gt_data_path = self.get_abspath(
                self.test_result[topic_belonging]['GroundTruth']['additional']['gt_data'])
            gt_data = (pd.read_csv(gt_data_path, index_col=False)
                       .sort_values(by=['time_stamp'], ascending=True).reset_index(drop=True))

            for topic in self.test_result[topic_belonging].keys():
                if topic == 'GroundTruth':
                    continue

                topic_tag = topic.replace('/', '')
                match_folder = os.path.join(self.DataFolder, topic_belonging, topic_tag, 'match')

                match_timestamp_path = self.get_abspath(
                    self.test_result[topic_belonging][topic]['match']['match_timestamp'])
                match_timestamp = pd.read_csv(match_timestamp_path, index_col=False)

                pred_data_path = self.get_abspath(
                    self.test_result[topic_belonging][topic]['additional']['pred_data'])
                pred_data = (pd.read_csv(pred_data_path, index_col=False)
                             .sort_values(by=['time_stamp'], ascending=True).reset_index(drop=True))

                input_parameter_container = {
                    'object_matching_tolerance': self.test_config['object_matching_tolerance'],
                }
                input_data = {
                    'match_timestamp': match_timestamp,
                    'gt_data': gt_data,
                    'pred_data': pred_data,
                }

                send_log(self, f'{topic_belonging} {topic} 目标匹配')
                data = match_tool.run(input_data, input_parameter_container)[match_column]
                path = os.path.join(match_folder, 'match_data.csv')
                self.test_result[topic_belonging][topic]['match']['match_data'] = self.get_relpath(path)
                data.to_csv(path, index=False, encoding='utf_8_sig')

    @sync_test_result
    def evaluate_metrics(self):
        for topic_belonging in self.test_result.keys():
            if topic_belonging == 'General':
                continue

            send_log(self, f'{topic_belonging}, 使用{topic_belonging}MetricEvaluator')
            metric_evaluator = eval(f'MetricEvaluator.{topic_belonging}MetricEvaluator()')
            metric_filter = eval(f'MetricEvaluator.{topic_belonging}MetricFilter()')
            evaluate_range = {metric: v['evaluate_range']
                              for metric, v in test_encyclopaedia['Information'][topic_belonging]['metrics'].items()}

            for topic in self.test_result[topic_belonging].keys():
                if topic == 'GroundTruth':
                    continue

                if topic not in self.test_config['test_item']:
                    send_log(self, f'{topic}没有在test_item中')
                    continue

                topic_tag = topic.replace('/', '')
                metric_folder = os.path.join(self.DataFolder, topic_belonging, topic_tag, 'metric')
                create_folder(metric_folder)
                if 'metric' not in self.test_result[topic_belonging][topic]:
                    self.test_result[topic_belonging][topic]['metric'] = {}

                match_data_path = self.test_result[topic_belonging][topic]['match']['match_data']
                match_data = pd.read_csv(self.get_abspath(match_data_path), index_col=False)
                input_parameter_container = {
                    'metric_type': self.test_config['test_item'][topic],
                    'evaluate_range': evaluate_range,
                }
                input_data = {
                    'data': match_data,
                }

                send_log(self, f'{topic_belonging} {topic} 指标评估')
                data_dict = metric_evaluator.run(input_data, input_parameter_container)
                for metric, metric_data in data_dict.items():
                    total_folder = os.path.join(metric_folder, 'total')
                    create_folder(total_folder, False)
                    if 'total' not in self.test_result[topic_belonging][topic]['metric']:
                        self.test_result[topic_belonging][topic]['metric']['total'] = {}

                    path = os.path.join(total_folder, f'{metric}.csv')
                    self.test_result[topic_belonging][topic]['metric']['total'][metric] = self.get_relpath(path)
                    metric_data.to_csv(path, index=False, encoding='utf_8_sig')

                    input_parameter_container = {
                        'characteristic_type': self.test_config['target_characteristic'],
                    }
                    input_data = {
                        'total_data': match_data,
                        'data_to_filter': metric_data,
                    }

                    characteristic_data_dict = metric_filter.run(input_data, input_parameter_container)
                    for characteristic, characteristic_data in characteristic_data_dict.items():
                        characteristic_folder = os.path.join(metric_folder, characteristic)
                        create_folder(characteristic_folder, False)
                        if characteristic not in self.test_result[topic_belonging][topic]['metric']:
                            self.test_result[topic_belonging][topic]['metric'][characteristic] = {}

                        path = os.path.join(characteristic_folder, f'{metric}.csv')
                        self.test_result[topic_belonging][topic]['metric'][characteristic][
                            metric] = self.get_relpath(path)
                        characteristic_data.to_csv(path, index=False, encoding='utf_8_sig')

    def start(self):

        if self.test_action['preprocess']:
            self.load_pred_data()
            self.load_gt_data()
            self.sync_timestamp()
            self.promote_additional()

        if self.test_action['match']:
            self.match_timestamp()
            self.match_object()

        if self.test_action['metric']:
            self.evaluate_metrics()

    def get_relpath(self, path: str) -> str:
        return os.path.relpath(path, self.scenario_unit_folder)

    def get_abspath(self, path: str) -> str:
        return os.path.join(self.scenario_unit_folder, path)

    def save_test_result(self):
        with open(self.test_result_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_result,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    def load_test_result(self):
        with open(self.test_result_yaml) as f:
            self.test_result = yaml.load(f, Loader=yaml.FullLoader)


class DataGrinderPilotOneTask:

    def __init__(self, task_folder):
        print('=' * 25 + self.__class__.__name__ + '=' * 25)
        scenario_config_yaml = os.path.join(task_folder, 'TestConfig.yaml')
        with open(scenario_config_yaml, 'r', encoding='utf-8') as file:
            self.test_config = yaml.safe_load(file)
        self.task_folder = task_folder

        # 加载测试相关的参数
        self.product = self.test_config['product']
        self.version = self.test_config['version']
        self.test_encyclopaedia = test_encyclopaedia[self.product]
        self.test_action = self.test_config['test_action']
        self.scenario_unit_folder = self.test_config['data_path']['intermediate']['scenario_unit']
        self.tag_combination_folder = self.test_config['data_path']['intermediate']['tag_combination']
        self.result_folder = self.test_config['data_path']['output_result']
        self.region_division = self.test_config['region_division']

        self.test_result_yaml = os.path.join(self.task_folder, 'TestResult.yaml')
        if not os.path.exists(self.test_result_yaml):
            self.test_result = {}
            for scenario_tag in self.test_config['scenario_tag']:
                tag_key = '&'.join(scenario_tag['tag'].values())
                self.test_result[tag_key] = {
                    'tag': scenario_tag['tag'], 'scenario_unit': {}, 'tag_combination': {}
                }
                for scenario_id in scenario_tag['scenario_id']:
                    self.test_result[tag_key]['scenario_unit'][scenario_id] = self.get_relpath(
                        os.path.join(self.scenario_unit_folder, scenario_id, 'TestResult.yaml'))

            self.save_test_result()

    @sync_test_result
    def analyze_scenario_unit(self):
        # 依次创建scenario_unit测试信息
        scenario_run_list = {}
        scenario_list = []
        for scenario_tag in self.test_config['scenario_tag']:
            for scenario_id in scenario_tag['scenario_id']:
                if scenario_id in scenario_list:
                    continue

                scenario_list.append(scenario_id)
                scenario_test_config = {
                    'product': self.test_config['product'],
                    'version': self.test_config['version'],
                    'pred_folder': os.path.join(self.test_config['data_path']['raw']['pred'], scenario_id),
                    'gt_folder': os.path.join(self.test_config['data_path']['raw']['gt'], scenario_id),
                    'test_action': self.test_action['scenario_unit'],
                    'test_item': self.test_config['test_item'],
                    'target_characteristic': ['is_coverageValid', 'is_keyObj'],
                    'scenario_tag': scenario_tag['tag'],
                    'scenario_id': scenario_id,
                    'pred_ROI': self.test_config['pred_ROI'],
                    'gt_ROI': self.test_config['gt_ROI'],
                    'coverage_reference_point': self.test_config['coverage_reference_point'],
                    'timestamp_matching_tolerance': self.test_config['timestamp_matching_tolerance'],
                    'coverage_threshold': self.test_config['coverage_threshold'],
                    'object_matching_tolerance': self.test_config['object_matching_tolerance'],
                }

                scenario_run_list[scenario_id] = {
                    'scenario_unit_folder': os.path.join(self.scenario_unit_folder, scenario_id),
                    'scenario_test_config': scenario_test_config,
                    'scenario_config_yaml': os.path.join(self.scenario_unit_folder, scenario_id, 'TestConfig.yaml')
                }

        for scenario_run_info in scenario_run_list.values():
            create_folder(scenario_run_info['scenario_unit_folder'], False)
            with open(scenario_run_info['scenario_config_yaml'], 'w', encoding='utf-8') as f:
                yaml.dump(scenario_run_info['scenario_test_config'],
                          f, encoding='utf-8', allow_unicode=True, sort_keys=False)
            DataGrinderPilotOneCase(scenario_run_info['scenario_unit_folder']).start()

    @sync_test_result
    def combine_scenario_tag(self):
        # 合并各个场景的match_data, 重新corresponding_index编号
        # 获得每一种特征的分类结果
        match_data_dict = {}
        for tag_key in self.test_result.keys():
            if tag_key == 'OutputResult':
                continue

            tag_combination_folder = os.path.join(self.tag_combination_folder, tag_key)
            create_folder(tag_combination_folder)
            match_data_dict[tag_key] = {}

            for scenario_id, scenario_test_result in self.test_result[tag_key]['scenario_unit'].items():
                scenario_unit_folder = os.path.join(self.scenario_unit_folder, scenario_id)
                with open(self.get_abspath(scenario_test_result)) as f:
                    scenario_test_result = yaml.load(f, Loader=yaml.FullLoader)

                # 遍历全部测试结果，按照topic和metrics合并
                for topic_belonging in scenario_test_result.keys():
                    if topic_belonging == 'General':
                        continue

                    if topic_belonging not in self.test_result[tag_key]['tag_combination']:
                        self.test_result[tag_key]['tag_combination'][topic_belonging] = {}
                    if topic_belonging not in match_data_dict[tag_key]:
                        match_data_dict[tag_key][topic_belonging] = {}

                    for topic in scenario_test_result[topic_belonging].keys():
                        if topic == 'GroundTruth':
                            frequency = round(scenario_test_result[topic_belonging][topic]['frequency'])
                            if 'frequency' not in self.test_result[tag_key]:
                                self.test_result[tag_key]['frequency'] = frequency
                            else:
                                self.test_result[tag_key]['frequency'] = (
                                    min(self.test_result[tag_key]['frequency'], frequency))
                            continue

                        topic_tag = topic.replace('/', '')
                        if topic not in self.test_result[tag_key]['tag_combination'][topic_belonging]:
                            self.test_result[tag_key]['tag_combination'][topic_belonging][topic] = {}
                            topic_folder = os.path.join(tag_combination_folder, topic_belonging, topic_tag)
                            create_folder(topic_folder)

                        if topic not in match_data_dict[tag_key][topic_belonging]:
                            match_data_dict[tag_key][topic_belonging][topic] = []

                        match_data_path = scenario_test_result[topic_belonging][topic]['match']['match_data']
                        match_data = pd.read_csv(os.path.join(scenario_unit_folder, match_data_path), index_col=False)
                        match_data.insert(0, 'scenario_id', scenario_id)
                        match_data_dict[tag_key][topic_belonging][topic].append(match_data)

        for tag_key in match_data_dict.keys():
            for topic_belonging in match_data_dict[tag_key].keys():

                send_log(self, f'{topic_belonging}, 使用{topic_belonging}MetricEvaluator')
                metric_evaluator = eval(f'MetricEvaluator.{topic_belonging}MetricEvaluator()')
                metric_filter = eval(f'MetricEvaluator.{topic_belonging}MetricFilter()')
                evaluate_range = {metric: v['evaluate_range']
                                  for metric, v in
                                  test_encyclopaedia['Information'][topic_belonging]['metrics'].items()}

                for topic, df_list in match_data_dict[tag_key][topic_belonging].items():
                    topic_tag = topic.replace('/', '')
                    total_match_data = pd.concat(df_list).reset_index(drop=True)
                    total_match_data['corresponding_index'] = total_match_data.index

                    input_parameter_container = {
                        'metric_type': self.test_config['test_item'][topic],
                        'evaluate_range': evaluate_range,
                    }
                    input_data = {
                        'data': total_match_data,
                    }

                    send_log(self, f'{topic_belonging} {topic} 指标评估')
                    data_dict = metric_evaluator.run(input_data, input_parameter_container)

                    for metric, metric_data in data_dict.items():
                        total_folder = os.path.join(self.tag_combination_folder,
                                                    tag_key, topic_belonging, topic_tag, 'total')
                        create_folder(total_folder, False)
                        if 'total' not in self.test_result[tag_key]['tag_combination'][topic_belonging][topic]:
                            self.test_result[tag_key]['tag_combination'][topic_belonging][topic]['total'] = {}

                        path = os.path.join(total_folder, f'{metric}.csv')
                        self.test_result[tag_key]['tag_combination'][topic_belonging][topic]['total'][
                            metric] = self.get_relpath(path)
                        metric_data.to_csv(path, index=False, encoding='utf_8_sig')

                        input_parameter_container = {
                            'characteristic_type': self.test_config['target_characteristic'],
                        }
                        input_data = {
                            'total_data': total_match_data,
                            'data_to_filter': metric_data,
                        }

                        characteristic_data_dict = metric_filter.run(input_data, input_parameter_container)
                        for characteristic, characteristic_data in characteristic_data_dict.items():
                            characteristic_folder = os.path.join(self.tag_combination_folder,
                                                                 tag_key, topic_belonging, topic_tag, characteristic)
                            create_folder(characteristic_folder, False)
                            if characteristic not in self.test_result[tag_key]['tag_combination'][topic_belonging][
                                topic]:
                                self.test_result[tag_key]['tag_combination'][topic_belonging][topic][
                                    characteristic] = {}

                            path = os.path.join(characteristic_folder, f'{metric}.csv')
                            self.test_result[tag_key]['tag_combination'][topic_belonging][topic][characteristic][
                                metric] = self.get_relpath(path)
                            characteristic_data.to_csv(path, index=False, encoding='utf_8_sig')

    @sync_test_result
    def compile_statistics(self):
        # 按照数据库数据单元的方式保存数据
        # 格式为json，在文件夹内平铺
        json_folder = os.path.join(self.result_folder, 'statistics')
        create_folder(json_folder)
        json_count = 0
        json_rows = []
        self.test_result['OutputResult'] = {}
        for tag_key in self.test_result.keys():
            if tag_key == 'OutputResult':
                continue

            tag = self.test_result[tag_key]['tag']
            scenario_list = list(self.test_result[tag_key]['scenario_unit'].keys())

            for topic_belonging in self.test_result[tag_key]['tag_combination'].keys():
                metric_statistics = eval(f'MetricStatistics.{topic_belonging}MetricStatistics()')

                for topic in self.test_result[tag_key]['tag_combination'][topic_belonging].keys():
                    topic_tag = topic.replace('/', '')

                    for characteristic in self.test_config['target_characteristic']:
                        test_result = self.test_result[tag_key]['tag_combination'][topic_belonging][topic][
                            characteristic]

                        info_json_data = {tag_type: tag_value for tag_type, tag_value in tag.items()}
                        info_json_data['scenario_list'] = scenario_list
                        info_json_data['topic'] = topic

                        data = {
                            metric: pd.read_csv(self.get_abspath(data_path), index_col=False)
                            for metric, data_path in test_result.items()
                        }

                        input_parameter_container = {
                            'region_division': self.region_division,
                            'characteristic': characteristic,
                        }

                        json_datas = metric_statistics.run(data, input_parameter_container)

                        for json_data in json_datas:
                            json_data = {**info_json_data, **json_data}

                            # 保存单个json文件
                            json_count += 1
                            json_name = (f'{json_count:06d}--{tag_key}--{topic_tag}--{json_data["type"]}'
                                         f'--{json_data["region"]}--{json_data["characteristic"]}--{json_data["metric"]}.json')
                            json_path = os.path.join(json_folder, json_name)

                            if json_data['result']['sample_count'] < self.test_result[tag_key]['frequency'] * 2:
                                print(
                                    f'{json_count} {json_name} 样本少于{self.test_result[tag_key]["frequency"] * 2}，不保存')
                                continue

                            print(f'{json_count} {json_name} 已保存')
                            with open(json_path, 'w', encoding='utf-8') as f:
                                json.dump(json_data, f, ensure_ascii=False, indent=4)

                            # 保存json文件的目录
                            json_rows.append(
                                [
                                    json_count, tag_key, topic, json_data['type'], json_data['region'],
                                    json_data['characteristic'], json_data['metric'],
                                    json_data['result']['sample_count'], json.dumps(json_data['result'])
                                ]
                            )

        path = os.path.join(json_folder, 'output_result.csv')
        pd.DataFrame(json_rows, columns=[
            'result_index', 'scenario_tag', 'topic', 'obstacle_type', 'region',
            'characteristic', 'metric', 'sample_count', 'result'
        ]).to_csv(path, index=False, encoding='utf_8_sig')
        self.test_result['OutputResult']['statistics'] = self.get_relpath(path)

    @sync_test_result
    def visualize_output(self):

        def plot_table(axes, columns, columns_width, values, index=None, index_width=0):
            if index is None:
                index = []
            axes.patch.set_facecolor('white')
            axes.patch.set_alpha(0.3)
            for pos in ['right', 'top', 'left', 'bottom']:
                axes.spines[pos].set_visible(False)

            # 画index
            if index_width and len(index):
                axes.add_patch(pc.Rectangle(
                    xy=(-index_width, 0),
                    width=index_width, height=1,
                    angle=0, alpha=1,
                    rotation_point='xy',
                    fill=True,
                    edgecolor='grey',
                    facecolor='bisque',
                    linewidth=1))
                axes.text(-index_width / 2, 0.5, 'Test Case',
                          va='center', ha='center',
                          fontdict={
                              'family': 'Ubuntu',
                              'style': 'normal',
                              'weight': 'bold',
                              'color': 'black',
                              'size': font_size * 1.5,
                          })

                for i, idx in enumerate(index):
                    axes.add_patch(pc.Rectangle(
                        xy=(-index_width, -1 - i),
                        width=index_width, height=1,
                        angle=0, alpha=1,
                        rotation_point='xy',
                        fill=True,
                        edgecolor='grey',
                        facecolor='linen',
                        linewidth=1))

                    axes.text(-index_width * 0.95, -1 - i + 0.5, idx,
                              va='center', ha='left',
                              fontdict={
                                  'family': 'sans-serif',
                                  'style': 'normal',
                                  'weight': 'normal',
                                  'color': 'black',
                                  'size': font_size * 1.5,
                              })

            # 画columns
            current_x = 0
            for i, (col, width) in enumerate(zip(columns, columns_width)):
                axes.add_patch(pc.Rectangle(
                    xy=(current_x, 0),
                    width=width, height=1,
                    angle=0, alpha=1,
                    rotation_point='xy',
                    fill=True,
                    edgecolor='grey',
                    facecolor='lightsteelblue',
                    linewidth=1))

                axes.text(current_x + width * 0.5, 0.5, col,
                          va='center', ha='center',
                          fontdict={
                              'family': 'Ubuntu',
                              'style': 'normal',
                              'weight': 'bold',
                              'color': 'black',
                              'size': font_size * 1.5,
                          })

                current_x += width

            # 画values
            ii, jj = values.shape
            for i in range(ii):
                current_x = 0
                for j in range(jj):
                    width = columns_width[j]
                    color = 'white'
                    value = values[i, j]

                    # if columns[j] == 'pass_ratio%':
                    #     sizes = [value, 1 - value]
                    #     labels = ['PASSED', 'FAILED']
                    #     colors = ['limegreen', 'lightcoral']
                    #     new_ax = fig.add_axes([current_x + width * 0.5, -i - 1 + 0.5, 0.5, 0.5])  # [x, y, width, height]
                    #     new_ax.pie(sizes, labels=labels, colors=colors, radius=0.4)

                    if '%' in columns[j]:
                        value = '{:.2%}'.format(value)

                    elif isinstance(value, float):
                        if int(value) == value:
                            value = int(value)
                        else:
                            value = round(value, 4)

                    if contains_chinese(str(value)):
                        fontdict = {
                            'family': 'sans-serif',
                            'style': 'normal',
                            'weight': 'normal',
                            'color': 'black',
                            'size': font_size * 1.5,
                        }
                    else:
                        fontdict = {
                            'family': 'Ubuntu',
                            'style': 'normal',
                            'weight': 'normal',
                            'color': 'black',
                            'size': font_size * 1.5,
                        }

                    axes.add_patch(pc.Rectangle(
                        xy=(current_x, -i - 1),
                        width=width, height=1,
                        angle=0, alpha=1,
                        rotation_point='xy',
                        fill=True,
                        edgecolor='grey',
                        facecolor=color,
                        linewidth=1))
                    axes.text(current_x + width * 0.5, -i - 1 + 0.5, value,
                              va='center', ha='center',
                              fontdict=fontdict)

                    current_x += width

            axes.set_xlim(-index_width - 0.1, np.sum(columns_width) + 0.1)
            axes.set_ylim(-len(df) - 0.1, 2.1)
            axes.set_xticks([])
            axes.set_yticks([])

        visualization_folder = os.path.join(self.result_folder, 'visualization')
        create_folder(visualization_folder)
        statistics = pd.read_csv(self.get_abspath(self.test_result['OutputResult']['statistics']), index_col=False)
        group_columns = ['scenario_tag', 'topic', 'obstacle_type', 'characteristic', 'metric']
        stat_group = statistics.groupby(group_columns)
        self.test_result['OutputResult']['visualization'] = {}

        df_tp_error = {}
        for df_name, df in stat_group:
            scenario_tag, topic, obstacle_type, characteristic, metric = df_name

            if characteristic == '静止目标' and '速度' in metric:
                continue

            for index, row in df.iterrows():
                result_dict = json.loads(row['result'])
                for key, value in result_dict.items():
                    df.at[index, key] = value
            df['type_cate'] = pd.Categorical(df['obstacle_type'],
                                             categories=['小车', '大巴', '货车', '自行车', '行人'], ordered=True)
            df.sort_values(by=['type_cate', 'region'], inplace=True)
            drop_columns = group_columns + ['result_index', 'type_cate', 'result']
            df.drop(drop_columns, axis=1, inplace=True)
            df.insert(0, 'obstacle_type', obstacle_type)
            df.rename(columns={'region': 'grid area division[m]'}, inplace=True)

            columns = list(df.columns)
            columns_width = [max(get_string_display_length(c) / 5.2, 2) for c in columns]
            values = df.values

            fig_size = (np.sum(columns_width) + 0.1, (len(df) + 2) + 0.1)
            fig = plt.figure(figsize=fig_size)
            plt.tight_layout()
            plt.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)
            plt.axis('off')
            grid = plt.GridSpec(1, 1, wspace=0, hspace=0.02)
            ax = fig.add_subplot(grid[0, 0])
            plot_table(ax, columns, columns_width, values)
            ax.text(np.sum(columns_width) / 2, 1.5, '-'.join(
                [topic, scenario_tag, characteristic, obstacle_type, metric]),
                    va='center', ha='center', fontsize=font_size * 2)

            pic_name = '--'.join(
                [topic, scenario_tag, characteristic, obstacle_type, metric]).replace('/', '')
            pic_path = os.path.join(visualization_folder, f'{pic_name}.jpg')
            print(f'{pic_path} 已保存')
            canvas = FigureCanvas(fig)
            canvas.print_figure(pic_path, facecolor='white', dpi=100)
            fig.clf()
            plt.close()

            # 汇总tp_error
            if metric == '准召信息':
                continue

            if topic not in df_tp_error.keys():
                df_tp_error[topic] = {}
            if scenario_tag not in df_tp_error[topic].keys():
                df_tp_error[topic][scenario_tag] = {}
            if characteristic not in df_tp_error[topic][scenario_tag].keys():
                df_tp_error[topic][scenario_tag][characteristic] = {}
            if obstacle_type not in df_tp_error[topic][scenario_tag][characteristic].keys():
                df_tp_error[topic][scenario_tag][characteristic][obstacle_type] = []

            df.rename(columns={'pass_ratio%': metric}, inplace=True)
            df_tp_error[topic][scenario_tag][characteristic][obstacle_type].append(
                df[['grid area division[m]', metric]])

        # 合并tp_error的metric数据，以及可视化为饼图
        for topic in df_tp_error.keys():
            if topic not in self.test_result['OutputResult']['visualization'].keys():
                self.test_result['OutputResult']['visualization'][topic] = {}

            for scenario_tag in df_tp_error[topic].keys():
                if scenario_tag not in self.test_result['OutputResult']['visualization'][topic].keys():
                    self.test_result['OutputResult']['visualization'][topic][scenario_tag] = {}

                for characteristic in df_tp_error[topic][scenario_tag].keys():
                    if characteristic not in self.test_result['OutputResult']['visualization'][topic][scenario_tag].keys():
                        self.test_result['OutputResult']['visualization'][topic][scenario_tag][characteristic] = {}

                    for obstacle_type in df_tp_error[topic][scenario_tag][characteristic].keys():
                        if obstacle_type not in self.test_result['OutputResult']['visualization'][topic][scenario_tag][characteristic].keys():
                            self.test_result['OutputResult']['visualization'][topic][scenario_tag][characteristic][obstacle_type] = {}

                        merged_df = pd.DataFrame(columns=['grid area division[m]'])
                        for one_df in df_tp_error[topic][scenario_tag][characteristic][obstacle_type]:
                            merged_df = merged_df.merge(one_df, on='grid area division[m]', how='outer')
                        csv_name = '--'.join([topic, scenario_tag, characteristic, obstacle_type]).replace('/', '')
                        csv_path = os.path.join(visualization_folder, f'{csv_name}.csv')
                        (merged_df.sort_values(by=['grid area division[m]'])
                         .to_csv(csv_path, index=False, encoding='utf_8_sig'))

                        self.test_result['OutputResult']['visualization'][topic][scenario_tag][characteristic][
                            obstacle_type]['data'] = self.get_relpath(csv_path)

    def start(self):
        if any([value for value in self.test_action['scenario_unit'].values()]):
            self.analyze_scenario_unit()

        if self.test_action['tag_combination']:
            self.combine_scenario_tag()

        if self.test_action['output_result']:
            # self.compile_statistics()
            self.visualize_output()

    def get_relpath(self, path: str) -> str:
        return os.path.relpath(path, self.task_folder)

    def get_abspath(self, path: str) -> str:
        return os.path.join(self.task_folder, path)

    def save_test_result(self):
        with open(self.test_result_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_result,
                      f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    def load_test_result(self):
        with open(self.test_result_yaml) as f:
            self.test_result = yaml.load(f, Loader=yaml.FullLoader)
