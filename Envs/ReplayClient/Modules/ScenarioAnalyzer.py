#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2023/8/10 上午9:41
# @Author  : Cen Zhengfeng

import json
import subprocess
import sys
import datetime
import glob
import os

import yaml
import pandas as pd
import numpy as np

from BenchDBMS import BenchDBMS

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Libs import get_project_path
sys.path.append(get_project_path())
from Utils.Libs import test_encyclopaedia, create_folder, bench_config, project_path

import warnings

warnings.filterwarnings("ignore")

scenario_analysis_path = os.path.join(project_path, 'Docs', 'Resources', 'scenario_info', 'scenario_analysis.json')
scenario_road_weather_path = os.path.join(project_path, 'Docs', 'Resources', 'scenario_info', 'scenario_road_weather.csv')
scenario_summary_path = os.path.join(project_path, 'Docs', 'Resources', 'scenario_info', 'scenario_summary.csv')
colors = ['#3682be', '#45a776', '#f05326', '#b3974e', '#38cb7d', '#ddae33', '#844bb3',
          '#93c555', '#5f6694', '#df3881']


class SingleScenarioObstaclesAnalyzer:

    def __init__(self, gt_raw_folder):
        self.gt_raw_folder = gt_raw_folder
        self.test_information = test_encyclopaedia['Information']['Obstacles']

        sid = os.path.basename(gt_raw_folder).split('_')
        if len(sid) > 3:
            self.scenario_id = '_'.join(sid[:3])
            self.truth_source = sid[-1]
        else:
            self.scenario_id = os.path.basename(gt_raw_folder)
            self.truth_source = 'undefined'

        print(f'正在解析{self.scenario_id}开始解析')
        self.tmp_folder = os.path.join('/tmp', self.scenario_id)
        self.analysis = {
            'scenario_id': self.scenario_id,
            'truth_source': self.truth_source,
            'label': {},
            'obstacles': {}
        }

    def load_gt_data(self):
        gt_config_path = os.path.join(self.gt_raw_folder, 'yaml_management.yaml')
        if not os.path.exists(gt_config_path):
            print('No yaml_management.yaml is found. Please check')
            return False

        with open(gt_config_path) as f:
            gt_config = yaml.load(f, Loader=yaml.FullLoader)

        gt_topic = 'od_csv'
        flag = f'{gt_topic}_flag'
        timestamp_csv_tag = f'{gt_topic.split("_")[0]}_timestamp'

        if not (flag in gt_config.keys() and gt_config[flag]):
            print(f'GroundTruth 未找到{gt_topic}对应的真值文件')
            return False

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
            gt_data = gt_data[gt_data['type'].isin([1, 2, 18])]
            gt_data = gt_data[gt_data['sub_type'] != 10]

        # 读取时间辍, 用于时间辍匹配
        gt_timestamp_csv = \
            glob.glob(os.path.join(self.gt_raw_folder, f'{timestamp_csv_tag}*hz.csv'))[0]
        gt_timestamp = pd.read_csv(gt_timestamp_csv).sort_values(
            by=['timestamp']).rename(columns={'timestamp': 'time_stamp'}) \
            .drop_duplicates(subset=['time_stamp'], keep='first')

        raw_column = self.test_information['raw_column']

        create_folder(self.tmp_folder)
        path = os.path.join(self.tmp_folder, 'gt_data.csv')
        gt_data[raw_column].to_csv(path, index=False, encoding='utf_8_sig')

        path = os.path.join(self.tmp_folder, 'gt_timestamp.csv')
        gt_timestamp.to_csv(path, index=False, encoding='utf_8_sig')

        # 自车速度用于对齐
        ego_data_path = os.path.join(self.gt_raw_folder, 'od_ego.csv')
        if not os.path.exists(ego_data_path):
            print('GroundTruth未找到自车速度数据')
            ego_data = pd.DataFrame(columns=['time_stamp', 'ego_vx'])
            ego_data['time_stamp'] = gt_timestamp['time_stamp']
            ego_data['ego_vx'] = 0
            self.analysis['label']['ego_vx_flag'] = False
        else:
            # print(f'GroundTruth 正在读取{os.path.basename(ego_data_path)}, 用于时间同步')
            ego_data = pd.read_csv(os.path.join(self.gt_raw_folder, 'od_ego.csv'))[
                ['ego_timestamp', 'INS_Speed']].rename(
                columns={'ego_timestamp': 'time_stamp', 'INS_Speed': 'ego_vx'})
            self.analysis['label']['ego_vx_flag'] = True
        path = os.path.join(self.tmp_folder, 'gt_ego.csv')
        ego_data.to_csv(path, index=False, encoding='utf_8_sig')

        return True

    def process_gt_data(self):
        # print('GroundTruth 正在预处理步骤')
        path = os.path.join(self.tmp_folder, 'gt_data.csv')

        input_parameter_container = {
            'coverage_reference_point': [2, 0, 1],
            'coverage_threshold': 0.6,
            'ROI': {
                'x': [-100, 150], 'y': [-8, 8],
            },
            'lane_width': 3.6,
            'moving_threshold': 2,
            'key_coverage_threshold': 0.1,
            'test_topic': 'Obstacles',
        }
        parameter_json_path = os.path.join(self.tmp_folder, 'process_api_parameter.json')
        with open(parameter_json_path, 'w', encoding='utf-8') as f:
            json.dump(input_parameter_container, f, ensure_ascii=False, indent=4)
        additional_gt_data_path = os.path.join(self.tmp_folder, f'additional_gt_data.csv')

        cmd = [
            f"{bench_config['ReplayClient']['sys_interpreter']}",
            "Api_ProcessRawData.py",
            "-r", path,
            "-j", parameter_json_path,
            "-p", additional_gt_data_path,
        ]

        # print(cmd)
        cwd = os.path.join(get_project_path(), 'Envs', 'Master', 'Interfaces')
        result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
        if result.stderr:
            print("stderr:", result.stderr)

        additional_column = self.test_information['raw_column'] + self.test_information['additional_column']
        data = pd.read_csv(additional_gt_data_path, index_col=False)[additional_column]
        data.to_csv(additional_gt_data_path, index=False, encoding='utf_8_sig')

    def analyze_time(self):
        d = self.scenario_id.split('_')[0]
        t = self.scenario_id.split('_')[1]
        index = int(self.scenario_id[-3:])
        s = int(t[0:2]) * 3600 + int(t[2:4]) * 60 + int(t[4:6]) + 300 * (index - 1)
        d_num = datetime.datetime.strptime(d, "%Y%m%d").date().timetuple().tm_yday
        if d_num <= 183:
            raise_time = (- d_num / 91 + 638 / 91) * 3600
            set_time = (d_num / 91 + 1546 / 91) * 3600
        else:
            raise_time = (d_num / 91 + 272 / 91) * 3600
            set_time = (- d_num / 91 + 1912 / 91) * 3600
        if raise_time - 1800 <= s <= raise_time + 1800:
            time_label = 'dawn'
        elif set_time - 1800 <= s <= set_time + 1800:
            time_label = 'dusk'
        elif raise_time + 1800 < s <= 43200:
            time_label = 'morning'
        elif 43200 < s < set_time - 1800:
            time_label = 'afternoon'
        else:
            time_label = 'night'

        # print(f'{d[:4]}年的第{d_num}天，日出时间{raise_time}秒，日落时间{set_time}秒')
        # print(f'时间标签为{time_label}')
        self.analysis['label']['time'] = time_label

    def analyze_ego(self):
        ego_data = pd.read_csv(os.path.join(self.tmp_folder, 'gt_ego.csv'))
        ego_data['ego_v'] = ego_data.apply(lambda v: np.abs(v['ego_vx']), axis=1)
        low_ego_data = ego_data[ego_data['ego_v'] < 3]
        high_ego_data = ego_data[ego_data['ego_v'] >= 3]
        self.analysis['label']['low_velocity_ratio[%]'] = len(low_ego_data) / len(ego_data)
        if len(high_ego_data):
            self.analysis['label']['average_ego_velocity[m/s]'] = float(high_ego_data['ego_v'].mean())
        else:
            self.analysis['label']['average_ego_velocity[m/s]'] = 0

    def analyze_obstacles(self):

        def get_res(df):
            p_quantity = len(df)
            if p_quantity:
                p_average_velocity = df['abs_v'].mean()
                p_flow = df['abs_v'].sum() / (2 * area_length)
                p_area = df['area[m2]'].sum() / (4 * area_length * area_width)
                return p_quantity, p_average_velocity, p_flow, p_area
            else:
                return np.nan, np.nan, np.nan, np.nan

        data = pd.read_csv(os.path.join(self.tmp_folder, f'additional_gt_data.csv'), index_col=False)
        data = data[data['is_detectedValid'] == 1].drop(columns=['is_detectedValid'], axis=1)

        types = data['type_classification'].unique()
        region = [-100, -50, 0, 50, 100, 150]
        for col in data.columns:
            if 'is_' in col:
                characteristic = col
                ch_data = data[data[characteristic] == 1]
                self.analysis['obstacles'][characteristic] = {}
                for type_ in types:
                    type_data = ch_data[ch_data['type_classification'] == type_]
                    self.analysis['obstacles'][characteristic][type_] = {
                        'all_region': len(type_data),
                    }

                    for i in range(len(region) - 1):
                        area_data = type_data[(type_data['x'] >= region[i]) & (type_data['x'] < region[i + 1])]
                        region_text = f'[{region[i]}, {region[i + 1]}]'
                        self.analysis['obstacles'][characteristic][type_][region_text] = len(area_data)

        # 分析交通流
        data = pd.read_csv(os.path.join(self.tmp_folder, f'additional_gt_data.csv'), index_col=False)
        area_length, area_width = 30, 8
        data = data[(data['x'] <= area_length)
                    & (data['x'] >= -area_length)
                    & (data['y'] <= area_width)
                    & (data['y'] >= -area_width)]
        data['abs_v'] = data.apply(lambda v: np.sqrt(v['vx'] ** 2 + v['vy'] ** 2), axis=1)
        data['area[m2]'] = data.apply(lambda v: v['length'] * v['width'], axis=1)
        data.loc[data['abs_v'] > 80, 'abs_v'] = data.loc[data['abs_v'] <= 80, 'abs_v'].mean()

        rows = []
        columns = ['time_stamp', 'type', 'quantity[]', 'average_velocity[m/s]', 'flow[/s]', 'area_occ[%]']
        timestamps = pd.read_csv(os.path.join(self.tmp_folder, 'gt_timestamp.csv'), index_col=False)
        for time_stamp in timestamps['time_stamp'].unique():
            time_data = data[data['time_stamp'] == time_stamp]
            rows.append([time_stamp, 'total', *get_res(time_data)])

            for type_ in types:
                type_data = time_data[time_data['type_classification'] == type_]
                rows.append([time_stamp, type_, *get_res(type_data)])

        res = pd.DataFrame(rows, columns=columns)
        total_data = res[res['type'] == 'total']
        self.analysis['label']['average_velocity[m/s]'] = float(
            total_data['average_velocity[m/s]'].sum() / len(total_data))
        self.analysis['label']['flow[/s]'] = float(
            total_data['flow[/s]'].sum() / len(total_data))
        self.analysis['label']['area_occ[%]'] = float(
            total_data['area_occ[%]'].sum() / len(total_data))

    def start(self):
        self.analyze_time()
        if not self.load_gt_data():
            return False

        self.process_gt_data()
        self.analyze_ego()
        self.analyze_obstacles()

        return self.analysis


class ScenarioAnalyzer:

    def __init__(self):
        self.annotation_folder = '/media/data/annotation'
        if os.path.exists(scenario_analysis_path):
            with open(scenario_analysis_path, 'r', encoding='utf-8') as f:
                self.scenario_analysis = json.load(f)
        else:
            self.scenario_analysis = {}

    def start(self, update=False):
        for i, f in enumerate(os.listdir(self.annotation_folder)):
            folder = os.path.join(self.annotation_folder, f)
            if os.path.isdir(folder) and os.path.exists(os.path.join(folder, 'yaml_management.yaml')):
                obstacle_analyzer = SingleScenarioObstaclesAnalyzer(folder)
                key = f"{obstacle_analyzer.analysis['scenario_id']}-{obstacle_analyzer.analysis['truth_source']}"
                if key not in self.scenario_analysis or update:
                    print(f'No.{i + 1} 分析场景 {f}')
                    res = obstacle_analyzer.start()
                    if not res:
                        print(f'No.{i + 1} 分析场景 {f}失败')
                    else:
                        self.scenario_analysis[key] = res
                        print(f'No.{i + 1} {key} 已保存')
                        with open(scenario_analysis_path, 'w', encoding='utf-8') as f:
                            json.dump(self.scenario_analysis, f, ensure_ascii=True, indent=4)
                else:
                    print(f'No.{i + 1} 场景 {f} 已存在，不进行再次分析')


class ScenarioInfoSummary:

    def __init__(self):

        # 汇总包含真值的场景列表
        if os.path.exists(scenario_analysis_path):
            with open(scenario_analysis_path, 'r', encoding='utf-8') as f:
                self.scenario_analysis = json.load(f)
            self.analysis_label = list(list(self.scenario_analysis.values())[0]['label'].keys())
        else:
            self.scenario_analysis = {}
            self.analysis_label = []

        scenario_list_with_analysis = {}
        for scenario_analysis in self.scenario_analysis.values():
            scenario_id = scenario_analysis['scenario_id']
            truth_source = scenario_analysis['truth_source']
            if truth_source not in scenario_list_with_analysis:
                scenario_list_with_analysis[truth_source] = []
            scenario_list_with_analysis[truth_source].append(scenario_id)

        print(scenario_list_with_analysis)

        # 汇总肉眼分析过weather和road的场景
        if os.path.exists(scenario_road_weather_path):
            scenario_rc = pd.read_csv(scenario_road_weather_path, index_col=False)
            self.scenario_road_weather = scenario_rc[(scenario_rc['road_level'].notna())
                        & (scenario_rc['weather'].notna())
                        & (scenario_rc['road_level'] != '其他')
                        & (scenario_rc['select4test'] == 1)]
            scenario_list_with_road_weather = self.scenario_road_weather['batch_id'].to_list()
        else:
            scenario_list_with_road_weather = []
            self.scenario_road_weather = pd.DataFrame()

        print(len(scenario_list_with_road_weather))
        print(scenario_list_with_road_weather)

        # 汇总包含视频文件的场景
        temp = BenchDBMS().id_11CAN11V2Lidar_path_DF.sort_index()
        temp.drop(temp[temp['Complete'] == False].index, inplace=True)
        scenario_list_with_video = temp['id'].to_list()

        print(len(scenario_list_with_video))
        print(scenario_list_with_video)

        self.scenario_list_with_analysis = scenario_list_with_analysis
        self.scenario_list_with_road_weather = scenario_list_with_road_weather
        self.scenario_list_with_video = scenario_list_with_video

    def get_scenario_combination(self):
        columns = ['scenario_id', 'truth_source', 'road_level', 'weather'] + self.analysis_label
        print(columns)
        rows = []
        used_content = []
        for truth_source in self.scenario_list_with_analysis:
            temp = sorted(list(set(self.scenario_list_with_analysis[truth_source])
                                           & (set(self.scenario_list_with_road_weather)
                                              & set(self.scenario_list_with_video))))

            for scenario_id in temp:
                if f'{scenario_id}-{truth_source}' in used_content:
                    continue

                used_content.append(f'{scenario_id}-{truth_source}')
                road_level = self.scenario_road_weather[self.scenario_road_weather['batch_id'] == scenario_id]['road_level'].iloc[0]
                weather = self.scenario_road_weather[self.scenario_road_weather['batch_id'] == scenario_id]['weather'].iloc[0]
                analysis_label = [v for k, v in self.scenario_analysis[f'{scenario_id}-{truth_source}']['label'].items() if k in self.analysis_label]
                rows.append([scenario_id, truth_source, road_level, weather, *analysis_label])

        pd.DataFrame(rows, columns=columns).sort_values(by='scenario_id').to_csv(scenario_summary_path, index=False)


if __name__ == "__main__":
    # SA = ScenarioAnalyzer()
    # SA.start(update=False)

    SS = ScenarioInfoSummary()
    SS.get_scenario_combination()