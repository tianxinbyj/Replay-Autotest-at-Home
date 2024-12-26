#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2023/8/10 上午9:41
# @Author  : Cen Zhengfeng
import datetime
import glob
import os
from itertools import groupby
from pathlib import Path
from time import sleep

import yaml
from tqdm import tqdm
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import warnings
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

from Function.JsonParser import TrueValueJsonParse

warnings.filterwarnings("ignore")

SUB_TYPE = {
    1: 'car',
    3: 'van',
    4: 'bus',
    5: 'truck',
    9: 'tricycle',
    10: 'special',
    11: 'tiny_car',
    12: 'lorry',
}

colors = ['#3682be', '#45a776', '#f05326', '#b3974e', '#38cb7d', '#ddae33', '#844bb3',
          '#93c555', '#5f6694', '#df3881']

CONSTRAINT_LONG = 6


class StatsAnalyzer:

    def __init__(self):
        self.total_time = 0
        self.num_total_vehicle = 0
        self.num_long_vehicle = 0
        self.num_large_vehicle = 0
        self.num_total_frame = 0
        self.max_ob_value = -1
        self.type_counter = [dict.fromkeys(SUB_TYPE.keys(), 0), 0, 0]
        self.unknown_type = 0
        self.tags = []
        # Initialize the dataframe.
        self.df = pd.DataFrame([0], columns=['max_ob_one_frame'])

    # Process obstacle types, includes vehicle, pedestrian and cyclist.
    # Also, sub_types could be processed.
    def _type_processor(self, data):
        for index in data.index:
            if data.loc[index, 'type'] == 1:
                self.type_counter[0][data.loc[index, 'sub_type']] += 1
            elif data.loc[index, 'type'] == 2:
                self.type_counter[1] += 1
            elif data.loc[index, 'type'] == 18:
                self.type_counter[2] += 1
            else:
                # If encounter some unknown types.
                self.unknown_type += 1

    # Set tags according to the given statistic.
    def set_tags(self):
        def compare(num_obstacle, obstacle_type, pos_ratio=0.1, neg_ratio=None, append=None):
            if neg_ratio is None:
                neg_ratio = 0.1 * pos_ratio
            if append is None:
                append = ["多", "少"]
            if num_obstacle == 0:
                self.tags.append(("没有" + obstacle_type, 0))
            elif num_obstacle >= pos_ratio * self.num_total_vehicle:
                self.tags.append((obstacle_type + append[0], num_obstacle / self.num_total_vehicle))
            elif num_obstacle <= neg_ratio * self.num_total_vehicle:
                self.tags.append((obstacle_type + append[1], num_obstacle / self.num_total_vehicle))

        compare(self.num_large_vehicle, "大车", )
        compare(self.num_long_vehicle, "长车")
        compare(self.type_counter[1], "步行人", 0.05, 0.005)
        compare(self.type_counter[2], "骑行人", 0.1, 0.005)

        # Add tags into the dataframe.
        index = 0
        for tag in self.tags:
            self.df['tag_' + str(index)] = tag[0]
            self.df['tag_' + str(index) + '_value'] = tag[1]
            index += 1

    # Analyze info and output the results.
    def analyze(self, path):
        # Extract the data of interest.
        data = pd.read_csv(path)
        self._type_processor(data)
        self.num_total_frame += data['frame_id'].max() - data['frame_id'].min() + 1

        # Use the number of same time_stamp value as the obstacle counter.
        # Calculate and record the number of obstacle per second.
        self.total_time += data['time_stamp'].max() - data['time_stamp'].min()
        self.df['ob_per_sec'] = self.num_total_frame / self.total_time

        temp = data['time_stamp'].value_counts().iat[0]
        if temp > self.max_ob_value:
            self.max_ob_value = temp
        self.df['max_ob_one_frame'] = self.max_ob_value

        self.num_total_vehicle = sum(self.type_counter[0].values())
        self.df['total_vehicle'] = self.num_total_vehicle
        self.df['vehicle_per_frame'] = sum(self.type_counter[0].values()) / self.num_total_frame

        # Vehicle length distribution.
        # Classified vehicle lengths within (0, 4.6], (4.6, 6], and (6, arbitrary].
        labels_list = ['0-4.6m', '4.6-6m', '6m+']
        result = pd.cut(data['length'].abs(), bins=[0, 4.6, 6, 10000], labels=labels_list, ordered=False)
        counts = pd.value_counts(result, sort=False)
        for key, value in dict(counts).items():
            # self.df[key] = counts[key] / self.num_total_vehicle
            if key in self.df.keys():
                self.df[key] += counts[key]
            else:
                self.df[key] = counts[key]

        for vehicle_type in self.type_counter[0].keys():
            self.df[SUB_TYPE[vehicle_type]] = self.type_counter[0][vehicle_type]

        self.df['pedestrian'] = self.type_counter[1]
        self.df['pedestrian_per_frame'] = self.type_counter[1] / self.num_total_frame
        self.df['cyclist'] = self.type_counter[2]
        self.df['cyclist_per_frame'] = self.type_counter[2] / self.num_total_frame

        # Set vehicles of type: {4: 'bus', 5: 'truck'} as large vehicles.
        self.num_large_vehicle = sum(self.type_counter[0][vehicle_type] for vehicle_type in [4, 5])
        # Set vehicles longer than 6 meters(CONSTRAINT_LONG) as long vehicles.
        self.num_long_vehicle += len(data[data['length'] > CONSTRAINT_LONG])

        # Vehicle distance.
        labels_list = ['0-25m', '25-50m', '50-75m', '75-100m', '100m+']
        result = pd.cut(data['x'].abs(), bins=[0, 25, 50, 75, 100, 10000], labels=labels_list, ordered=False)
        counts = pd.value_counts(result, sort=False)
        for key, value in dict(counts).items():
            # self.df[key] = counts[key] / self.num_total_vehicle
            if key in self.df.keys():
                self.df[key] += counts[key]
            else:
                self.df[key] = counts[key]

    def save_csv(self, in_path, out_path):
        # Add tags for the result.
        self.set_tags()
        out_path = out_path.joinpath(in_path.parts[-3])
        out_path.mkdir(parents=True, exist_ok=True)
        new_file_name = str(out_path) + in_path.anchor + 'Analysis_' + in_path.name
        self.df.to_csv(new_file_name, mode='w+', index=False)


class ScenarioAnalyzer:
    db_od_type_list = [
        'car', 'bus', 'truck', 'tricycle', 'other', 'pedestrian', 'cyclist'
    ]

    # 获取以下信息
    # 1. 自车速度信息: 速度
    # 2. 他车速度信息: 速度, 数量, 流量, 空间占据
    def __init__(self, data_folder, test_encyclopaedia, product='1J5'):
        self.high_timestamp = None
        self.low_timestamp = None
        self.ego_data = None
        self.product_relevant = test_encyclopaedia['product_relevant']
        self.sensor_label = product
        self.annotation_folder = data_folder
        self.area_length = 50
        self.area_width = 20

        annotation_topics = {
            'od_csv': '/VA/Obstacles',
            'object_csv': '/VA/Objects',
            'lanebev_csv': '/VA/Lines',
        }
        self.AnnotationDataParser = TrueValueJsonParse(self.annotation_folder)
        config = os.path.join(self.annotation_folder, 'yaml_management.yaml')
        if not os.path.exists(config):
            self.AnnotationDataParser.getTrueVal2Csv()
        with open(config) as f:
            annotation_value_config = yaml.load(f, Loader=yaml.FullLoader)
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
                    path = os.path.join(self.annotation_folder, annotation_topic, csv_file)
                    csv_data.append(pd.read_csv(path, index_col=False))

                annotation_timestamp_csv = \
                    glob.glob(os.path.join(self.annotation_folder, f'{timestamp_csv_tag}*hz.csv'))[0]
                self.annotation_data[topic] = pd.concat(csv_data).sort_values(by=['timestamp'])
                self.annotation_timestamp[topic] = pd.read_csv(annotation_timestamp_csv).sort_values(
                    by=['timestamp']).rename(columns={'timestamp': 'time_stamp'}) \
                    .drop_duplicates(subset=['time_stamp'], keep='first')['time_stamp'].values
                self.annotation_hz[topic] = 10

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

        self.analysis_data = {}
        self.analyze_time()
        if '/VA/Obstacles' in self.annotation_data.keys():
            self.analyze_ego()
            self.analyze_obstacles()

        analysis_yaml = os.path.join(self.annotation_folder, 'scenario_analysis.yaml')
        with open(analysis_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(self.analysis_data, f, encoding='utf-8', allow_unicode=True, sort_keys=False)

    def analyze_time(self):
        d = os.path.basename(self.annotation_folder).split('_')[0]
        t = os.path.basename(self.annotation_folder).split('_')[1]
        s = int(t[0:2]) * 3600 + int(t[2:4]) * 60 + int(t[4:6])
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

        print(f'{d[:4]}年的第{d_num}天，日出时间{raise_time}秒，日落时间{set_time}秒')
        print(f'时间标签为{time_label}')
        self.analysis_data['time_label'] = time_label

    def analyze_ego(self):
        self.ego_data = pd.read_csv(os.path.join(self.annotation_folder, 'od_ego.csv'))[
            ['ego_timestamp', 'INS_Speed']].rename(columns={'ego_timestamp': 'time_stamp', 'INS_Speed': 'ego_vx'})
        low_ego_data = self.ego_data[(self.ego_data['ego_vx'] < 5) & (self.ego_data['ego_vx'] > -5)]
        high_ego_data = self.ego_data[(self.ego_data['ego_vx'] >= 5) | (self.ego_data['ego_vx'] <= -5)]
        self.high_timestamp = high_ego_data['time_stamp'].values.tolist()
        self.analysis_data['low_duration'] = len(low_ego_data) / len(self.ego_data)
        self.analysis_data['average_ego_velocity'] = float(high_ego_data['ego_vx'].mean())

    def analyze_obstacles(self):
        font_size = 13
        title_font = {
            'family': 'Ubuntu',
            'style': 'normal',
            'weight': 'normal',
            'color': 'lightskyblue',
            'size': font_size * 1.3,
        }
        axis_font = {
            'family': 'Ubuntu',
            'style': 'normal',
            'weight': 'normal',
            'color': 'black',
            'size': font_size,
        }
        legend_font = {
            'family': 'Ubuntu',
            'style': 'normal',
            'weight': 'normal',
            'size': font_size * 0.8,
        }

        def get_res(df):
            p_quantity = len(df)
            p_average_velocity = df['abs_v'].mean()
            p_flow = df['abs_v'].sum() / (2 * self.area_length)
            p_area = df['area[m2]'].mean()
            if p_quantity:
                return p_quantity, p_average_velocity, p_flow, p_area
            else:
                return np.nan, np.nan, np.nan, np.nan

        data = self.annotation_data['/VA/Obstacles']
        data = data[(data['x'] <= self.area_length)
                    & (data['x'] >= -self.area_length)
                    & (data['y'] <= self.area_width)
                    & (data['y'] >= -self.area_width)]
        self.analysis_data['small_medium_ratio'] = len(data[data['length'] < 6]) / len(data)
        data['abs_v'] = data.apply(lambda v: np.sqrt(v['vx'] ** 2 + v['vy'] ** 2), axis=1)
        data['area[m2]'] = data.apply(lambda v: v['length'] * v['width'], axis=1)
        data = data[data['abs_v'] < 50].replace('vehicle', 'other')

        od_type_list = data['type_text'].drop_duplicates().values.tolist()
        counts = data['type_text'].value_counts()
        valid_type_list = [t for t in od_type_list if counts[t] >= 200]
        data = data[data['type_text'].isin(valid_type_list)]

        timestamp_data = self.annotation_timestamp['/VA/Obstacles']
        columns = ['time_stamp', 'type', 'quantity[]', 'average_velocity[m/s]', 'flow[/s]', 'area[m2]']
        rows = []
        for timestamp in timestamp_data:
            time_data = data[data['time_stamp'] == timestamp]
            rows.append([timestamp, 'total', *get_res(time_data)])

            for od_type in valid_type_list:
                type_data = time_data[time_data['type_text'] == od_type]
                rows.append([timestamp, od_type, *get_res(type_data)])

        res = pd.DataFrame(rows, columns=columns)
        res.to_csv('123.csv', index=False)

        # 提取特征信息
        self.analysis_data['od_info'] = {}
        high_res = res[res['time_stamp'].isin(self.high_timestamp)]
        for type_text in self.db_od_type_list + ['total']:
            type_data = high_res[high_res['type'] == type_text]
            if len(type_data):
                self.analysis_data['od_info'][type_text] = {
                    'valid': 1,
                    'quantity': float(type_data['quantity[]'].sum() / len(type_data)),
                    'average_velocity[m/s]': float(type_data['average_velocity[m/s]'].sum() / len(type_data)),
                    'flow[/s]': float(type_data['flow[/s]'].sum() / len(type_data)),
                    'area[m2]': float(type_data['area[m2]'].sum() / len(type_data)),
                }
            else:
                self.analysis_data['od_info'][type_text] = {
                    'valid': 0,
                    'quantity': 0,
                    'average_velocity[m/s]': 0,
                    'flow[/s]': 0,
                    'area[m2]': 0,
                }

        self.analysis_data['quantity'] = self.analysis_data['od_info']['total']['quantity']
        self.analysis_data['average_velocity[m/s]'] = self.analysis_data['od_info']['total']['average_velocity[m/s]']
        self.analysis_data['flow[/s]'] = self.analysis_data['od_info']['total']['flow[/s]']
        self.analysis_data['area[m2]'] = self.analysis_data['od_info']['total']['area[m2]']
        del self.analysis_data['od_info']['total']

        # 可视化
        fig = plt.figure(figsize=(20, 12))
        fig.tight_layout()
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)

        grid = plt.GridSpec(3, 1, wspace=0.1, hspace=0.25)
        for fig_type in columns[2:-1]:
            columns[2:-1].index(fig_type) % 2
            ax = fig.add_subplot(grid[columns[2:-1].index(fig_type), 0])
            new_valid_type_list = ['total'] + valid_type_list

            for od_type in new_valid_type_list:
                color = colors[new_valid_type_list.index(od_type)]
                type_data = res[res['type'] == od_type]
                alpha = 0.6 if od_type == 'total' else 1
                ax.plot(type_data['time_stamp'], type_data[fig_type], color=color, linestyle='solid',
                        linewidth=1, label=od_type, alpha=alpha)

            if fig_type == 'average_velocity[m/s]':
                ax.plot(self.ego_data['time_stamp'], self.ego_data['ego_vx'], color='black',
                        linestyle='dashed', linewidth=2, label='ego')
            else:
                ax2 = ax.twinx()
                ax2.plot(self.ego_data['time_stamp'], self.ego_data['ego_vx'], color='black',
                         linestyle='dashed', linewidth=2, label='ego')
                ax2.set_ylabel('ego_velocity[m/s]', fontdict=axis_font)

            ax.set_title(fig_type, fontdict=title_font)
            ax.set_xlabel('time[second]', fontdict=axis_font)
            ax.set_ylabel(fig_type, fontdict=axis_font)
            ax.grid('--', color='gainsboro')
            ax.tick_params(direction='out', labelsize=font_size / 1.1, length=2)
            ax.legend(loc=0, prop=legend_font)

        plot_path = os.path.join(self.annotation_folder, 'scenario_plot.png')
        canvas = FigureCanvas(fig)
        canvas.print_figure(plot_path, facecolor='white', dpi=100)
        fig.clf()
        plt.close()


if __name__ == "__main__":
    input_folder_path = '/home/flouqd/Downloads/rosbag_V6'
    output_folder_path = '/home/flouqd/Downloads/rosbag_V6/analysis_result'

    # Get all the csv files starts with VAObstacles
    csv_paths = sorted(Path(input_folder_path).glob('**/VAObstacles*data.csv'))

    tqdm_loop = tqdm([list(j) for i, j in groupby([csv_path for csv_path in csv_paths], key=lambda x: str(x.parent))])
    for grouped_path in tqdm_loop:
        sa = StatsAnalyzer()
        for csv_path in grouped_path:
            sa.analyze(csv_path)

        current_path = grouped_path[0]
        sa.save_csv(current_path, Path(output_folder_path))
        tqdm_loop.set_postfix_str(str(current_path.parent))
        sleep(0.01)
