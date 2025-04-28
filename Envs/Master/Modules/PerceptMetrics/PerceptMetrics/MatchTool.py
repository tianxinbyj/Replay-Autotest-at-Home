"""  
Created on 2024/7/5.  
@author: Bu Yujun  
"""
import os
import sys
from typing import List
import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment
from shapely.geometry import LineString
from shapely.geometry import CAP_STYLE, JOIN_STYLE
from shapely.geometry import Polygon
from scipy.spatial import ConvexHull

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from ObjectModel import Slot

import warnings
warnings.filterwarnings("ignore")


def match_timestamp(
        prediction_timestamp: List[float],
        groundtruth_timestamp: List[float],
        match_tolerance: float):
    rows = []
    for gt_t_idx, gt_t in enumerate(sorted(groundtruth_timestamp)):
        for pred_t_idx, pred_t in enumerate(sorted(prediction_timestamp)):
            delta = abs(gt_t - pred_t)
            if delta < match_tolerance:
                rows.append([gt_t_idx, gt_t, pred_t_idx, pred_t, delta])

    temp_data = pd.DataFrame(rows, columns=['gt_t_idx', 'gt_timestamp', 'pred_t_idx',
                                            'pred_timestamp', 'delta']).sort_values(by=['delta'])

    pred_timestamp, gt_timestamp = [], []
    while len(temp_data):
        gt_timestamp.append(temp_data.at[0, 'gt_timestamp'])
        gt_t_idx = temp_data.at[0, 'gt_t_idx']
        pred_timestamp.append(temp_data.at[0, 'pred_timestamp'])
        pred_t_idx = temp_data.at[0, 'pred_t_idx']
        temp_data.drop(temp_data[(temp_data['gt_t_idx'] == gt_t_idx)
                                 | (temp_data['pred_t_idx'] == pred_t_idx)].index, axis=0, inplace=True)
        if len(temp_data):
            temp_data = temp_data.sort_values(by=['delta']).reset_index(drop=True)
        else:
            break

    return zip(*sorted(zip(pred_timestamp, gt_timestamp)))


class ObstaclesMatchTool:

    def __init__(self):
        self.object_matching_tolerance = {
            'x': [6, 0.3],
            'y': [2, 0.1],
        }

    def run(self, input_data, input_parameter_container=None):
        if input_parameter_container is not None:
            self.object_matching_tolerance = input_parameter_container['object_matching_tolerance']

        match_timestamp = input_data['match_timestamp']
        gt_data = input_data['gt_data']
        pred_data = input_data['pred_data']
        gt_column = gt_data.columns
        pred_column = pred_data.columns
        total_column = (['gt.flag', 'pred.flag']
                        + [f'gt.{col}' for col in gt_column]
                        + [f'pred.{col}' for col in pred_column])
        match_data_rows = []

        for row_idx, row in match_timestamp.iterrows():
            frame_gt_data = gt_data[gt_data['time_stamp'] == row['gt_timestamp']]
            if not len(frame_gt_data):
                print(f'{row["gt_timestamp"]} sec GroundTruth 无目标')

            frame_pred_data = pred_data[pred_data['time_stamp'] == row['pred_timestamp']]
            if not len(frame_pred_data):
                print(f'{row["pred_timestamp"]} sec Prediction 无目标')

            # 将匹配的行的id填入，用于筛选TP,TF和FP
            match_pred_idx_list, match_gt_idx_list = [], []

            # TP样本
            if len(frame_gt_data) and len(frame_pred_data):

                loss_data = []
                for _, gt_row in frame_gt_data.iterrows():
                    gt_x = gt_row['x']
                    gt_y = gt_row['y']
                    gt_type = gt_row['type']
                    gt_pt_0_x = gt_row['pt_0_x']
                    gt_pt_0_y = gt_row['pt_0_y']
                    gt_pt_1_x = gt_row['pt_1_x']
                    gt_pt_1_y = gt_row['pt_1_y']
                    gt_pt_2_x = gt_row['pt_2_x']
                    gt_pt_2_y = gt_row['pt_2_y']
                    gt_pt_3_x = gt_row['pt_3_x']
                    gt_pt_3_y = gt_row['pt_3_y']

                    loss_data_row = []
                    for _, pred_row in frame_pred_data.iterrows():
                        pred_x = pred_row['x']
                        pred_y = pred_row['y']
                        pred_type = pred_row['type']
                        pred_pt_0_x = pred_row['pt_0_x']
                        pred_pt_0_y = pred_row['pt_0_y']
                        pred_pt_1_x = pred_row['pt_1_x']
                        pred_pt_1_y = pred_row['pt_1_y']
                        pred_pt_2_x = pred_row['pt_2_x']
                        pred_pt_2_y = pred_row['pt_2_y']
                        pred_pt_3_x = pred_row['pt_3_x']
                        pred_pt_3_y = pred_row['pt_3_y']

                        if self.get_match_flag(gt_type, gt_x, gt_y,
                                               gt_pt_0_x, gt_pt_0_y, gt_pt_1_x, gt_pt_1_y,
                                               gt_pt_2_x, gt_pt_2_y, gt_pt_3_x, gt_pt_3_y,
                                               pred_type, pred_x, pred_y,
                                               pred_pt_0_x, pred_pt_0_y, pred_pt_1_x, pred_pt_1_y,
                                               pred_pt_2_x, pred_pt_2_y, pred_pt_3_x, pred_pt_3_y):

                            ratio = 1 if gt_type == pred_type else 1.5
                            distance = np.sqrt((pred_x - gt_x) ** 2 + (pred_y - gt_y) ** 2) * ratio
                        else:
                            distance = 500
                        loss_data_row.append(distance)
                    loss_data.append(loss_data_row)

                # 使用匈牙利算法找到最小总距离的匹配
                loss_data = np.array(loss_data)
                loss_threshold = 100
                row_ind, col_ind = linear_sum_assignment(loss_data)
                for i, j in zip(row_ind, col_ind):
                    if loss_data[i, j] >= loss_threshold:
                        continue

                    gt_idx = frame_gt_data.index[i]
                    pred_idx = frame_pred_data.index[j]
                    match_gt_idx_list.append(gt_idx)
                    match_pred_idx_list.append(pred_idx)

                    gt_flag, pred_flag = 1, 1
                    this_row = [gt_flag, pred_flag]
                    for col in gt_column:
                        this_row.append(gt_data.at[gt_idx, col])
                    for col in pred_column:
                        this_row.append(pred_data.at[pred_idx, col])
                    match_data_rows.append(this_row)

            no_match_gt_idx_list = [idx for idx in frame_gt_data.index if idx not in match_gt_idx_list]
            no_match_pred_idx_list = [idx for idx in frame_pred_data.index if idx not in match_pred_idx_list]
            print(f'{row_idx}，{row["gt_timestamp"]:.3f}, 结算完毕, '
                  f'TP-{len(match_pred_idx_list)}, '
                  f'FN-{len(no_match_gt_idx_list)}, '
                  f'FP-{len(no_match_pred_idx_list)}')

            for gt_idx in no_match_gt_idx_list:
                gt_flag, pred_flag = 1, 0
                this_row = [gt_flag, pred_flag]
                for col in gt_column:
                    this_row.append(gt_data.at[gt_idx, col])
                for col in pred_column:
                    if col == 'time_stamp':
                        this_row.append(row['pred_timestamp'])
                    else:
                        this_row.append(None)
                match_data_rows.append(this_row)

            for pred_idx in no_match_pred_idx_list:
                gt_flag, pred_flag = 0, 1
                this_row = [gt_flag, pred_flag]
                for col in gt_column:
                    if col == 'time_stamp':
                        this_row.append(row['gt_timestamp'])
                    else:
                        this_row.append(None)
                for col in pred_column:
                    this_row.append(pred_data.at[pred_idx, col])
                match_data_rows.append(this_row)

        data = pd.DataFrame(match_data_rows, columns=total_column)

        # 进一步过滤极端异常数据
        # 过滤内容为速度和位置
        data.drop(data[
                      (data['gt.x'] > 300) | (data['gt.x'] < -300)
                      | (data['gt.y'] > 200) | (data['gt.y'] < -200)
                  ].index, axis=0, inplace=True)
        data.insert(0, 'corresponding_index', range(len(data)))
        return data

    def get_match_flag(self, gt_type, gt_x, gt_y,
                       gt_pt_0_x, gt_pt_0_y, gt_pt_1_x, gt_pt_1_y,
                       gt_pt_2_x, gt_pt_2_y, gt_pt_3_x, gt_pt_3_y,
                       pred_type, pred_x, pred_y,
                       pred_pt_0_x, pred_pt_0_y, pred_pt_1_x, pred_pt_1_y,
                       pred_pt_2_x, pred_pt_2_y, pred_pt_3_x, pred_pt_3_y):
        # 如果目标识别结果为人，但真值为车，或者相反，则直接显示为不匹配
        if (gt_type == 1 and pred_type == 2) or (gt_type == 2 and pred_type == 1):
            return False

        x_error = pred_x - gt_x
        x_error_rel = np.sign(x_error) * x_error / max(20, abs(gt_x), abs(pred_x))
        y_error = pred_y - gt_y
        y_error_rel = np.sign(y_error) * y_error / max(20, abs(gt_y), abs(pred_y))

        if ((abs(x_error) <= self.object_matching_tolerance['x'][0]
             or abs(x_error_rel) <= self.object_matching_tolerance['x'][1])
                and (abs(y_error) <= self.object_matching_tolerance['y'][0]
                     or abs(y_error_rel) <= self.object_matching_tolerance['y'][1])):
            return True

        def create_polygon(points):
            """将四个无序点转换为有序矩形多边形"""
            points_array = np.array(points)
            hull = ConvexHull(points_array)  # 计算凸包以排序顶点
            ordered_points = points_array[hull.vertices]
            return Polygon(ordered_points)  # 创建Shapely多边形

        def calculate_iou(rect1_points, rect2_points):
            """计算两个矩形的交并比（IoU）"""
            # 创建多边形对象
            poly1 = create_polygon(rect1_points)
            poly2 = create_polygon(rect2_points)
            # 计算交/并集面积
            intersection = poly1.intersection(poly2).area
            union = poly1.area + poly2.area - intersection
            return intersection / union if union != 0 else 0.0

        if ((abs(x_error) <= self.object_matching_tolerance['x'][0] * 1.5
             or abs(x_error_rel) <= self.object_matching_tolerance['x'][1]) * 1.5
                and (abs(y_error) <= self.object_matching_tolerance['y'][0] * 1.5
                     or abs(y_error_rel) <= self.object_matching_tolerance['y'][1]) * 1.5):

            gt_rect = [
                (gt_pt_0_x, gt_pt_0_y), (gt_pt_1_x, gt_pt_1_y),
                (gt_pt_2_x, gt_pt_2_y), (gt_pt_3_x, gt_pt_3_y)
            ]
            pred_rect = [
                (pred_pt_0_x, pred_pt_0_y), (pred_pt_1_x, pred_pt_1_y),
                (pred_pt_2_x, pred_pt_2_y), (pred_pt_3_x, pred_pt_3_y)
            ]
            if calculate_iou(gt_rect, pred_rect) >= 0.05:
                return True

        return False


class LinesMatchTool:

    def __init__(self):
        self.lane_matching_width = 1

    def run(self, input_data, input_parameter_container=None):
        if input_parameter_container is not None:
            self.lane_matching_width = input_parameter_container['lane_matching_width']

        match_timestamp = input_data['match_timestamp']
        gt_data = input_data['gt_data']
        pred_data = input_data['pred_data']
        gt_column = gt_data.columns
        pred_column = pred_data.columns
        total_column = (['gt.flag', 'pred.flag']
                        + [f'gt.{col}' for col in gt_column]
                        + [f'pred.{col}' for col in pred_column] + ['IOU'])
        match_data_rows = []

        for row_idx, row in match_timestamp.iterrows():
            frame_gt_data = gt_data[gt_data['time_stamp'] == row['gt_timestamp']]

            if not len(frame_gt_data):
                print(f'{row["gt_timestamp"]} sec GroundTruth 无目标')

            frame_pred_data = pred_data[pred_data['time_stamp'] == row['pred_timestamp']]
            if not len(frame_pred_data):
                print(f'{row["pred_timestamp"]} sec Prediction 无目标')

            # 将匹配的行的id填入，用于筛选TP,TF和FP
            match_pred_idx_list, match_gt_idx_list = [], []

            # TP样本
            if len(frame_gt_data) and len(frame_pred_data):

                gt_lines_shapely = []
                gt_lines_type = []
                for gt_idx, gt_row in frame_gt_data.iterrows():
                    x_pts = gt_row['x_points'].split(',')
                    y_pts = gt_row['y_points'].split(',')
                    gt_line = [[float(x), float(y)] for x, y in zip(x_pts, y_pts)]
                    gt_lines_type.append(gt_row['type'])
                    gt_lines_shapely.append(LineString(gt_line).buffer(self.lane_matching_width,
                                                                       cap_style=CAP_STYLE.flat,
                                                                       join_style=JOIN_STYLE.mitre))

                pred_lines_shapely = []
                pred_lines_type = []
                for pred_idx, pred_row in frame_pred_data.iterrows():
                    x_pts = pred_row['x_points'].split(',')
                    y_pts = pred_row['y_points'].split(',')
                    pred_line = [[float(x), float(y)] for x, y in zip(x_pts, y_pts)]
                    pred_lines_type.append(pred_row['type'])
                    pred_lines_shapely.append(LineString(pred_line).buffer(self.lane_matching_width,
                                                                           cap_style=CAP_STYLE.flat,
                                                                           join_style=JOIN_STYLE.mitre))

                loss_data = []
                for gt_i, gt_line in enumerate(gt_lines_shapely):

                    loss_data_row = []
                    for pred_i, pred_line in enumerate(pred_lines_shapely):
                        union = pred_line.union(gt_line)
                        intersection = pred_line.intersection(gt_line)
                        IOU = intersection.area/union.area
                        if gt_lines_type[gt_i] != pred_lines_type[pred_i]:
                            IOU /= 1.5

                        loss_data_row.append(-IOU)

                    loss_data.append(loss_data_row)

                # 使用匈牙利算法找到最小总距离的匹配
                loss_data = np.array(loss_data)

                if np.sum(loss_data):
                    row_ind, col_ind = linear_sum_assignment(loss_data)
                    for i, j in zip(row_ind, col_ind):
                        if loss_data[i, j] >= -0.1:
                            continue

                        gt_idx = frame_gt_data.index[i]
                        pred_idx = frame_pred_data.index[j]
                        match_gt_idx_list.append(gt_idx)
                        match_pred_idx_list.append(pred_idx)

                        gt_flag, pred_flag = 1, 1
                        this_row = [gt_flag, pred_flag]
                        for col in gt_column:
                            this_row.append(gt_data.at[gt_idx, col])
                        for col in pred_column:
                            this_row.append(pred_data.at[pred_idx, col])
                        this_row.append(-loss_data[i, j])
                        match_data_rows.append(this_row)

            no_match_gt_idx_list = [idx for idx in frame_gt_data.index if idx not in match_gt_idx_list]
            no_match_pred_idx_list = [idx for idx in frame_pred_data.index if idx not in match_pred_idx_list]
            print(f'{row_idx}，{row["gt_timestamp"]:.3f}, 结算完毕, '
                  f'TP-{len(match_pred_idx_list)}, '
                  f'FN-{len(no_match_gt_idx_list)}, '
                  f'FP-{len(no_match_pred_idx_list)}')

            for gt_idx in no_match_gt_idx_list:
                gt_flag, pred_flag = 1, 0
                this_row = [gt_flag, pred_flag]
                for col in gt_column:
                    this_row.append(gt_data.at[gt_idx, col])
                for col in pred_column:
                    if col == 'time_stamp':
                        this_row.append(row['pred_timestamp'])
                    else:
                        this_row.append(None)
                this_row.append(0)
                match_data_rows.append(this_row)

            for pred_idx in no_match_pred_idx_list:
                gt_flag, pred_flag = 0, 1
                this_row = [gt_flag, pred_flag]
                for col in gt_column:
                    if col == 'time_stamp':
                        this_row.append(row['gt_timestamp'])
                    else:
                        this_row.append(None)
                for col in pred_column:
                    this_row.append(pred_data.at[pred_idx, col])
                this_row.append(0)
                match_data_rows.append(this_row)

        data = pd.DataFrame(match_data_rows, columns=total_column)

        data.insert(0, 'corresponding_index', range(len(data)))
        return data.dropna(subset=['gt.type_classification', 'pred.type_classification'], how='all')


class SlotsMatchTool:

    def __init__(self):
        self.slot_matching_tolerance = {
            'x': 0.5,
            'y': 0.5,
        }

    def run(self, input_data, input_parameter_container=None):
        if input_parameter_container is not None:
            self.slot_matching_tolerance = input_parameter_container['slot_matching_tolerance']

        match_timestamp = input_data['match_timestamp']
        gt_data = input_data['gt_data']
        pred_data = input_data['pred_data']
        gt_column = gt_data.columns
        pred_column = pred_data.columns
        total_column = (['gt.flag', 'pred.flag']
                        + [f'gt.{col}' for col in gt_column]
                        + [f'pred.{col}' for col in pred_column])
        match_data_rows = []

        for row_idx, row in match_timestamp.iterrows():
            frame_gt_data = gt_data[gt_data['time_stamp'] == row['gt_timestamp']]
            if not len(frame_gt_data):
                print(f'{row["gt_timestamp"]} sec GroundTruth 无目标')

            frame_pred_data = pred_data[pred_data['time_stamp'] == row['pred_timestamp']]
            if not len(frame_pred_data):
                print(f'{row["pred_timestamp"]} sec Prediction 无目标')

            # 将匹配的行的id填入，用于筛选TP,TF和FP
            match_pred_idx_list, match_gt_idx_list = [], []

            if len(frame_gt_data) and len(frame_pred_data):

                loss_data = []
                for _, gt_row in frame_gt_data.iterrows():
                    gt_x = gt_row['center_x']
                    gt_y = gt_row['center_y']

                    loss_data_row = []
                    for _, pred_row in frame_pred_data.iterrows():
                        pred_x = pred_row['center_x']
                        pred_y = pred_row['center_y']

                        if self.get_match_flag(gt_x, gt_y, pred_x, pred_y):
                            distance = np.sqrt((pred_x - gt_x) ** 2 + (pred_y - gt_y) ** 2)
                        else:
                            distance = 500
                        loss_data_row.append(distance)
                    loss_data.append(loss_data_row)

                # 使用匈牙利算法找到最小总距离的匹配
                loss_data = np.array(loss_data)
                loss_threshold = 100
                row_ind, col_ind = linear_sum_assignment(loss_data)
                for i, j in zip(row_ind, col_ind):
                    if loss_data[i, j] >= loss_threshold:
                        continue

                    gt_idx = frame_gt_data.index[i]
                    pred_idx = frame_pred_data.index[j]
                    match_gt_idx_list.append(gt_idx)
                    match_pred_idx_list.append(pred_idx)

                    # 更换gt点的顺序和类型
                    gt_data.loc[gt_idx, ['pt_0_x', 'pt_0_y', 'pt_1_x', 'pt_1_y', 'pt_2_x', 'pt_2_y', 'pt_3_x', 'pt_3_y']] = self.sort_gt_corner_point(
                        *gt_data.loc[gt_idx, ['pt_0_x', 'pt_0_y', 'pt_1_x', 'pt_1_y', 'pt_2_x', 'pt_2_y', 'pt_3_x', 'pt_3_y']],
                        *pred_data.loc[pred_idx, ['pt_0_x', 'pt_0_y', 'pt_1_x', 'pt_1_y', 'pt_2_x', 'pt_2_y', 'pt_3_x', 'pt_3_y']]
                    )
                    pred_type = pred_data.at[pred_idx, 'type']
                    if pred_type == 1:
                        type_classification = 'vertical'
                    elif pred_type == 2:
                        type_classification = 'parallel'
                    elif pred_type == 3:
                        type_classification = 'oblique'
                    else:
                        type_classification = 'unknown'

                    gt_data.at[gt_idx, 'type'] = pred_data.at[pred_idx, 'type']
                    gt_data.at[gt_idx, 'type_classification'] = pred_data.at[pred_idx, 'type_classification'] = type_classification

                    # 计算车位的属性
                    gt_slot = Slot(*gt_data.loc[gt_idx,
                    ['type', 'pt_0_x', 'pt_0_y', 'pt_1_x', 'pt_1_y',
                     'pt_2_x', 'pt_2_y', 'pt_3_x', 'pt_3_y',
                     'stopper_0_x', 'stopper_0_y', 'stopper_1_x', 'stopper_1_y']])
                    gt_data.loc[gt_idx,
                    ['in_border_distance', 'in_border_length', 'slot_length',
                     'slot_heading', 'stopper_depth', 'slot_distance']] = [
                        gt_slot.in_border_distance, gt_slot.in_border_length, gt_slot.slot_length,
                        gt_slot.slot_heading, gt_slot.stopper_depth, gt_slot.get_slot_distance()
                    ]

                    pred_slot = Slot(*pred_data.loc[pred_idx,
                    ['type', 'pt_0_x', 'pt_0_y', 'pt_1_x', 'pt_1_y',
                     'pt_2_x', 'pt_2_y', 'pt_3_x', 'pt_3_y',
                     'stopper_0_x', 'stopper_0_y', 'stopper_1_x', 'stopper_1_y']])
                    pred_data.loc[pred_idx,
                    ['in_border_distance', 'in_border_length', 'slot_length',
                     'slot_heading', 'stopper_depth', 'slot_distance']] = [
                        pred_slot.in_border_distance, pred_slot.in_border_length, pred_slot.slot_length,
                        pred_slot.slot_heading, pred_slot.stopper_depth, pred_slot.get_slot_distance()
                    ]

                    gt_flag, pred_flag = 1, 1
                    this_row = [gt_flag, pred_flag]
                    for col in gt_column:
                        this_row.append(gt_data.at[gt_idx, col])
                    for col in pred_column:
                        this_row.append(pred_data.at[pred_idx, col])
                    match_data_rows.append(this_row)

            no_match_gt_idx_list = [idx for idx in frame_gt_data.index if idx not in match_gt_idx_list]
            no_match_pred_idx_list = [idx for idx in frame_pred_data.index if idx not in match_pred_idx_list]
            print(f'{row_idx}，{row["gt_timestamp"]:.3f}, 结算完毕, '
                  f'TP-{len(match_pred_idx_list)}, '
                  f'FN-{len(no_match_gt_idx_list)}, '
                  f'FP-{len(no_match_pred_idx_list)}')

            for gt_idx in no_match_gt_idx_list:
                gt_flag, pred_flag = 1, 0
                this_row = [gt_flag, pred_flag]
                for col in gt_column:
                    this_row.append(gt_data.at[gt_idx, col])
                for col in pred_column:
                    if col == 'time_stamp':
                        this_row.append(row['pred_timestamp'])
                    else:
                        this_row.append(None)
                match_data_rows.append(this_row)

            for pred_idx in no_match_pred_idx_list:
                gt_flag, pred_flag = 0, 1
                this_row = [gt_flag, pred_flag]
                for col in gt_column:
                    if col == 'time_stamp':
                        this_row.append(row['gt_timestamp'])
                    else:
                        this_row.append(None)
                for col in pred_column:
                    this_row.append(pred_data.at[pred_idx, col])
                match_data_rows.append(this_row)

        data = pd.DataFrame(match_data_rows, columns=total_column)
        data.insert(0, 'corresponding_index', range(len(data)))
        return data

    def get_match_flag(self, gt_x, gt_y, pred_x, pred_y):
        if (abs(gt_x - pred_x) <= self.slot_matching_tolerance['x']
                and abs(gt_y - pred_y) <= self.slot_matching_tolerance['y']):
            return True
        return False

    def sort_gt_corner_point(self,
                             gt_pt_0_x, gt_pt_0_y, gt_pt_1_x, gt_pt_1_y,
                             gt_pt_2_x, gt_pt_2_y, gt_pt_3_x, gt_pt_3_y,
                             pred_pt_0_x, pred_pt_0_y, pred_pt_1_x, pred_pt_1_y,
                             pred_pt_2_x, pred_pt_2_y, pred_pt_3_x, pred_pt_3_y):

        gt_points = [
            (gt_pt_0_x, gt_pt_0_y), (gt_pt_1_x, gt_pt_1_y), (gt_pt_2_x, gt_pt_2_y), (gt_pt_3_x, gt_pt_3_y),
        ]
        pred_points = [
            (pred_pt_0_x, pred_pt_0_y), (pred_pt_1_x, pred_pt_1_y), (pred_pt_2_x, pred_pt_2_y), (pred_pt_3_x, pred_pt_3_y),
        ]

        # 构建成本矩阵（4x4）
        cost_matrix = np.zeros((4, 4))
        for i, pred_pt in enumerate(pred_points):
            for j, gt_pt in enumerate(gt_points):
                # 计算欧几里得距离作为成本
                cost_matrix[i, j] = np.linalg.norm(np.array(pred_pt) - np.array(gt_pt))

        # 应用匈牙利算法找到最优匹配
        # row_ind是预测点的索引，col_ind是GT点的索引
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # 按照预测点的顺序对GT点进行排序
        # 先创建一个临时数组来存储排序结果
        sorted_gt = [None] * 4
        for pred_idx, gt_idx in zip(row_ind, col_ind):
            sorted_gt[pred_idx] = gt_points[gt_idx]

        return [coord for pt in sorted_gt for coord in pt]


if __name__ == '__main__':
    pred_data_path = '/home/hp/下载/44444/test_bevlines/04_TestData/2-Lines/01_ScenarioUnit/20240129_155339_n000004/01_Data/Lines/VABevLines/additional/pred_data.csv'
    gt_data_path = '/home/hp/下载/44444/test_bevlines/04_TestData/2-Lines/01_ScenarioUnit/20240129_155339_n000004/01_Data/Lines/GroundTruth/additional/VABevLines_gt_data.csv'
    match_timestamp_path = '/home/hp/下载/44444/test_bevlines/04_TestData/2-Lines/01_ScenarioUnit/20240129_155339_n000004/01_Data/Lines/VABevLines/match/match_timestamp.csv'

    pred_data = pd.read_csv(pred_data_path, index_col=False)
    gt_data = pd.read_csv(gt_data_path, index_col=False)
    match_timestamp = pd.read_csv(match_timestamp_path, index_col=False)

    parameter_json = {
        'lane_matching_width': 1
    }

    match_tool = LinesMatchTool()

    input_data = {
        'match_timestamp': match_timestamp,
        'gt_data': gt_data,
        'pred_data': pred_data,
    }

    data = match_tool.run(input_data, parameter_json)
    data.to_csv('123.csv', index=False, encoding='utf_8_sig')