"""  
Created on 2024/7/5.  
@author: Bu Yujun  
"""
from typing import List

import numpy as np
import pandas as pd
from scipy.optimize import linear_sum_assignment


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
            'x': [4, 0.2],
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
        total_column = (['gt_flag', 'pred_flag']
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
                    loss_data_row = []

                    for _, pred_row in frame_pred_data.iterrows():
                        pred_x = pred_row['x']
                        pred_y = pred_row['y']
                        pred_type = pred_row['type']

                        if self.get_match_flag(gt_type, gt_x, gt_y, pred_type, pred_x, pred_y):
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

    def get_match_flag(self, gt_type, gt_x, gt_y, pred_type, pred_x, pred_y):
        # 如果目标识别结果为人，但真值为车，或者相反，则直接显示为不匹配
        if (gt_type == 1 and pred_type != 1) or (gt_type != 1 and pred_type == 1):
            return False

        x_error = pred_x - gt_x
        x_error_rel = np.sign(x_error) * x_error / max(20, abs(gt_x))
        y_error = pred_y - gt_y
        y_error_rel = np.sign(y_error) * y_error / max(10, abs(gt_y))

        if ((abs(x_error) <= self.object_matching_tolerance['x'][0]
             or abs(x_error_rel) <= self.object_matching_tolerance['x'][1])
                and (abs(y_error) <= self.object_matching_tolerance['y'][0]
                     or abs(y_error_rel) <= self.object_matching_tolerance['y'][1])):
            return True

        return False


if __name__ == '__main__':
    OBT = ObstaclesMatchTool()

    gt_type, gt_x, gt_y, pred_type, pred_x, pred_y = 1, 10, -3, 1, 11, -4
    print(OBT.get_match_flag(gt_type, gt_x, gt_y, pred_type, pred_x, pred_y))
