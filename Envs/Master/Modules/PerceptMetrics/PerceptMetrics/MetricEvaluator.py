"""  
Created on 2024/7/24.  
@author: Bu Yujun  
"""
import numpy as np
import pandas as pd


def change_name(var):
    return ''.join([v.title() for v in var.split('_')])


class RecallPrecision:
    """
    为准召信息表格增加range筛选的

    """

    def __init__(self):
        self.columns = [
            'TP',
            'FP',
            'FN',
            'CTP',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            (gt_flag, gt_x, gt_y, gt_type,
             pred_flag, pred_x, pred_y, pred_type) = (
                input_data['gt.flag'], input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['pred.flag'], input_data['pred.x'], input_data['pred.y'],
                input_data['pred.type_classification'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 8):
            (gt_flag, gt_x, gt_y, gt_type,
             pred_flag, pred_x, pred_y, pred_type) = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        CTP = 0
        if gt_flag == 1 and pred_flag == 1:
            TP, FP, FN = 1, 0, 0
            if pred_type == gt_type:
                CTP = 1

        elif gt_flag == 0 and pred_flag == 1:
            TP, FP, FN = 0, 1, 0

        elif gt_flag == 1 and pred_flag == 0:
            TP, FP, FN = 0, 0, 1

        else:
            TP, FP, FN = 0, 0, 0

        return TP, FP, FN, CTP


class XError:
    """
    计算物体的纵向距离偏差

    """

    def __init__(self):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'pred.x',
            'x.error',
            'x.error_abs',
            'x.error%',
            'x.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_road_user, pred_x = (
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['pred.x'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            gt_x, gt_y, gt_type, gt_road_user, pred_x = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        x_error = pred_x - gt_x
        x_error_abs = abs(x_error)
        x_error_p = x_error / max(20, abs(gt_x))
        x_error_p_abs = abs(x_error_p)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if x_error_abs > 2:
                is_abnormal = 1
        else:
            if x_error_p_abs > 0.1:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type, gt_road_user,
                pred_x, x_error, x_error_abs, x_error_p, x_error_p_abs,
                is_abnormal)


class YError:
    """
    计算物体的横向距离偏差

    """

    def __init__(self):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'pred.y',
            'y.error',
            'y.error_abs',
            'y.error%',
            'y.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_road_user, pred_y = (
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['pred.y'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            gt_x, gt_y, gt_type, gt_road_user, pred_y = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        y_error = pred_y - gt_y
        y_error_abs = abs(y_error)
        y_error_p = y_error / max(10, abs(gt_y))
        y_error_p_abs = abs(y_error_p)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if y_error_abs > 1:
                is_abnormal = 1
        else:
            if y_error_abs > 1.5:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type, gt_road_user,
                pred_y, y_error, y_error_abs, y_error_p, y_error_p_abs,
                is_abnormal)


class VxError:
    """
    计算物体的纵向速度偏差

    """

    def __init__(self):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'gt.vx',
            'pred.vx',
            'vx.error',
            'vx.error_abs',
            'vx.error%',
            'vx.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_road_user, gt_vx, pred_vx = (
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.vx'],
                input_data['pred.vx'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 6):
            gt_x, gt_y, gt_type, gt_road_user, gt_vx, pred_vx = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        vx_error = pred_vx - gt_vx
        vx_error_abs = abs(vx_error)
        vx_error_p = vx_error / max(2, abs(gt_vx))
        vx_error_p_abs = abs(vx_error_p)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if vx_error_abs > 2 and vx_error_p > 0.2:
                is_abnormal = 1
        else:
            if vx_error_abs > 4 and vx_error_p > 0.2:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type, gt_road_user,
                gt_vx, pred_vx, vx_error, vx_error_abs, vx_error_p, vx_error_p_abs,
                is_abnormal)


class VyError:
    """
    计算物体的横向速度偏差

    """

    def __init__(self):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'gt.vy',
            'pred.vy',
            'vy.error',
            'vy.error_abs',
            'vy.error%',
            'vy.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_road_user, gt_vy, pred_vy = (
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.vy'], input_data['pred.vy'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 6):
            gt_x, gt_y, gt_type, gt_road_user, gt_vy, pred_vy = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        vy_error = pred_vy - gt_vy
        vy_error_abs = abs(vy_error)
        vy_error_p = vy_error / max(2, abs(gt_vy))
        vy_error_p_abs = abs(vy_error_p)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if vy_error_abs > 1 and vy_error_p_abs > 0.2:
                is_abnormal = 1
        else:
            if vy_error_abs > 2 and vy_error_p_abs > 0.2:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type, gt_road_user,
                gt_vy, pred_vy, vy_error, vy_error_abs, vy_error_p, vy_error_p_abs,
                is_abnormal)


class YawError:
    """
    计算物体的航向角偏差

    """

    def __init__(self):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'gt.yaw',
            'pred.yaw',
            'yaw.error',
            'yaw.error_abs',
            'yaw.is_reverse',
            'is_abnormal',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_road_user, gt_yaw, pred_yaw = (
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.yaw'], input_data['pred.yaw'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 6):
            gt_x, gt_y, gt_type, gt_road_user, gt_yaw, pred_yaw = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 将角度修正到-pi到pi之间
        pred_yaw = self.make_yaw_pi(pred_yaw)
        gt_yaw = self.make_yaw_pi(gt_yaw)
        yaw_error = self.make_yaw_pi(pred_yaw - gt_yaw)

        # 识别反向角度，计算error时排除反向干扰
        if abs(yaw_error) > 2 * np.pi / 3:
            if pred_yaw < 0:
                pred_yaw = pred_yaw + np.pi
            else:
                pred_yaw = pred_yaw - np.pi
            is_reverse = 1
            yaw_error = self.make_yaw_pi(pred_yaw - gt_yaw)
        else:
            is_reverse = 0

        is_abnormal = 0
        if is_reverse:
            is_abnormal = 1
        else:
            if abs(gt_x) <= 20:
                if abs(yaw_error) > np.deg2rad(10):
                    is_abnormal = 1
            else:
                if abs(yaw_error) > np.deg2rad(20):
                    is_abnormal = 1

        return (gt_x, gt_y, gt_type, gt_road_user,
                np.rad2deg(gt_yaw), np.rad2deg(pred_yaw), np.rad2deg(yaw_error), np.rad2deg(abs(yaw_error)),
                is_reverse, is_abnormal)

    def make_yaw_pi(self, yaw):
        while True:
            if yaw <= np.pi:
                break
            yaw -= 2 * np.pi

        while True:
            if yaw >= -np.pi:
                break
            yaw += 2 * np.pi

        return yaw


class LengthError:

    def __init__(self):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'gt.length',
            'pred.length',
            'length.error',
            'length.error_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_road_user, gt_length, pred_length = (
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.length'], input_data['pred.length'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 6):
            gt_x, gt_y, gt_type, gt_road_user, gt_length, pred_length = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        length_error = pred_length - gt_length
        length_error_abs = abs(length_error)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if length_error > 1 and length_error_abs > 0.2:
                is_abnormal = 1
        else:
            if length_error > 2 and length_error_abs > 0.2:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type, gt_road_user,
                gt_length, pred_length, length_error, length_error_abs,
                is_abnormal)


class WidthError:

    def __init__(self):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'gt.width',
            'pred.width',
            'width.error',
            'width.error_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_road_user, gt_width, pred_width = (
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.width'], input_data['pred.width'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 6):
            gt_x, gt_y, gt_type, gt_road_user, gt_width, pred_width = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        width_error = pred_width - gt_width
        width_error_abs = abs(width_error)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if width_error > 0.4:
                is_abnormal = 1
        else:
            if width_error > 0.8:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type, gt_road_user,
                gt_width, pred_width, width_error, width_error_abs,
                is_abnormal)


class HeightError:

    def __init__(self):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'gt.height',
            'pred.height',
            'height.error',
            'height.error_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_road_user, gt_height, pred_height = (
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.height'], input_data['pred.height'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 6):
            gt_x, gt_y, gt_type, gt_road_user, gt_height, pred_height = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        height_error = pred_height - gt_height
        height_error_abs = abs(height_error)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if height_error > 0.4:
                is_abnormal = 1
        else:
            if height_error > 0.8:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type, gt_road_user,
                gt_height, pred_height, height_error, height_error_abs,
                is_abnormal)


class ObstaclesMetricEvaluator:

    def __init__(self):
        self.metric_type = [
            'recall_precision',
            'x_error', 'y_error',
            'vx_error', 'vy_error',
            'yaw_error', 'length_error',
            'width_error', 'height_error',
        ]

    def run(self, input_data, input_parameter_container=None):
        if input_parameter_container is not None:
            self.metric_type = input_parameter_container['metric_type']

        data_dict = {}
        data = input_data['data']
        tp_data = data[(data['gt.flag'] == 1) & (data['pred.flag'] == 1)]

        for metric in self.metric_type:
            print(f'正在评估指标 {metric}')
            metric_class = change_name(metric)
            func = eval(f'{metric_class}()')

            if metric == 'recall_precision':
                result_df = data.apply(lambda row: func(row.to_dict()), axis=1, result_type='expand')
                result_df.columns = func.columns
                result_df = pd.concat([data, result_df], axis=1)
            else:
                result_df = tp_data.apply(lambda row: func(row.to_dict()), axis=1, result_type='expand')
                result_df.columns = func.columns
                result_df['corresponding_index'] = tp_data['corresponding_index']

            data_dict[metric] = result_df

        return data_dict


class ObstaclesMetricFilter:

    def __init__(self):
        self.characteristic_type = [
            'is_coverageValid',
            'is_cipv',
            'is_keyObj',
            'is_sameDir',
            'is_oppositeDir',
            'is_crossingDir',
            'is_moving',
            'is_static',
        ]

    def run(self, input_data, input_parameter_container=None):
        if input_parameter_container is not None:
            self.characteristic_type = input_parameter_container['characteristic_type']

        # 符合有效探测范围的数据
        total_data = input_data['total_data']
        total_data.drop(total_data[(total_data['gt.is_detectedValid'] == 0)
                                   | (total_data['pred.is_detectedValid'] == 0)].index, axis=0, inplace=True)
        total_data.index = total_data['corresponding_index'].to_list()

        data = input_data['data_to_filter']

        # 获得各种characteristic的corresponding_index
        # 不管是何种characteristic, 遮挡率必须符合要求的数据
        corresponding_index_dict = {characteristic: [] for characteristic in self.characteristic_type}
        for idx, row in total_data.iterrows():
            for characteristic in self.characteristic_type:
                if row['gt.flag'] == 1:
                    if row[f'gt.{characteristic}'] == 1 and row['gt.is_coverageValid'] == 1:
                        corresponding_index_dict[characteristic].append(idx)
                elif row['pred.flag'] == 1:
                    if row[f'pred.{characteristic}'] == 1 and row['pred.is_coverageValid'] == 1:
                        corresponding_index_dict[characteristic].append(idx)

        characteristic_data_dict = {}
        for characteristic in self.characteristic_type:
            print(f'正在评估特征 {characteristic}')
            characteristic_data_dict[characteristic] = data[
                data['corresponding_index'].isin(corresponding_index_dict[characteristic])]

        return characteristic_data_dict
