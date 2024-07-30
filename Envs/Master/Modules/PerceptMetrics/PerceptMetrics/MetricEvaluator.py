"""  
Created on 2024/7/24.  
@author: Bu Yujun  
"""
import numpy as np


def change_name(var):
    return ''.join([v.title() for v in var.split('_')])


class XError:
    """
    计算物体的纵向距离偏差

    """

    def __init__(self, evaluate_range):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'pred.x',
            'x.error',
            'x.error_abs',
            'x.error%',
            'x.error%_abs',
            'x.is_abnormal',
            'is_valid',
        ]
        self.evaluate_range = evaluate_range

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, pred_x = (
                input_data['gt.x'], input_data['gt.y'], input_data['gt.type_classification'],
                input_data['pred.x'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 4):
            gt_x, gt_y, gt_type, pred_x = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 判断是否需要计算该指标
        is_valid = 1
        if gt_type not in self.evaluate_range:
            is_valid = 0
        else:
            x_range = self.evaluate_range[gt_type]['x']
            y_range = self.evaluate_range[gt_type]['y']
            if not (x_range[0] <= gt_x <= x_range[1] and y_range[0] <= gt_y <= y_range[1]):
                is_valid = 0

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

        return (gt_x, gt_y, gt_type,
                pred_x, x_error, x_error_abs, x_error_p, x_error_p_abs,
                is_abnormal, is_valid)


class YError:
    """
    计算物体的横向距离偏差

    """

    def __init__(self, evaluate_range):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'pred.y',
            'y.error',
            'y.error_abs',
            'y.error%',
            'y.error%_abs',
            'y.is_abnormal',
            'is_valid',
        ]
        self.evaluate_range = evaluate_range

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, pred_y = (
                input_data['gt.x'], input_data['gt.y'], input_data['gt.type_classification'],
                input_data['pred.y'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 4):
            gt_x, gt_y, gt_type, pred_y = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 判断是否需要计算该指标
        is_valid = 1
        if gt_type not in self.evaluate_range:
            is_valid = 0
        else:
            x_range = self.evaluate_range[gt_type]['x']
            y_range = self.evaluate_range[gt_type]['y']
            if not (x_range[0] <= gt_x <= x_range[1] and y_range[0] <= gt_y <= y_range[1]):
                is_valid = 0

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

        return (gt_x, gt_y, gt_type,
                pred_y, y_error, y_error_abs, y_error_p, y_error_p_abs,
                is_abnormal, is_valid)


class VxError:
    """
    计算物体的纵向速度偏差

    """

    def __init__(self, evaluate_range):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.vx',
            'pred.vx',
            'vx.error',
            'vx.error_abs',
            'vx.error%',
            'vx.error%_abs',
            'vx.is_abnormal',
            'is_valid',
        ]
        self.evaluate_range = evaluate_range

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_vx, pred_vx = (
                input_data['gt.x'], input_data['gt.y'], input_data['gt.type_classification'],
                input_data['gt.vx'], input_data['pred.vx'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            gt_x, gt_y, gt_type, gt_vx, pred_vx = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 判断是否需要计算该指标
        is_valid = 1
        if gt_type not in self.evaluate_range:
            is_valid = 0
        else:
            x_range = self.evaluate_range[gt_type]['x']
            y_range = self.evaluate_range[gt_type]['y']
            if not (x_range[0] <= gt_x <= x_range[1] and y_range[0] <= gt_y <= y_range[1]):
                is_valid = 0

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

        return (gt_x, gt_y, gt_type,
                gt_vx, pred_vx, vx_error, vx_error_abs, vx_error_p, vx_error_p_abs,
                is_abnormal, is_valid)


class VyError:
    """
    计算物体的横向速度偏差

    """

    def __init__(self, evaluate_range):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.vy',
            'pred.vy',
            'vy.error',
            'vy.error_abs',
            'vy.error%',
            'vy.error%_abs',
            'vy.is_abnormal',
            'is_valid',
        ]
        self.evaluate_range = evaluate_range

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_vy, pred_vy = (
                input_data['gt.x'], input_data['gt.y'], input_data['gt.type_classification'],
                input_data['gt.vy'], input_data['pred.vy'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            gt_x, gt_y, gt_type, gt_vy, pred_vy = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 判断是否需要计算该指标
        is_valid = 1
        if gt_type not in self.evaluate_range:
            is_valid = 0
        else:
            x_range = self.evaluate_range[gt_type]['x']
            y_range = self.evaluate_range[gt_type]['y']
            if not (x_range[0] <= gt_x <= x_range[1] and y_range[0] <= gt_y <= y_range[1]):
                is_valid = 0

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

        return (gt_x, gt_y, gt_type,
                gt_vy, pred_vy, vy_error, vy_error_abs, vy_error_p, vy_error_p_abs,
                is_abnormal, is_valid)


class YawError:
    """
    计算物体的航向角偏差

    """

    def __init__(self, evaluate_range):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.yaw',
            'pred.yaw',
            'yaw.error',
            'yaw.error_abs',
            'yaw.is_reverse',
            'yaw.is_abnormal',
            'is_valid',
        ]
        self.evaluate_range = evaluate_range

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_yaw, pred_yaw = (
                input_data['gt.x'], input_data['gt.y'], input_data['gt.type_classification'],
                input_data['gt.yaw'], input_data['pred.yaw'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            gt_x, gt_y, gt_type, gt_yaw, pred_yaw = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 判断是否需要计算该指标
        is_valid = 1
        if gt_type not in self.evaluate_range:
            is_valid = 0
        else:
            x_range = self.evaluate_range[gt_type]['x']
            y_range = self.evaluate_range[gt_type]['y']
            if not (x_range[0] <= gt_x <= x_range[1] and y_range[0] <= gt_y <= y_range[1]):
                is_valid = 0

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

        return (gt_x, gt_y, gt_type,
                np.rad2deg(gt_yaw), np.rad2deg(pred_yaw), np.rad2deg(yaw_error), np.rad2deg(abs(yaw_error)),
                is_reverse, is_abnormal, is_valid)

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

    def __init__(self, evaluate_range):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.length',
            'pred.length',
            'length.error',
            'length.error_abs',
            'length.is_abnormal',
            'is_valid',
        ]
        self.evaluate_range = evaluate_range

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_length, pred_length = (
                input_data['gt.x'], input_data['gt.y'], input_data['gt.type_classification'],
                input_data['gt.length'], input_data['pred.length'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            gt_x, gt_y, gt_type, gt_length, pred_length = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 判断是否需要计算该指标
        is_valid = 1
        if gt_type not in self.evaluate_range:
            is_valid = 0
        else:
            x_range = self.evaluate_range[gt_type]['x']
            y_range = self.evaluate_range[gt_type]['y']
            if not (x_range[0] <= gt_x <= x_range[1] and y_range[0] <= gt_y <= y_range[1]):
                is_valid = 0

        length_error = pred_length - gt_length
        length_error_abs = abs(length_error)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if length_error > 1 and length_error_abs > 0.2:
                is_abnormal = 1
        else:
            if length_error > 2 and length_error_abs > 0.2:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type,
                gt_length, pred_length, length_error, length_error_abs,
                is_abnormal, is_valid)


class WidthError:

    def __init__(self, evaluate_range):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.width',
            'pred.width',
            'width.error',
            'width.error_abs',
            'width.is_abnormal',
            'is_valid',
        ]
        self.evaluate_range = evaluate_range

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_width, pred_width = (
                input_data['gt.x'], input_data['gt.y'], input_data['gt.type_classification'],
                input_data['gt.width'], input_data['pred.width'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            gt_x, gt_y, gt_type, gt_width, pred_width = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 判断是否需要计算该指标
        is_valid = 1
        if gt_type not in self.evaluate_range:
            is_valid = 0
        else:
            x_range = self.evaluate_range[gt_type]['x']
            y_range = self.evaluate_range[gt_type]['y']
            if not (x_range[0] <= gt_x <= x_range[1] and y_range[0] <= gt_y <= y_range[1]):
                is_valid = 0

        width_error = pred_width - gt_width
        width_error_abs = abs(width_error)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if width_error > 0.4:
                is_abnormal = 1
        else:
            if width_error > 0.8:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type,
                gt_width, pred_width, width_error, width_error_abs,
                is_abnormal, is_valid)


class HeightError:

    def __init__(self, evaluate_range):
        self.columns = [
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.height',
            'pred.height',
            'height.error',
            'height.error_abs',
            'height.is_abnormal',
            'is_valid',
        ]
        self.evaluate_range = evaluate_range

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, gt_y, gt_type, gt_height, pred_height = (
                input_data['gt.x'], input_data['gt.y'], input_data['gt.type_classification'],
                input_data['gt.height'], input_data['pred.height'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            gt_x, gt_y, gt_type, gt_height, pred_height = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 判断是否需要计算该指标
        is_valid = 1
        if gt_type not in self.evaluate_range:
            is_valid = 0
        else:
            x_range = self.evaluate_range[gt_type]['x']
            y_range = self.evaluate_range[gt_type]['y']
            if not (x_range[0] <= gt_x <= x_range[1] and y_range[0] <= gt_y <= y_range[1]):
                is_valid = 0

        height_error = pred_height - gt_height
        height_error_abs = abs(height_error)

        is_abnormal = 0
        if abs(gt_x) <= 20:
            if height_error > 0.4:
                is_abnormal = 1
        else:
            if height_error > 0.8:
                is_abnormal = 1

        return (gt_x, gt_y, gt_type,
                gt_height, pred_height, height_error, height_error_abs,
                is_abnormal, is_valid)


class ObstaclesMetricEvaluator:

    def __init__(self):
        self.metric_type = [
            'recall_precision',
            'x_error', 'y_error',
            'vx_error', 'vy_error',
            'yaw_error', 'length_error',
            'width_error', 'height_error',
        ]
        self.evaluate_range = {'x_error': {1: {'x': [-100, 150], 'y': [-30, 30]}, 2: {'x': [-20, 60], 'y': [-20, 20]},
                                           4: {'x': [-100, 150], 'y': [-30, 30]}, 5: {'x': [-100, 150], 'y': [-30, 30]},
                                           18: {'x': [-20, 60], 'y': [-20, 20]}},
                               'y_error': {1: {'x': [-100, 150], 'y': [-30, 30]}, 2: {'x': [-20, 60], 'y': [-20, 20]},
                                           4: {'x': [-100, 150], 'y': [-30, 30]}, 5: {'x': [-100, 150], 'y': [-30, 30]},
                                           18: {'x': [-20, 60], 'y': [-20, 20]}},
                               'vx_error': {1: {'x': [-20, 60], 'y': [-20, 20]}, 2: {'x': [-20, 60], 'y': [-20, 20]},
                                            4: {'x': [-20, 60], 'y': [-20, 20]}, 5: {'x': [-20, 60], 'y': [-20, 20]},
                                            18: {'x': [-20, 60], 'y': [-20, 20]}},
                               'vy_error': {1: {'x': [-60, 60], 'y': [-20, 20]}, 2: {'x': [-20, 60], 'y': [-20, 20]},
                                            4: {'x': [-60, 60], 'y': [-20, 20]}, 5: {'x': [-60, 60], 'y': [-20, 20]},
                                            18: {'x': [-20, 60], 'y': [-20, 20]}},
                               'yaw_error': {1: {'x': [-60, 60], 'y': [-20, 20]}, 4: {'x': [-60, 60], 'y': [-20, 20]},
                                             5: {'x': [-60, 60], 'y': [-20, 20]}, 18: {'x': [-20, 60], 'y': [-20, 20]}},
                               'length_error': {1: {'x': [-60, 60], 'y': [-20, 20]},
                                                4: {'x': [-60, 60], 'y': [-20, 20]},
                                                5: {'x': [-60, 60], 'y': [-20, 20]},
                                                18: {'x': [-20, 60], 'y': [-20, 20]}},
                               'width_error': {1: {'x': [-60, 60], 'y': [-20, 20]}, 4: {'x': [-60, 60], 'y': [-20, 20]},
                                               5: {'x': [-60, 60], 'y': [-20, 20]},
                                               18: {'x': [-20, 60], 'y': [-20, 20]}},
                               'height_error': {1: {'x': [-60, 60], 'y': [-20, 20]},
                                                4: {'x': [-60, 60], 'y': [-20, 20]},
                                                5: {'x': [-60, 60], 'y': [-20, 20]}}}

    def run(self, input_data, input_parameter_container=None):
        if input_parameter_container is not None:
            self.metric_type = input_parameter_container['metric_type']
            self.evaluate_range = input_parameter_container['evaluate_range']

        data_dict = {}
        data = input_data['data']
        tp_data = data[(data['gt.flag'] == 1) & (data['pred.flag'] == 1)]

        for metric in self.metric_type:
            print(f'正在评估指标 {metric}')
            if metric == 'recall_precision':
                data_dict['recall_precision'] = data
            else:
                metric_class = change_name(metric)
                func = eval(f'{metric_class}(self.evaluate_range[metric])')
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
