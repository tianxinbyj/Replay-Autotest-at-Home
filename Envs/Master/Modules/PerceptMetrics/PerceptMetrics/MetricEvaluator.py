"""  
Created on 2024/7/24.  
@author: Bu Yujun  
"""
import numpy as np


def change_name(var):
    return ''.join([v.title() for v in var.split('_')])


class YawError:
    """
    计算物体的航向角偏差

    """

    def __init__(self):
        self.columns = [
            'gt.yaw',
            'pred.yaw',
            'yaw.error',
            'yaw.error_abs',
            'yaw.is_reverse'
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_yaw, pred_yaw = input_data['gt.yaw'], input_data['pred.yaw']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 2):
            gt_yaw, pred_yaw = input_data

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

        return np.rad2deg(gt_yaw), np.rad2deg(pred_yaw), np.rad2deg(yaw_error), np.rad2deg(abs(yaw_error)), is_reverse

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


class XError:
    """
    计算物体的纵向距离偏差

    """

    def __init__(self):
        self.columns = [
            'gt.x',
            'pred.x',
            'x.error',
            'x.error_abs',
            'x.error%',
            'x.error%_abs',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_x, pred_x = input_data['gt.x'], input_data['pred.x']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 2):
            gt_x, pred_x = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        x_error = pred_x - gt_x
        # 误差需要取反
        if pred_x < 0 and gt_x < 0:
            x_error *= -1
        x_error_abs = abs(x_error)
        x_error_p = x_error / max(20, abs(gt_x))
        x_error_p_abs = abs(x_error_p)

        return gt_x, pred_x, x_error, x_error_abs, x_error_p, x_error_p_abs


class YError:
    """
    计算物体的纵向距离偏差

    """

    def __init__(self):
        self.columns = [
            'gt.y',
            'pred.y',
            'y.error',
            'y.error_abs',
            'y.error%',
            'y.error%_abs',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_y, pred_y = input_data['gt.y'], input_data['pred.y']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 2):
            gt_y, pred_y = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        y_error = pred_y - gt_y
        # 误差需要取反
        if pred_y < 0 and gt_y < 0:
            y_error *= -1
        y_error_abs = abs(y_error)
        y_error_p = y_error / max(10, abs(gt_y))
        y_error_p_abs = abs(y_error_p)

        return gt_y, pred_y, y_error, y_error_abs, y_error_p, y_error_p_abs


class VxError:
    """
    计算物体的纵向距离偏差

    """

    def __init__(self):
        self.columns = [
            'gt.vx',
            'pred.vx',
            'vx.error',
            'vx.error_abs',
            'vx.error%',
            'vx.error%_abs',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_vx, pred_vx = input_data['gt.vx'], input_data['pred.vx']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 2):
            gt_vx, pred_vx = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        vx_error = pred_vx - gt_vx
        # 误差需要取反
        if pred_vx < 0 and gt_vx < 0:
            vx_error *= -1
        vx_error_abs = abs(vx_error)
        vx_error_p = vx_error / max(2, abs(gt_vx))
        vx_error_p_abs = abs(vx_error_p)

        return gt_vx, pred_vx, vx_error, vx_error_abs, vx_error_p, vx_error_p_abs


class VyError:
    """
    计算物体的纵向距离偏差

    """

    def __init__(self):
        self.columns = [
            'gt.vy',
            'pred.vy',
            'vy.error',
            'vy.error_abs',
            'vy.error%',
            'vy.error%_abs',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_vy, pred_vy = input_data['gt.vy'], input_data['pred.vy']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 2):
            gt_vy, pred_vy = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        vy_error = pred_vy - gt_vy
        # 误差需要取反
        if pred_vy < 0 and gt_vy < 0:
            vy_error *= -1
        vy_error_abs = abs(vy_error)
        vy_error_p = vy_error / max(2, abs(gt_vy))
        vy_error_p_abs = abs(vy_error_p)

        return gt_vy, pred_vy, vy_error, vy_error_abs, vy_error_p, vy_error_p_abs


class LengthError:

    def __init__(self):
        self.columns = [
            'gt.length',
            'pred.length',
            'length.error',
            'length.error_abs',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_length, pred_length = input_data['gt.length'], input_data['pred.length']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 2):
            gt_length, pred_length = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        length_error = pred_length - gt_length
        length_error_abs = abs(length_error)

        return gt_length, pred_length, length_error, length_error_abs


class WidthError:

    def __init__(self):
        self.columns = [
            'gt.width',
            'pred.width',
            'width.error',
            'width.error_abs',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_width, pred_width = input_data['gt.width'], input_data['pred.width']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 2):
            gt_width, pred_width = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        width_error = pred_width - gt_width
        width_error_abs = abs(width_error)

        return gt_width, pred_width, width_error, width_error_abs


class HeightError:

    def __init__(self):
        self.columns = [
            'gt.height',
            'pred.height',
            'height.error',
            'height.error_abs',
        ]

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            gt_height, pred_height = input_data['gt.height'], input_data['pred.height']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 2):
            gt_height, pred_height = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        height_error = pred_height - gt_height
        height_error_abs = abs(height_error)

        return gt_height, pred_height, height_error, height_error_abs


class ObstaclesMetricEvaluator:

    def __init__(self):
        self.metric_type = [
            'recall&precision',
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
            if metric == 'recall&precision':
                data_dict['recall&precision'] = data
            else:
                var_class = change_name(metric)
                func = eval(f'{var_class}()')
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

        # 有效探测范围且符合遮挡率要求的数据
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
            characteristic_data_dict[characteristic] = data[data['corresponding_index'].isin(corresponding_index_dict[characteristic])]

        return characteristic_data_dict
