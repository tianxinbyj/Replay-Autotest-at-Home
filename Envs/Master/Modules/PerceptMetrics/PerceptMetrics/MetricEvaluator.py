"""  
Created on 2024/7/24.  
@author: Bu Yujun  
"""

import os
import sys
import warnings

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from KpiGenerator import ObstaclesKpi, LinesKpi, change_name
from KpiGenerator import obstacles_type_classification_text, lines_type_classification_text

warnings.filterwarnings("ignore")


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
            (gt_flag, gt_type, pred_flag, pred_type) = (
                input_data['gt.flag'],
                input_data['gt.type_classification'],
                input_data['pred.flag'],
                input_data['pred.type_classification'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 8):
            (gt_flag, gt_type, pred_flag, pred_type) = input_data

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
            'gt.id',
            'pred.id',
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'pred.type',
            'pred.x',
            'x.error',
            'x.error_abs',
            'x.error%',
            'x.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, pred_type, pred_x, gt_closest_x, pred_closest_x = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['pred.type_classification'],
                input_data['pred.x'],
                input_data['gt.closest_pt_x'],
                input_data['pred.closest_pt_x'],
            )

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 10):
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, pred_type, pred_x, gt_closest_x, pred_closest_x = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = obstacles_type_classification_text[gt_type]

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('纵向距离误差', 'x_abs_95[m]', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            x_limit = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('纵向距离误差', 'x_abs_95[m]', type_classification, kpi_date_label)
            x_limit = kpi_threshold * kpi_ratio

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('纵向距离误差', 'x%_abs_95', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            x_limit_p = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('纵向距离误差', 'x%_abs_95', type_classification, kpi_date_label)
            x_limit_p = kpi_threshold * kpi_ratio

        if abs(gt_x) < 50:
            x_error = pred_x - gt_x
            x_error_p = x_error / max(20, abs(gt_x))
        else:
            x_error = pred_closest_x - gt_closest_x
            x_error_p = x_error / abs(gt_closest_x)
        x_error_abs = abs(x_error)
        x_error_p_abs = abs(x_error_p)

        # 双重阈值下，近处看绝对值，远处看相对值
        if x_limit is not None and x_limit_p is not None and x_limit_p != 0:
            distance_point = x_limit / x_limit_p
            if abs(gt_x) > abs(distance_point):
                x_error = np.nan
                x_error_abs = np.nan
            else:
                x_error_p = np.nan
                x_error_p_abs = np.nan

        is_abnormal = []
        if x_limit is not None:
            if x_error_abs > x_limit:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if x_limit_p is not None:
            if x_error_p_abs > x_limit_p:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if len(is_abnormal):
            is_abnormal = int(all(is_abnormal))
        else:
            is_abnormal = 0

        return (gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user,
                pred_type, pred_x, x_error, x_error_abs, x_error_p, x_error_p_abs,
                is_abnormal)


class YError:
    """
    计算物体的横向距离偏差

    """

    def __init__(self):
        self.columns = [
            'gt.id',
            'pred.id',
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'pred.type',
            'pred.y',
            'y.error',
            'y.error_abs',
            'y.error%',
            'y.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, pred_type, pred_y = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['pred.type_classification'],
                input_data['pred.y'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 8):
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, pred_type, pred_y = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = obstacles_type_classification_text[gt_type]

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('横向距离误差', 'y_abs_95[m]', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            y_limit = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('横向距离误差', 'y_abs_95[m]', type_classification, kpi_date_label)
            y_limit = kpi_threshold * kpi_ratio

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('横向距离误差', 'y%_abs_95', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            y_limit_p = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('横向距离误差', 'y%_abs_95', type_classification, kpi_date_label)
            y_limit_p = kpi_threshold * kpi_ratio

        y_error = pred_y - gt_y
        y_error_abs = abs(y_error)
        y_error_p = y_error / max(10, abs(gt_y))
        y_error_p_abs = abs(y_error_p)

        is_abnormal = []
        if y_limit is not None:
            if y_error_abs > y_limit:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if y_limit_p is not None:
            if y_error_p_abs > y_limit_p:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if len(is_abnormal):
            is_abnormal = int(all(is_abnormal))
        else:
            is_abnormal = 0

        return (gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user,
                pred_type, pred_y, y_error, y_error_abs, y_error_p, y_error_p_abs,
                is_abnormal)


class VxError:
    """
    计算物体的纵向速度偏差

    """

    def __init__(self):
        self.columns = [
            'gt.id',
            'pred.id',
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'gt.vx',
            'gt.vel',
            'pred.type',
            'pred.vx',
            'vx.error',
            'vx.error_abs',
            'vx.error%',
            'vx.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, gt_vx, gt_vy, pred_type, pred_vx = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.vx'], input_data['gt.vy'],
                input_data['pred.type_classification'],
                input_data['pred.vx'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 10):
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, gt_vx, gt_vy, pred_type, pred_vx = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = obstacles_type_classification_text[gt_type]

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('纵向速度误差', 'vx_abs_95[m/s]', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            vx_limit = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('纵向速度误差', 'vx_abs_95[m/s]', type_classification, kpi_date_label)
            vx_limit = kpi_threshold * kpi_ratio

        vx_error = pred_vx - gt_vx
        vx_error_abs = abs(vx_error)
        vx_error_p = vx_error / max(2, abs(gt_vx))
        vx_error_p_abs = abs(vx_error_p)

        is_abnormal = []
        if vx_limit is not None:
            if vx_error_abs > vx_limit:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if len(is_abnormal):
            is_abnormal = int(all(is_abnormal))
        else:
            is_abnormal = 0

        return (gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user,
                gt_vx, np.sqrt(gt_vx ** 2 + gt_vy ** 2), pred_type, pred_vx,
                vx_error, vx_error_abs, vx_error_p, vx_error_p_abs,
                is_abnormal)


class VyError:
    """
    计算物体的横向速度偏差

    """

    def __init__(self):
        self.columns = [
            'gt.id',
            'pred.id',
            'gt.x',
            'gt.y',
            'gt.type',
            'gt.road_user',
            'gt.vy',
            'gt.vel',
            'pred.type',
            'pred.vy',
            'vy.error',
            'vy.error_abs',
            'vy.error%',
            'vy.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, gt_vx, gt_vy, pred_type, pred_vy = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.vx'], input_data['gt.vy'],
                input_data['pred.type_classification'],
                input_data['pred.vy'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 10):
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, gt_vx, gt_vy, pred_type, pred_vy = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = obstacles_type_classification_text[gt_type]

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('横向速度误差', 'vy_abs_95[m/s]', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            vy_limit = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('横向速度误差', 'vy_abs_95[m/s]', type_classification, kpi_date_label)
            vy_limit = kpi_threshold * kpi_ratio

        vy_error = pred_vy - gt_vy
        vy_error_abs = abs(vy_error)
        vy_error_p = vy_error / max(2, abs(gt_vy))
        vy_error_p_abs = abs(vy_error_p)

        is_abnormal = []
        if vy_limit is not None:
            if vy_error_abs > vy_limit:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if len(is_abnormal):
            is_abnormal = int(all(is_abnormal))
        else:
            is_abnormal = 0

        return (gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user,
                gt_vy, np.sqrt(gt_vx ** 2 + gt_vy ** 2), pred_type, pred_vy,
                vy_error, vy_error_abs, vy_error_p, vy_error_p_abs,
                is_abnormal)


class YawError:
    """
    计算物体的航向角偏差

    """

    def __init__(self):
        self.columns = [
            'gt.id',
            'pred.id',
            'gt.x',
            'gt.y',
            'gt.vel',
            'gt.type',
            'gt.road_user',
            'gt.yaw',
            'pred.type',
            'pred.yaw',
            'yaw.error',
            'yaw.error_abs',
            'yaw.is_reverse',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            (gt_id, pred_id, gt_x, gt_y, gt_vx, gt_vy,
             gt_type, gt_road_user, gt_yaw, pred_type, pred_yaw) = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.vx'], input_data['gt.vy'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.yaw'],
                input_data['pred.type_classification'],
                input_data['pred.yaw'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 11):
            (gt_id, pred_id, gt_x, gt_y, gt_vx, gt_vy,
             gt_type, gt_road_user, gt_yaw, pred_type, pred_yaw) = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = obstacles_type_classification_text[gt_type]

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('航向角误差', 'yaw_abs_95[deg]', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            yaw_limit = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('航向角误差', 'yaw_abs_95[deg]', type_classification, kpi_date_label)
            yaw_limit = kpi_threshold * kpi_ratio

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
        if yaw_limit is not None:
            if is_reverse or np.rad2deg(abs(yaw_error)) > yaw_limit:
                is_abnormal = 1

        return (gt_id, pred_id, gt_x, gt_y, np.sqrt(gt_vx ** 2 + gt_vy ** 2), gt_type, gt_road_user,
                np.rad2deg(gt_yaw), pred_type, np.rad2deg(pred_yaw),
                np.rad2deg(yaw_error), np.rad2deg(abs(yaw_error)),
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
            'gt.id',
            'pred.id',
            'gt.x',
            'gt.y',
            'gt.type',
            'pred.type',
            'gt.road_user',
            'gt.length',
            'pred.length',
            'length.error',
            'length.error_abs',
            'length.error%',
            'length.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            gt_id, pred_id, gt_x, gt_y, gt_type, pred_type, gt_road_user, gt_length, pred_length = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'], input_data['pred.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.length'], input_data['pred.length'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 9):
            gt_id, pred_id, gt_x, gt_y, gt_type, pred_type, gt_road_user, gt_length, pred_length = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = obstacles_type_classification_text[gt_type]

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('长度误差', 'length_abs_95[m]', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            length_limit = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('长度误差', 'length_abs_95[m]', type_classification, kpi_date_label)
            length_limit = kpi_threshold * kpi_ratio

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('长度误差', 'length%_abs_95', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            length_limit_p = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('长度误差', 'length%_abs_95', type_classification, kpi_date_label)
            length_limit_p = kpi_threshold * kpi_ratio

        length_error = pred_length - gt_length
        length_error_abs = abs(length_error)
        length_error_p = length_error / gt_length
        length_error_p_abs = abs(length_error_p)

        is_abnormal = []
        if length_limit is not None:
            if length_error_abs > length_limit:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if length_limit_p is not None:
            if length_error_p_abs > length_limit_p:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if len(is_abnormal):
            is_abnormal = int(all(is_abnormal))
        else:
            is_abnormal = 0

        return (gt_id, pred_id, gt_x, gt_y, gt_type, pred_type, gt_road_user,
                gt_length, pred_length, length_error, length_error_abs,
                length_error_p, length_error_p_abs,
                is_abnormal)


class WidthError:

    def __init__(self):
        self.columns = [
            'gt.id',
            'pred.id',
            'gt.x',
            'gt.y',
            'gt.type',
            'pred.type',
            'gt.road_user',
            'gt.width',
            'pred.width',
            'width.error',
            'width.error_abs',
            'width.error%',
            'width.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            gt_id, pred_id, gt_x, gt_y, gt_type, pred_type, gt_road_user, gt_width, pred_width = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'], input_data['pred.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.width'], input_data['pred.width'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 9):
            gt_id, pred_id, gt_x, gt_y, gt_type, pred_type, gt_road_user, gt_width, pred_width = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = obstacles_type_classification_text[gt_type]

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('宽度误差', 'width_abs_95[m]', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            width_limit = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('宽度误差', 'width_abs_95[m]', type_classification, kpi_date_label)
            width_limit = kpi_threshold * kpi_ratio

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('宽度误差', 'width%_abs_95', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            width_limit_p = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('宽度误差', 'width%_abs_95', type_classification, kpi_date_label)
            width_limit_p = kpi_threshold * kpi_ratio

        width_error = pred_width - gt_width
        width_error_abs = abs(width_error)
        width_error_p = width_error / gt_width
        width_error_p_abs = abs(width_error_p)

        is_abnormal = []
        if width_limit is not None:
            if width_error_abs > width_limit:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if width_limit_p is not None:
            if width_error_p_abs > width_limit_p:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if len(is_abnormal):
            is_abnormal = int(all(is_abnormal))
        else:
            is_abnormal = 0

        return (gt_id, pred_id, gt_x, gt_y, gt_type, pred_type, gt_road_user,
                gt_width, pred_width, width_error, width_error_abs,
                width_error_p, width_error_p_abs,
                is_abnormal)


class HeightError:

    def __init__(self):
        self.columns = [
            'gt.id',
            'pred.id',
            'gt.x',
            'gt.y',
            'gt.type',
            'pred.type',
            'gt.road_user',
            'gt.height',
            'pred.height',
            'height.error',
            'height.error_abs',
            'height.error%',
            'height.error%_abs',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            gt_id, pred_id, gt_x, gt_y, gt_type, pred_type, gt_road_user, gt_height, pred_height = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'], input_data['pred.type_classification'],
                input_data['gt.road_user'],
                input_data['gt.height'], input_data['pred.height'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 9):
            gt_id, pred_id, gt_x, gt_y, gt_type, pred_type, gt_road_user, gt_height, pred_height = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = obstacles_type_classification_text[gt_type]

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('高度误差', 'height_abs_95[m]', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            height_limit = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('高度误差', 'height_abs_95[m]', type_classification, kpi_date_label)
            height_limit = kpi_threshold * kpi_ratio

        kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('高度误差', 'height%_abs_95', type_classification, (gt_x, gt_y))
        if kpi_threshold is None:
            height_limit_p = None
        else:
            kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('高度误差', 'height%_abs_95', type_classification, kpi_date_label)
            height_limit_p = kpi_threshold * kpi_ratio

        height_error = pred_height - gt_height
        height_error_abs = abs(height_error)
        height_error_p = height_error / gt_height
        height_error_p_abs = abs(height_error_p)

        is_abnormal = []
        if height_limit is not None:
            if height_error_abs > height_limit:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if height_limit_p is not None:
            if height_error_p_abs > height_limit_p:
                is_abnormal.append(True)
            else:
                is_abnormal.append(False)
        if len(is_abnormal):
            is_abnormal = int(all(is_abnormal))
        else:
            is_abnormal = 0

        return (gt_id, pred_id, gt_x, gt_y, gt_type, pred_type, gt_road_user,
                gt_height, pred_height, height_error, height_error_abs,
                height_error_p, height_error_p_abs,
                is_abnormal)


class ObstaclesMetricEvaluator:

    def __init__(self):
        self.kpi_date_label = '20250228'
        self.metric_type = [
            'recall_precision',
            'x_error', 'y_error',
            'vx_error', 'vy_error',
            'yaw_error', 'length_error',
            'width_error', 'height_error',
        ]

    def run(self, input_data, input_parameter_container=None):
        if input_parameter_container is not None:
            metric_type = input_parameter_container['metric_type']
            kpi_date_label = input_parameter_container['kpi_date_label']
        else:
            metric_type = self.metric_type
            kpi_date_label = self.kpi_date_label

        data_dict = {}
        data = input_data
        tp_data = data[(data['gt.flag'] == 1) & (data['pred.flag'] == 1)]

        for metric in metric_type:
            print(f'正在评估指标 {metric}')
            metric_class = change_name(metric)
            func = eval(f'{metric_class}()')

            if metric == 'recall_precision':
                result_df = data.apply(lambda row: func(row.to_dict()), axis=1, result_type='expand')
                result_df.columns = func.columns
                result_df = pd.concat([data, result_df], axis=1)
            else:
                if metric in ['vx_error', 'vy_error']:
                    result_df = tp_data[(data['gt.vx'] <= 60)
                                        & (data['gt.vx'] >= -60)
                                        & (data['gt.vy'] <= 60)
                                        & (data['gt.vy'] >= -60)
                                  ].apply(lambda row: func(row.to_dict(), kpi_date_label), axis=1,
                                              result_type='expand')
                else:
                    result_df = tp_data.apply(lambda row: func(row.to_dict(), kpi_date_label), axis=1, result_type='expand')
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
        total_data.index = total_data['corresponding_index'].to_list()
        data = input_data['data_to_filter']

        # 获得各种characteristic的corresponding_index
        # 不管是何种characteristic, 探测有效性和遮挡率必须符合要求的数据
        corresponding_index_dict = {characteristic: [] for characteristic in self.characteristic_type}
        for idx, row in total_data.iterrows():
            for characteristic in self.characteristic_type:
                if row['gt.flag'] == 1:
                    if (row[f'gt.{characteristic}'] == 1
                            and row['gt.is_detectedValid'] == 1 and row['gt.is_coverageValid'] == 1):
                        corresponding_index_dict[characteristic].append(idx)
                elif row['pred.flag'] == 1:
                    if (row[f'pred.{characteristic}'] == 1
                            and row['pred.is_detectedValid'] == 1 and row['pred.is_coverageValid'] == 1):
                        corresponding_index_dict[characteristic].append(idx)

        characteristic_data_dict = {}
        for characteristic in self.characteristic_type:
            print(f'正在评估特征 {characteristic}')
            characteristic_data_dict[characteristic] = data[
                data['corresponding_index'].isin(corresponding_index_dict[characteristic])]

        return characteristic_data_dict


class LateralError:

    def __init__(self):
        self.columns = [
            'gt.id',
            'pred.id',
            'gt.type',
            'pred.type',
            'gt.radius',
            'pred.radius',
            '0-30.lateral.error',
            '30-60.lateral.error',
            '60-120.lateral.error',
            'lateral.error',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            (gt_id, pred_id, gt_type, pred_type, gt_radius, pred_radius,
             gt_x_points, gt_y_points, pred_x_points, pred_y_points) = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.type_classification'], input_data['pred.type_classification'],
                input_data['gt.radius'], input_data['pred.radius'],
                input_data['gt.x_points'], input_data['gt.y_points'],
                input_data['pred.x_points'], input_data['pred.y_points'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 10):
            (gt_id, pred_id, gt_type, pred_type, gt_radius, pred_radius,
             gt_x_points, gt_y_points, pred_x_points, pred_y_points) = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = lines_type_classification_text[gt_type]

        kpi_threshold = LinesKpi.get_lines_kpi_threshold('横向位置误差', '0-30_abs_95[m]', type_classification, gt_radius)
        if kpi_threshold is None:
            limit_0_30 = None
        else:
            kpi_ratio = LinesKpi.get_lines_kpi_ratio('横向位置误差', '0-30_abs_95[m]', type_classification, kpi_date_label)
            limit_0_30 = kpi_threshold * kpi_ratio

        kpi_threshold = LinesKpi.get_lines_kpi_threshold('横向位置误差', '30-60_abs_95[m]', type_classification, gt_radius)
        if kpi_threshold is None:
            limit_30_60 = None
        else:
            kpi_ratio = LinesKpi.get_lines_kpi_ratio('横向位置误差', '30-60_abs_95[m]', type_classification, kpi_date_label)
            limit_30_60 = kpi_threshold * kpi_ratio

        kpi_threshold = LinesKpi.get_lines_kpi_threshold('横向位置误差', '60-120_abs_95[m]', type_classification, gt_radius)
        if kpi_threshold is None:
            limit_60_120 = None
        else:
            kpi_ratio = LinesKpi.get_lines_kpi_ratio('横向位置误差', '60-120_abs_95[m]', type_classification, kpi_date_label)
            limit_60_120 = kpi_threshold * kpi_ratio

        gt_x_points = [float(x) for x in gt_x_points.split(',')]
        gt_y_points = [float(x) for x in gt_y_points.split(',')]
        pred_x_points = [float(x) for x in pred_x_points.split(',')]
        pred_y_points = [float(x) for x in pred_y_points.split(',')]
        int_pred_x_points, int_pred_y_points = [], []
        for x, y in zip(pred_x_points, pred_y_points):
            if min(gt_x_points) <= x <= max(gt_x_points):
                int_pred_x_points.append(x)
                int_pred_y_points.append(y)

        f = interp1d(gt_x_points, gt_y_points, kind='linear')
        int_gt_y_points = f(int_pred_x_points)
        lateral_error = {
            '0-30.error': [],
            '30-60.error': [],
            '60-120.error': [],
        }
        error_0_30 = None
        error_30_60 = None
        error_60_120 = None
        for i, x in enumerate(int_pred_x_points):
            if 0 <= x < 30:
                lateral_error['0-30.error'].append(abs(int_pred_y_points[i] - int_gt_y_points[i]))
            elif 30 <= x < 60:
                lateral_error['30-60.error'].append(abs(int_pred_y_points[i] - int_gt_y_points[i]))
            elif 60 <= x < 120:
                lateral_error['60-120.error'].append(abs(int_pred_y_points[i] - int_gt_y_points[i]))

        if len(lateral_error['0-30.error']) >= 3:
            error_0_30 = np.mean(lateral_error['0-30.error'])
        if len(lateral_error['30-60.error']) >= 3:
            error_30_60 = np.mean(lateral_error['30-60.error'])
        if len(lateral_error['60-120.error']) >= 3:
            error_60_120 = np.mean(lateral_error['60-120.error'])

        over_limit_error = []
        is_abnormal = []
        if (limit_0_30 is not None) and (error_0_30 is not None):
            if error_0_30 > limit_0_30:
                is_abnormal.append(True)
                over_limit_error.append(error_0_30 - limit_0_30)
            else:
                is_abnormal.append(False)
                over_limit_error.append(0)

        if (limit_30_60 is not None) and (error_30_60 is not None):
            if error_30_60 > limit_30_60:
                is_abnormal.append(True)
                over_limit_error.append(error_30_60 - limit_30_60)
            else:
                is_abnormal.append(False)
                over_limit_error.append(0)

        if (limit_60_120 is not None) and (error_60_120 is not None):
            if error_60_120 > limit_60_120:
                is_abnormal.append(True)
                over_limit_error.append(error_60_120 - limit_60_120)
            else:
                is_abnormal.append(False)
                over_limit_error.append(0)

        if len(is_abnormal):
            is_abnormal = int(any(is_abnormal))
            over_limit_error = np.mean(over_limit_error)
        else:
            is_abnormal = 0
            over_limit_error = 0

        return (gt_id, pred_id, gt_type, pred_type, gt_radius, pred_radius,
                error_0_30, error_30_60, error_60_120, over_limit_error,
                is_abnormal)


class HeadingError:

    def __init__(self):
        self.columns = [
            'gt.id',
            'pred.id',
            'gt.type',
            'pred.type',
            'gt.radius',
            'pred.radius',
            'gt.heading_0',
            'pred.heading_0',
            'gt.heading_50',
            'pred.heading_50',
            '0.heading.error',
            '50.heading.error',
            'heading.error',
            'is_abnormal',
        ]

    def __call__(self, input_data, kpi_date_label):

        if isinstance(input_data, dict):
            (gt_id, pred_id, gt_type, pred_type, gt_radius, pred_radius,
             gt_heading_0, pred_heading_0, gt_heading_50, pred_heading_50) = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.type_classification'], input_data['pred.type_classification'],
                input_data['gt.radius'], input_data['pred.radius'],
                input_data['gt.heading_0'], input_data['pred.heading_0'],
                input_data['gt.heading_50'], input_data['pred.heading_50'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 10):
            (gt_id, pred_id, gt_type, pred_type, gt_radius, pred_radius,
             gt_heading_0, pred_heading_0, gt_heading_50, pred_heading_50) = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = lines_type_classification_text[gt_type]

        kpi_threshold = LinesKpi.get_lines_kpi_threshold('偏航角误差', '0_heading_abs_95[deg]', type_classification, gt_radius)
        if kpi_threshold is None:
            limit_0 = None
        else:
            kpi_ratio = LinesKpi.get_lines_kpi_ratio('偏航角误差', '0_heading_abs_95[deg]', type_classification, kpi_date_label)
            limit_0 = kpi_threshold * kpi_ratio

        kpi_threshold = LinesKpi.get_lines_kpi_threshold('偏航角误差', '50_heading_abs_95[deg]', type_classification, gt_radius)
        if kpi_threshold is None:
            limit_50 = None
        else:
            kpi_ratio = LinesKpi.get_lines_kpi_ratio('偏航角误差', '50_heading_abs_95[deg]', type_classification, kpi_date_label)
            limit_50 = kpi_threshold * kpi_ratio

        if (pred_heading_0 is not None) and (gt_heading_0 is not None):
            heading_0_error = abs(pred_heading_0 - gt_heading_0)
        else:
            heading_0_error = None

        if (pred_heading_50 is not None) and (gt_heading_50 is not None):
            heading_50_error = abs(pred_heading_50 - gt_heading_50)
        else:
            heading_50_error = None

        over_limit_error = []
        is_abnormal = []
        if (limit_0 is not None) and (heading_0_error is not None):
            if heading_0_error > limit_0:
                is_abnormal.append(True)
                over_limit_error.append(heading_0_error - limit_0)
            else:
                is_abnormal.append(False)
                over_limit_error.append(0)

        if (limit_50 is not None) and (heading_50_error is not None):
            if heading_50_error > limit_50:
                is_abnormal.append(True)
                over_limit_error.append(heading_50_error - limit_50)
            else:
                is_abnormal.append(False)
                over_limit_error.append(0)

        if len(is_abnormal):
            is_abnormal = int(any(is_abnormal))
            over_limit_error = np.mean(over_limit_error)
        else:
            is_abnormal = 0
            over_limit_error = 0

        return (gt_id, pred_id, gt_type, pred_type, gt_radius, pred_radius,
                gt_heading_0, pred_heading_0, gt_heading_50, pred_heading_50,
                heading_0_error, heading_50_error, over_limit_error,
                is_abnormal)


class LinesMetricEvaluator:

    def __init__(self):
        self.kpi_date_label = '20250228'
        self.metric_type = [
            'recall_precision',
            'lateral_error',
            'heading_error',
            'length_error',
        ]

    def run(self, input_data, input_parameter_container=None):
        if input_parameter_container is not None:
            metric_type = input_parameter_container['metric_type']
            kpi_date_label = input_parameter_container['kpi_date_label']
        else:
            metric_type = self.metric_type
            kpi_date_label = self.kpi_date_label

        data_dict = {}
        data = input_data[~((input_data['gt.flag'] == 1)
                                   & ((input_data['gt.type_classification'].isna())
                                      | (input_data['gt.radius'].isna())))]
        data = data[~((data['gt.flag'] == 0)
                      & ((data['pred.type_classification'].isna())
                         | (data['pred.radius'].isna())))]
        tp_data = data[(data['gt.flag'] == 1) & (data['pred.flag'] == 1)]

        for metric in metric_type:
            print(f'正在评估指标 {metric}')
            metric_class = change_name(metric)
            func = eval(f'{metric_class}()')

            if metric == 'recall_precision':
                result_df = data.apply(lambda row: func(row.to_dict()), axis=1, result_type='expand')
                result_df.columns = func.columns
                result_df = pd.concat([data, result_df], axis=1)
            else:
                result_df = tp_data.apply(lambda row: func(row.to_dict(), kpi_date_label), axis=1, result_type='expand')
                result_df.columns = func.columns
                result_df['corresponding_index'] = tp_data['corresponding_index']

            data_dict[metric] = result_df

        return data_dict


if __name__ == '__main__':
    # kpi_date_label = 20241130
    # type_classification, gt_x, gt_y = '小车',-5.477241039276123,-0.092785932123661
    #
    # kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('纵向距离误差', 'x_abs_95[m]', type_classification, x=gt_x, y=gt_y)
    #
    # if kpi_threshold is None:
    #     x_limit = None
    # else:
    #     kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('纵向距离误差', 'x_abs_95[m]', type_classification, kpi_date_label)
    #     x_limit = kpi_threshold * kpi_ratio
    # print(kpi_threshold, x_limit)
    #
    # kpi_threshold = ObstaclesKpi.get_obstacles_kpi_threshold('纵向距离误差', 'x%_abs_95', type_classification, x=gt_x, y=gt_y)
    #
    # if kpi_threshold is None:
    #     x_limit_p = None
    # else:
    #     kpi_ratio = ObstaclesKpi.get_obstacles_kpi_ratio('纵向距离误差', 'x%_abs_95', type_classification, kpi_date_label)
    #     x_limit_p = kpi_threshold * kpi_ratio
    # print(kpi_threshold, x_limit_p)

    lme = LinesMetricEvaluator()
    input_parameter_container = {
        'metric_type': [
            'recall_precision',
            'lateral_error',
            'heading_error',
            # 'length_error',
        ],
        'kpi_date_label': 20250330,
    }

    path = '/home/hp/下载/44444/test_bevlines/04_TestData/2-Lines/01_ScenarioUnit/20240129_155339_n000004/01_Data/Lines/VABevLines/match/match_data.csv'
    match_data = pd.read_csv(path, index_col=False)

    data_dict = lme.run(match_data, input_parameter_container)
    for metric, metric_data in data_dict.items():
        metric_data.to_csv(f'{metric}.csv', index=False)