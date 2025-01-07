"""  
Created on 2024/7/24.  
@author: Bu Yujun  
"""
import os

import numpy as np
import pandas as pd

import warnings
warnings.filterwarnings("ignore")


def get_project_path():
    """
    获取项目路径的函数。
    """
    folder = os.path.dirname(os.path.abspath(__file__))  # 获取当前文件所在的绝对路径的目录
    while True:
        if os.path.exists(os.path.join(folder, 'requirements.txt')):
            return folder
        parent_folder = os.path.dirname(folder)
        if parent_folder == folder:
            raise Exception("未找到项目路径")
        folder = parent_folder


kpi_target_file_path = os.path.join(get_project_path(), 'Docs', 'Resources', 'ObstaclesKpi.xlsx')
kpi_target_threshold = pd.read_excel(kpi_target_file_path, sheet_name=0, header=[0, 1, 2], index_col=[0, 1])
kpi_target_ratio = pd.read_excel(kpi_target_file_path, sheet_name=1, header=[0, 1, 2], index_col=[0, 1])

# 对车辆进行分类，分为大车，小车，行人，两轮车
type_classification_text = {
    'car': '小车',
    'pedestrian': '行人',
    'truck_bus': '大车',
    'cyclist': '两轮车',
}


def change_name(var):
    return ''.join([v.title() for v in var.split('_')])


def get_obstacles_kpi_threshold(col_1, col_2, target_type, x=0, y=0, region=None):

    def get_region_text(x, y):
        x_text = None
        if -100 < x <= -50:
            x_text = 'x(-100~-50)'
        elif -50 < x <= 0:
            x_text = 'x(-50~0)'
        elif 0 < x <= 50:
            x_text = 'x(0~50)'
        elif 50 < x <= 100:
            x_text = 'x(50~100)'
        elif 100 < x <= 150:
            x_text = 'x(100~150)'

        y_text = None
        if -8 <= y <= 8:
            y_text = 'y(-8~8)'

        if x_text is None or y_text is None:
            return None

        return f'{x_text},{y_text}'

    df = kpi_target_threshold
    if region is not None:
        region_text = region
    else:
        region_text = get_region_text(x, y)
        if region_text is None:
            return None

    index = (target_type, region_text)
    col = (col_1, col_2, '/VA/Obstacles')
    threshold = df.at[index, col]
    if np.isnan(threshold):
        return None
    else:
        return threshold


def get_obstacles_kpi_ratio(col_1, col_2, target_type, kpi_date_label):
    df = kpi_target_ratio
    index = (target_type, int(kpi_date_label))
    col = (col_1, col_2, '/VA/Obstacles')
    ratio = df.at[index, col]
    if np.isnan(ratio):
        return 1
    else:
        return ratio


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
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, pred_type, pred_x = (
                input_data['gt.id'], input_data['pred.id'],
                input_data['gt.x'], input_data['gt.y'],
                input_data['gt.type_classification'],
                input_data['gt.road_user'],
                input_data['pred.type_classification'],
                input_data['pred.x'])

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 8):
            gt_id, pred_id, gt_x, gt_y, gt_type, gt_road_user, pred_type, pred_x = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        type_classification = type_classification_text[gt_type]

        kpi_threshold = get_obstacles_kpi_threshold('纵向距离误差', 'x_abs_95[m]', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            x_limit = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('纵向距离误差', 'x_abs_95[m]', type_classification, kpi_date_label)
            x_limit = kpi_threshold * kpi_ratio

        kpi_threshold = get_obstacles_kpi_threshold('纵向距离误差', 'x%_abs_95', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            x_limit_p = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('纵向距离误差', 'x%_abs_95', type_classification, kpi_date_label)
            x_limit_p = kpi_threshold * kpi_ratio

        x_error = pred_x - gt_x
        x_error_abs = abs(x_error)
        x_error_p = x_error / max(20, abs(gt_x))
        x_error_p_abs = abs(x_error_p)

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

        type_classification = type_classification_text[gt_type]

        kpi_threshold = get_obstacles_kpi_threshold('横向距离误差', 'y_abs_95[m]', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            y_limit = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('横向距离误差', 'y_abs_95[m]', type_classification, kpi_date_label)
            y_limit = kpi_threshold * kpi_ratio

        kpi_threshold = get_obstacles_kpi_threshold('横向距离误差', 'y%_abs_95', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            y_limit_p = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('横向距离误差', 'y%_abs_95', type_classification, kpi_date_label)
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

        type_classification = type_classification_text[gt_type]

        kpi_threshold = get_obstacles_kpi_threshold('纵向速度误差', 'vx_abs_95[m/s]', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            vx_limit = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('纵向速度误差', 'vx_abs_95[m/s]', type_classification, kpi_date_label)
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

        type_classification = type_classification_text[gt_type]

        kpi_threshold = get_obstacles_kpi_threshold('横向速度误差', 'vy_abs_95[m/s]', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            vy_limit = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('横向速度误差', 'vy_abs_95[m/s]', type_classification, kpi_date_label)
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

        type_classification = type_classification_text[gt_type]

        kpi_threshold = get_obstacles_kpi_threshold('航向角误差', 'yaw_abs_95[deg]', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            yaw_limit = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('航向角误差', 'yaw_abs_95[deg]', type_classification, kpi_date_label)
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

        type_classification = type_classification_text[gt_type]

        kpi_threshold = get_obstacles_kpi_threshold('长度误差', 'length_abs_95[m]', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            length_limit = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('长度误差', 'length_abs_95[m]', type_classification, kpi_date_label)
            length_limit = kpi_threshold * kpi_ratio

        kpi_threshold = get_obstacles_kpi_threshold('长度误差', 'length%_abs_95', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            length_limit_p = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('长度误差', 'length%_abs_95', type_classification, kpi_date_label)
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

        type_classification = type_classification_text[gt_type]

        kpi_threshold = get_obstacles_kpi_threshold('宽度误差', 'width_abs_95[m]', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            width_limit = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('宽度误差', 'width_abs_95[m]', type_classification, kpi_date_label)
            width_limit = kpi_threshold * kpi_ratio

        kpi_threshold = get_obstacles_kpi_threshold('宽度误差', 'width%_abs_95', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            width_limit_p = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('宽度误差', 'width%_abs_95', type_classification, kpi_date_label)
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

        type_classification = type_classification_text[gt_type]

        kpi_threshold = get_obstacles_kpi_threshold('高度误差', 'height_abs_95[m]', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            height_limit = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('高度误差', 'height_abs_95[m]', type_classification, kpi_date_label)
            height_limit = kpi_threshold * kpi_ratio

        kpi_threshold = get_obstacles_kpi_threshold('高度误差', 'height%_abs_95', type_classification, x=gt_x, y=gt_y)
        if kpi_threshold is None:
            height_limit_p = None
        else:
            kpi_ratio = get_obstacles_kpi_ratio('高度误差', 'height%_abs_95', type_classification, kpi_date_label)
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


if __name__ == '__main__':
    kpi_date_label = 20241130
    type_classification, gt_x, gt_y = '小车',-5.477241039276123,-0.092785932123661

    kpi_threshold = get_obstacles_kpi_threshold('纵向距离误差', 'x_abs_95[m]', type_classification, x=gt_x, y=gt_y)

    if kpi_threshold is None:
        x_limit = None
    else:
        kpi_ratio = get_obstacles_kpi_ratio('纵向距离误差', 'x_abs_95[m]', type_classification, kpi_date_label)
        x_limit = kpi_threshold * kpi_ratio
    print(kpi_threshold, x_limit)

    kpi_threshold = get_obstacles_kpi_threshold('纵向距离误差', 'x%_abs_95', type_classification, x=gt_x, y=gt_y)

    if kpi_threshold is None:
        x_limit_p = None
    else:
        kpi_ratio = get_obstacles_kpi_ratio('纵向距离误差', 'x%_abs_95', type_classification, kpi_date_label)
        x_limit_p = kpi_threshold * kpi_ratio
    print(kpi_threshold, x_limit_p)