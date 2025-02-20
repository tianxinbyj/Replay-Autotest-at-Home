"""  
@Author: BU YUJUN
@Date: 2024/7/8 上午9:57  
"""
from scipy.integrate import quad
from scipy.signal import savgol_filter

if 'numpy' not in globals():
    import numpy as np

from typing import List, Tuple
import pandas as pd
from scipy.interpolate import interp1d, CubicSpline

import warnings
warnings.filterwarnings("ignore")


obstacles_parameter_container = {
    'coverage_reference_point': [2, 0, 1],
    'coverage_threshold': 0.6,
    'lane_width': 3.6,
    'moving_threshold': 2,
    'key_coverage_threshold': 0.1,
    'ROI': {'x': [-100, 150], 'y': [-20, 20]},
}


lines_parameter_container = {
    'lane_width': 3.6,
    'if_gt': True
}


def calculate_time_gap(
        baseline_time_series: List[float],
        baseline_velocity_series: List[float],
        calibrated_time_series: List[float],
        calibrated_velocity_series: List[float]) -> Tuple[float, float]:
    # 提取时间序列和速度序列
    t1 = baseline_time_series
    v1 = baseline_velocity_series
    t2 = calibrated_time_series
    v2 = calibrated_velocity_series

    # 对基准时间序列和速度序列进行线性插值
    f1 = interp1d(t1, v1, kind='linear')
    new_t1_min = round(min(t1) + 20, 2)  # 设定新的时间范围，避开起始和结束的不稳定数据
    new_t1_max = round(max(t1) - 20, 2)
    new_t1 = np.arange(new_t1_min, new_t1_max, 0.005)  # 生成新的时间点
    new_v1 = f1(new_t1)  # 计算新的速度值

    # 对校准时间序列和速度序列进行线性插值
    f2 = interp1d(t2, v2, kind='linear')
    new_t2_min = round(min(t2) + 20, 2)
    new_t2_max = round(max(t2) - 20, 2)
    new_t2 = np.arange(new_t2_min, new_t2_max, 0.005)
    new_v2 = f2(new_t2)

    # 初始化结果列表
    total_res = []

    # 获取两个速度序列的长度
    l1, l2 = len(new_v1), len(new_v2)

    # 设定初始的平移索引
    idx = 9000

    # 遍历所有可能的平移索引
    while idx <= l1 + l2 - 9000:
        # 计算平移后的重叠区域
        a = max(idx, l2)
        b = min(idx, l1)
        v1_overlay = new_v1[a - l2: b]
        v2_overlay = new_v2[a - idx: b + l2 - idx]

        # 记录重叠区域的时间点
        tt1, tt2 = new_t1[a - l2], new_t2[a - idx]

        # 计算速度误差的平方
        v_error_list = [(vv1 - vv2) ** 2 for vv1, vv2 in zip(v1_overlay, v2_overlay)]

        # 计算平均速度误差
        v_error = np.sqrt(np.mean(v_error_list))

        # 将结果添加到列表中
        total_res.append([idx, tt1 - tt2, len(v_error_list), v_error])

        # 如果速度误差小于0.5，打印结果
        if v_error < 0.5:
            if __name__ == '__main__':
                print(idx, tt1 - tt2, len(v_error_list), v_error)

                # 更新平移索引，加速搜索过程
        idx += max(1, 2 * round(v_error ** 2))

        # 将结果列表转换为DataFrame
    res = pd.DataFrame(total_res, columns=['idx', 'time_delta', 'overlay_num', 'v_error'])

    # 找到最小速度误差对应的时间间隔
    t_delta = res.at[res['v_error'].idxmin(), 'time_delta']

    # 打印最佳时间间隔和对应的最小速度误差
    if __name__ == '__main__':
        print('最佳时间间隔 = ', t_delta, '平均误差 =', res['v_error'].min())

    # 返回最佳时间间隔
    return t_delta, res['v_error'].min()


def calculate_angle(x, y, z=None):
    """
    返回的azimuth是0-360度内的弧度, elevation是-90到90的弧度

    """

    azimuth = np.arctan2(y, x)
    if azimuth < 0:
        azimuth += 2 * np.pi

    if z is None:
        return azimuth

    elevation = np.arctan2(z, np.sqrt(x ** 2 + y ** 2))
    return azimuth, elevation


class ObstaclesTypeClassification:
    """
    对车辆进行分类，分为大车，小车，行人，两轮车

    """

    def __init__(self, input_parameter_container=None):
        self.columns = [
            'road_user',
            'type_classification'
        ]
        self.type = 'by_row'

        self.type_text = {
            1: 'car',
            2: 'pedestrian',
            4: 'truck_bus',
            5: 'truck_bus',
            18: 'cyclist'
        }

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            type_ = input_data['type']
            sub_type = input_data['sub_type']
            length = input_data['length']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 3):
            type_, sub_type, length = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 第一个为保留位
        if type_ in [2, 18]:
            return 'VRU', self.type_text[type_]

        elif type_ == 1:
            if sub_type in [1, 3, 9, 11]:
                return 'DRU', self.type_text[1]
            elif sub_type in [5, 12]:
                return 'DRU', self.type_text[5]
            elif sub_type == 4:
                return 'DRU', self.type_text[4]
            else:
                if length <= 5.99:
                    return 'DRU', self.type_text[1]
                else:
                    return 'DRU', self.type_text[5]

        return None, None


class RectPoints:
    """
    计算距离和四个角点

    """

    def __init__(self, input_parameter_container=None):
        self.columns = [
            'distance',
            'pt_0_x', 'pt_0_y',
            'pt_1_x', 'pt_1_y',
            'pt_2_x', 'pt_2_y',
            'pt_3_x', 'pt_3_y',
            'closest_pt_x', 'closest_pt_y',
        ]
        self.type = 'by_row'

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            x = input_data['x']
            y = input_data['y']
            type_ = input_data['type_classification']
            yaw = input_data['yaw']
            length = input_data['length']
            width = input_data['width']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 6):
            x, y, type_, yaw, length, width = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 计算距离
        distance = np.sqrt(x ** 2 + y ** 2)

        if type_ == 'car':
            width_limit = length * 0.255 + 0.7
            width = np.minimum(width_limit, width)
        elif type_ == 'truck_bus':
            width = np.minimum(width, 2.6)
        elif type_ == 'pedestrian':
            width = np.minimum(width, 0.8)
        elif type_ == 'cyclist':
            width = np.minimum(width, 1)

        # 使用旋转矩阵计算矩形左下角点a的坐标
        a = (np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]) @ np.array(
            [[-length / 2], [-width / 2]])).reshape(1, 2) + np.array([[x, y]])

        # 使用旋转矩阵计算矩形右下角点b的坐标
        b = (np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]) @ np.array(
            [[length / 2], [-width / 2]])).reshape(1, 2) + np.array([[x, y]])

        # 使用旋转矩阵计算矩形右上角点c的坐标
        c = (np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]) @ np.array(
            [[length / 2], [width / 2]])).reshape(1, 2) + np.array([[x, y]])

        # 使用旋转矩阵计算矩形左上角点d的坐标
        d = (np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]) @ np.array(
            [[-length / 2], [width / 2]])).reshape(1, 2) + np.array([[x, y]])

        # 返回矩形的四个顶点的坐标
        pt_0_x, pt_0_y = a[0]
        pt_1_x, pt_1_y = b[0]
        pt_2_x, pt_2_y = c[0]
        pt_3_x, pt_3_y = d[0]
        pts = [[pt_0_x, pt_0_y], [pt_1_x, pt_1_y], [pt_2_x, pt_2_y], [pt_3_x, pt_3_y]]
        sorted_pts = sorted(pts, key=lambda p: abs(p[0]))
        closest_pt_x, closest_pt_y = sorted_pts[0]
        return distance, *a[0], *b[0], *c[0], *d[0], closest_pt_x, closest_pt_y


class VisionAngleRange:
    """
    计算物体在参考点的观察角度占据

    """

    def __init__(self, input_parameter_container=None):
        self.columns = [
            'azimuth_right',
            'azimuth_left',
            'elevation_bottom',
            'elevation_top',
        ]
        self.type = 'by_row'

        # x0, y0, z0为观察点的坐标
        if isinstance(input_parameter_container, dict):
            self.x0, self.y0, self.z0 = input_parameter_container['coverage_reference_point']
        else:
            self.x0, self.y0, self.z0 = obstacles_parameter_container['coverage_reference_point']

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            pts = {
                '0': {'x': input_data['pt_0_x'], 'y': input_data['pt_0_y']},
                '1': {'x': input_data['pt_1_x'], 'y': input_data['pt_1_y']},
                '2': {'x': input_data['pt_2_x'], 'y': input_data['pt_2_y']},
                '3': {'x': input_data['pt_3_x'], 'y': input_data['pt_3_y']},
            }
            height = input_data['height']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 9):
            pts = {
                '0': {'x': input_data[0], 'y': input_data[1]},
                '1': {'x': input_data[2], 'y': input_data[3]},
                '2': {'x': input_data[4], 'y': input_data[5]},
                '3': {'x': input_data[6], 'y': input_data[7]},
            }
            height = input_data[8]

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        azimuth_list, elevation_list = [], []
        for i in range(8):
            x = pts[f'{i % 4}']['x'] - self.x0
            y = pts[f'{i % 4}']['y'] - self.y0
            z = height * (i // 4) - self.z0
            azimuth, elevation = calculate_angle(x, y, z)
            azimuth_list.append(azimuth)
            elevation_list.append(elevation)

        # 若横跨0度，应该较靠近360度的角度化为负数,
        if max(azimuth_list) - min(azimuth_list) > np.pi:
            azimuth_list = [azi if azi < np.pi else azi - 2 * np.pi for azi in azimuth_list]

        return min(azimuth_list), max(azimuth_list), min(elevation_list), max(elevation_list)


class DuplicateId:
    """
    去除相同的id

    """

    def __init__(self, input_parameter_container=None):
        self.type = 'by_frame'

    def __call__(self, input_data):

        return input_data.drop_duplicates(subset=['time_stamp', 'id'], keep='first')


class IsCoverageValid:
    """
    计算物体的被遮挡率和是否有效

    """

    def __init__(self, input_parameter_container=None):
        self.type = 'by_frame'

        if isinstance(input_parameter_container, dict):
            self.threshold = input_parameter_container['coverage_threshold']
        else:
            self.threshold = obstacles_parameter_container['coverage_threshold']

    def __call__(self, input_data):

        if isinstance(input_data, pd.DataFrame):
            data = input_data.sort_values(by=['time_stamp', 'distance'], ascending=[True, False]).reset_index(drop=True)

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # intvs = []
        coverages = []
        is_coverage_valid = []

        for time_stamp in data['time_stamp'].drop_duplicates():
            frame_data = data[data['time_stamp'] == time_stamp].reset_index(drop=True)

            for i, one_row in frame_data.iterrows():
                # 计算水平遮挡率
                b = self.gen_interval_by_angle(one_row['azimuth_right'], one_row['azimuth_left'])
                b_union = self.merge_intervals(b)
                b_length = 0
                for b in b_union:
                    b_length += b[1] - b[0]

                a = []
                for j in range(i + 1, len(frame_data)):
                    a.extend(self.gen_interval_by_angle(
                        frame_data.at[j, 'azimuth_right'], frame_data.at[j, 'azimuth_left']
                    ))
                a_union = self.merge_intervals(a)
                overlaps = self.find_overlapping_intervals(a_union, b_union)
                covered_length = 0
                for overlap in overlaps:
                    covered_length += overlap[1] - overlap[0]

                if b_length != 0:
                    azimuth_coverage = covered_length / b_length
                else:
                    azimuth_coverage = 1

                if azimuth_coverage:
                    # 计算高度这档率
                    b_union = [[one_row['elevation_bottom'], one_row['elevation_top']]]
                    b_length = b_union[0][1] - b_union[0][0]

                    a = []
                    for j in range(i + 1, len(frame_data)):
                        a.extend([[
                            frame_data.at[j, 'elevation_bottom'], frame_data.at[j, 'elevation_top']
                        ]])
                    a_union = self.merge_intervals(a)
                    overlaps = self.find_overlapping_intervals(a_union, b_union)
                    covered_length = 0
                    for overlap in overlaps:
                        covered_length += overlap[1] - overlap[0]

                    if b_length != 0:
                        elevation_coverage = covered_length / b_length
                    else:
                        elevation_coverage = 1

                    coverage = azimuth_coverage * elevation_coverage
                    coverages.append(coverage)
                    is_coverage_valid.append(1 if coverage <= self.threshold else 0)
                    # intvs.append(intv)

                else:
                    coverages.append(0)
                    is_coverage_valid.append(1)
                    # intvs.append('')

        data['coverage'] = coverages
        data['is_coverageValid'] = is_coverage_valid
        # data['intv'] = intvs

        return data

    def gen_interval_by_angle(self, min_angle, max_angle):
        # 如果小角度小于0，那么将区间分割成两端
        if min_angle < 0:
            res_interval = [
                [min_angle + 2 * np.pi, 2 * np.pi],
                [0, max_angle],
            ]
        else:
            res_interval = [
                [min_angle, max_angle]
            ]
        return res_interval

    def merge_intervals(self, intervals):
        # Sort intervals by their start times
        intervals.sort(key=lambda x: x[0])

        merged = []
        for interval in intervals:
            # If the list is empty or the new interval doesn't overlap with the previous, append it
            if not merged or merged[-1][1] < interval[0]:
                merged.append(interval)
            else:
                # Otherwise, merge with the previous interval
                merged[-1][1] = max(merged[-1][1], interval[1])

        return merged

    def find_overlapping_intervals(self, a_union, b_union):
        overlaps = []
        i, j = 0, 0
        while i < len(a_union) and j < len(b_union):
            a_start, a_end = a_union[i]
            b_start, b_end = b_union[j]

            # Check for overlap
            if a_start <= b_end and b_start <= a_end:
                overlap_start = max(a_start, b_start)
                overlap_end = min(a_end, b_end)
                overlaps.append([overlap_start, overlap_end])

                # Move the pointer for the list whose interval ends first
                if a_end < b_end:
                    i += 1
                else:
                    j += 1
            elif a_end < b_end:
                i += 1
            else:
                j += 1

        return overlaps


class DruDirection:
    """
    计算物体的运动和朝向状态

    """

    def __init__(self, input_parameter_container=None):
        self.columns = [
            'is_sameDir',
            'is_oppositeDir',
            'is_crossingDir',
            'is_moving',
            'is_static',
        ]
        self.type = 'by_row'

        if isinstance(input_parameter_container, dict):
            self.moving_threshold = input_parameter_container['moving_threshold']
        else:
            self.moving_threshold = obstacles_parameter_container['moving_threshold']

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            obj_type, yaw, vx, vy = input_data['type'], input_data['yaw'], input_data['vx'], input_data['vy']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 4):
            obj_type, yaw, vx, vy = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        yaw_degree = np.rad2deg(yaw)
        while True:

            if -180 <= yaw_degree <= 180:
                break
            if yaw_degree < -180:
                yaw_degree += 360
            if yaw_degree > 180:
                yaw_degree -= 360

        velocity = np.sqrt(vx ** 2 + vy ** 2)

        if obj_type == 1:

            if velocity >= self.moving_threshold:
                is_moving = 1
                is_static = 0
            else:
                is_moving = 0
                is_static = 1

            if -30 <= yaw_degree <= 30:
                return 1, 0, 0, is_moving, is_static
            elif -180 <= yaw_degree < -150 or 150 < yaw_degree <= 180:
                return 0, 1, 0, is_moving, is_static
            else:
                return 0, 0, 1, is_moving, is_static

        return 0, 0, 0, 0, 0


class IsKeyObj:
    """
    计算物体是否为关键目标

    """

    def __init__(self, input_parameter_container=None):
        self.type = 'by_frame'

        if isinstance(input_parameter_container, dict):
            self.lane_width = input_parameter_container['lane_width']
            self.key_coverage_threshold = input_parameter_container['key_coverage_threshold']
        else:
            self.lane_width = obstacles_parameter_container['lane_width']
            self.key_coverage_threshold = obstacles_parameter_container['key_coverage_threshold']

    def __call__(self, input_data):

        if isinstance(input_data, pd.DataFrame):
            data = input_data.sort_values(by=['time_stamp'], ascending=[True]).reset_index(drop=True)

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data['is_cipv'] = 0
        data['is_keyObj'] = 0

        # 正前方
        data_front = data[
            (data['x'] > 0) & (data['y'] > -self.lane_width / 2)
            & (data['y'] < self.lane_width / 2)
            & (data['coverage'] < self.key_coverage_threshold)].sort_values(
            by=['x'], ascending=[True])

        for time_stamp in data_front['time_stamp'].drop_duplicates():
            frame_data = data_front[data_front['time_stamp'] == time_stamp]

            for i, one_row in frame_data.iterrows():
                data.at[i, 'is_cipv'] = 1
                data.at[i, 'is_keyObj'] = 1
                break

        # 正后方
        data_rear = data[
            (data['x'] < 0) & (data['y'] > -self.lane_width / 2)
            & (data['y'] < self.lane_width / 2)
            & (data['coverage'] < self.key_coverage_threshold)].sort_values(
            by=['x'], ascending=[False])

        for time_stamp in data_rear['time_stamp'].drop_duplicates():
            frame_data = data_rear[data_rear['time_stamp'] == time_stamp]

            for i, one_row in frame_data.iterrows():
                data.at[i, 'is_keyObj'] = 1
                break

        # 左右
        data_other = data[(((data['y'] < -self.lane_width / 2) & (data['y'] > -3 * self.lane_width / 2)) | (
                (data['y'] > self.lane_width / 2) & (data['y'] < 3 * self.lane_width / 2))) & (
                                  data['coverage'] < self.key_coverage_threshold)]
        for i in data_other.index:
            data.at[i, 'is_keyObj'] = 1

        return data


class IsObstaclesDetectedValid:
    """
    计算动态障碍物是否在标定的探测范围内

    """

    def __init__(self, input_parameter_container=None):
        self.columns = [
            'reserved',
            'is_detectedValid',
        ]
        self.type = 'by_row'

        # ROI列表内的关系为或，字典内的关系为和
        if isinstance(input_parameter_container, dict):
            self.ROI = input_parameter_container['ROI']
        else:
            self.ROI = obstacles_parameter_container['ROI']

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            pts = {
                '0': {'x': input_data['pt_0_x'], 'y': input_data['pt_0_y']},
                '1': {'x': input_data['pt_1_x'], 'y': input_data['pt_1_y']},
                '2': {'x': input_data['pt_2_x'], 'y': input_data['pt_2_y']},
                '3': {'x': input_data['pt_3_x'], 'y': input_data['pt_3_y']},
            }

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 8):
            pts = {
                '0': {'x': input_data[0], 'y': input_data[1]},
                '1': {'x': input_data[2], 'y': input_data[3]},
                '2': {'x': input_data[4], 'y': input_data[5]},
                '3': {'x': input_data[6], 'y': input_data[7]},
            }

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 4个点都在roi内才认为True
        is_pts_valid_list = []

        for pt in pts.values():

            # ROI是列表，则内部关系为或； 是字典，则内部关系为和
            if isinstance(self.ROI, dict):
                is_pts_valid = self.check_with_dict(pt, self.ROI)
                is_pts_valid_list.append(is_pts_valid)

            elif isinstance(self.ROI, list):
                is_pts_valid = any([self.check_with_dict(pt, sub_roi) for sub_roi in self.ROI])
                is_pts_valid_list.append(is_pts_valid)

        if all(is_pts_valid_list):
            return 0, 1
        else:
            return 0, 0

    def check_with_dict(self, pt, roi_dict):
        is_valid_list = []
        for range_type, range_value in roi_dict.items():
            if range_type in ['x', 'y']:

                # 需要判断是否嵌套了表格，他们的关系为或
                if isinstance(range_value[0], float) or isinstance(range_value[0], int):
                    is_valid = range_value[0] <= pt[range_type] <= range_value[1]
                    is_valid_list.append(is_valid)

                elif isinstance(range_value[0], list) or isinstance(range_value[0], tuple):
                    is_valid = any(
                        [sub_range_value[0] <= pt[range_type] <= sub_range_value[1] for sub_range_value in range_value])
                    is_valid_list.append(is_valid)

            elif range_type == 'azimuth':

                # 需要判断是否嵌套了表格，他们的关系为或
                if isinstance(range_value[0], float) or isinstance(range_value[0], int):
                    angle_min, angle_max = range_value[0], range_value[1]
                    x_ref, y_ref, _ = range_value[2]

                    is_valid = self.check_angle(pt['x'], pt['y'], x_ref, y_ref, angle_min, angle_max)
                    is_valid_list.append(is_valid)

                elif isinstance(range_value[0], list) or isinstance(range_value[0], tuple):
                    sub_is_valid_list = []

                    for sub_range_value in range_value:
                        angle_min, angle_max = sub_range_value[0], sub_range_value[1]
                        x_ref, y_ref, _ = sub_range_value[2]

                        sub_is_valid = self.check_angle(pt['x'], pt['y'], x_ref, y_ref, angle_min, angle_max)
                        sub_is_valid_list.append(sub_is_valid)

                    is_valid = any(sub_is_valid_list)
                    is_valid_list.append(is_valid)

        return all(is_valid_list)

    def check_angle(self, x, y, x_ref, y_ref, angle_min, angle_max):
        azimuth = calculate_angle(x - x_ref, y - y_ref)
        angle_min, angle_max = np.deg2rad(angle_min), np.deg2rad(angle_max)

        if angle_min >= 0:
            if angle_min <= azimuth < angle_max:
                return True

        else:
            if angle_min + 2 * np.pi <= azimuth or azimuth <= angle_max:
                return True

        return False


class ObstaclesPreprocess:

    def __init__(self, preprocess_types=None):
        if preprocess_types is None:
            self.preprocess_types = [
                'DuplicateId',
                'ObstaclesTypeClassification',
                'RectPoints', 'VisionAngleRange', 'IsObstaclesDetectedValid',
                'IsCoverageValid', 'IsKeyObj', 'DruDirection'
            ]
        else:
            self.preprocess_types = preprocess_types

    def run(self, data, input_parameter_container):
        # 增加age
        id_counts = data['id'].value_counts()
        data['age'] = data['id'].map(id_counts)

        for preprocess_type in self.preprocess_types:
            print(f'正在预处理 {preprocess_type}')
            func = eval(f'{preprocess_type}(input_parameter_container)')
            if func.type == 'by_row':
                result_df = data.apply(lambda row: func(row.to_dict()), axis=1, result_type='expand')
                result_df.columns = func.columns
                data = pd.concat([data, result_df], axis=1)

            elif func.type == 'by_frame':
                data = func(data)

        return data


class LinesTypeClassification:
    """
    对车道线进行分类，分为主车道线，次车道线，道路边沿

    """

    def __init__(self, input_parameter_container=None):
        self.columns = [
            'reserved_lines_type',
            'type_classification'
        ]
        self.type = 'by_row'

        if isinstance(input_parameter_container, dict):
            self.lane_width = input_parameter_container['lane_width']
        else:
            self.lane_width = lines_parameter_container['lane_width']

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            type_ = input_data['type']
            position = input_data['position']
            c0 = input_data['c0']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 3):
            type_, position, c0 = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 第一个为保留位
        if type_ == 2:
            return 'reserved_lines_type', 'fence'

        elif type_ == 1:
            if position in [1, 8]:
                return 'reserved_lines_type', 'main_lane'
            elif position in [2, 3, 9, 10]:
                return 'reserved_lines_type', 'secondary_lane'

        return 'reserved_lines_type', None


class LinesCoefficient:

    def __init__(self, input_parameter_container=None):

        self.columns = [
            'c0', 'c1', 'c2', 'c3', 'heading_0', 'heading_50', 'length', 'radius',
        ]
        self.type = 'by_row'

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            x_points = input_data['x_points']
            y_points = input_data['y_points']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 2):
            x_points, y_points = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 计算车道线系数
        x_points = [float(x) for x in x_points.split(',')]
        y_points = [float(y) for y in y_points.split(',')]
        combined = list(zip(x_points, y_points))
        sorted_combined = sorted(combined, key=lambda t: t[0])
        sorted_list1, sorted_list2 = zip(*sorted_combined)
        f_x_points = list(sorted_list1)
        f_y_points = list(sorted_list2)
        ac3, ac2, ac1, _ = np.polyfit(f_x_points, f_y_points, 3)
        length = self.cubic_curve_length(ac3, ac2, ac1, min(f_x_points), max(f_x_points))

        # 计算曲率半径只考虑0-80米的长度
        radius_x, radius_y = [], []
        for x_, y_ in zip(f_x_points, f_y_points):
            if 0 <= x_ <= 80:
                radius_x.append(x_)
                radius_y.append(y_)

        if len(radius_x) > 2:
            c3, c2, c1, c0 = np.polyfit(radius_x, radius_y, 3)

            if min(radius_x) > 8:
                heading_0 = None
            else:
                heading_0 = np.rad2deg(np.arctan(25 * c3 + 5 * c2 + c1))

            if max(radius_x) < 52 or min(radius_x) > 48:
                heading_50 = None
            else:
                heading_50 = np.rad2deg(np.arctan(7500 * c3 + 100 * c2 + c1))

            if len(radius_x) < 5 or self.cubic_curve_length(c3, c2, c1, min(radius_x), max(radius_x)) <= 10:
                radius = None
            else:
                x_smooth, y_smooth, radius = self.smooth_and_calculate_curvature(radius_x, radius_y)
                # radius = min(9999, np.mean(radius[np.isfinite(radius)]))
                radius = min(9999, round(np.percentile(radius, 50)))

        else:
            c0, c1, c2, c3, heading_0, heading_50, radius = None, None, None, None, None, None, None

        return c0, c1, c2, c3, heading_0, heading_50, length, radius

    def cubic_curve_length(self, a, b, c, x1, x2):
        """
        计算一元三次函数在给定区间内的曲线长度。
        参数:
        a, b, c, d : float
            一元三次函数的系数，即 f(x) = ax^3 + bx^2 + cx + d
        x1, x2 : float
            计算曲线长度的区间端点
        返回:
        float
            曲线长度
        """
        # 定义一元三次函数的导数
        def df(x):
            return 3 * a * x ** 2 + 2 * b * x + c

        # 被积函数
        integrand = lambda x: np.sqrt(1 + (df(x)) ** 2)
        # 计算积分（曲线长度）
        result, error = quad(integrand, x1, x2)

        return result

    def smooth_and_calculate_curvature(self, x, y, window_length=15, polyorder=3, resample_factor=2):
        """平滑曲线并计算曲率半径"""
        # 参数化处理（累积弧长）
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.sqrt(dx ** 2 + dy ** 2)
        s = np.concatenate(([0], np.cumsum(ds)))

        # 生成等间距采样点
        num_new = max(len(x) * resample_factor, 50)  # 确保足够采样点
        s_new = np.linspace(s[0], s[-1], num_new)
        f_x = interp1d(s, x, kind='cubic', fill_value="extrapolate")
        f_y = interp1d(s, y, kind='cubic', fill_value="extrapolate")
        x_resampled = f_x(s_new)
        y_resampled = f_y(s_new)

        # Savitzky-Golay平滑
        window_length = min(window_length, len(x_resampled) // 2 * 2 - 1)  # 确保为奇数且合理
        if window_length < polyorder + 1:
            window_length = polyorder + 1 + (polyorder + 1) % 2
        x_smooth = savgol_filter(x_resampled, window_length, polyorder)
        y_smooth = savgol_filter(y_resampled, window_length, polyorder)

        # 计算导数
        delta_s = s_new[1] - s_new[0]
        dx_ds = np.gradient(x_smooth, delta_s)
        dy_ds = np.gradient(y_smooth, delta_s)
        d2x_ds2 = np.gradient(dx_ds, delta_s)
        d2y_ds2 = np.gradient(dy_ds, delta_s)

        # 计算曲率半径
        numerator = dx_ds * d2y_ds2 - dy_ds * d2x_ds2
        denominator = (dx_ds ** 2 + dy_ds ** 2) ** 1.5
        epsilon = 1e-6
        denominator = np.where(np.abs(denominator) < epsilon, epsilon, denominator)
        kappa = numerator / denominator
        radius = np.divide(1, np.abs(kappa), out=np.full_like(kappa, np.inf), where=np.abs(kappa) != 0)

        return x_smooth, y_smooth, radius


class ConnectLines:
    """
    首尾相连连接车道线

    """

    def __init__(self, input_parameter_container=None):
        self.type = 'by_frame'

        if isinstance(input_parameter_container, dict):
            self.if_gt = input_parameter_container['if_gt']
        else:
            self.if_gt = lines_parameter_container['if_gt']

    def __call__(self, input_data):
        rows = []
        cols = ['time_stamp', 'frame_id', 'id', 'confidence', 'type', 'position', 'color', 'points_num', 'x_points', 'y_points']
        for time_stamp, frame_data in input_data.groupby('time_stamp'):

            frame_id = frame_data['frame_id'].iloc[0]

            frame_data['max_y_points'] = frame_data['y_points'].apply(lambda x: max([abs(float(t)) for t in x.split(',')]))
            sorted_df = frame_data.sort_values(by='max_y_points', ascending=False)
            frame_data = sorted_df.drop(columns=['max_y_points'])

            lines = []
            for idx, row in frame_data.iterrows():

                # 筛选出前方的点
                x_points = row['x_points'].split(',')
                y_points = row['y_points'].split(',')
                f_x_points, f_y_points = [], []
                for x, y in zip(x_points, y_points):
                    if (float(x) not in f_x_points and float(y) not in f_y_points
                            and 150 >= float(x) >= -0.1 and 20 >= float(y) >= -20):
                        f_x_points.append(float(x))
                        f_y_points.append(float(y))

                points_num = len(f_x_points)
                if not points_num:
                    continue

                points = sorted(list(zip(f_x_points, f_y_points)), key=lambda t: t[0])
                lines.append({
                    'points': points,
                    'start': points[0],  # 线段起点
                    'end': points[-1],  # 线段终点
                    'type': row['type'],
                    'position': row['position'],
                    'id': row['id'],
                    'color': row['color'],
                    'confidence': row['confidence'],
                })

            merged = True
            while merged:
                merged = False
                new_lines = []
                used = set()

                for i in range(len(lines)):
                    if i in used:
                        continue
                    current_line = lines[i]

                    # 尝试与其他线段合并
                    for j in range(i + 1, len(lines)):
                        if j in used:
                            continue
                        target_line = lines[j]

                        res = self.can_merge(current_line, target_line)
                        if res[0]:
                            if res[1] == 1:
                                merged_points = current_line['points'] + target_line['points']
                                new_lines.append({
                                    'points': merged_points,
                                    'start': merged_points[0],
                                    'end': merged_points[-1],
                                    'type': current_line['type'],
                                    'position': current_line['position'],
                                    'id': f"{current_line['id']},{target_line['id']}",
                                    'color': current_line['color'],
                                    'confidence': current_line['confidence'],
                                })
                            else:
                                merged_points = target_line['points'] + current_line['points']
                                new_lines.append({
                                    'points': merged_points,
                                    'start': merged_points[0],
                                    'end': merged_points[-1],
                                    'type': target_line['type'],
                                    'position': target_line['position'],
                                    'id': f"{target_line['id']},{current_line['id']}",
                                    'color': target_line['color'],
                                    'confidence': target_line['confidence'],
                                })

                            used.add(i)
                            used.add(j)
                            merged = True
                            break

                    # 如果未合并，保留当前线段
                    if i not in used:
                        new_lines.append(current_line)
                        used.add(i)

                lines = new_lines

            for i, line in enumerate(lines):
                points = line['points']
                x_points, y_points = [], []
                for pt in points:
                    if str(pt[0]) not in x_points and str(pt[1]) not in y_points:
                        x_points.append(str(pt[0]))
                        y_points.append(str(pt[1]))

                rows.append([
                    time_stamp, frame_id, line['id'], line['confidence'], line['type'],
                    line['position'], line['color'], len(x_points),
                    ','.join(x_points), ','.join(y_points)])

        return pd.DataFrame(rows, columns=cols)

    # 检查两条线段是否可以合并
    def can_merge(self, line1, line2):
        def distance(p1, p2):
            return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

        if self.if_gt:
            epsilon = 0.1
        else:
            epsilon = 0.00001

        if distance(line1['end'], line2['start']) < epsilon:
            return True, 1
        elif distance(line1['start'], line2['end']) < epsilon:
            return True, 2
        return False, 0


class DefinePosition:
    """
    确定车道线的L1L2L3R1R2R3的位置

    """

    def __init__(self, input_parameter_container=None):
        self.type = 'by_frame'

        if isinstance(input_parameter_container, dict):
            self.lane_width = input_parameter_container['lane_width']
        else:
            self.lane_width = lines_parameter_container['lane_width']

    def __call__(self, input_data):

        def calculate_lane_head(row):
            x_points = sorted(map(float, row['x_points'].split(',')), key=lambda x: abs(x))[:3]
            y_points = [float(y) for x, y in zip(row['x_points'].split(','), row['y_points'].split(',')) if
                        float(x) in x_points]
            return sum(x_points) / len(x_points), sum(y_points) / len(y_points)

        df_type_1 = input_data[input_data['type'] == 1]
        df_type_1['head'] = df_type_1.apply(calculate_lane_head, axis=1)
        df_type_1['head_x'] = df_type_1['head'].apply(lambda x: x[0])
        df_type_1['head_y'] = df_type_1['head'].apply(lambda x: x[1])

        for index in df_type_1[(df_type_1['head_x'] > 15)
                               | (df_type_1['head_y'] > 3.5 * self.lane_width)
                               | (df_type_1['head_y'] < -3.5 * self.lane_width)].index:
            input_data.at[index, 'position'] = 0

        for time_stamp, frame_data in df_type_1.groupby('time_stamp'):
            # 左侧
            df_type_left = frame_data[(frame_data['head_y'] >= 0) & (frame_data['head_x'] <= 15)]
            if len(df_type_left):
                df_type_left.sort_values(by='head_y', ascending=True, inplace=True)
                current_row = 0
                position_list = np.arange(1, len(df_type_left) + 1, 1)
                for idx, row in df_type_left.iterrows():
                    current_row += 1
                    i = 0
                    while True:
                        if i * self.lane_width <= row['head_y'] < (0.2 + i + current_row) * self.lane_width:
                            break
                        else:
                            position_list += 1
                            i += 1
                            if current_row + i > 7:
                                break

                for i, p in zip(df_type_left.index, position_list):
                    input_data.at[i, 'position'] = int(p)

            # 右侧
            df_type_right = frame_data[(frame_data['head_y'] < 0) & (frame_data['head_x'] <= 15)]
            if len(df_type_right):
                df_type_right.sort_values(by='head_y', ascending=False, inplace=True)
                current_row = 0
                position_list = np.arange(1, len(df_type_right) + 1, 1)
                for idx, row in df_type_right.iterrows():
                    current_row += 1
                    i = 0
                    while True:
                        if i * self.lane_width <= abs(row['head_y']) < (0.2 + i + current_row) * self.lane_width:
                            break
                        else:
                            position_list += 1
                            i += 1
                            if current_row + i > 7:
                                break

                for i, p in zip(df_type_right.index, position_list):
                    input_data.at[i, 'position'] = int(p) + 7

        return input_data


class DefineId:
    """
    确定真值的id

    """

    def __init__(self, input_parameter_container=None):
        self.type = 'by_frame'

    def __call__(self, input_data):

        def get_lists_interaction(lst1, lst2):
            res = [t for t in lst1 if t in lst2]
            if len(res):
                return True
            else:
                return False

        id_count = 0
        new_ids = []
        last_id_dict = {}
        for time_stamp, frame_data in input_data.groupby('time_stamp'):

            current_id_dict = {}
            for idx, row in frame_data.iterrows():
                raw_id = row['id']
                id_list = str(raw_id).split(',')
                for last_raw_id, last_new_id in last_id_dict.items():
                    last_id_list = str(last_raw_id).split(',')
                    if get_lists_interaction(id_list, last_id_list):
                        current_id_dict[str(raw_id)] = last_new_id
                        break

                if raw_id not in current_id_dict:
                    id_count += 1
                    current_id_dict[str(raw_id)] = id_count

                new_ids.append(current_id_dict[str(raw_id)])

            last_id_dict = current_id_dict

        input_data['origin_id'] = input_data['id']
        input_data['id'] = new_ids
        input_data['id'] = input_data['id'].astype(int)

        return input_data


class LinesPreprocess:

    def __init__(self, preprocess_types=None):
        if preprocess_types is None:
            self.preprocess_types = [
                'DuplicateId',
                'ConnectLines',
                'DefinePosition',
                'DefineId',
                'LinesCoefficient',
                'LinesTypeClassification',
            ]
        else:
            self.preprocess_types = preprocess_types

    def run(self, data, input_parameter_container):
        # 增加age
        id_counts = data['id'].value_counts()
        data['age'] = data['id'].map(id_counts)

        if not input_parameter_container['if_gt']:
            self.preprocess_types.remove('DefinePosition')
            self.preprocess_types.remove('DefineId')

        for preprocess_type in self.preprocess_types:
            print(f'正在预处理 {preprocess_type}')
            func = eval(f'{preprocess_type}(input_parameter_container)')
            if func.type == 'by_row':
                result_df = data.apply(lambda row: func(row.to_dict()), axis=1, result_type='expand')
                result_df.columns = func.columns
                data = pd.concat([data, result_df], axis=1)

            elif func.type == 'by_frame':
                data = func(data)

        return data[data['length'] >= 2].sort_values(by=['time_stamp', 'c0'])


if __name__ == '__main__':
    raw_data_path = '/home/hp/下载/44444/test_bevlines/04_TestData/2-Lines/01_ScenarioUnit/20240129_155339_n000004/01_Data/Lines/GroundTruth/additional/gt_data.csv'
    raw_data = pd.read_csv(raw_data_path, index_col=False)

    parameter_json = {
        'lane_width': 3.6,
        'test_topic': 'Lines',
        'if_gt': True
    }

    preprocess_instance = LinesPreprocess()
    data = preprocess_instance.run(raw_data, parameter_json)
    data.to_csv('456.csv', index=False, encoding='utf_8_sig')
