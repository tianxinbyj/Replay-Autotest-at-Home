"""  
@Author: BU YUJUN
@Date: 2024/7/8 上午9:57  
"""
from typing import List, Tuple

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d

parameter_container = {
    'coverage_reference_point': [2, 0, 1],
    'coverage_threshold': 0.6,
    'lane_width': 3.6,
    'moving_threshold': 2,
    'key_coverage_threshold': 0.1,
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
        ]
        self.type = 'by_row'

    def __call__(self, input_data):

        if isinstance(input_data, dict):  # 假设传入的是一行数据的字典形式
            x = input_data['x']
            y = input_data['y']
            yaw = input_data['yaw']
            length = input_data['length']
            width = input_data['width']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            x, y, yaw, length, width = input_data

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        # 计算距离
        distance = np.sqrt(x ** 2 + y ** 2)

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
        return distance, *a[0], *b[0], *c[0], *d[0]


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
            self.x0, self.y0, self.z0 = parameter_container['coverage_reference_point']

    def __call__(self, input_data):

        if isinstance(input_data, dict):
            pt_0_x = input_data['pt_0_x']
            pt_0_y = input_data['pt_0_y']
            pt_1_x = input_data['pt_1_x']
            pt_1_y = input_data['pt_1_y']
            pt_2_x = input_data['pt_2_x']
            pt_2_y = input_data['pt_2_y']
            pt_3_x = input_data['pt_3_x']
            pt_3_y = input_data['pt_3_y']
            height = input_data['height']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 9):
            pt_0_x = input_data[0]
            pt_0_y = input_data[1]
            pt_1_x = input_data[2]
            pt_1_y = input_data[3]
            pt_2_x = input_data[4]
            pt_2_y = input_data[5]
            pt_3_x = input_data[6]
            pt_3_y = input_data[7]
            height = input_data[8]

        else:
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        azimuth_list, elevation_list = [], []
        for i in range(8):
            x, y = eval(f'pt_{i % 4}_x') - self.x0, eval(f'pt_{i % 4}_y') - self.y0
            z = height * (i // 4) - self.z0
            azimuth = np.arctan2(y, x)
            if azimuth < 0:
                azimuth += 2 * np.pi
            azimuth_list.append(azimuth)

            elevation = np.arctan2(z, np.sqrt(x ** 2 + y ** 2))
            elevation_list.append(elevation)

        # 若横跨0度，应该较靠近360度的角度化为负数,
        if max(azimuth_list) - min(azimuth_list) > np.pi:
            azimuth_list = [azi if azi < np.pi else azi - 2 * np.pi for azi in azimuth_list]

        return min(azimuth_list), max(azimuth_list), min(elevation_list), max(elevation_list)


class IsCoverageValid:
    """
    计算物体的被遮挡率和是否有效

    """

    def __init__(self, input_parameter_container=None):
        self.type = 'by_frame'

        if isinstance(input_parameter_container, dict):
            self.threshold = input_parameter_container['coverage_threshold']
        else:
            self.threshold = parameter_container['coverage_threshold']

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

                azimuth_coverage = covered_length / b_length
                # intv = f'{a_union}-{b_union}-{overlaps}'

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

                    elevation_coverage = covered_length / b_length
                    # intv += f'                {a_union}-{b_union}-{overlaps}'

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
        ]
        self.type = 'by_row'

        if isinstance(input_parameter_container, dict):
            self.moving_threshold = input_parameter_container['moving_threshold']
        else:
            self.moving_threshold = parameter_container['moving_threshold']

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
            else:
                is_moving = 0

            if -30 <= yaw_degree <= 30:
                return 1, 0, 0, is_moving
            elif -180 <= yaw_degree < -150 or 150 < yaw_degree <= 180:
                return 0, 1, 0, is_moving
            else:
                return 0, 0, 1, is_moving

        return 0, 0, 0, 0


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
            self.lane_width = parameter_container['lane_width']
            self.key_coverage_threshold = parameter_container['key_coverage_threshold']

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

    def __init__(self, ):
        pass


class ObstaclesPreprocess:

    def __init__(self, preprocess_types=None):
        if preprocess_types is None:
            self.preprocess_types = [
                'RectPoints', 'VisionAngleRange', 'IsCoverageValid', 'IsKeyObj', 'DruDirection'
            ]
        else:
            self.preprocess_types = preprocess_types

    def run(self, data, input_parameter_container):
        # 增加age
        id_counts = data['id'].value_counts()
        data['age'] = data['id'].map(id_counts)

        for preprocess_type in self.preprocess_types:
            func = eval(f'{preprocess_type}(input_parameter_container)')
            if func.type == 'by_row':
                result_df = data.apply(lambda row: func(row.to_dict()), axis=1, result_type='expand')
                result_df.columns = func.columns
                data = pd.concat([data, result_df], axis=1)

            elif func.type == 'by_frame':
                data = func(data)

        return data
