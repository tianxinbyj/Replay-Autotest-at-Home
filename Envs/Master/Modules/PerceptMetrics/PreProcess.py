"""  
@Author: BU YUJUN
@Date: 2024/7/8 上午9:57  
"""
import time
from typing import List, Tuple, Any

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d


def calculate_time_gap(
        baseline_time_series: List[float],
        baseline_velocity_series: List[float],
        calibrated_time_series: List[float],
        calibrated_velocity_series: List[float]
) -> Tuple[Any, Any]:
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

    def __init__(self):
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
            raise ValueError("Invalid input format for get_rect_points function")

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


class DruDirection:

    def __init__(self):
        self.columns = [
            'is_sameDir',
            'is_oppositeDir',
            'is_crossingDir',
        ]
        self.type = 'by_row'

    def __call__(self, input_data):

        if isinstance(input_data, dict):  # 假设传入的是一行数据的字典形式
            obj_type, yaw = input_data['type'], input_data['yaw']

        elif ((isinstance(input_data, tuple) or isinstance(input_data, list))
              and len(input_data) == 5):
            obj_type, yaw = input_data

        else:
            raise ValueError("Invalid input format for get_rect_points function")

        yaw_degree = np.rad2deg(yaw)
        while True:
            if -180 <= yaw_degree <= 180:
                break
            if yaw_degree < -180:
                yaw_degree += 360
            if yaw_degree > 180:
                yaw_degree -= 360

        if obj_type == 1:
            if -30 <= yaw_degree <= 30:
                return 1, 0, 0
            elif -180 <= yaw_degree < -150 or 150 < yaw_degree < 180:
                return 0, 1, 0
            else:
                return 0, 0, 1



        return 0, 0, 0


class ObstaclesPreprocess:

    def __init__(self, data, preprocess_types=None):
        if preprocess_types is None:
            self.preprocess_types = [
                'RectPoints', 'DruDirection'
            ]
        else:
            self.preprocess_types = []

        for preprocess_type in self.preprocess_types:
            func = eval(f'{preprocess_type}()')
            if func.type == 'by_row':
                result_df = data.apply(lambda row: func(row.to_dict()), axis=1, result_type='expand')
                result_df.columns = func.columns
                data = pd.concat([data, result_df], axis=1)
            else:
                pass

        self.data = data


if __name__ == '__main__':
    # df = pd.DataFrame({
    #     'x': [10, 20],
    #     'y': [30, 40],
    #     'yaw': [np.pi / 4, np.pi / 2],
    #     'length': [5, 6],
    #     'width': [2, 3]
    # })
    #
    # d = ObstaclesPreprocess(df, preprocess_types=['rectPoints'])

    t0 = time.time()
    # rays = get_rays()
    print(time.time() - t0)
