"""  
Created on 2024/7/5.  
@author: Bu Yujun  
"""
import copy
import csv
import datetime
import glob
import inspect
import os.path
import re
import shutil
import threading
import time
import uuid
import warnings
from typing import List
from collections import OrderedDict

import matplotlib.patches as pc
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from scipy.interpolate import interp1d
from scipy.optimize import linear_sum_assignment


def calculate_time_gap(
        baseline_time_series: List[float],
        baseline_velocity_series: List[float],
        calibrated_time_series: List[float],
        calibrated_velocity_series: List[float]
) -> float:
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
    print('最佳时间间隔 = ', t_delta, '平均误差为', res['v_error'].min())

    # 返回最佳时间间隔
    return t_delta


if __name__ == '__main__':
    pass