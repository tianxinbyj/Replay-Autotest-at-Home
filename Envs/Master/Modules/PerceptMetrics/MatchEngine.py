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
    t1 = baseline_time_series
    v1 = baseline_velocity_series
    t2 = calibrated_time_series
    v2 = calibrated_velocity_series

    f1 = interp1d(t1, v1, kind='linear')
    new_t1_min = round(min(t1) + 20, 2)
    new_t1_max = round(max(t1) - 20, 2)
    new_t1 = np.arange(new_t1_min, new_t1_max, 0.005)
    new_v1 = f1(new_t1)
    f2 = interp1d(t2, v2, kind='linear')
    new_t2_min = round(min(t2) + 20, 2)
    new_t2_max = round(max(t2) - 20, 2)
    new_t2 = np.arange(new_t2_min, new_t2_max, 0.005)
    new_v2 = f2(new_t2)

    # 以v1为基准,平移v2,平移的长度应该[1, L1+L2-1]中
    total_res = []
    l1, l2 = len(new_v1), len(new_v2)
    idx = 9000
    while idx <= l1 + l2 - 9000:
        a = max(idx, l2)
        b = min(idx, l1)
        v1_overlay = new_v1[a - l2: b]
        v2_overlay = new_v2[a - idx: b + l2 - idx]
        tt1, tt2 = new_t1[a - l2], new_t2[a - idx]
        v_error_list = [(vv1 - vv2) ** 2 for vv1, vv2 in zip(v1_overlay, v2_overlay)]
        v_error = np.sqrt(np.mean(v_error_list))
        total_res.append([idx, tt1 - tt2, len(v_error_list), v_error])
        if v_error < 0.5:
            print(idx, tt1 - tt2, len(v_error_list), v_error)
        idx += max(1, 2 * round(v_error ** 2))

    res = pd.DataFrame(total_res, columns=['idx', 'time_delta', 'overlay_num', 'v_error'])
    t_delta = res.at[res['v_error'].idxmin(), 'time_delta']
    print('最佳时间间隔 = ', t_delta, '平均误差为', res['v_error'].min())
    return t_delta


if __name__ == '__main__':
    pass