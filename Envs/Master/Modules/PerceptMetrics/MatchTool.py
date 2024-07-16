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


def match_timestamp(
        prediction_timestamp: List[float], 
        groundtruth_timestamp: List[float], 
        match_tolerance: float):

    rows = []
    for gt_t_idx, gt_t in enumerate(sorted(groundtruth_timestamp)):
        for pred_t_idx, pred_t in enumerate(sorted(prediction_timestamp)):
            delta = abs(gt_t - pred_t)
            if delta < match_tolerance:
                rows.append([gt_t_idx, gt_t, pred_t_idx, pred_t, delta])

    temp_data = pd.DataFrame(rows, columns=['gt_t_idx', 'gt_timestamp', 'pred_t_idx',
                                            'pred_timestamp', 'delta']).sort_values(by=['delta'])

    pred_timestamp, gt_timestamp = [], []
    while len(temp_data):
        gt_timestamp.append(temp_data.at[0, 'gt_timestamp'])
        gt_t_idx = temp_data.at[0, 'gt_t_idx']
        pred_timestamp.append(temp_data.at[0, 'pred_timestamp'])
        pred_t_idx = temp_data.at[0, 'pred_t_idx']
        temp_data.drop(temp_data[(temp_data['gt_t_idx'] == gt_t_idx)
                                 | (temp_data['pred_t_idx'] == pred_t_idx)].index, axis=0, inplace=True)
        if len(temp_data):
            temp_data = temp_data.sort_values(by=['delta']).reset_index(drop=True)
        else:
            break

    return zip(*sorted(zip(pred_timestamp, gt_timestamp)))


if __name__ == '__main__':
    pass