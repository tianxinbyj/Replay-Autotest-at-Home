"""
Created on 2025/02/21.
@author: Bu Yujun
"""

import os
import pandas as pd
import numpy as np
import yaml


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


class Kpi:

    def __init__(self, kpi_path=None):
        self.kpi_target_threshold = pd.read_excel(kpi_path, sheet_name=0, header=[0, 1, 2], index_col=[0, 1])
        self.kpi_target_ratio = pd.read_excel(kpi_path, sheet_name=1, header=[0, 1, 2], index_col=[0, 1])

    def get_obstacles_kpi_threshold(self, col_1, col_2, target_type, pt=(10, 10), region_text=None):

        def get_region_text(pt):
            x_text = None
            x, y = pt
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

        df = self.kpi_target_threshold
        if region_text is None:
            region_text = get_region_text(pt)

        if region_text is None:
            return None

        index = (target_type, region_text)
        col = (col_1, col_2, '/VA/Obstacles')
        threshold = df.at[index, col]
        if np.isnan(threshold):
            return None
        else:
            return threshold

    def get_obstacles_kpi_ratio(self, col_1, col_2, target_type, kpi_date_label):
        df = self.kpi_target_ratio
        index = (target_type, int(kpi_date_label))
        col = (col_1, col_2, '/VA/Obstacles')
        ratio = df.at[index, col]
        if np.isnan(ratio) or ratio is None:
            return 1
        else:
            return ratio

    def get_lines_kpi_threshold(self, col_1, col_2, target_type, r=1000, radius_text=None):

        def get_radius_text(r):
            r_text = None
            if 20 < r <= 250:
                r_text = 'R(20~250)'
            elif 250 < r <= 1000:
                r_text = 'R(250~1000)'
            elif 1000 < r <= 5000:
                r_text = 'R(1000~5000)'
            elif 5000 < r <= 1e10:
                r_text = 'R(5000~99999)'

            return r_text

        df = self.kpi_target_threshold
        if radius_text is None:
            radius_text = get_radius_text(r)

        if radius_text is None:
            return None

        index = (target_type, radius_text)
        col = (col_1, col_2, '/VA/BevLines')
        threshold = df.at[index, col]
        if np.isnan(threshold):
            return None
        else:
            return threshold

    def get_lines_kpi_ratio(self, col_1, col_2, target_type, kpi_date_label):
        df = self.kpi_target_ratio
        index = (target_type, int(kpi_date_label))
        col = (col_1, col_2, '/VA/BevLines')
        ratio = df.at[index, col]
        if np.isnan(ratio) or ratio is None:
            return 1
        else:
            return ratio

    def get_slots_kpi_threshold(self, col_1, col_2, target_type, pt=(10, 10), region_text=None):

        def get_region_text(pt):
            x_text = None
            x, y = pt
            if -1 < x <= 4:
                x_text = 'x(-1~4)'
            elif 4 < x <= 6 or -3 < x <= -1:
                x_text = 'x(4~6)(-3~-1)'
            elif 6 < x <= 9 or -6 < x <= -3:
                x_text = 'x(6~9)(-6~-3)'
            elif 9 < x <= 14 or -11 < x <= -6:
                x_text = 'x(9~14)(-11~-6)'

            y_text = None
            if -3 <= y <= 3:
                y_text = 'y(-3~3)'
            elif 3 < y <= 10 or -10 < y <= -3:
                y_text = 'y(3~10)(-10~-3)'

            if x_text is None or y_text is None:
                return None

            return f'{x_text},{y_text}'

        df = self.kpi_target_threshold
        if region_text is None:
            region_text = get_region_text(pt)

        if region_text is None:
            return None

        index = (target_type, region_text)
        col = (col_1, col_2, '/VA/PK/Slots')
        threshold = df.at[index, col]
        if np.isnan(threshold):
            return None
        else:
            return threshold

    def get_slots_kpi_ratio(self, col_1, col_2, target_type, kpi_date_label):
        df = self.kpi_target_ratio
        index = (target_type, int(kpi_date_label))
        col = (col_1, col_2, '/VA/PK/Slots')
        ratio = df.at[index, col]
        if np.isnan(ratio) or ratio is None:
            return 1
        else:
            return ratio


def change_name(var):
    return ''.join([v.title() for v in var.split('_')])


kpi_target_file_path = os.path.join(get_project_path(), 'Docs', 'Resources', 'ObstaclesKpi.xlsx')
ObstaclesKpi = Kpi(kpi_target_file_path)

kpi_target_file_path = os.path.join(get_project_path(), 'Docs', 'Resources', 'LinesKpi.xlsx')
LinesKpi = Kpi(kpi_target_file_path)

kpi_target_file_path = os.path.join(get_project_path(), 'Docs', 'Resources', 'SlotsKpi.xlsx')
SlotsKpi = Kpi(kpi_target_file_path)

test_encyclopaedia_yaml = os.path.join(get_project_path(), 'Docs', 'Resources', 'test_encyclopaedia.yaml')
with open(test_encyclopaedia_yaml, 'r', encoding='utf-8') as file:
    test_encyclopaedia = yaml.safe_load(file)

obstacles_type_classification_text = {
    k: v['name'] for k, v in test_encyclopaedia['Information']['Obstacles']['type'].items()
}

lines_type_classification_text = {
    k: v['name'] for k, v in test_encyclopaedia['Information']['Lines']['type'].items()
}

slots_type_classification_text = {
    k: v['name'] for k, v in test_encyclopaedia['Information']['Slots']['type'].items()
}

obstacles_characteristic_text = {
    k: v['name'] for k, v in test_encyclopaedia['Information']['Obstacles']['characteristic'].items()
}

lines_characteristic_text = {
    k: v['name'] for k, v in test_encyclopaedia['Information']['Lines']['characteristic'].items()
}

slots_characteristic_text = {
    k: v['name'] for k, v in test_encyclopaedia['Information']['Slots']['characteristic'].items()
}

obstacles_metric_text = {
    k: v['name'] for k, v in test_encyclopaedia['Information']['Obstacles']['metric'].items()
}

lines_metric_text = {
    k: v['name'] for k, v in test_encyclopaedia['Information']['Lines']['metric'].items()
}

slots_metric_text = {
    k: v['name'] for k, v in test_encyclopaedia['Information']['Slots']['metric'].items()
}