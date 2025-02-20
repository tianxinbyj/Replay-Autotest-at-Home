"""  
@Author: BU YUJUN
@Date: 2024/7/25 下午3:27  
"""
import warnings
import pandas as pd

import warnings
warnings.filterwarnings("ignore")

# 屏蔽特定的警告
warnings.filterwarnings("ignore", category=pd.errors.SettingWithCopyWarning)


# 对车辆进行分类，分为大车，小车，行人，两轮车
obstacles_type_classification_text = {
    'car': '小车',
    'pedestrian': '行人',
    'truck_bus': '大车',
    'cyclist': '两轮车',
}

# 对车道线进行分类，分为主车道线，次车道线，道路边沿和其他车道线
lines_type_classification_text = {
    'main_lane': '主车道线',
    'secondary_lane': '次车道线',
    'fence': '道路边沿',
}

obstacles_characteristic_text = {
    'is_coverageValid': '全局目标',
    'is_cipv': 'cipv目标',
    'is_keyObj': '关键目标',
    'is_sameDir': '同向目标',
    'is_oppositeDir': '对向目标',
    'is_crossingDir': '横向目标',
    'is_moving': '运动目标',
    'is_static': '静止目标',
}

lines_characteristic_text = {
    'total': '全局目标',
}

obstacles_metric_text = {
    'recall_precision': '准召信息',
    'x_error': '纵向距离误差',
    'y_error': '横向距离误差',
    'vx_error': '纵向速度误差',
    'vy_error': '横向速度误差',
    'yaw_error': '航向角误差',
    'length_error': '长度误差',
    'width_error': '宽度误差',
    'height_error': '高度误差',
}

lines_metric_text = {
    'recall_precision': '准召信息',
    'heading_error': '偏航角误差',
    'lateral_error': '横向位置误差',
}


def change_name(var):
    return ''.join([v.title() for v in var.split('_')])


class RecallPrecision:
    """
    统计准召信息

    """

    def __call__(self, input_data):

        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        TP = data['TP'].sum()
        FP = data['FP'].sum()
        FN = data['FN'].sum()
        CTP = data['CTP'].sum()
        gt_count = TP + FN
        pred_count = TP + FP

        recall = TP / (TP + FN) if gt_count != 0 else 0
        precision = TP / (TP + FP) if pred_count != 0 else 0
        type_accuracy = CTP / TP if TP != 0 else 0

        res = {
            'TP': int(TP), 'FP': int(FP), 'FN': int(FN), 'CTP': int(CTP),
            'recall%': recall, 'precision%': precision, 'type_accuracy%': type_accuracy,
            'sample_count': int(max(pred_count, gt_count)),
        }

        return res


class XError:
    """
    统计纵向距离误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        x_abs_mean = data['x.error_abs'].mean()
        x_abs_68 = data['x.error_abs'].quantile(0.68)
        x_abs_95 = data['x.error_abs'].quantile(0.95)

        x_origin_mean = data['x.error'].mean()
        x_origin_std = data['x.error'].std()

        x_p_abs_mean = data['x.error%_abs'].mean()
        x_p_abs_68 = data['x.error%_abs'].quantile(0.68)
        x_p_abs_95 = data['x.error%_abs'].quantile(0.95)

        x_p_origin_mean = data['x.error%'].mean()
        x_p_origin_std = data['x.error%'].std()

        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            'x_abs_mean[m]': x_abs_mean, 'x_abs_68[m]': x_abs_68, 'x_abs_95[m]': x_abs_95,
            'x_origin_mean[m]': x_origin_mean, 'x_origin_std[m]': x_origin_std,
            'x%_abs_mean': x_p_abs_mean, 'x%_abs_68': x_p_abs_68, 'x%_abs_95': x_p_abs_95,
            'x%_origin_mean': x_p_origin_mean, 'x%_origin_std': x_p_origin_std,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class YError:
    """
    统计横向距离误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        y_abs_mean = data['y.error_abs'].mean()
        y_abs_68 = data['y.error_abs'].quantile(0.68)
        y_abs_95 = data['y.error_abs'].quantile(0.95)

        y_origin_mean = data['y.error'].mean()
        y_origin_std = data['y.error'].std()

        y_p_abs_mean = data['y.error%_abs'].mean()
        y_p_abs_68 = data['y.error%_abs'].quantile(0.68)
        y_p_abs_95 = data['y.error%_abs'].quantile(0.95)

        y_p_origin_mean = data['y.error%'].mean()
        y_p_origin_std = data['y.error%'].std()

        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            'y_abs_mean[m]': y_abs_mean, 'y_abs_68[m]': y_abs_68, 'y_abs_95[m]': y_abs_95,
            'y_origin_mean[m]': y_origin_mean, 'y_origin_std[m]': y_origin_std,
            'y%_abs_mean': y_p_abs_mean, 'y%_abs_68': y_p_abs_68, 'y%_abs_95': y_p_abs_95,
            'y%_origin_mean': y_p_origin_mean, 'y%_origin_std': y_p_origin_std,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class VxError:
    """
    统计纵向速度误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        vx_abs_mean = data['vx.error_abs'].mean()
        vx_abs_68 = data['vx.error_abs'].quantile(0.68)
        vx_abs_95 = data['vx.error_abs'].quantile(0.95)

        vx_origin_mean = data['vx.error'].mean()
        vx_origin_std = data['vx.error'].std()

        vx_p_abs_mean = data['vx.error%_abs'].mean()
        vx_p_abs_68 = data['vx.error%_abs'].quantile(0.68)
        vx_p_abs_95 = data['vx.error%_abs'].quantile(0.95)

        vx_p_origin_mean = data['vx.error%'].mean()
        vx_p_origin_std = data['vx.error%'].std()

        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            'vx_abs_mean[m/s]': vx_abs_mean, 'vx_abs_68[m/s]': vx_abs_68, 'vx_abs_95[m/s]': vx_abs_95,
            'vx_origin_mean[m/s]': vx_origin_mean, 'vx_origin_std[m/s]': vx_origin_std,
            'vx%_abs_mean': vx_p_abs_mean, 'vx%_abs_68': vx_p_abs_68, 'vx%_abs_95': vx_p_abs_95,
            'vx%_origin_mean': vx_p_origin_mean, 'vx%_origin_std': vx_p_origin_std,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class VyError:
    """
    统计横向速度误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        vy_abs_mean = data['vy.error_abs'].mean()
        vy_abs_68 = data['vy.error_abs'].quantile(0.68)
        vy_abs_95 = data['vy.error_abs'].quantile(0.95)

        vy_origin_mean = data['vy.error'].mean()
        vy_origin_std = data['vy.error'].std()

        vy_p_abs_mean = data['vy.error%_abs'].mean()
        vy_p_abs_68 = data['vy.error%_abs'].quantile(0.68)
        vy_p_abs_95 = data['vy.error%_abs'].quantile(0.95)

        vy_p_origin_mean = data['vy.error%'].mean()
        vy_p_origin_std = data['vy.error%'].std()

        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            'vy_abs_mean[m/s]': vy_abs_mean, 'vy_abs_68[m/s]': vy_abs_68, 'vy_abs_95[m/s]': vy_abs_95,
            'vy_origin_mean[m/s]': vy_origin_mean, 'vy_origin_std[m/s]': vy_origin_std,
            'vy%_abs_mean': vy_p_abs_mean, 'vy%_abs_68': vy_p_abs_68, 'vy%_abs_95': vy_p_abs_95,
            'vy%_origin_mean': vy_p_origin_mean, 'vy%_origin_std': vy_p_origin_std,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class YawError:
    """
    统计航向角误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        yaw_abs_mean = data['yaw.error_abs'].mean()
        yaw_abs_68 = data['yaw.error_abs'].quantile(0.68)
        yaw_abs_95 = data['yaw.error_abs'].quantile(0.95)

        yaw_origin_mean = data['yaw.error'].mean()
        yaw_origin_std = data['yaw.error'].std()

        reverse_ratio = data['yaw.is_reverse'].sum() / len(data)
        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            'yaw_abs_mean[deg]': yaw_abs_mean, 'yaw_abs_68[deg]': yaw_abs_68, 'yaw_abs_95[deg]': yaw_abs_95,
            'yaw_origin_mean[deg]': yaw_origin_mean, 'yaw_origin_std[deg]': yaw_origin_std,
            'reverse%': reverse_ratio,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class LengthError:
    """
    统计长度误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data[input_data['gt.type'] == input_data['pred.type']]

        length_abs_mean = data['length.error_abs'].mean()
        length_abs_68 = data['length.error_abs'].quantile(0.68)
        length_abs_95 = data['length.error_abs'].quantile(0.95)

        length_origin_mean = data['length.error'].mean()
        length_origin_std = data['length.error'].std()

        length_p_abs_mean = data['length.error%_abs'].mean()
        length_p_abs_68 = data['length.error%_abs'].quantile(0.68)
        length_p_abs_95 = data['length.error%_abs'].quantile(0.95)

        length_p_origin_mean = data['length.error%'].mean()
        length_p_origin_std = data['length.error%'].std()

        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            'length_abs_mean[m]': length_abs_mean, 'length_abs_68[m]': length_abs_68, 'length_abs_95[m]': length_abs_95,
            'length_origin_mean[m]': length_origin_mean, 'length_origin_std[m]': length_origin_std,
            'length%_abs_mean': length_p_abs_mean, 'length%_abs_68': length_p_abs_68, 'length%_abs_95': length_p_abs_95,
            'length%_origin_mean': length_p_origin_mean, 'length%_origin_std': length_p_origin_std,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class WidthError:
    """
    统计宽度误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data[input_data['gt.type'] == input_data['pred.type']]

        width_abs_mean = data['width.error_abs'].mean()
        width_abs_68 = data['width.error_abs'].quantile(0.68)
        width_abs_95 = data['width.error_abs'].quantile(0.95)

        width_origin_mean = data['width.error'].mean()
        width_origin_std = data['width.error'].std()

        width_p_abs_mean = data['width.error%_abs'].mean()
        width_p_abs_68 = data['width.error%_abs'].quantile(0.68)
        width_p_abs_95 = data['width.error%_abs'].quantile(0.95)

        width_p_origin_mean = data['width.error%'].mean()
        width_p_origin_std = data['width.error%'].std()

        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            'width_abs_mean[m]': width_abs_mean, 'width_abs_68[m]': width_abs_68, 'width_abs_95[m]': width_abs_95,
            'width_origin_mean[m]': width_origin_mean, 'width_origin_std[m]': width_origin_std,
            'width%_abs_mean': width_p_abs_mean, 'width%_abs_68': width_p_abs_68, 'width%_abs_95': width_p_abs_95,
            'width%_origin_mean': width_p_origin_mean, 'width%_origin_std': width_p_origin_std,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class HeightError:
    """
    统计高度误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data[input_data['gt.type'] == input_data['pred.type']]

        height_abs_mean = data['height.error_abs'].mean()
        height_abs_68 = data['height.error_abs'].quantile(0.68)
        height_abs_95 = data['height.error_abs'].quantile(0.95)

        height_origin_mean = data['height.error'].mean()
        height_origin_std = data['height.error'].std()

        height_p_abs_mean = data['height.error%_abs'].mean()
        height_p_abs_68 = data['height.error%_abs'].quantile(0.68)
        height_p_abs_95 = data['height.error%_abs'].quantile(0.95)

        height_p_origin_mean = data['height.error%'].mean()
        height_p_origin_std = data['height.error%'].std()

        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            'height_abs_mean[m]': height_abs_mean, 'height_abs_68[m]': height_abs_68, 'height_abs_95[m]': height_abs_95,
            'height_origin_mean[m]': height_origin_mean, 'height_origin_std[m]': height_origin_std,
            'height%_abs_mean': height_p_abs_mean, 'height%_abs_68': height_p_abs_68, 'height%_abs_95': height_p_abs_95,
            'height%_origin_mean': height_p_origin_mean, 'height%_origin_std': height_p_origin_std,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class ObstaclesMetricStatistics:

    def run(self, input_data, input_parameter_container):
        region_division = input_parameter_container['region_division']
        characteristic = input_parameter_container['characteristic']

        total_data = input_data['recall_precision']
        total_data.index = total_data['corresponding_index'].to_list()

        # 获取每种目标类型的index历表
        type_classification_list = set(total_data['pred.type_classification'].drop_duplicates().dropna().to_list()
                                       + total_data['gt.type_classification'].drop_duplicates().dropna().to_list())
        type_corresponding_index_dict = {type_classification: [] for type_classification in type_classification_list}

        # 获取每个区域的index列表
        region_corresponding_index_dict = {}

        for idx, row in total_data.iterrows():

            ru = row['gt.road_user'] if row['gt.flag'] == 1 else row['pred.road_user']
            if ru == 'DRU':
                ru_region_division = region_division['DRU'] + region_division['VRU']
            else:
                ru_region_division = region_division['VRU']

            for region in ru_region_division:
                region_text = self.get_region_text(region)
                if row['gt.flag'] == 1:
                    pt = {
                        'x': row['gt.x'], 'y': row['gt.y'],
                    }
                    if self.check_region(pt, region):
                        if region_text not in region_corresponding_index_dict:
                            region_corresponding_index_dict[region_text] = []
                        region_corresponding_index_dict[region_text].append(idx)

                elif row['pred.flag'] == 1:
                    pt = {
                        'x': row['pred.x'], 'y': row['pred.y'],
                    }
                    if self.check_region(pt, region):
                        if region_text not in region_corresponding_index_dict:
                            region_corresponding_index_dict[region_text] = []
                        region_corresponding_index_dict[region_text].append(idx)

            if row['gt.flag'] == 1:
                type_corresponding_index_dict[row['gt.type_classification']].append(idx)

            elif row['pred.flag'] == 1:
                type_corresponding_index_dict[row['pred.type_classification']].append(idx)

        json_datas = []
        for metric, data in input_data.items():

            for region_text, region_index in region_corresponding_index_dict.items():
                for type_classification, type_index in type_corresponding_index_dict.items():

                    # 获取共同的index
                    common_index = sorted(list(set(region_index) & set(type_index)))
                    selected_data = data[data['corresponding_index'].isin(common_index)]
                    if len(selected_data) == 0:
                        continue

                    metric_class = change_name(metric)
                    func = eval(f'{metric_class}()')
                    res = func(selected_data)

                    json_datas.append(
                        {
                            'FeatureDetail': obstacles_characteristic_text[characteristic],
                            'RangeDetails': region_text,
                            'ObstacleName': obstacles_type_classification_text[type_classification],
                            'MetricTypeName': obstacles_metric_text[metric],
                            'Output': res,
                        })

        return json_datas

    def get_region_text(self, region):
        region_text = []
        for i in ['x', 'y']:
            r = region[i]
            if not isinstance(r[0], list):
                region_text.append(f'{i}({r[0]}~{r[1]})')
            else:
                region_text.append(f'{i}({r[0][0]}~{r[0][1]})({r[1][0]}~{r[1][1]})')

        return ','.join(region_text)

    def check_region(self, pt, region):
        is_valid_list = []
        for range_type in ['x', 'y']:
            if isinstance(region[range_type][0], float) or isinstance(region[range_type][0], int):
                is_valid = region[range_type][0] <= pt[range_type] < region[range_type][1]
                is_valid_list.append(is_valid)

            elif isinstance(region[range_type][0], list) or isinstance(region[range_type][0], tuple):
                is_valid = any(
                    [sub_range_value[0] <= pt[range_type] < sub_range_value[1] for sub_range_value in
                     region[range_type]])
                is_valid_list.append(is_valid)

        return all(is_valid_list)


class LateralError:

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        lateral_0_30_mean = data['0-30.lateral.error'].mean()
        lateral_0_30_68 = data['0-30.lateral.error'].quantile(0.68)
        lateral_0_30_95 = data['0-30.lateral.error'].quantile(0.95)

        lateral_30_60_mean = data['30-60.lateral.error'].mean()
        lateral_30_60_68 = data['30-60.lateral.error'].quantile(0.68)
        lateral_30_60_95 = data['30-60.lateral.error'].quantile(0.95)

        lateral_60_120_mean = data['60-120.lateral.error'].mean()
        lateral_60_120_68 = data['60-120.lateral.error'].quantile(0.68)
        lateral_60_120_95 = data['60-120.lateral.error'].quantile(0.95)

        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            '0-30_abs_mean[m]': lateral_0_30_mean, '0-30_abs_68[m]': lateral_0_30_68, '0-30_abs_95[m]': lateral_0_30_95,
            '30-60_abs_mean[m]': lateral_30_60_mean, '30-60_abs_68[m]': lateral_30_60_68, '30-60_abs_95[m]': lateral_30_60_95,
            '60-120_abs_mean[m]': lateral_60_120_mean, '60-120_abs_68[m]': lateral_60_120_68, '60-120_abs_95[m]': lateral_60_120_95,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class HeadingError:

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        heading_0_mean = data['0.heading.error'].mean()
        heading_0_68 = data['0.heading.error'].quantile(0.68)
        heading_0_95 = data['0.heading.error'].quantile(0.95)

        heading_50_mean = data['50.heading.error'].mean()
        heading_50_68 = data['50.heading.error'].quantile(0.68)
        heading_50_95 = data['50.heading.error'].quantile(0.95)

        pass_ratio = 1 - data['is_abnormal'].sum() / len(data)

        res = {
            '0_heading_abs_mean[deg]': heading_0_mean, '0_heading_abs_68[deg]': heading_0_68, '0_heading_abs_95[deg]': heading_0_95,
            '50_heading_abs_mean[deg]': heading_50_mean, '50_heading_abs_68[deg]': heading_50_68, '50_heading_abs_95[deg]': heading_50_95,
            'sample_count': len(data), 'pass_ratio%': pass_ratio,
        }

        return res


class LinesMetricStatistics:

    def run(self, input_data, input_parameter_container):
        radius_division = input_parameter_container['radius_division']
        characteristic = input_parameter_container['characteristic']

        total_data = input_data['recall_precision']
        total_data.index = total_data['corresponding_index'].to_list()

        # 获取每种目标类型的index历表
        type_classification_list = set(total_data['pred.type_classification'].drop_duplicates().dropna().to_list()
                                       + total_data['gt.type_classification'].drop_duplicates().dropna().to_list())
        type_corresponding_index_dict = {type_classification: [] for type_classification in type_classification_list}

        # 获取每个区域的index列表
        radius_corresponding_index_dict = {}

        for idx, row in total_data.iterrows():

            for radius in radius_division:
                radius_text = self.get_radius_text(radius)
                if row['gt.flag'] == 1:
                    r = row['gt.radius']
                    if self.check_radius(r, radius):
                        if radius_text not in radius_corresponding_index_dict:
                            radius_corresponding_index_dict[radius_text] = []
                        radius_corresponding_index_dict[radius_text].append(idx)

                elif row['pred.flag'] == 1:
                    r = row['pred.radius']
                    if self.check_radius(r, radius):
                        if radius_text not in radius_corresponding_index_dict:
                            radius_corresponding_index_dict[radius_text] = []
                        radius_corresponding_index_dict[radius_text].append(idx)

            if row['gt.flag'] == 1:
                type_corresponding_index_dict[row['gt.type_classification']].append(idx)

            elif row['pred.flag'] == 1:
                type_corresponding_index_dict[row['pred.type_classification']].append(idx)

        json_datas = []
        for metric, data in input_data.items():

            for radius_text, radius_index in radius_corresponding_index_dict.items():
                for type_classification, type_index in type_corresponding_index_dict.items():

                    # 获取共同的index
                    common_index = sorted(list(set(radius_index) & set(type_index)))
                    selected_data = data[data['corresponding_index'].isin(common_index)]
                    if len(selected_data) == 0:
                        continue

                    metric_class = change_name(metric)
                    func = eval(f'{metric_class}()')
                    res = func(selected_data)

                    json_datas.append(
                        {
                            'FeatureDetail': lines_characteristic_text[characteristic],
                            'CurvatureRadius': radius_text,
                            'LinesName': lines_type_classification_text[type_classification],
                            'MetricTypeName': lines_metric_text[metric],
                            'Output': res,
                        })

        return json_datas

    def get_radius_text(self, radius):
        radius_text = '~'.join([f'{r:.0f}' for r in radius])
        return f'R({radius_text})'

    def check_radius(self, r, radius):
        return radius[0] < r <= radius[1]