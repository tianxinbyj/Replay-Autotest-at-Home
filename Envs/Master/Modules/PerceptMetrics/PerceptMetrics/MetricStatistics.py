"""  
@Author: BU YUJUN
@Date: 2024/7/25 下午3:27  
"""
import warnings
import pandas as pd

# 屏蔽特定的警告
warnings.filterwarnings("ignore", category=pd.errors.SettingWithCopyWarning)


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

        TP = len(data[(data['gt.flag'] == 1) & (data['pred.flag'] == 1)])
        FP = len(data[(data['gt.flag'] == 0) & (data['pred.flag'] == 1)])
        FN = len(data[(data['gt.flag'] == 1) & (data['pred.flag'] == 0)])

        gt_count = TP + FN
        pred_count = TP + FP

        result_series = input_data.apply(lambda row: self.check_CTP(row.to_dict()), axis=1)
        data.loc[:, 'CTP'] = result_series
        CTP = len(data[(data['gt.flag'] == 1) & (data['pred.flag'] == 1) & (data['CTP'] == 1)])

        recall = TP / (TP + FN) if gt_count != 0 else 0
        precision = TP / (TP + FP) if pred_count != 0 else 0
        type_accuracy = CTP / TP if TP != 0 else 0

        res = {
            'TP': TP, 'FP': FP, 'FN': FN, 'CTP': CTP,
            'recall': recall, 'precision': precision, 'type_accuracy': type_accuracy
        }

        return res

    def check_CTP(self, row):

        gt_type = row['gt.type']
        pred_type = row['pred.type']
        gt_sub_type = row['gt.sub_type']
        pred_sub_type = row['pred.sub_type']

        if gt_type != pred_type:
            return 0
        else:
            if gt_type in [2, 18]:
                return 1
            else:
                if gt_sub_type == pred_sub_type:
                    return 1
                else:
                    return 0


class XError:
    """
    统计纵向距离误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        x_abs_mean = data['x.error_abs'].mean()
        x_abs_90 = data['x.error_abs'].quantile(0.9)
        x_abs_95 = data['x.error_abs'].quantile(0.95)

        x_origin_mean = data['x.error'].mean()
        x_origin_std = data['x.error'].std()

        x_p_abs_mean = data['x.error%_abs'].mean()
        x_p_abs_90 = data['x.error%_abs'].quantile(0.9)
        x_p_abs_95 = data['x.error%_abs'].quantile(0.95)

        x_p_origin_mean = data['x.error%'].mean()
        x_p_origin_std = data['x.error%'].std()

        res = {
            'x_abs_mean': x_abs_mean, 'x_abs_90': x_abs_90, 'x_abs_95': x_abs_95,
            'x_origin_mean': x_origin_mean, 'x_origin_std': x_origin_std,
            'x%_abs_mean': x_p_abs_mean, 'x%_abs_90': x_p_abs_90, 'x%_abs_95': x_p_abs_95,
            'x%_origin_mean': x_p_origin_mean, 'x%_origin_std': x_p_origin_std,
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
        y_abs_90 = data['y.error_abs'].quantile(0.9)
        y_abs_95 = data['y.error_abs'].quantile(0.95)

        y_origin_mean = data['y.error'].mean()
        y_origin_std = data['y.error'].std()

        y_p_abs_mean = data['y.error%_abs'].mean()
        y_p_abs_90 = data['y.error%_abs'].quantile(0.9)
        y_p_abs_95 = data['y.error%_abs'].quantile(0.95)

        y_p_origin_mean = data['y.error%'].mean()
        y_p_origin_std = data['y.error%'].std()

        res = {
            'y_abs_mean': y_abs_mean, 'y_abs_90': y_abs_90, 'y_abs_95': y_abs_95,
            'y_origin_mean': y_origin_mean, 'y_origin_std': y_origin_std,
            'y%_abs_mean': y_p_abs_mean, 'y%_abs_90': y_p_abs_90, 'y%_abs_95': y_p_abs_95,
            'y%_origin_mean': y_p_origin_mean, 'y%_origin_std': y_p_origin_std,
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
        vx_abs_90 = data['vx.error_abs'].quantile(0.9)
        vx_abs_95 = data['vx.error_abs'].quantile(0.95)

        vx_origin_mean = data['vx.error'].mean()
        vx_origin_std = data['vx.error'].std()

        vx_p_abs_mean = data['vx.error%_abs'].mean()
        vx_p_abs_90 = data['vx.error%_abs'].quantile(0.9)
        vx_p_abs_95 = data['vx.error%_abs'].quantile(0.95)

        vx_p_origin_mean = data['vx.error%'].mean()
        vx_p_origin_std = data['vx.error%'].std()

        res = {
            'vx_abs_mean': vx_abs_mean, 'vx_abs_90': vx_abs_90, 'vx_abs_95': vx_abs_95,
            'vx_origin_mean': vx_origin_mean, 'vx_origin_std': vx_origin_std,
            'vx%_abs_mean': vx_p_abs_mean, 'vx%_abs_90': vx_p_abs_90, 'vx%_abs_95': vx_p_abs_95,
            'vx%_origin_mean': vx_p_origin_mean, 'vx%_origin_std': vx_p_origin_std,
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
        vy_abs_90 = data['vy.error_abs'].quantile(0.9)
        vy_abs_95 = data['vy.error_abs'].quantile(0.95)

        vy_origin_mean = data['vy.error'].mean()
        vy_origin_std = data['vy.error'].std()

        vy_p_abs_mean = data['vy.error%_abs'].mean()
        vy_p_abs_90 = data['vy.error%_abs'].quantile(0.9)
        vy_p_abs_95 = data['vy.error%_abs'].quantile(0.95)

        vy_p_origin_mean = data['vy.error%'].mean()
        vy_p_origin_std = data['vy.error%'].std()

        res = {
            'vy_abs_mean': vy_abs_mean, 'vy_abs_90': vy_abs_90, 'vy_abs_95': vy_abs_95,
            'vy_origin_mean': vy_origin_mean, 'vy_origin_std': vy_origin_std,
            'vy%_abs_mean': vy_p_abs_mean, 'vy%_abs_90': vy_p_abs_90, 'vy%_abs_95': vy_p_abs_95,
            'vy%_origin_mean': vy_p_origin_mean, 'vy%_origin_std': vy_p_origin_std,
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
        yaw_abs_90 = data['yaw.error_abs'].quantile(0.9)
        yaw_abs_95 = data['yaw.error_abs'].quantile(0.95)

        yaw_origin_mean = data['yaw.error'].mean()
        yaw_origin_std = data['yaw.error'].std()

        reverse_ratio = data['yaw.is_reverse'].sum() / len(data)

        res = {
            'yaw_abs_mean': yaw_abs_mean, 'yaw_abs_90': yaw_abs_90, 'yaw_abs_95': yaw_abs_95,
            'yaw_origin_mean': yaw_origin_mean, 'yaw_origin_std': yaw_origin_std,
            'reverse%': reverse_ratio,
        }

        return res


class LengthError:
    """
    统计长度误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        length_abs_mean = data['length.error_abs'].mean()
        length_abs_90 = data['length.error_abs'].quantile(0.9)
        length_abs_95 = data['length.error_abs'].quantile(0.95)

        length_origin_mean = data['length.error'].mean()
        length_origin_std = data['length.error'].std()

        res = {
            'length_abs_mean': length_abs_mean, 'length_abs_90': length_abs_90, 'length_abs_95': length_abs_95,
            'length_origin_mean': length_origin_mean, 'length_origin_std': length_origin_std,
        }

        return res


class WidthError:
    """
    统计宽度误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        width_abs_mean = data['width.error_abs'].mean()
        width_abs_90 = data['width.error_abs'].quantile(0.9)
        width_abs_95 = data['width.error_abs'].quantile(0.95)

        width_origin_mean = data['width.error'].mean()
        width_origin_std = data['width.error'].std()

        res = {
            'width_abs_mean': width_abs_mean, 'width_abs_90': width_abs_90, 'width_abs_95': width_abs_95,
            'width_origin_mean': width_origin_mean, 'width_origin_std': width_origin_std,
        }

        return res


class HeightError:
    """
    统计长度误差的绝对值平均值，绝对值90%分位数, 绝对值95%分位数，原始值平均值，原始值标准差

    """

    def __call__(self, input_data):
        if not isinstance(input_data, pd.DataFrame):
            raise ValueError(f'Invalid input format for {self.__class__.__name__}')

        data = input_data

        height_abs_mean = data['height.error_abs'].mean()
        height_abs_90 = data['height.error_abs'].quantile(0.9)
        height_abs_95 = data['height.error_abs'].quantile(0.95)

        height_origin_mean = data['height.error'].mean()
        height_origin_std = data['height.error'].std()

        res = {
            'height_abs_mean': height_abs_mean, 'height_abs_90': height_abs_90, 'height_abs_95': height_abs_95,
            'height_origin_mean': height_origin_mean, 'height_origin_std': height_origin_std,
        }

        return res


class ObstaclesMetricStatistics:

    def __init__(self):
        self.region_division = [
            {'x': [-60, -20], 'y': [-8, 8]},
            {'x': [-20, 0], 'y': [-8, 8]},
            {'x': [0, 20], 'y': [-8, 8]},
            {'x': [20, 60], 'y': [-8, 8]},
            {'x': [60, 120], 'y': [-8, 8]},
            {'x': [-60, -20], 'y': [[--20, -8], [8, 20]]},
            {'x': [-20, 0], 'y': [[--20, -8], [8, 20]]},
            {'x': [0, 20], 'y': [[--20, -8], [8, 20]]},
            {'x': [20, 60], 'y': [[--20, -8], [8, 20]]},
            {'x': [60, 120], 'y': [[--20, -8], [8, 20]]},
        ]

    def run(self, input_data, input_parameter_container=None):
        if input_parameter_container is not None:
            self.region_division = input_parameter_container['region_division']

        total_data = input_data['recall_precision']
        total_data.index = total_data['corresponding_index'].to_list()

        # 获取每种目标类型的index历表
        type_classification_list = set(total_data['pred.type_classification'].drop_duplicates().dropna().to_list()
                                       + total_data['gt.type_classification'].drop_duplicates().dropna().to_list())
        type_corresponding_index_dict = {type_classification: [] for type_classification in type_classification_list}

        # 获取每个区域的index列表
        region_corresponding_index_dict = {self.get_region_text(region): [] for region in self.region_division}

        for idx, row in total_data.iterrows():
            for region in self.region_division:
                region_text = self.get_region_text(region)
                if row['gt.flag'] == 1:
                    pt = {
                        'x': row['gt.x'], 'y': row['gt.y'],
                    }
                    if self.check_region(pt, region):
                        region_corresponding_index_dict[region_text].append(idx)

                elif row['pred.flag'] == 1:
                    pt = {
                        'x': row['pred.x'], 'y': row['pred.y'],
                    }
                    if self.check_region(pt, region):
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
                    common_index = set(region_index) & set(type_index)
                    selected_data = data[data['corresponding_index'].isin(common_index)]
                    if len(selected_data) == 0:
                        continue

                    metric_class = change_name(metric)
                    func = eval(f'{metric_class}()')
                    res = func(selected_data)

                    json_datas.append(
                        {
                            'region': region_text,
                            'type': type_classification,
                            'metric': metric,
                            'result': res,
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
