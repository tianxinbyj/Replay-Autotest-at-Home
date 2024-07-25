"""  
@Author: BU YUJUN
@Date: 2024/7/25 下午3:27  
"""


class RecallPrecision:
    """
    计算准召信息的统计

    """

    def __init__(self):
        self.columns = [
            'TP', 'FP', 'FN', 'CTP',
            'recall', 'precision',
            'type_accuracy'
        ]


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

        total_data = input_data['recall&precision']

        corresponding_index_dict = {self.get_region_text(region): [] for region in self.region_division}
        for idx, row in total_data.iterrows():
            for region in self.region_division:
                region_text = self.get_region_text(region)
                if row['gt.flag'] == 1:
                    pt = {
                        'x': row['gt.x'], 'y': row['gt.y'],
                    }
                    if self.check_region(pt, region):
                        corresponding_index_dict[region_text].append(idx)
                elif row['pred.flag'] == 1:
                    pt = {
                        'x': row['pred.x'], 'y': row['pred.y'],
                    }
                    if self.check_region(pt, region):
                        corresponding_index_dict[region_text].append(idx)

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
            if isinstance(region[0], float) or isinstance(region[0], int):
                is_valid = region[0] <= pt[range_type] < region[1]
                is_valid_list.append(is_valid)

            elif isinstance(region[0], list) or isinstance(region[0], tuple):
                is_valid = any(
                    [sub_range_value[0] <= pt[range_type] < sub_range_value[1] for sub_range_value in region])
                is_valid_list.append(is_valid)

        return all(is_valid_list)