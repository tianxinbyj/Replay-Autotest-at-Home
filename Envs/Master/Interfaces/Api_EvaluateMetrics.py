"""
@Author: BU YUJUN
@Date: 2024/8/16 下午2:02
"""
import argparse
import json
import os
import sys
import pandas as pd

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.Master.Modules.PerceptMetrics.PerceptMetrics.MetricEvaluator import ObstaclesMetricEvaluator
from Envs.Master.Modules.PerceptMetrics.PerceptMetrics.MetricEvaluator import ObstaclesMetricFilter


def main():
    parser = argparse.ArgumentParser(description="get time gap")
    parser.add_argument("-m", "--match_data_path", type=str, required=True, help="pred data path")
    parser.add_argument("-j", "--parameter_json_path", type=str, required=True, help="parameter json path")
    parser.add_argument("-f", "--metric_folder", type=str, required=True, help="metric folder")

    args = parser.parse_args()

    match_data = pd.read_csv(args.match_data_path, index_col=False)
    with open(args.parameter_json_path, 'r', encoding='utf-8') as f:
        parameter_json = json.load(f)

    metric_evaluator = ObstaclesMetricEvaluator()
    data_dict = metric_evaluator.run(match_data, parameter_json)
    for metric, metric_data in data_dict.items():
        total_folder = os.path.join(args.metric_folder, 'total')
        if not os.path.exists(total_folder):
            os.makedirs(total_folder)

        path = os.path.join(total_folder, f'{metric}.csv')
        metric_data.to_csv(path, index=False, encoding='utf_8_sig')

        input_data = {
            'total_data': match_data,
            'data_to_filter': metric_data,
        }

        metric_filter = ObstaclesMetricFilter()
        characteristic_data_dict = metric_filter.run(input_data, parameter_json)
        for characteristic, characteristic_data in characteristic_data_dict.items():
            characteristic_folder = os.path.join(args.metric_folder, characteristic)
            if not os.path.exists(characteristic_folder):
                os.makedirs(characteristic_folder)

            path = os.path.join(characteristic_folder, f'{metric}.csv')
            characteristic_data.to_csv(path, index=False, encoding='utf_8_sig')


if __name__ == '__main__':
    main()
    # match_data_path = '/home/hp/aeb_test_result/04_TestData/1-Obstacles/01_ScenarioUnit/20240724_171737_n000001_0/01_Data/Obstacles/VAFrontWideObstacles2dDet/match/match_data.csv'
    # parameter_json_path = '/home/hp/Replay-Autotest-at-Home/Temp/evaluate_api_parameter.json'
    # metric_folder = '/home/hp/aeb_test_result/04_TestData/1-Obstacles/01_ScenarioUnit/20240724_171737_n000001_0/01_Data/Obstacles/VAFrontWideObstacles2dDet/metric'
    #
    # match_data = pd.read_csv(match_data_path, index_col=False)
    # with open(parameter_json_path, 'r', encoding='utf-8') as f:
    #     parameter_json = json.load(f)
    #
    # metric_evaluator = ObstaclesMetricEvaluator()
    # data_dict = metric_evaluator.run(match_data, parameter_json)
    # for metric, metric_data in data_dict.items():
    #     total_folder = os.path.join(metric_folder, 'total')
    #     if not os.path.exists(total_folder):
    #         os.makedirs(total_folder)
    #
    #     path = os.path.join(total_folder, f'{metric}.csv')
    #     metric_data.to_csv(path, index=False, encoding='utf_8_sig')
    #
    #     input_data = {
    #         'total_data': match_data,
    #         'data_to_filter': metric_data,
    #     }
    #
    #     metric_filter = ObstaclesMetricFilter()
    #     characteristic_data_dict = metric_filter.run(input_data, parameter_json)
    #     for characteristic, characteristic_data in characteristic_data_dict.items():
    #         characteristic_folder = os.path.join(metric_folder, characteristic)
    #         if not os.path.exists(characteristic_folder):
    #             os.makedirs(characteristic_folder)
    #
    #         path = os.path.join(characteristic_folder, f'{metric}.csv')
    #         characteristic_data.to_csv(path, index=False, encoding='utf_8_sig')