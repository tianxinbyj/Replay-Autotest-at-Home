"""  
@Author: BU YUJUN
@Date: 2024/8/15 下午3:59  
"""
import argparse
import json
import sys
import pandas as pd

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.Master.Modules.PerceptMetrics.PerceptMetrics.MatchTool import ObstaclesMatchTool


def main():
    parser = argparse.ArgumentParser(description="match obstacles")
    parser.add_argument("-p", "--pred_data_path", type=str, required=True, help="pred data path")
    parser.add_argument("-g", "--gt_data_path", type=str, required=True, help="gt data path")
    parser.add_argument("-t", "--match_timestamp_path", type=str, required=True, help="match timestamp path")
    parser.add_argument("-j", "--parameter_json_path", type=str, required=True, help="parameter json path")
    parser.add_argument("-m", "--match_data_path", type=str, required=True, help="match data path")
    args = parser.parse_args()

    pred_data = pd.read_csv(args.pred_data_path, index_col=False)
    gt_data = pd.read_csv(args.gt_data_path, index_col=False)
    match_timestamp = pd.read_csv(args.match_timestamp_path, index_col=False)
    with open(args.parameter_json_path, 'r', encoding='utf-8') as f:
        parameter_json = json.load(f)

    input_data = {
        'match_timestamp': match_timestamp,
        'gt_data': gt_data,
        'pred_data': pred_data,
    }

    match_tool = ObstaclesMatchTool()
    data = match_tool.run(input_data, parameter_json)
    data.to_csv(args.match_data_path, index=False, encoding='utf_8_sig')


if __name__ == '__main__':
    main()