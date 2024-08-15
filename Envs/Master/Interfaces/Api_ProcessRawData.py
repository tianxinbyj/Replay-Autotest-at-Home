"""  
@Author: BU YUJUN
@Date: 2024/8/15 下午2:46  
"""
import argparse
import sys
import pandas as pd
import json

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.Master.Modules.PerceptMetrics.PerceptMetrics.PreProcess import ObstaclesPreprocess


def main():
    parser = argparse.ArgumentParser(description="process raw data")
    parser.add_argument("-r", "--raw_data_path", type=str, required=True, help="raw data path")
    parser.add_argument("-j", "--parameter_json_path", type=str, required=True, help="parameter json path")
    parser.add_argument("-p", "--process_data_path", type=str, required=True, help="process data path")
    args = parser.parse_args()

    raw_data = pd.read_csv(args.raw_data_path, index_col=False)
    with open(args.parameter_json_path, 'r', encoding='utf-8') as f:
        parameter_json = json.load(f)

    preprocess_instance = ObstaclesPreprocess()
    data = preprocess_instance.run(raw_data, parameter_json)
    data.to_csv(args.process_data_path, index=False, encoding='utf_8_sig')


if __name__ == '__main__':
    main()
