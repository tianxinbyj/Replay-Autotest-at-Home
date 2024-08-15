"""  
@Author: BU YUJUN
@Date: 2024/8/15 下午3:19  
"""
import argparse
import sys

import pandas as pd

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.Master.Modules.PerceptMetrics.PerceptMetrics.MatchTool import match_timestamp


def main():
    parser = argparse.ArgumentParser(description="match timestamp")
    parser.add_argument("-p", "--pred_timestamp_path", type=str, required=True, help="pred timestamp path")
    parser.add_argument("-g", "--gt_timestamp_path", type=str, required=True, help="gt timestamp path")
    parser.add_argument("-t", "--match_tolerance", type=float, required=True, help="match tolerance")
    parser.add_argument("-f", "--match_timestamp_path", type=str, required=True, help="match timestamp path")
    args = parser.parse_args()

    print(args.match_tolerance)

    pred_timestamp = pd.read_csv(args.pred_timestamp_path, index_col=False)['time_stamp'].to_list()
    gt_timestamp = pd.read_csv(args.gt_timestamp_path, index_col=False)['time_stamp'].to_list()

    match_pred_timestamp, match_gt_timestamp = match_timestamp(
        pred_timestamp, gt_timestamp, args.match_tolerance)

    # 保存时间辍匹配数据
    match_timestamp_data = pd.DataFrame(columns=['gt_timestamp', 'pred_timestamp', 'match_time_gap'])
    if len(match_pred_timestamp):
        match_timestamp_data['gt_timestamp'] = match_gt_timestamp
        match_timestamp_data['pred_timestamp'] = match_pred_timestamp
        match_timestamp_data['match_time_gap'] = (match_timestamp_data['pred_timestamp']
                                                  - match_timestamp_data['gt_timestamp'])

    match_timestamp_data.to_csv(args.match_timestamp_path, index=False, encoding='utf_8_sig')


if __name__ == '__main__':
    main()