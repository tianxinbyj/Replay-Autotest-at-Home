"""  
@Author: BU YUJUN
@Date: 2024/8/15 下午2:02  
"""
import argparse
import sys
import pandas as pd

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.Master.Modules.PerceptMetrics.PerceptMetrics.PreProcess import calculate_time_gap


def main():
    parser = argparse.ArgumentParser(description="get time gap")
    parser.add_argument("-b", "--baseline_data_path", type=str, required=True, help="baseline ego data path")
    parser.add_argument("-c", "--calibrated_data_path", type=str, required=True, help="calibrated ego data path")
    args = parser.parse_args()

    baseline_data = pd.read_csv(args.baseline_data_path, index_col=False)
    baseline_time_series = baseline_data['time_stamp'].to_list()
    baseline_velocity_series = baseline_data['ego_vx'].to_list()

    calibrated_data = pd.read_csv(args.calibrated_data_path, index_col=False)
    calibrated_time_series = calibrated_data['time_stamp'].to_list()
    calibrated_velocity_series = calibrated_data['ego_vx'].to_list()

    t_delta, v_error = calculate_time_gap(
        baseline_time_series=baseline_time_series,
        baseline_velocity_series=baseline_velocity_series,
        calibrated_time_series=calibrated_time_series,
        calibrated_velocity_series=calibrated_velocity_series
    )
    print(t_delta, v_error)


if __name__ == '__main__':
    main()
