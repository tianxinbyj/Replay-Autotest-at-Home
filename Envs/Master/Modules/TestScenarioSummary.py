"""
@Author: BU YUJUN
@Date: 2025/2/7 下午14:00
"""
import copy
import json
import subprocess
import sys
import datetime
import glob
import os

import yaml
import pandas as pd
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Libs import get_project_path
sys.path.append(get_project_path())
from Utils.Libs import test_encyclopaedia, create_folder, bench_config, project_path

import warnings

warnings.filterwarnings("ignore")

scenario_summary_path = os.path.join(project_path, 'Docs', 'Resources', 'scenario_info', 'scenario_summary.csv')
scenario_test_record_path = os.path.join(project_path, 'Docs', 'Resources', 'scenario_info', 'scenario_test_record.csv')
colors = ['#3682be', '#45a776', '#f05326', '#b3974e', '#38cb7d', '#ddae33', '#844bb3',
          '#93c555', '#5f6694', '#df3881']


class TestScenarioSummary:

    def __init__(self):
        if not os.path.exists(scenario_summary_path):
            return

        scenario_summary = pd.read_csv(scenario_summary_path, index_col=False)
        scenario_summary = scenario_summary
        scenario_summary['test_count'] = 0
        scenario_summary['test_history'] = '[]'
        scenario_summary['bug_excluding'] = '[]'

        if not os.path.exists(scenario_test_record_path):
            self.scenario_test_record = scenario_summary
        else:
            scenario_test_record = pd.read_csv(scenario_test_record_path, index_col=False)
            rows = []
            for idx, row in scenario_summary.iterrows():
                scenario_id = row['scenario_id']
                truth_source = row['truth_source']
                new_row = row.values.tolist()
                temp = scenario_test_record[
                    (scenario_test_record['scenario_id'] == scenario_id)
                    & (scenario_test_record['truth_source'] == truth_source)]
                if len(temp) == 1:
                    new_row[-3] = temp['test_count'].iloc[0]
                    new_row[-2] = temp['test_history'].iloc[0]
                    new_row[-1] = temp['bug_excluding'].iloc[0]
                rows.append(new_row)

            self.scenario_test_record = pd.DataFrame(rows, columns=scenario_summary.columns)

    def register_test_project(self, test_project_folder, update=True):
        for test_task_folder in glob.glob(os.path.join(test_project_folder, '04_TestData', '*-*')):
            test_config_path = os.path.join(test_task_folder, 'TestConfig.yaml')
            with open(test_config_path, 'r') as f:
                test_config = yaml.load(f, Loader=yaml.FullLoader)

            version = test_config['version']
            test_topic = test_config['test_topic']
            test_date = test_config['test_date']
            truth_source = test_config['test_action']['ros2bag']['truth_source']

            print(f'增加了{test_date} {version} {test_topic}的测试记录')
            test_result_path = os.path.join(test_task_folder, 'TestResult.yaml')
            with open(test_result_path, 'r') as f:
                test_result = yaml.load(f, Loader=yaml.FullLoader)

            test_scenario_list = test_result['ScenarioUnit'].keys()
            bugItems_path = os.path.join(test_task_folder, test_result['OutputResult']['bugItems'])
            bugItems = pd.read_csv(bugItems_path, index_col=False)
            bug_excludes = bugItems[
                (bugItems['is_valid'].isin([2, 3]))
                & (bugItems['bug_type'] != '正样本误检测')]

            for scenario_id in test_scenario_list:
                temp = self.scenario_test_record[(self.scenario_test_record['scenario_id'] == scenario_id)
                                 & (self.scenario_test_record['truth_source'] == truth_source)]
                if len(temp) != 1:
                    print(f'未找到{scenario_id} {truth_source}的场景信息')
                    continue

                index = temp.index[0]
                test_history = []
                bug_excluding = []
                if not update:
                    test_history = json.loads(self.scenario_test_record.at[index, 'test_history'])
                    bug_excluding = json.loads(self.scenario_test_record.at[index, 'bug_excluding'])

                new_test_record = {
                    'test_date': str(test_date),
                    'test_topic': test_topic,
                    'truth_source': truth_source,
                    'version': version,
                }
                bug_exclude = bug_excludes[bug_excludes['scenario_id'] == scenario_id]
                bug_exclude_id = [int(id_) for id_ in set(bug_exclude['gt_target_id'].values.tolist())]
                print(bug_exclude_id)

                if new_test_record not in test_history:
                    print(f'增加了测试记录 {scenario_id} {truth_source}')
                    test_history.append(new_test_record)
                    test_history.sort(key=lambda x: x['test_date'])

                    bug_excluding.extend(bug_exclude_id)
                    bug_excluding = list(sorted(set(bug_excluding)))

                    self.scenario_test_record.at[index, 'test_history'] = json.dumps(test_history)
                    self.scenario_test_record.at[index, 'test_count'] = len(test_history)
                    self.scenario_test_record.at[index, 'bug_excluding'] = json.dumps(bug_excluding)
                else:
                    print(f'重复的测试记录 {scenario_id} {truth_source}')

            self.scenario_test_record.to_csv(scenario_test_record_path, index=False)


if __name__ == "__main__":
    tss = TestScenarioSummary()
    for test_project_folder in [
        '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_manual_20241205_181840',
        '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_20250107_010621',
        '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_20250120_010000',
    ]:
        tss.register_test_project(test_project_folder, False)