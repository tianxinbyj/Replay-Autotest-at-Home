"""
@Author: BU YUJUN
@Date: 2025/2/7 下午14:00
"""
import glob
import json
import os
import shutil
import sys

import numpy as np
import pandas as pd
import yaml
from matplotlib import pyplot as plt
from matplotlib.backends.backend_template import FigureCanvas

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Libs import get_project_path
sys.path.append(get_project_path())
from Utils.Libs import project_path

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


class TestResultSummary:

    def __init__(self):
        self.visualization_kpi_threshold = {
            'recall%': 0.04,
            'precision%': 0.15,
            'x_abs_95[m]': 0.25,
            'x%_abs_95[m]': 0.25,
            'y_abs_95[m]': 0.25,
            'yaw_abs_95[deg]': 5,
            'length_abs_95[m]': 0.25,
            'width_abs_95[m]': 0.25,
        }

    def load_test_result(self, test_project_folder_list):
        test_result_summary = {}
        for test_project_folder in test_project_folder_list:
            for test_task_folder in glob.glob(os.path.join(test_project_folder, '04_TestData', '*-*')):
                test_config_path = os.path.join(test_task_folder, 'TestConfig.yaml')
                with open(test_config_path, 'r') as f:
                    test_config = yaml.load(f, Loader=yaml.FullLoader)

                version = test_config['version']
                test_topic = test_config['test_topic']
                test_date = test_config['test_date']
                truth_source = test_config['test_action']['ros2bag']['truth_source']

                if test_topic not in test_result_summary:
                    test_result_summary[test_topic] = {}

                if 'version' not in test_result_summary[test_topic]:
                    test_result_summary[test_topic]['version'] = []
                    test_result_summary[test_topic]['test_date'] = []
                test_result_summary[test_topic]['version'].append(version)
                test_result_summary[test_topic]['test_date'].append(str(test_date))

                test_result_path = os.path.join(test_task_folder, 'TestResult.yaml')
                with open(test_result_path, 'r') as f:
                    test_result = yaml.load(f, Loader=yaml.FullLoader)

                if test_topic == 'Obstacles':
                    excel_path = os.path.join(test_task_folder, test_result['OutputResult']['report_table']['关键目标'])
                    for _, scenario_tag in enumerate(pd.ExcelFile(excel_path).sheet_names):
                        data = pd.read_excel(excel_path, sheet_name=scenario_tag, header=[0, 1, 2], index_col=[0, 1, 2])
                        for col in data.columns:
                            if col[1] not in self.visualization_kpi_threshold or col[2] == 'kpi_target':
                                continue
                            for index in data.index:
                                title = f'{col[1]}--{scenario_tag}--{col[2]}--{index[0]}--{index[1]}'
                                if title not in test_result_summary[test_topic]:
                                    test_result_summary[test_topic][title] = []

                                test_result_summary[test_topic][title].append(float(data.at[index, col]))

        visualization_item = {}
        # 最新的版本和次新的版本的差异大于一定值
        for title, res in test_result_summary['Obstacles'].items():
            kpi_type = title.split('--')[0]
            if kpi_type not in self.visualization_kpi_threshold.keys():
                visualization_item['INFO--' + title] = res
            elif all([not np.isnan(v) for v in res]):
                limit = self.visualization_kpi_threshold[kpi_type]
                if kpi_type in ['recall%', 'precision%']:
                    diff = res[-2] - res[-1]
                elif kpi_type in ['yaw_abs_95[deg]']:
                    diff = res[-1] - res[-2]
                else:
                    diff = (res[-1] - res[-2]) / max(res[-1], res[-2])
                if abs(diff) > limit:
                    if diff < 0:
                        visualization_item['BETTER--' + title] = res
                    else:
                        visualization_item['WORSE--' + title] = res

        # 使用 sorted 函数对字典的键进行排序，排序依据是键按 '--' 分割后的第一个元素
        sorted_keys = sorted(visualization_item.keys(), key=lambda x: (x.split('--')[0], x.split('--')[1]))
        # 根据排序后的键构建新的字典
        visualization_item = {key: visualization_item[key] for key in sorted_keys}
        print(len(visualization_item))

        folder = '/home/zhangliwei01/ZONE/123/pic'
        shutil.rmtree(folder, ignore_errors=True)

        for title, res in visualization_item.items():
            if 'INFO' in title:
                continue

            if '行人' in title or '两轮车' in title or 'width' in title or 'length' in title:
                continue

            compare_res = title.split('--')[0]
            os.makedirs(os.path.join(folder, compare_res), exist_ok=True)

            kpi_type = title.split('--')[1]
            fig = plt.figure(figsize=(10, 5.625))
            fig.tight_layout()
            plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
            grid = plt.GridSpec(1, 1, wspace=0.2, hspace=0.25)
            ax = fig.add_subplot(grid[0, 0])

            x = [f'{v}\n{t}' for v, t in zip(visualization_item['INFO--version'], visualization_item['INFO--test_date'])]
            color = 'limegreen' if 'BETTER' in title else 'lightcoral'
            ax.plot(x, res,
                    marker='o', markeredgecolor='black',
                    markerfacecolor=color, markersize=10,
                    linestyle='--', label=kpi_type)

            ax.tick_params(axis='x', labelrotation=0)
            ax.legend(loc='best')
            ax.set_title(title.replace('--', ','))
            ax.grid(linestyle='-', linewidth=0.5, color='lightgray', alpha=0.6)

            if kpi_type in ['recall%', 'precision%']:
                ax.set_ylim(0, 1)
            else:
                ax.set_ylim(max(0, 1.5 * min(res) - 0.5 * max(res)), 1.5 * max(res) - 0.5 * min(res))

            path = os.path.join(folder, compare_res, f"{title.replace('/', '')}.png")
            canvas = FigureCanvas(fig)
            canvas.print_figure(path, facecolor='white', dpi=100)
            fig.clf()
            plt.close()


if __name__ == "__main__":
    # tss = TestScenarioSummary()
    # for test_project_folder in [
    #     '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_manual_20241205_181840',
    #     '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_20250107_010621',
    #     '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_20250120_010000',
    # ]:
    #     tss.register_test_project(test_project_folder, False)

    test_project_folder_list = [
        '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_manual_20241205_181840',
        '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_20250107_010621',
        '/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_20250120_010000',
    ]

    trs = TestResultSummary()
    trs.load_test_result(test_project_folder_list)