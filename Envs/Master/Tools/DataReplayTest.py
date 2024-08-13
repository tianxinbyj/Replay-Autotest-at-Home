"""
@Author: BU YUJUN
@Date: 2024/8/9 上午12:00
"""
import glob
import os
import threading

import yaml

from Envs.Master.Modules.ReplayController import ReplayController
from Envs.Master.Modules.DataGrinderPilot import DataGrinderPilotOneTask
from Utils.Logger import UDPLogServer


class DataReplayTestPilot:

    def __init__(self, test_project_path):
        self.log_server = None
        workspace_folder = os.path.join(test_project_path, '03_Workspace')
        if not os.path.exists(os.path.join(workspace_folder, 'install')):
            print('未找到 03_Workspace/install 文件！')
            return

        # 寻找所有TestConfig.yaml
        test_config_yaml_list = glob.glob(os.path.join(test_project_path, '04_TestData', '*', 'TestConfig.yaml'))
        if not test_config_yaml_list:
            print('未找到TestConfig.yaml文件！')
            return

        self.test_project_path = test_project_path
        pred_folder = os.path.join(test_project_path, '01_Prediction')
        gt_folder = os.path.join(test_project_path, '02_GroundTruth')

        self.task_folder_dict = {}
        self.test_config_dict = {}
        for test_config_yaml in test_config_yaml_list:
            with open(test_config_yaml) as f:
                test_config = yaml.load(f, Loader=yaml.FullLoader)
            feature_group = test_config['feature_group']
            test_config['pred_folder'] = pred_folder
            test_config['gt_folder'] = gt_folder

            with open(test_config_yaml, 'w', encoding='utf-8') as f:
                yaml.dump(test_config,
                          f, encoding='utf-8', allow_unicode=True, sort_keys=False)

            if feature_group not in self.task_folder_dict:
                self.task_folder_dict[feature_group] = []
            self.task_folder_dict[feature_group].append(os.path.dirname(test_config_yaml))

            if feature_group not in self.test_config_dict:
                self.test_config_dict[feature_group] = []
            self.test_config_dict[feature_group].append(test_config)

    def start_log_server(self):
        self.log_server = UDPLogServer()
        t = threading.Thread(target=self.log_server.start)
        t.daemon = True
        t.start()

    def replay_and_record(self):
        # 使用第一个test_config的值作为录包的test_action依据
        for feature_group in self.test_config_dict.keys():
            print(f'录制 {feature_group} ros2bag')
            test_config = self.test_config_dict[feature_group][0]
            replay_config = {
                'product': test_config['product'],
                'feature_group': test_config['feature_group'],
                'replay_action': test_config['test_action']['ros2bag'],
                'pred_folder': test_config['pred_folder'],
                'gt_folder': test_config['gt_folder'],
                'workspace': os.path.join(self.test_project_path, '03_Workspace')
            }

            # 合并scenario_list
            scenario_list = []
            for test_config in self.test_config_dict[feature_group]:
                for scenario_tag in test_config['scenario_tag']:
                    scenario_list.extend(scenario_tag['scenario_id'])

            replay_config['scenario_id'] = sorted(set(scenario_list))
            print(replay_config)
            ReplayController(replay_config).start()

    def data_grinder(self):
        for feature_group in self.test_config_dict.keys():
            for task_folder in self.task_folder_dict[feature_group]:
                if feature_group == 'pilot':
                    DataGrinderPilotOneTask(task_folder).start()

    def start(self):
        self.start_log_server()
        self.replay_and_record()
        self.data_grinder()
        self.log_server.stop()


if __name__ == '__main__':
    test_project_path = '/home/caobingqi/ZONE/Data/TestProject/1J5/Pilot/debug'
    drt = DataReplayTestPilot(test_project_path)
    drt.start()

    scenario_id = [
        '20230602_144755_n000003',
        # '20230627_170934_n000001',
        # '20230703_103858_n000003',
        # '20230703_105701_n000001',
        # # '20230706_160503_n000001',
        # # '20230706_161116_n000001',
        # # '20230706_162037_n000001',
        # '20230602_144755_n000005',
        # '20230614_135643_n000001',
        # '20230614_142204_n000004',
        # '20230627_173157_n000001',
        # # '20230706_165109_n000002',
        # # '20230706_184054_n000001',
    ]
