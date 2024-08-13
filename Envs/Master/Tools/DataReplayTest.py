"""
@Author: BU YUJUN
@Date: 2024/8/9 上午12:00
"""
import glob
import os

from Envs.Master.Modules.ReplayController import ReplayController
from Envs.Master.Modules.DataGrinderPilot import DataGrinderPilotOneTask


class DataReplayTestPilot:

    def __init__(self, test_project_path):
        workspace_folder = os.path.join(test_project_path, '03_Workspace')
        if not os.path.exists(os.path.join(workspace_folder, 'install')):
            print('未找到install文件！')
            return

        # 寻找所有TestConfig.yaml
        test_config_yaml_list = glob.glob(os.path.join(test_project_path, '04_TestData', '*', 'TestConfig.yaml'))
        if not test_config_yaml_list:
            print('未找到TestConfig.yaml文件！')
            return

        prediction_folder = os.path.join(test_project_path, '01_Prediction')


if __name__ == '__main__':
    test_project_path = '/home/zhangliwei01/ZONE/TestProject/2J5/123'
    drt = DataGrinderPilotOneTask(test_project_path)