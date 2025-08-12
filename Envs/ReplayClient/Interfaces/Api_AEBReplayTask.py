
import argparse
import os
import shutil
import sys
import time
import yaml
from pathlib import Path

from Libs import get_project_path
sys.path.append(get_project_path())

from Envs.Master.Tools.DataReplayTest import DataReplayTest
from Utils.Libs import project_path


class AEBReplayTask:

    def __init__(self, replay_data_path):
        self.replay_config = None
        self.replay_data_path = Path(replay_data_path)
        self.workspace_path = self.replay_data_path / 'ReplayWorkspace'

    def create_replay_config(self):
        for rosbag_dir in self.replay_data_path.rglob('rosbag'):
            replay_config = {
                'install': str(rosbag_dir.parent / 'install'),
                'params': str(rosbag_dir.parent / 'params'),
                'rosbag': {},
            }
            scenario_count = 0
            for file in rosbag_dir.glob('*'):
                if file.is_dir() and (file / 'stats.txt').exists():
                    with open(file / 'stats.txt', 'r') as f:
                        stats = f.read()
                        if 'transfer_OK' in stats and 'replay_OK' not in stats:
                            rosbag_path = (file / 'ROSBAG' / 'COMBINE')
                            if rosbag_path.exists():
                                replay_config['rosbag'][file.name] = str(rosbag_path.absolute())
                                scenario_count += 1
                                if scenario_count > 20:
                                    break

            if scenario_count:
                self.replay_config = replay_config
                break

        if self.replay_config is None:
            return None

        test_config_path = Path(project_path) / 'Envs' / 'Master' / 'Tools' / 'TestConfig.yaml'
        with open(test_config_path, 'r') as f:
            test_config = yaml.load(f, Loader=yaml.FullLoader)

        test_config['test_date'] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        test_config['pred_folder'] = str(self.workspace_path / '01_Prediction')
        test_config['gt_folder'] = str(self.workspace_path / '02_GroundTruth')
        test_config['scenario_tag'][0]['scenario_id'] = self.replay_config['rosbag']
        print(f'测试场景包含{list(self.replay_config["rosbag"].keys())}')

        return test_config

    def create_workspace(self):
        test_config = self.create_replay_config()
        if test_config is None:
            print('没有找到可用的test_config')
            return None

        if self.workspace_path.exists():
            shutil.rmtree(self.workspace_path)

        os.makedirs(self.workspace_path / '01_Prediction', exist_ok=True)
        os.makedirs(self.workspace_path / '02_GroundTruth', exist_ok=True)
        os.makedirs(self.workspace_path / '03_Workspace', exist_ok=True)
        os.makedirs(self.workspace_path / '04_TestData' / '1-Obstacles', exist_ok=True)
        shutil.copytree(self.replay_config['install'], self.workspace_path / '03_Workspace' / 'install')
        test_config_path = self.workspace_path / '04_TestData' / '1-Obstacles' / 'TestConfig.yaml'
        with open(test_config_path, 'w') as f:
            yaml.dump(test_config, f, sort_keys=False)

        return True

    def monitor_replay_status(self):
        for scenario_id,  rosbag_path in self.replay_config['rosbag'].items():
            pred_raw_folder = self.workspace_path / '01_Prediction' / scenario_id

    def run_aeb_replay(self):
        if not self.create_workspace():
            print('未创建工作空间')
            return False

        ddd = DataReplayTest(self.workspace_path)
        ddd.start()


def main():
    parser = argparse.ArgumentParser(description="start aeb replay task")
    parser.add_argument("-a", "--action", type=str, required=True, help="replay action")
    parser.add_argument("-f", "--workspace", type=str, required=True, help="replay_data_path")

    args = parser.parse_args()
    action = args.action
    replay_data_path = args.replay_data_path

    aeb_replay_task = AEBReplayTask(replay_data_path)
    if action == 'create':
        if aeb_replay_task.create_workspace():
            print(aeb_replay_task.workspace_path)

    elif action == 'run':
        aeb_replay_task.run_aeb_replay()