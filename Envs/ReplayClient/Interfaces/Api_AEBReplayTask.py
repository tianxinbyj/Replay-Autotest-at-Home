
import argparse
import os
import shutil
import sys
import time
import yaml
import getpass
from pathlib import Path

from Libs import get_project_path
sys.path.append(get_project_path())

from Envs.Master.Tools.DataReplayTest import DataReplayTest
from Utils.Libs import project_path, kill_tmux_session_if_exists, bench_config, bench_id, get_folder_size, check_tmux_session_exists


class AEBReplayTask:

    def __init__(self, replay_data_path=None):
        self.replay_config = None
        if replay_data_path:
            self.replay_data_path = Path(replay_data_path)
        else:
            self.replay_data_path = Path(f'/home/{getpass.getuser()}') / 'ZONE' / 'AEBReplayRoom'
        if not self.replay_data_path.exists():
            os.makedirs(self.replay_data_path)

        self.workspace_path = self.replay_data_path / 'ReplayWorkspace'
        self.tmux_session = f'{bench_id}_ReplayTestSes'
        self.tmux_window = f'{bench_id}_ReplayTestWin'

    def create_replay_config(self, scenario_num):
        for rosbag_dir in self.replay_data_path.rglob('rosbag'):
            if not ((rosbag_dir.parent / 'install').exists() and (rosbag_dir.parent / 'params').exists()):
                continue

            replay_config = {
                'install': str(rosbag_dir.parent / 'install'),
                'params': str(rosbag_dir.parent / 'params'),
                'rosbag': {},
                's3_level': [
                    str(rosbag_dir.parent.parent.parent.name),
                    str(rosbag_dir.parent.parent.name),
                    str(rosbag_dir.parent.name),
                ]
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
                                if scenario_count >= scenario_num:
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
        test_config['s3_level'] = self.replay_config['s3_level']
        print(f'测试场景包含{list(self.replay_config["rosbag"].keys())}')

        return test_config

    def create_workspace(self, scenario_num=20):
        test_config = self.create_replay_config(scenario_num)
        if test_config is None:
            print('没有找到可用的test_config')
            return None

        if self.workspace_path.exists():
            shutil.rmtree(self.workspace_path)

        os.makedirs(self.workspace_path / '01_Prediction', exist_ok=True)
        os.makedirs(self.workspace_path / '02_GroundTruth', exist_ok=True)
        os.makedirs(self.workspace_path / '03_Workspace', exist_ok=True)
        os.makedirs(self.workspace_path / '04_TestData' / '1-Obstacles', exist_ok=True)
        os.makedirs(self.workspace_path / (test_config['s3_level'][0]+'-Replay') / test_config['s3_level'][1] / test_config['s3_level'][2], exist_ok=True)
        shutil.copytree(self.replay_config['install'], self.workspace_path / '03_Workspace' / 'install')
        shutil.copytree(self.replay_config['params'], self.workspace_path / '03_Workspace' / 'params')
        test_config_path = self.workspace_path / '04_TestData' / '1-Obstacles' / 'TestConfig.yaml'
        with open(test_config_path, 'w') as f:
            yaml.dump(test_config, f, sort_keys=False)

        return True

    def monitor_replay_status(self):
        for scenario_id,  rosbag_path in self.replay_config['rosbag'].items():
            pred_raw_folder = self.workspace_path / '01_Prediction' / scenario_id

    def start_replay(self):
        kill_tmux_session_if_exists(self.tmux_session)

        os.system(f"tmux new-session -s {self.tmux_session} -n {self.tmux_window} -d")
        api_folder = os.path.join(project_path, 'Envs', 'Master', 'Interfaces')
        os.system(f"tmux send-keys -t {self.tmux_session}:{self.tmux_window} 'cd {api_folder}' C-m")
        time.sleep(1)
        sys_interpreter = bench_config['Master']['sys_interpreter']
        os.system(f"tmux send-keys -t {self.tmux_session}:{self.tmux_window} '{sys_interpreter} Api_StartReplayTest.py -t {self.workspace_path}' C-m")

    def get_replay_process(self):
        print(get_folder_size(str(self.workspace_path)))

    def stop_replay(self):
        for _ in range(3):
            if check_tmux_session_exists(self.tmux_session):
                kill_tmux_session_if_exists(self.tmux_session)
            else:
                break
            time.sleep(1)

        for f in self.workspace_path.glob('*play'):
            for ff in f.glob('*'):
                for upload_dir in ff.glob('*'):
                    shutil.move(self.workspace_path / '01_Prediction', upload_dir / 'rosbag')
                    shutil.move(self.workspace_path / '03_Workspace' / 'install', upload_dir / 'install')
                    shutil.move(self.workspace_path / '03_Workspace' / 'params', upload_dir / 'params')
                    shutil.rmtree(self.workspace_path / '02_GroundTruth')
                    shutil.rmtree(self.workspace_path / '03_Workspace')
                    shutil.rmtree(self.workspace_path / '04_TestData')
                    print(str(upload_dir.absolute()))
                    return str(upload_dir.absolute())

        print(0)
        return 0

    # def upload_directory(self):
    #             self.s3_client = boto3.client(
    #         's3',
    #         endpoint_url=endpoint_url,  # 你的S3 endpoint
    #         aws_access_key_id=aws_access_key_id,  # 替换为你的Access Key
    #         aws_secret_access_key=aws_secret_access_key,  # 替换为你的Secret Key
    #         config=Config(signature_version='s3v4'),
    #         region_name='cn-north-1'  # 区域名称, 可根据实际情况修改
    #     )


def main():
    parser = argparse.ArgumentParser(description="start aeb replay task")
    parser.add_argument("-a", "--action", type=str, required=True, help="replay action")
    parser.add_argument("-n", "--scenario_num", type=int, default=20, help="scenario num")

    args = parser.parse_args()
    action = args.action
    scenario_num = args.scenario_num

    aeb_replay_task = AEBReplayTask()
    if action == 'create':
        if aeb_replay_task.create_workspace(scenario_num):
            print(aeb_replay_task.workspace_path)

    elif action == 'start':
        aeb_replay_task.start_replay()

    elif action == 'process':
        aeb_replay_task.get_replay_process()

    elif action == 'stop':
        aeb_replay_task.stop_replay()


if __name__ == '__main__':
    main()
