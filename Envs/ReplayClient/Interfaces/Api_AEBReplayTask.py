
import argparse
import getpass
import os
import shutil
import sys
import time
from pathlib import Path

import pandas as pd
import yaml

from Libs import get_project_path

sys.path.append(get_project_path())

from Utils.Libs import project_path, kill_tmux_session_if_exists, bench_config, bench_id, get_folder_size, check_tmux_session_exists
from Envs.ReplayClient.Modules.ScpConfig2ECU import ChangeConfigVer

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
        self.upload_scenario_list = self.replay_data_path / 'uploaded_scenario_list.txt'
        self.replay_session = f'{bench_id}_ReplayTestSes'
        self.replay_window = f'{bench_id}_ReplayTestWin'
        self.upload_session = f'{bench_id}_UploadTestSes'
        self.upload_window = f'{bench_id}_UploadTestWin'

    def create_replay_config(self, scenario_num):
        if self.workspace_path.exists():
            shutil.rmtree(self.workspace_path)

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
            for file in sorted(rosbag_dir.glob('*')):
                if file.is_dir() and (file / 'stats.txt').exists():
                    with open(file / 'stats.txt', 'r') as f:
                        stats = f.read()
                        if 'transfer_OK' in stats:
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
            print('没有找到可用的testconfig')
            print(0)
            return None

        if self.workspace_path.exists():
            shutil.rmtree(self.workspace_path)

        if self.upload_scenario_list.exists():
            os.remove(self.upload_scenario_list)

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

        print(f'{str(self.workspace_path)}')

    def start_replay(self):
        kill_tmux_session_if_exists(self.replay_session)

        os.system(f"tmux new-session -s {self.replay_session} -n {self.replay_window} -d")
        api_folder = os.path.join(project_path, 'Envs', 'Master', 'Interfaces')
        os.system(f"tmux send-keys -t {self.replay_session}:{self.replay_window} 'cd {api_folder}' C-m")
        time.sleep(1)
        sys_interpreter = bench_config['Master']['sys_interpreter']
        os.system(f"tmux send-keys -t {self.replay_session}:{self.replay_window} '{sys_interpreter} Api_StartReplayTest.py -t {self.workspace_path}' C-m")

    def get_replay_process(self):
        print(get_folder_size(str(self.workspace_path)))

    def stop_replay(self):
        for _ in range(3):
            if check_tmux_session_exists(self.replay_session):
                kill_tmux_session_if_exists(self.replay_session)
            else:
                break
            time.sleep(1)

        output_statistic_path = self.workspace_path / '01_Prediction' / 'topic_output_statistics.csv'
        output_statistic = pd.read_csv(output_statistic_path, index_col=0)
        valid_scenario_list = output_statistic[output_statistic['isValid'] == 1].index.tolist()

        if len(valid_scenario_list):
            for f in self.workspace_path.glob('*play'):
                for ff in f.glob('*'):
                    for upload_dir in ff.glob('*'):
                        test_config_path = self.workspace_path / '04_TestData' / '1-Obstacles' / 'TestConfig.yaml'
                        with open(test_config_path) as f:
                            car_id = yaml.load(f, Loader=yaml.FullLoader)['s3_level'][0]

                        (upload_dir / 'rosbag').mkdir(parents=True, exist_ok=True)
                        for scenario_id in valid_scenario_list:
                            shutil.move(
                                self.workspace_path / '01_Prediction' / scenario_id, 
                                upload_dir / 'rosbag' / scenario_id)

                        new_output_statistic_path = upload_dir / 'rosbag' / f'ReplayResult@{car_id}@{time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())}.csv'
                        output_statistic_path.rename(new_output_statistic_path)

                        (upload_dir / 'install').mkdir(parents=True, exist_ok=True)
                        shutil.make_archive(
                            base_name=str(upload_dir / 'install' / 'install'),  # 移除.tar.gz后缀
                            format='gztar',  # 关键：指定为tar.gz格式
                            root_dir=str(self.workspace_path / '03_Workspace'),  # 压缩的根目录
                            base_dir='install'  # 相对于根目录的源文件夹
                        )

                        (upload_dir / 'params').mkdir(parents=True, exist_ok=True)
                        shutil.make_archive(
                            base_name=str(upload_dir / 'params' / 'params'),  # 移除.tar.gz后缀
                            format='gztar',  # 关键：指定为tar.gz格式
                            root_dir=str(self.workspace_path / '03_Workspace'),  # 压缩的根目录
                            base_dir='params'  # 相对于根目录的源文件夹
                        )

                        shutil.rmtree(self.workspace_path / '01_Prediction')
                        shutil.rmtree(self.workspace_path / '02_GroundTruth')
                        shutil.rmtree(self.workspace_path / '03_Workspace')
                        shutil.rmtree(self.workspace_path / '04_TestData')
                        print(' '.join(valid_scenario_list))
                        return str(upload_dir.absolute())

        print(0)
        return None

    def start_upload(self, endpoint_url, aws_access_key_id, aws_secret_access_key, bucket_name, s3_path):
        kill_tmux_session_if_exists(self.upload_session)

        matching_files = []
        for file in self.workspace_path.rglob('*'):
            if file.is_file() and 'ReplayResult@' in file.name:
                matching_files.append(str(file.absolute()))

        if not matching_files:
            print(f'{self.workspace_path} 没有回灌结果可上传, 上传失败')
            return None

        output_statistic_path = matching_files[0]
        car_id = Path(output_statistic_path).name.split('@')[1]
        output_statistic = pd.read_csv(output_statistic_path, index_col=0)
        valid_scenario_list = output_statistic[output_statistic['isValid'] == 1].index.tolist()
        with open(self.upload_scenario_list, 'w', encoding='utf-8') as ff:
            for scenario_id in valid_scenario_list:
                for f in self.replay_data_path.rglob('*'):
                    if (f.is_dir() and 'ReplayWorkspace' not in str(f.absolute())
                            and car_id in str(f.absolute()) and scenario_id in str(f.absolute())):
                        print(f'删除原始数据 {str(f.absolute())}')
                        shutil.rmtree(f)
                        ff.write(f"{str(f.relative_to(self.replay_data_path))}\n")
                        break

        os.system(f"tmux new-session -s {self.upload_session} -n {self.upload_window} -d")
        api_folder = os.path.join(project_path, 'Envs', 'Master', 'Interfaces')
        os.system(f"tmux send-keys -t {self.upload_session}:{self.upload_window} 'cd {api_folder}' C-m")
        time.sleep(1)
        sys_interpreter = bench_config['Master']['sys_interpreter']
        os.system(f"tmux send-keys -t {self.upload_session}:{self.upload_window} '{sys_interpreter} "
                  f"Api_CloudS3.py -a upload -u {endpoint_url} -k {aws_access_key_id} -s {aws_secret_access_key} "
                  f"-b {bucket_name} -p {s3_path} -f {self.workspace_path}' C-m")

    def stop_upload(self):
        while self.workspace_path.exists():
            time.sleep(10)
        print(f'{self.workspace_path}上传完成, 本地文件处理完成')

        for _ in range(3):
            if check_tmux_session_exists(self.upload_session):
                kill_tmux_session_if_exists(self.upload_session)
            else:
                break
            time.sleep(1)

        print(self.upload_scenario_list)

    def prepare_replay(self, config_path=None):
        """
        准备回灌测试前，更换相机参数，
        """
        if not config_path:
            config_path = self.workspace_path / '03_Workspace' / 'params'

        elif Path(config_path).exists():
            config_path = Path(config_path)

        elif not Path(config_path).exists():
            raise FileNotFoundError(f"指定参数文件的文件夹不存在：{config_path}")

        change_cam_config = ChangeConfigVer(config_path)
        change_c_flag = change_cam_config.change_config_in_one()
        # if change_c_flag:
        replay_flag = change_cam_config.replay_model_start_success()

        if change_c_flag and replay_flag:
            print("更换相机参数成功，重启电源，并开启了回灌模式成功") # 文字固定，尽量不要改
            return True
        elif change_c_flag and not replay_flag:
            print("更换相机参数成功，重启电源，开启回灌模式失败")  # 文字固定，尽量不要改
            return False
        elif not change_c_flag and replay_flag:
            print("更换相机参数失败，重启电源，开启回灌模式失败")  # 文字固定，尽量不要改
            return False
        elif not change_c_flag and not replay_flag:
            print("更换相机参数失败，重启电源，开启回灌模式失败")  # 文字固定，尽量不要改
            return False
        return None


def main():
    parser = argparse.ArgumentParser(description="start aeb replay task")
    parser.add_argument("-a", "--action", type=str, required=True, help="replay action")
    parser.add_argument("-n", "--scenario_num", type=int, default=20, help="scenario num")
    parser.add_argument("-u", "--endpoint_url", type=str, default='http://10.192.53.221:8080', help="endpoint url")
    parser.add_argument("-k", "--aws_access_key_id", type=str, default='44JAMVA71J5L90D9DK77', help="aws access key id")
    parser.add_argument("-s", "--aws_secret_access_key", type=str, default='h1cY4WzpNxmQCpsXlXFpO4nWjNp3pbH0ZuBsuGmu', help="aws secret access key")
    parser.add_argument("-b", "--bucket_name", type=str, default='aeb', help="bucket name")
    parser.add_argument("-p", "--s3_path", type=str, default='ALL/', help="s3 path")

    args = parser.parse_args()
    action = args.action
    scenario_num = args.scenario_num
    endpoint_url = args.endpoint_url
    aws_access_key_id = args.aws_access_key_id
    aws_secret_access_key = args.aws_secret_access_key
    bucket_name = args.bucket_name
    s3_path = args.s3_path

    aeb_replay_task = AEBReplayTask()
    if action == 'create':
        if aeb_replay_task.create_workspace(scenario_num):
            print(aeb_replay_task.workspace_path)

    elif action == 'prepare':
        # 做好更换参数并校验，重新下上电，开启回灌模式的准备
        print("before pass prepare")
        print("after pass prepare")
        aeb_replay_task.prepare_replay()
        time.sleep(1)

    elif action == 'start':
        aeb_replay_task.start_replay()

    elif action == 'process':
        aeb_replay_task.get_replay_process()

    elif action == 'stop':
        aeb_replay_task.stop_replay()

    elif action == 'upload':
        aeb_replay_task.start_upload(
            endpoint_url, aws_access_key_id, aws_secret_access_key, bucket_name, s3_path
        )

    elif action == 'clear':
        aeb_replay_task.stop_upload()


if __name__ == '__main__':
    main()

    # endpoint_url='http://10.192.53.221:8080'  # 你的S3 endpoint
    # aws_access_key_id='44JAMVA71J5L90D9DK77'  # 替换为你的Access Key
    # aws_secret_access_key='h1cY4WzpNxmQCpsXlXFpO4nWjNp3pbH0ZuBsuGmu'  # 替换为你的Secret Key
    # bucket_name = 'aeb'
    # s3_path = 'ALL/'
    # delete_flag = 0
    # aeb_replay_task = AEBReplayTask()
    # aeb_replay_task.start_upload(
    #     endpoint_url, aws_access_key_id, aws_secret_access_key, bucket_name, s3_path
    #     )
