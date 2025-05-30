"""  
@Author: BU YUJUN
@Date: 2025/5/26 16:21  
"""

import os
import sys
import time
import yaml

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from Libs import get_project_path

sys.path.append(get_project_path())
from Utils.VideoProcess import convert_video_h265, gen_h265_timestamp, normalize_h265_startcodes
from Utils.Libs import docker_path, variables, kill_tmux_session_if_exists, project_path, get_folder_size


class EthernetReplayDataGenerator:

    def __init__(self, config_path, install_path, ros2bag_path):
        with open(config_path, 'r', encoding='utf-8') as file:
            self.video_config = yaml.safe_load(file)

        self.h265_config_path = os.path.join(os.path.dirname(config_path), 'H265.yaml')
        self.h265_config = {
            topic: {
                'timestamp_path': '0',
                'H265_path': '0',
            } for topic in self.video_config.keys()
        }

        self.tmux_session = variables['tmux_node']['h265_gen'][0]
        self.tmux_window = variables['tmux_node']['h265_gen'][1]
        self.install_path = install_path
        self.ros2bag_path = ros2bag_path

    def transfer_h265(self):
        for topic, info in self.video_config.items():
            fps = info['fps']
            video_path = info['path']
            h265_path = os.path.join(os.path.dirname(video_path), f"{topic.replace('/', '')}.h265")
            timestamp_path = os.path.join(os.path.dirname(video_path), f"{topic.replace('/', '')}.csv")
            normalized_h265_path = os.path.join(os.path.dirname(video_path), f"{topic.replace('/', '')}_norm.h265")
            self.h265_config[topic]['timestamp_path'] = timestamp_path
            self.h265_config[topic]['H265_path'] = normalized_h265_path

            convert_video_h265(video_path, fps, h265_path)
            gen_h265_timestamp(h265_path, timestamp_path)
            normalize_h265_startcodes(h265_path, normalized_h265_path)
            os.remove(h265_path)

        with open(self.h265_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(self.h265_config, file, sort_keys=False, indent=2, allow_unicode=True)

    def gen_h265_db3(self):
        kill_tmux_session_if_exists(self.tmux_session)
        os.system(f'tmux new-session -s {self.tmux_session} -n {self.tmux_window} -d')
        time.sleep(0.1)

        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "bash {docker_path}" C-m')
        os.system('sleep 3')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"source {self.install_path}/setup.bash" C-m')

        h265_rosnode_path = os.path.join(project_path, 'Envs', 'Master', 'Modules', 'RosNode')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"cd {h265_rosnode_path}" C-m')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"python3 H265ToRosbagConverter.py -f {self.h265_config_path} -r {self.ros2bag_path}" C-m')

    def stop(self):
        count = 3
        while count:
            time.sleep(3)
            folder_size_1 = get_folder_size(self.ros2bag_path)
            time.sleep(3)
            folder_size_2 = get_folder_size(self.ros2bag_path)
            print(f'ros2bag size == {round(folder_size_2 / 1024 / 1024, 2)} MB')
            if folder_size_2 == folder_size_1:
                count -= 1
        print(f'ros2bag gen stops')
        kill_tmux_session_if_exists(self.tmux_session)

        with open(self.h265_config_path, 'r', encoding='utf-8') as file:
            h265_config = yaml.safe_load(file)

        for topic in h265_config:
            os.remove(h265_config[topic]['H265_path'])
            os.remove(h265_config[topic]['timestamp_path'])

        os.remove(self.h265_config_path)

    # def combine


if __name__ == '__main__':
    t0 = time.time()
    config_path = '/home/hp/ZONE/temp/Config.yaml'
    install_path = '/home/hp/artifacts/ZPD_EP39/RC11/install'
    ros2bag_path = '/home/hp/ZONE/temp/debug'
    ee = EthernetReplayDataGenerator(config_path, install_path, ros2bag_path)
    ee.transfer_h265()
    ee.gen_h265_db3()
    ee.stop()
    print(time.time() - t0)