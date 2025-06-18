"""  
Created on 2024/7/1.  
@author: Bu Yujun  
"""
import os
import shutil
import sys
import time

from Libs import get_project_path
sys.path.append(get_project_path())

from Utils.Libs import kill_tmux_session_if_exists, check_tmux_session_exists
from Utils.Libs import variables, ros_docker_path


class Ros2BagRecorder:

    def __init__(self, workspace):
        self.install = os.path.join(workspace, 'install')
        self.tmux_session = variables['tmux_node']['ros2bag'][0]
        self.tmux_window = variables['tmux_node']['ros2bag'][1]

    def start_record(self, scenario_id, folder, topic_list, bag_name=None):
        if topic_list and 'all' not in topic_list:
            topic_str = ' '.join(topic_list)
        else:
            topic_str = '-a'

        if not bag_name:
            t0 = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(time.time()))
            bag_name = f'{scenario_id}_{t0}'

        print(f'录制的topic为{topic_str}')
        kill_tmux_session_if_exists(self.tmux_session)

        work_folder = os.path.join(folder, scenario_id)
        if not os.path.exists(work_folder):
            os.makedirs(work_folder)

        # 删除旧的包数据
        for f in os.listdir(work_folder):
            if scenario_id in f:
                bag_related_file = os.path.join(work_folder, f)
                if os.path.isdir(bag_related_file):
                    shutil.rmtree(bag_related_file)
                else:
                    os.remove(bag_related_file)

        bag_folder = os.path.join(work_folder, bag_name)
        if os.path.exists(bag_folder):
            shutil.rmtree(bag_folder)

        parser_folder = os.path.join(work_folder, 'RawData')
        if os.path.exists(parser_folder):
            shutil.rmtree(parser_folder)
        os.makedirs(parser_folder)

        os.system(f'tmux new-session -s {self.tmux_session} -n {self.tmux_window} -d')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "bash {ros_docker_path}" C-m')
        time.sleep(3)
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "cd {work_folder}" C-m')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"source {self.install}/setup.bash" C-m')

        qos_config = os.path.join(get_project_path(),
                                  'Docs', 'Resources', 'qos_config', 'best_effort_driving.yaml')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"ros2 bag record {topic_str} -o {bag_name} --qos-profile-overrides-path {qos_config}" C-m')

        print(f'rosbag: {bag_folder}')
        print(f'rosbag 解析结果: {parser_folder}')
        return bag_folder, parser_folder

    def stop_record(self):
        if check_tmux_session_exists(self.tmux_session):
            os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "exit" Enter')
            time.sleep(0.5)

        kill_tmux_session_if_exists(self.tmux_session)
        time.sleep(1)
