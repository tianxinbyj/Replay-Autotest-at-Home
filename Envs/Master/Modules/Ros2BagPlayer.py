import os
import sys
import time

import yaml

from Libs import get_project_path

sys.path.append(get_project_path())
from Utils.Libs import kill_tmux_session_if_exists, check_tmux_session_exists, bench_config
from Utils.Libs import variables, docker_path


class Ros2BagPlayer:
    """
    ros2 bag 的播放器
    """
    def __init__(self, workspace):
        self.install = os.path.join(workspace, 'install')
        self.play_tmux_session = variables['tmux_node']['replay'][0]
        self.play_tmux_window = variables['tmux_node']['replay'][1]
        self.sil_tmux_session = variables['tmux_node']['sil_pkg'][0]
        self.sil_tmux_window = variables['tmux_node']['sil_pkg'][1]

    def start_play(self, ros2bag_path, play_topic_list=None, play_remap_flag=True):
        """
        开始播放ros2 bag
        topic_list: 需要播放的ros2bag
        play_remap_flag： flag 确定是否需要将播放的topic remap改名，
        """

        if not play_topic_list:
            play_topic_list = ['/Camera/Rear/H265', '/Camera/FrontWide/H265',
                          '/Camera/SorroundRear/H265' ,'/Camera/SorroundFront/H265',
                          '/Camera/SorroundRight/H265' ,'/Camera/SorroundLeft/H265',
                          '/FM/FctReq' ,'/SA/INSPVA','/SA/IMU','/PK/DR/Result',
                          '/VA/VehicleStatusIpd' ,'/VA/VehicleMotionIpd', '/PI/EG/EgoMotionInfo']

        # 打开sil转发节点
        self.run_sil_pkg()

        kill_tmux_session_if_exists(self.play_tmux_session)
        os.system(f'tmux new-session -s {self.play_tmux_session} -n {self.play_tmux_window} -d')
        time.sleep(0.1)

        os.system(f'tmux send-keys -t {self.play_tmux_session}:{self.play_tmux_window} "bash {docker_path}" C-m')
        os.system('sleep 3')
        os.system(f'tmux send-keys -t {self.play_tmux_session}:{self.play_tmux_window} '
                  f'"source {self.install}/setup.bash" C-m')

        play_topic_str = ' '.join(play_topic_list)
        remap_topic_str = ' '.join([topic + ':=/o' + topic for topic in play_topic_list])

        if play_remap_flag:
            ros_play_cmd = f'ros2 bag play {ros2bag_path} --topic {play_topic_str} --remap {remap_topic_str}'

        elif not play_remap_flag:
            ros_play_cmd = f'ros2 bag play {ros2bag_path} --topic {play_topic_str}'

        else:
            raise Exception('play_remap_flag must be True or False')

        os.system(f'tmux send-keys -t {self.play_tmux_session}:{self.play_tmux_window} '
                  f'"{ros_play_cmd}" C-m')

    def get_ros2bag_metadata_yaml(self, ros2bag_path):
        """
        从对应的metadata.yaml文件中获取到对应ros2bag的信息
        """
        if not os.path.exists(ros2bag_path):
            raise FileNotFoundError(ros2bag_path)

        if os.path.isdir(ros2bag_path) and os.path.exists(os.path.join(ros2bag_path, 'metadata.yaml')):
            metadata_yaml_path = os.path.join(ros2bag_path, 'metadata.yaml')
        else:
            metadata_yaml_path = os.path.join(os.path.dirname(ros2bag_path), 'metadata.yaml')

        with open(metadata_yaml_path, 'r') as meta_yf:
            rosbag2_bagfile_information_dict = yaml.load(meta_yf, Loader=yaml.FullLoader)

            topic_in_bag_list = [topics_with_message_count['topic_metadata']['name'] for topics_with_message_count in
                                 rosbag2_bagfile_information_dict['rosbag2_bagfile_information']['topics_with_message_count']
                                 if topics_with_message_count['message_count'] != 0]

            ros2bag_duration_nanosec = rosbag2_bagfile_information_dict['rosbag2_bagfile_information']['duration']['nanoseconds']

        return ros2bag_duration_nanosec / 1e9, topic_in_bag_list

    def run_sil_pkg(self):
        kill_tmux_session_if_exists(self.sil_tmux_session)
        os.system(f'tmux new-session -s {self.sil_tmux_session} -n {self.sil_tmux_window} -d')
        time.sleep(0.1)

        os.system(f'tmux send-keys -t {self.sil_tmux_session}:{self.sil_tmux_window} "bash {docker_path}" C-m')
        os.system('sleep 3')

        sil_pkg_path = bench_config['Master']['sil_pkg_path']
        os.system(f'tmux send-keys -t {self.sil_tmux_session}:{self.sil_tmux_window} "cd {sil_pkg_path}" C-m')
        os.system(f'tmux send-keys -t {self.sil_tmux_session}:{self.sil_tmux_window} "source install/setup.bash" C-m')
        os.system(f'tmux send-keys -t {self.sil_tmux_session}:{self.sil_tmux_window} "ros2 launch src/sil_pkg/launch/rosbag_play_sub.py" C-m')

    def get_play_process(self, start_time, ros2bag_path):
        if start_time > time.time():
            return 0
        elif self.get_ros2bag_metadata_yaml(ros2bag_path)[0] < time.time() - start_time:
            return 1
        else:
            return (time.time() - start_time) / self.get_ros2bag_metadata_yaml(ros2bag_path)[0]

    def stop_play(self):
        if check_tmux_session_exists(self.play_tmux_session):
            os.system(f'tmux send-keys -t {self.play_tmux_session}:{self.play_tmux_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {self.play_tmux_session}:{self.play_tmux_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {self.play_tmux_session}:{self.play_tmux_window} "exit" Enter')
            time.sleep(0.5)

        if check_tmux_session_exists(self.sil_tmux_session):
            os.system(f'tmux send-keys -t {self.sil_tmux_session}:{self.sil_tmux_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {self.sil_tmux_session}:{self.sil_tmux_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {self.sil_tmux_session}:{self.sil_tmux_window} "exit" Enter')
            time.sleep(0.5)

        kill_tmux_session_if_exists(self.play_tmux_session)
        kill_tmux_session_if_exists(self.sil_tmux_session)
        time.sleep(1)


if __name__ == '__main__':
    RBP = Ros2BagPlayer('/home/zhangliwei01/ZONE/horizon/parking_test/workspace/RC8')

    db3_bag = '/home/zhangliwei01/ZONE/20231130_152434_n000001_driving_rosbag/20231130_152434_n000001_driving_rosbag_0.db3'
    duration , topic_in_bag_list = RBP.get_ros2bag_metadata_yaml(db3_bag)
    RBP.start_play(db3_bag, play_topic_list=None)