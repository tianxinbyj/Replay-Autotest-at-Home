"""
@Author: BU YUJUN
@Date: 2025/6/24 上午10:17
"""
import json
import os
import shutil
import time
from datetime import datetime

import pandas as pd
import yaml

from Envs.Master.Modules.DataTransformer import DataTransformer, DataLoggerAnalysis
from Utils.Libs import ros_docker_path, kill_tmux_session_if_exists, create_folder

vehicle_id = {
    '005': 'Q3402',
    '006': 'Q3401',
}


class AEBDataProcessor:

    def __init__(self, folder):
        self.data_group = {}
        self.folder = folder

        '''
        # data group的架构
        20250614-005-AEB
        -- ZoneRos
        ---- rosbag2_2025_06_14-14_45_27
        ---- rosbag2_2025_06_14-15_19_47
        -- Qcraft
        ---- 20250614_105002_Q3402
        ---- 20250614_113118_Q3402
        ---- 20250614_122620_Q3402
        ---- 20250614_125956_Q3402
        ---- 20250614_133921_Q3402
        ---- 20250614_144736_Q3402
        '''

        for f in os.listdir(folder):
            if os.path.exists(os.path.join(folder, f, 'rosbag')):
                self.data_group[f] = {
                    'ZoneRos': os.path.join(folder, f),
                    'Qcraft': []
                }

        for k in self.data_group.keys():
            date = k.split('-')[0]
            zone_v = k.split('-')[1]
            qcraft_v = vehicle_id[zone_v]
            for f in os.listdir(folder):
                if (date in f and qcraft_v in f
                        and os.path.exists(os.path.join(folder, f, 'camera_jpg_img.stf'))):
                    self.data_group[k]['Qcraft'].append(os.path.join(folder, f))
                self.data_group[k]['Qcraft'].sort()

    def reindex(self, install_path, bag_folder):
        tmux_session = 'reindex_ses'
        tmux_window = 'reindex_win'
        os.system(f'tmux new-session -s {tmux_session} -n {tmux_window} -d')
        time.sleep(0.1)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "bash {ros_docker_path}" C-m')
        time.sleep(3)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} '
                  f'"source {install_path}/setup.bash" C-m')
        time.sleep(1)
        os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "ros2 bag reindex {bag_folder}" C-m')

        while not os.path.exists(os.path.join(bag_folder, 'metadata.yaml')):
            time.sleep(1)
        kill_tmux_session_if_exists(tmux_session)

    def run(self):
        for k, v in self.data_group.items():
            data_logger = DataLoggerAnalysis(v['ZoneRos'])
            install, ros2bag_info_path = data_logger.load_data()
            data_transformer = DataTransformer(install)

            with open(ros2bag_info_path, 'r', encoding='utf-8') as file:
                ros2bag_info = json.load(file)

            for qcraft_package_path in v['Qcraft']:
                print(f'===========正在解析 {qcraft_package_path} ===========')
                qcraft_package_path = qcraft_package_path[len(data_transformer.q_docker_base_path):]
                h265_config_path = data_transformer.qStf_to_h265(qcraft_package_path)
                # h265_config_path = '/home/vcar/ZONE/temp/h265_config.yaml'
                with open(h265_config_path, 'r', encoding='utf-8') as file:
                    h265_config = yaml.safe_load(file)

                q_start_time = h265_config['start_time']
                q_end_time = h265_config['end_time']
                print(f'{os.path.basename(qcraft_package_path)}的时间区间为 {q_start_time} - {q_end_time} seconds, {q_end_time - q_start_time} seconds')

                for i, ros2bag in enumerate(ros2bag_info):
                    z_start_time = ros2bag['start_time']
                    z_end_time = ros2bag['end_time']
                    print(f'第{i + 1}个rosbag {os.path.basename(ros2bag["path"])} 时间区间为 {z_start_time} - {z_end_time} seconds, {z_end_time - z_start_time} seconds')
                    t0 = max(q_start_time, z_start_time)
                    t1 = min(q_end_time, z_end_time)
                    if t1 - t0 < 30:
                        print(f'==========> 与Q重叠没有区间段')
                        continue

                    print(f'==========> 与Q重叠区间段为{t0} - {t1} seconds, {t1 - t0} seconds')
                    d0 = datetime.fromtimestamp(int(t0)).strftime("%Y_%m_%d-%H_%M_%S")
                    d1 = datetime.fromtimestamp(int(t1)).strftime("%Y_%m_%d-%H_%M_%S")

                    ros2bag_dir = os.path.join(self.folder, 'COMBINE_BAG', k, f'{d0}={d1}')
                    os.makedirs(ros2bag_dir, exist_ok=True)
                    corr_install = os.path.join(self.folder, 'COMBINE_BAG', k, 'install')
                    if not os.path.exists(corr_install):
                        shutil.move(install, corr_install)

                    # H265转成rosbag
                    single_h265_config = {
                        'h265': {},
                        'start_time': t0,
                        'end_time': t1,
                    }
                    for topic in h265_config['h265']:
                        h265_list = h265_config['h265'][topic]['H265_path']
                        timestamp = pd.read_csv(h265_config['h265'][topic]['timestamp_path'], index_col=False)
                        timestamp = timestamp[(timestamp['time_stamp'] < t1) & (timestamp['time_stamp'] > t0)]

                        timestamp_path = os.path.join(ros2bag_dir, f"{topic.replace('/', '')}.csv")
                        timestamp.to_csv(timestamp_path, index=False)
                        h265_list = h265_list[timestamp.index.min() : timestamp.index.max()+1]
                        single_h265_config['h265'][topic] = {
                            'timestamp_path': timestamp_path,
                            'H265_path': h265_list
                        }

                    single_h265_config_path = os.path.join(ros2bag_dir, 'h265_config.yaml')
                    with open(single_h265_config_path, 'w', encoding='utf-8') as file:
                        yaml.dump(single_h265_config, file)

                    # 生成q_ros2bag
                    q_ros2bag_path = data_transformer.h265_to_db3(single_h265_config_path, ros2bag_dir)
                    # 生成z_ros2bag
                    z_ros2bag_zip_path = ros2bag['path']
                    z_ros2bag_path = os.path.join(ros2bag_dir, os.path.basename(z_ros2bag_zip_path).split('.tar')[0])
                    create_folder(z_ros2bag_path, update=True)
                    cmd = f'tar -xzvf {z_ros2bag_zip_path} -C {z_ros2bag_path}'
                    p = os.popen(cmd)
                    p.read()
                    self.reindex(corr_install, z_ros2bag_path)
                    # 合并q和z
                    combined_ros2bag_folder = os.path.join(ros2bag_dir, 'ROSBAG', 'COMBINE')
                    create_folder(combined_ros2bag_folder, update=True)
                    data_transformer.combine_db3(
                        [q_ros2bag_path, z_ros2bag_path],
                        combined_ros2bag_folder,
                        t0, t1
                    )


if __name__ == '__main__':
    folder = '/media/data/Q_DATA/debug_data'
    AEBDataProcessor(folder).run()
