"""
@Author: BU YUJUN
@Date: 2024/10/16 下午2:02
"""
import argparse
import glob
import os
import shutil
import sys

import pandas as pd

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.Master.Modules.Ros2BagParser import Ros2BagClip, Ros2BagParser


def get_time_offset(bag_path, workspace):
    temp_folder = os.path.join(get_project_path(), 'Temp')
    if os.path.exists(temp_folder):
        shutil.rmtree(temp_folder)
    os.makedirs(temp_folder)

    RBP = Ros2BagParser(workspace)
    RBP.getMsgInfo(bag_path, ['/PI/EG/EgoMotionInfo'], temp_folder, 'xxxxxxxx')

    ego_csv = glob.glob(os.path.join(temp_folder, '*Ego*hz.csv'))[0]
    ego_data = pd.read_csv(ego_csv)

    if os.path.exists(temp_folder):
        shutil.rmtree(temp_folder)
    os.makedirs(temp_folder)

    if len(ego_data):
        return ego_data['local_time'].mean() - ego_data['time_stamp'].mean()
    else:
        return False

def clip_rosbag(workspace, topic_list, time_range_list, src_path, dst_path_list):
    dd = Ros2BagClip(workspace)
    for time_range, dst_path in zip(time_range_list, dst_path_list):
        if os.path.exists(dst_path):
            shutil.rmtree(dst_path)

        dd.cutRosbag(src_path, dst_path, topic_list, [time_range[0], time_range[1]])


def main():
    parser = argparse.ArgumentParser(description="clip rosbag")
    parser.add_argument("-w", "--workspace", type=str, required=True, help="workspace")
    parser.add_argument("-b", "--bag_path", type=str, required=True, help="bag path")
    parser.add_argument("-t", "--topic_list", type=str, nargs='+', required=True, help="topic list")
    parser.add_argument("-r", "--time_range_list", type=float, nargs='+', required=True, help="time range list")
    parser.add_argument("-d", "--dst_path_list", type=str, nargs='+', required=True, help="dst path list")
    parser.add_argument("-o", "--offset", type=float, default=0, help="offset")

    args = parser.parse_args()
    workspace = args.workspace
    bag_path = args.bag_path
    time_range_list = args.time_range_list
    topic_list = args.topic_list
    dst_path_list = args.dst_path_list
    offset = args.offset

    if len(time_range_list) % 2 != 0:
        print("列表中的元素个数必须是偶数")
        return

    time_offset = get_time_offset(bag_path, workspace)
    if time_offset is False:
        print('没有找到时间差')
        return

    time_range_list = [float(t + time_offset + offset) for t in time_range_list]
    time_range_list = [time_range_list[i:i + 2] for i in range(0, len(time_range_list), 2)]
    print(time_range_list)

    if len(time_range_list) != len(dst_path_list):
        print(f'时间区间数量为{time_range_list}, 截取rosbag目标路径数量为{len(dst_path_list)}')
        return

    clip_rosbag(workspace, topic_list, time_range_list, bag_path, dst_path_list)


if __name__ == '__main__':
    main()
    cmd = '/home/zhangliwei01/ZONE/PythonProject/Replay-Autotest-at-Home/venv/bin/python3 Api_ClipRosbag.py -w /home/zhangliwei01/ZONE/TestProject/ES39/p_feature_20241022_123455/03_Workspace -b /home/zhangliwei01/ZONE/1018_2024/rosbag/rosbag2_2024_10_18-15_46_57/rosbag -t /SA/INSPVA /PK/DR/Result /SOA/SDNaviLinkInfo /SOA/SDNaviStsInfo /PI/EG/EgoMotionInfo -r 1729237932.95 1729238232.95 1729239097.319 1729239397.319 -d /home/zhangliwei01/ZONE/replay_rosbag/20241018_154712_n000002 /home/zhangliwei01/ZONE/replay_rosbag/20241018_160637_n000002 -o 0.6'

