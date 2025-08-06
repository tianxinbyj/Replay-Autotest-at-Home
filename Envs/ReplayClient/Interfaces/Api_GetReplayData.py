"""
@Author: BU YUJUN
@Date: 2025/8/1 下午2:11
"""
import argparse
import sys

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.Master.Tools.AEBDataProcessor import AEBDataManager


def get_replay_list():
    print('id | path | prepared_size | prepared_num | transferred_size | transferred_num | replayed_size | replayed_num')
    aeb_data_manager = AEBDataManager()
    for k, v in aeb_data_manager.aeb_data_package_list.items():
        print(k, v['path'],
              v['prepared_size'], v['prepared_num'],
              v['transferred_size'], v['transferred_num'],
              v['replayed_size'], v['replayed_num'])

def transfer_data(host, username, password, data_label, remote_base_dir):
    aeb_data_manager = AEBDataManager()
    aeb_data_manager.transfer_data(host, username, password, data_label, remote_base_dir)


def main():
    parser = argparse.ArgumentParser(description="get replay data")
    parser.add_argument("-a", "--action", type=str, default='ls', help="what do you want?")
    parser.add_argument("-l", "--data_label", type=str, default='abc', help="data label")
    parser.add_argument("-r", "--remote_base_dir", type=str, default='abc', help="remote base dir")
    parser.add_argument("-i", "--host", type=str, default='abc', help="host")
    parser.add_argument("-u", "--username", type=str, default='abc', help="username")
    parser.add_argument("-p", "--password", type=str, default='abc', help="password")

    args = parser.parse_args()
    action = args.action

    if action == 'ls':
        get_replay_list()

    elif action == 'dd':
        data_label = args.data_label
        remote_base_dir = args.remote_base_dir
        host = args.host
        username = args.username
        password = args.password
        transfer_data(host, username, password, data_label, remote_base_dir)


if __name__ == '__main__':
    get_replay_list()