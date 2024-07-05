"""  
Author: Bu Yujun
Date: 6/21/24  
"""
import argparse
import sys

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.ReplayClient.Modules.ReplayManagement import ReplayManagement
from Envs.ReplayClient.Modules.DataSelector import DataSelector


def main():
    parser = argparse.ArgumentParser(description="Start Replay.")
    parser.add_argument("-s", "--scenario_id", type=str, default='s', help="Specify scenario id.")
    args = parser.parse_args()

    if args.scenario_id != 's':
        data_selector = DataSelector()
        scenario_id = args.scenario_id
        is_existed = data_selector.gen_video_config(scenario_id)
        if not is_existed:
            print(0)
            return is_existed
        data_selector.gen_can_config(scenario_id)
        data_selector.gen_video_shot(scenario_id)

    replay_management = ReplayManagement()
    res = replay_management.start_replay()
    if res:
        print(replay_management.task_id)
    else:
        print(0)


if __name__ == '__main__':
    main()
