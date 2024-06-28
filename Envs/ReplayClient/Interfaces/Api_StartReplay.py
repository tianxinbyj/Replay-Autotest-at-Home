"""  
Author: Bu Yujun
Date: 6/21/24  
"""
import sys

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.ReplayClient.Modules.ReplayManagement import ReplayManagement


def main():
    replay_management = ReplayManagement()
    res = replay_management.start_replay()
    if res:
        print(replay_management.task_id)
    else:
        print(0)


if __name__ == '__main__':
    main()
