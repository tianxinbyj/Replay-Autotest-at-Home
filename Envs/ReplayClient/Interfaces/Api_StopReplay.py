"""  
Author: Bu Yujun
Date: 6/21/24  
"""
import os
import sys

from Libs import get_project_path
sys.path.append(get_project_path())

from Envs.ReplayClient.Modules.ReplayManagement import ReplayManagement


def main():
    replay_management = ReplayManagement()
    replay_management.stop_replay()


if __name__ == '__main__':
    main()
