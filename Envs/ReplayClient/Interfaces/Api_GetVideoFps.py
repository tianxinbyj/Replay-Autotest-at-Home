"""  
Author: Bu Yujun
Date: 7/2/24  
"""
import argparse
import sys

from Libs import get_project_path

sys.path.append(get_project_path())

from Utils.VideoProcess import parse_video
from Envs.ReplayClient.Modules.DataSelector import data_selector


def main():
    parser = argparse.ArgumentParser(description="get video fps and duration")
    parser.add_argument("-s", "--scenario_id", type=str, required=True, help="specify scenario id")
    args = parser.parse_args()

    video_path = data_selector.data.at[args.scenario_id, 'CAM_FRONT_120']

    parse_video(video_path)


if __name__ == "__main__":
    main()
