"""  
Created on 2024/8/2.  
@author: Bu Yujun  
"""
import argparse
import os.path
import sys

from Libs import get_project_path

sys.path.append(get_project_path())

from Utils.VideoProcess import extract_frames
from Envs.ReplayClient.Modules.DataSelector import data_selector


def main():
    parser = argparse.ArgumentParser(description="extract frame from video.")
    parser.add_argument("-s", "--scenario_id", type=str, required=True, help="specify scenario id")
    parser.add_argument("-f", "--frame_list", type=int, nargs='+', required=True, help="specify frame number")
    parser.add_argument("-c", "--camera", type=str, default='CAM_FRONT_120', help="specify camera")
    parser.add_argument("-p", "--pic_folder", type=str, default='None', help="specify pic path")
    args = parser.parse_args()

    video_path = data_selector.data.at[args.scenario_id, args.camera]

    if args.pic_folder == 'None':
        pic_folder = os.path.join(get_project_path(), 'Temp')
    else:
        pic_folder = args.pic_folder

    extract_frames(video_path, args.frame_list, pic_folder)
    print(pic_folder)


if __name__ == "__main__":
    main()
