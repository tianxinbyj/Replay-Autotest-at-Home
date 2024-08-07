"""  
Author: Bu Yujun
Date: 7/2/24  
"""
import argparse
import os.path
import sys

from Libs import get_project_path

sys.path.append(get_project_path())

from Utils.VideoProcess import extract_frame
from Envs.ReplayClient.Modules.DataSelector import data_selector


def main():
    parser = argparse.ArgumentParser(description="extract frame from video.")
    parser.add_argument("-s", "--scenario_id", type=str, required=True, help="specify scenario id")
    parser.add_argument("-f", "--frame_number", type=int, required=True, help="specify frame number")
    parser.add_argument("-c", "--camera", type=str, default='CAM_FRONT_120', help="specify camera")
    parser.add_argument("-p", "--pic_path", type=str, default='None', help="specify pic path")
    args = parser.parse_args()

    video_path = data_selector.data.at[args.scenario_id, args.camera]

    if args.pic_path == 'None':
        pic_path = os.path.join(get_project_path(), 'Temp', f'{args.camera}_{args.scenario_id}_{args.frame_number}.jpg')
    else:
        pic_path = args.pic_path

    extract_frame(video_path, args.frame_number, pic_path)
    print(pic_path)


if __name__ == "__main__":
    main()
