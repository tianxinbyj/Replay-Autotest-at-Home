"""  
Author: Bu Yujun
Date: 7/2/24  
"""
import argparse
import sys

from Libs import get_project_path

sys.path.append(get_project_path())

from Utils.VideoProcess import extract_frame


def main():
    # video_path, frame_number, pic_path
    parser = argparse.ArgumentParser(description="extract frame from video.")
    parser.add_argument("-v", "--video_path", type=str, required=True, help="specify video_path")
    parser.add_argument("-f", "--frame_number", type=int, required=True, help="specify frame number")
    parser.add_argument("-p", "--pic_path", type=str, required=True, help="specify pic path")
    args = parser.parse_args()

    extract_frame(args.video_path, args.frame_number, args.pic_path)


if __name__ == "__main__":
    main()