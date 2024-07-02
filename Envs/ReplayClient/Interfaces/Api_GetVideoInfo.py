"""  
Author: Bu Yujun
Date: 7/2/24  
"""
import argparse
import os
import subprocess
import sys
import time

import yaml

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.ReplayClient.Modules.DataSelector import data_selector
from Utils.VideoProcess import extract_frame


def main():
    parser = argparse.ArgumentParser(description="get video info")
    parser.add_argument("-s", "--scenario_id", type=str, required=True, help="specify scenario id")
    args = parser.parse_args()

    scenario_fps_yaml = os.path.join(get_project_path(), 'Docs', 'Resources', 'scenario_info', 'scenario_fps.yaml')
    with open(scenario_fps_yaml, 'r', encoding='utf-8') as file:
        scenario_fps = yaml.safe_load(file)
    video_path = data_selector.data.at[args.scenario_id, 'CAM_FRONT_120']

    if args.scenario_id in scenario_fps:
        video_info = scenario_fps[args.scenario_id]
    else:
        y, m, d, H, M, S, ms = os.path.basename(video_path)[8:].split('_')[0].split('-')
        start_time = time.mktime(time.strptime(f'{y}-{m}-{d}-{H}-{M}-{S}', "%Y-%m-%d-%H-%M-%S")) + int(ms) / 1000
        interface_path = os.path.join(get_project_path(), 'Envs', 'ReplayClient', 'Interfaces')
        command = [
            '/usr/bin/python3', 'Api_GetVideoFps.py',
            '-s', args.scenario_id
        ]

        res = subprocess.run(command,
                             cwd=interface_path,
                             capture_output=True,
                             text=True)

        fps, duration = res.stdout.strip().split('\n')[-1].split(' ')

        video_info = {
            'fps': float(fps),
            'duration': float(duration),
            'start_date': time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time)),
            'start_time': start_time,
        }

        scenario_fps[args.scenario_id] = video_info
        with open(scenario_fps_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(scenario_fps, f, allow_unicode=True)

    # 保存視頻信息
    video_info_yaml = os.path.join(get_project_path(), 'Temp', f'{args.scenario_id}.yaml')
    with open(video_info_yaml, 'w', encoding='utf-8') as f:
        yaml.dump(video_info, f, encoding='utf-8', allow_unicode=True)
    print(video_info_yaml)

    for frame_number in [300, 3000]:
        pic_path = os.path.join('/media/data/video_info', f'{args.scenario_id}_{frame_number}.jpg')
        extract_frame(video_path, frame_number, pic_path)
        print(pic_path)


if __name__ == "__main__":
    main()
