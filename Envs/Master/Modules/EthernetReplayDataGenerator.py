"""  
@Author: BU YUJUN
@Date: 2025/5/26 16:21  
"""
import os
import sys
import time

import yaml

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from Libs import get_project_path
sys.path.append(get_project_path())

from Utils.VideoProcess import convert_video_h265, gen_h265_timestamp, normalize_h265_startcodes


class EthernetReplayDataGenerator:

    def __init__(self, config_path):
        with open(config_path, 'r', encoding='utf-8') as file:
            video_config = yaml.safe_load(file)
            print(video_config)

        h265_config_path = os.path.join(os.path.dirname(config_path), 'H265.yaml')
        h265_config = {
            topic: {
                'timestamp_path': '0',
                'H265_path': '0',
            } for topic in video_config.keys()
        }
        for topic, info in video_config.items():
            fps = info['fps']
            video_path = info['path']
            h265_path = os.path.join(os.path.dirname(video_path), f"{topic.replace('/', '')}.h265")
            convert_video_h265(video_path, fps, h265_path)
            timestamp_path = os.path.join(os.path.dirname(video_path), f"{topic.replace('/', '')}.csv")
            gen_h265_timestamp(h265_path, timestamp_path)
            normalized_h265_path = os.path.join(os.path.dirname(video_path), f"{topic.replace('/', '')}_norm.h265")
            normalize_h265_startcodes(h265_path, normalized_h265_path)
            h265_config[topic]['timestamp_path'] = timestamp_path
            h265_config[topic]['H265_path'] = normalized_h265_path

        with open(h265_config_path, 'w', encoding='utf-8') as file:
            yaml.dump(h265_config, file, sort_keys=False, indent=2, allow_unicode=True)


if __name__ == '__main__':
    t0 = time.time()
    config_path = '/home/hp/ZONE/temp/Config.yaml'
    ee = EthernetReplayDataGenerator(config_path)
    print(time.time() - t0)