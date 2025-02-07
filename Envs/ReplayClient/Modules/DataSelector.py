"""  
Author: Bu Yujun
Date: 6/21/24  
"""
import os
import sys
import subprocess

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Libs import get_project_path
from BenchDBMS import BenchDBMS

sys.path.append(get_project_path())
from Utils.Libs import bench_id


class DataSelector:

    def __init__(self):
        self.BenchDBMS = BenchDBMS()
        self.data = self.BenchDBMS.id_11CAN11V2Lidar_path_DF.sort_index()
        # self.data.drop(self.data[self.data['Complete'] == False].index, inplace=True)

    def gen_video_config(self, index):
        if index not in self.data.index:
            return False

        video_toml = '/home/vcar/work/injectFile/input/video.toml'

        if bench_id == 'Replay02':
            card_struct = {
                '视频注入卡-1': {
                    # 'CH1': 'CAM_FRONT_30',
                    'CH2': 'CAM_FRONT_120',
                    'CH3': 'CAM_BACK',
                },
                # '视频注入卡-2': {
                #     'CH4': 'CAM_BACK_LEFT',
                #     'CH5': 'CAM_BACK_RIGHT',
                #     'CH6': 'CAM_FRONT_LEFT',
                #     'CH7': 'CAM_FRONT_RIGHT',
                # },
                '视频注入卡-3': {
                    'CH8': 'CAM_FISHEYE_BACK',
                    'CH9': 'CAM_FISHEYE_FRONT',
                    'CH10': 'CAM_FISHEYE_LEFT',
                    'CH11': 'CAM_FISHEYE_RIGHT',
                },
            }
            video_fps = {}

        elif bench_id == 'Replay01':
            card_struct = {
                '视频注入卡-1': {
                    # 'CH1': 'CAM_FRONT_30',
                    'CH1': 'CAM_FRONT_120',
                    'CH2': 'CAM_BACK',
                },
                '视频注入卡-2': {
                    'CH4': 'CAM_BACK_RIGHT',
                    'CH5': 'CAM_FRONT_RIGHT',
                    'CH6': 'CAM_FRONT_LEFT',
                    'CH7': 'CAM_BACK_LEFT',
                },
                '视频注入卡-3': {
                    'CH8': 'CAM_FISHEYE_FRONT',
                    'CH9': 'CAM_FISHEYE_BACK',
                    'CH10': 'CAM_FISHEYE_LEFT',
                    'CH11': 'CAM_FISHEYE_RIGHT',
                },
            }
            video_fps = {
                'CH1': 15,
                'CH2': 15,
                'CH4': 15,
                'CH5': 15,
                'CH6': 15,
                'CH7': 15,
                'CH8': 30,
                'CH9': 30,
                'CH10': 30,
                'CH11': 30,
            }

        else:
            return None

        toml_content = []
        video = {}
        for card in card_struct.keys():
            toml_content.append('[[video]]')
            toml_content.append(f"card_id = '{card}'")
            for channel, camera in card_struct[card].items():
                toml_content.append('')
                toml_content.append('    [[video.channels]]')
                toml_content.append("    file = '{:s}'".format(self.data.at[index, camera]))
                toml_content.append(f"    id = '{channel}'")
                toml_content.append('    video_start_time_offset = 0')

                if video_fps:
                    toml_content.append('    fps = {:d}'.format(video_fps[channel]))

                # 生成video组，给播放使用
                video[camera.split('.')[0]] = self.data.at[index, camera]

            toml_content.append('')

        with open(video_toml, 'w') as f:
            for c in toml_content:
                f.write(c + '\n')
        print(f'{video_toml} 生成完毕')

        return video

    def gen_can_config(self, index):
        can_toml = '/home/vcar/work/injectFile/input/can.toml'

        card_struct = {
            'VCI-1': {
                'CH1': ['Safety', 4],
                'CH2': ['BKP', 6],
                'CH3': ['CH', 1],
                'CH4': ['IPS', 3],
                'CH5': ['PT', 2],
                'CH6': ['ADAS1', 7],
                'CH7': ['ADAS2', 8],
                'CH8': ['ADAS3', 9],
            },
        }

        toml_content = []
        for card in card_struct.keys():
            toml_content.append('[[can]]')
            toml_content.append(f"card_id = '{card}'")

            for channel, can in card_struct[card].items():
                toml_content.append('')
                toml_content.append('    [[can.channels]]')
                toml_content.append('    can_start_time_offset = 0')
                toml_content.append(f'    channel_in_file = {can[1]}')
                toml_content.append(f'    cmd = 0')
                toml_content.append("    file = '{:s}'".format(self.data.at[index, can[0]]))
                toml_content.append(f"    id = '{channel}'")
                toml_content.append('    is_canfd = true')

            toml_content.append('')

        with open(can_toml, 'w') as f:
            for c in toml_content:
                f.write(c + '\n')
        print(f'{can_toml} 生成完毕')

    def gen_video_shot(self, index):
        pic_path = os.path.join('/media/data/video_info', f'{index}.png')
        frame = 100

        if not os.path.exists(pic_path):
            interface_path = os.path.join(get_project_path(), 'Envs', 'ReplayClient', 'Interfaces')
            command = ['/usr/bin/python3', 'Api_CutOneFrame.py',
                       '-s', index,
                       '-f', str(frame),
                       '-p', pic_path]

            # 在指定的工作目录中执行命令
            subprocess.run(command,
                           cwd=interface_path,
                           capture_output=True,
                           text=True)

            print(f'生成{pic_path}')

        return pic_path


data_selector = DataSelector()


if __name__ == '__main__':
    DS = DataSelector()
    index = '20241018_160637_n000002'
    print(DS.data.index)
    print(index in DS.data.index)

    DS.gen_video_config(index)
    DS.gen_can_config(index)
