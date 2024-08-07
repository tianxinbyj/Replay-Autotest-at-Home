"""
Author: Bu Yujun
Date: 7/2/24
"""
import argparse
import glob
import os
import shutil
import sys

import time

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.ReplayClient.Modules.DataSelector import data_selector
from Envs.ReplayClient.Modules.BirdEyeView import ConvertJsonFile, transfer_2j5_2_1j5


def Gen_ES37_Calib_Folder():
    """

    """
    # print(os.getcwd())

    # Temp_Path = Path(os.getcwd())
    now = os.path.abspath(__file__)
    # print('now', now)
    for _ in range(4):
        now = os.path.dirname(now)
    Temp_Path = Path(os.path.join(now, 'Temp'))

    print(Temp_Path.parents[1])
    Temp_100_Path = os.path.join(Temp_Path, 'json_calib', '100')
    Temp_101_Path = os.path.join(Temp_Path, 'json_calib', '101')
    print(Temp_100_Path)
    ES37_Calib_Folder = os.path.join(Temp_Path, 'ES37', 'camera')
    # 创建ES37的文件夹
    # os.system(f'mkdir -p {ES37_Calib_Folder}')
    time.sleep(0.3)

    # 创建11个相机各自的文件
    for i in range(11):
        time.sleep(0.01)
        os.system(f'mkdir -p {os.path.join(ES37_Calib_Folder, "camera-" + str(i))}')  # TODO 创建文件夹名字，maybe change
    time.sleep(0.2)
    camera_match_json = {'rearright.json': 'camera-1',
                         'frontleft.json': 'camera-2',
                         'rearleft.json': 'camera-3',
                         'rear.json': 'camera-4',
                         'front.json': 'camera-5',
                         'fisheye_rear.json': 'camera-6',
                         'fisheye_front.json': 'camera-7',
                         'fisheye_left.json': 'camera-8',
                         'fisheye_right.json': 'camera-9',
                         'front_30fov.json': 'camera-10',
                         'frontright.json': 'camera-0',
                         }  # TODO 匹配的字典，maybe change
    for camera_json in os.listdir(Temp_100_Path):
        print(camera_json)
        print(os.path.join(Temp_100_Path, camera_json))
        os.system(
            f'cp {os.path.join(Temp_100_Path, camera_json)} {os.path.join(ES37_Calib_Folder, camera_match_json[camera_json], "camera_0.json")}')
        # print('resssssss ',
        #       f'cp {os.path.join(Temp_101_Path, camera_json)} {os.path.join(ES37_Calib_Folder, camera_match_json[camera_json], "camera_0.json")}')

    for camera_json in os.listdir(Temp_101_Path):
        print(camera_json)
        print(os.path.join(Temp_101_Path, camera_json))
        # print('resssssss ',f'cp {os.path.join(Temp_101_Path, camera_json)} {os.path.join(ES37_Calib_Folder, camera_match_json[camera_json], "camera_0.json")}')
        os.system(
            f'cp {os.path.join(Temp_101_Path, camera_json)} {os.path.join(ES37_Calib_Folder, camera_match_json[camera_json], "camera_0.json")}')


def main():
    parser = argparse.ArgumentParser(description="get video info")
    parser.add_argument("-s", "--scenario_id", type=str, required=True, help="specify scenario id")
    args = parser.parse_args()

    # 獲取標定文件
    video_path = data_selector.data.at[args.scenario_id, 'CAM_FRONT_120']
    calib_folder = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(video_path))), 'Config')
    calib_f = glob.glob(os.path.join(calib_folder, '*calibration.json'))
    if len(calib_f):
        kunyi_calib_file = calib_f[0]
        json_calib_folder = os.path.join(get_project_path(), 'Temp', 'json_calib')
        if os.path.exists(json_calib_folder):
            shutil.rmtree(json_calib_folder)
        os.makedirs(json_calib_folder)
        ConvertJsonFile(kunyi_calib_file, json_calib_folder)
        print('folder', json_calib_folder)

        yaml_calib_folder = os.path.join(get_project_path(), 'Temp', 'yaml_calib')
        if os.path.exists(yaml_calib_folder):
            shutil.rmtree(yaml_calib_folder)
        os.makedirs(yaml_calib_folder)

        transfer_2j5_2_1j5(json_calib_folder, yaml_calib_folder)
        print('folder', yaml_calib_folder)

        Gen_ES37_Calib_Folder()


if __name__ == "__main__":
    main()
    # Gen_ES37_Calib_Folder()
