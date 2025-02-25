"""
Author: Bu Yujun
Date: 7/2/24
"""
import argparse
import glob
import json
import os
import shutil
import sys

import time
from pathlib import Path

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.ReplayClient.Modules.DataSelector import data_selector
from Envs.ReplayClient.Modules.BirdEyeView import ConvertJsonFile, transfer_2j5_2_1j5, transfer_es39_2_1j5


def Gen_ES37_Calib_Folder():
    """

    """

    now = get_project_path()
    Temp_Path = Path(os.path.join(now, 'Temp'))

    Temp_100_Path = os.path.join(Temp_Path, 'json_calib', '100')
    Temp_101_Path = os.path.join(Temp_Path, 'json_calib', '101')
    ES37_Calib_Folder = os.path.join(Temp_Path, 'es37_calib', 'camera')

    time.sleep(0.3)
    # 创建 es37_calib 的文件夹
    # 创建11个相机各自的文件
    for i in range(11):
        time.sleep(0.01)
        os.system(f'mkdir -p {os.path.join(ES37_Calib_Folder, "camera-" + str(i))}')
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
                         }
    for camera_json in os.listdir(Temp_100_Path):
        #
        os.system(
            f'cp {os.path.join(Temp_100_Path, camera_json)} {os.path.join(ES37_Calib_Folder, camera_match_json[camera_json], "camera_0.json")}')

    for camera_json in os.listdir(Temp_101_Path):
        #
        os.system(
            f'cp {os.path.join(Temp_101_Path, camera_json)} {os.path.join(ES37_Calib_Folder, camera_match_json[camera_json], "camera_0.json")}')

    for camera_i in camera_match_json.values():

        # 给相机的json文件添加内容
        """
            "base_calib_done" : 1,
            "binning_camera_update" : 0,
            "calib_done_ts" : 0,
            "calib_src" : 1,
            "camera_intrinsic_changed" : 0,
        """
        # Done 添加字段
        camera_i_json_path = os.path.join(ES37_Calib_Folder, camera_i, "camera_0.json")
        with open(camera_i_json_path, 'r') as camera_i_file:
            camera_i_data = json.load(camera_i_file)
            camera_i_data["base_calib_done"] = 1
            camera_i_data["binning_camera_update"] = 0
            camera_i_data["calib_done_ts"] = 0
            camera_i_data["calib_src"] = 1
            camera_i_data["camera_intrinsic_changed"] = 0

            # TODO 解决 前窄30 fov÷2 的问题
            if camera_i == 'camera-10':
                camera_i_data["center_u"] = camera_i_data["center_u"] / 2
                camera_i_data["center_v"] = camera_i_data["center_v"] / 2
                camera_i_data["focal_u"] = camera_i_data["focal_u"] / 2
                camera_i_data["focal_v"] = camera_i_data["focal_v"] / 2
                camera_i_data["image_height"] = camera_i_data["image_height"] / 2
                camera_i_data["image_width"] = camera_i_data["image_width"] / 2
                camera_i_data["valid_height"][0] = camera_i_data["valid_height"][0] / 2
                camera_i_data["valid_height"][1] = camera_i_data["valid_height"][1] / 2
            time.sleep(0.2)
        with open(camera_i_json_path, 'w') as camera_i_file_change:
            json.dump(camera_i_data, camera_i_file_change, indent=4)
            time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="get video info")
    parser.add_argument("-s", "--scenario_id", type=str, required=True, help="specify scenario id")
    args = parser.parse_args()

    # 獲取標定文件
    video_path = data_selector.data.at[args.scenario_id, 'CAM_FRONT_120']
    calib_folder = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(video_path))), 'Config')
    calib_old = glob.glob(os.path.join(calib_folder, '*calibration.json'))
    calib_new = glob.glob(os.path.join(calib_folder, '*.json'))
    if len(calib_old):
        kunyi_calib_file = calib_old[0]
        origin_calib_folder = os.path.join(get_project_path(), 'Temp', 'origin_calib')
        if os.path.exists(origin_calib_folder):
            shutil.rmtree(origin_calib_folder)
        os.makedirs(origin_calib_folder)
        new_calib_file = os.path.join(origin_calib_folder, 'calibration.json')
        shutil.copyfile(kunyi_calib_file, new_calib_file)
        print('folder', origin_calib_folder)

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

        es37_calib_folder = os.path.join(get_project_path(), 'Temp', 'es37_calib')
        Gen_ES37_Calib_Folder()
        print('folder', es37_calib_folder)

    elif len(calib_new):
        json_calib_folder = os.path.join(get_project_path(), 'Temp', 'json_calib')
        if os.path.exists(json_calib_folder):
            shutil.rmtree(json_calib_folder)
        os.makedirs(json_calib_folder)
        for f in calib_new:
            shutil.copy(f, json_calib_folder)
        print('folder', json_calib_folder)

        yaml_calib_folder = os.path.join(get_project_path(), 'Temp', 'yaml_calib')
        if os.path.exists(yaml_calib_folder):
            shutil.rmtree(yaml_calib_folder)
        os.makedirs(yaml_calib_folder)
        transfer_es39_2_1j5(json_calib_folder, yaml_calib_folder)
        print('folder', yaml_calib_folder)


if __name__ == "__main__":
    main()
