"""
Author: Bu Yujun
Date: 7/2/24
"""
import argparse
import glob
import os
import shutil
import sys

from Libs import get_project_path

sys.path.append(get_project_path())

from Envs.ReplayClient.Modules.DataSelector import data_selector
from Envs.ReplayClient.Modules.BirdEyeView import ConvertJsonFile, transfer_2j5_2_1j5


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


if __name__ == "__main__":
    main()
