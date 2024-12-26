"""  
Created on 2024/6/20.  
@author: Bu Yujun
"""

import os
import shutil

import requests


def get_project_path():
    """
    获取项目路径的函数。
    遍历当前文件的父目录，直到找到包含'Tests/conftest.py'的路径为止。
    """
    folder = os.path.dirname(os.path.abspath(__file__))  # 获取当前文件所在的绝对路径的目录
    while True:
        if os.path.exists(os.path.join(folder, 'requirements.txt')):
            return folder
        parent_folder = os.path.dirname(folder)
        if parent_folder == folder:
            raise Exception("未找到项目路径")
        folder = parent_folder


def replace_path_in_dict(input_dict, old_path, new_path):
    """
    递归遍历字典，找到所有是路径的值，并替换路径中的特定部分。
    """
    if isinstance(input_dict, dict):
        for key, value in input_dict.items():
            if isinstance(value, dict):
                replace_path_in_dict(value, old_path, new_path)
            elif isinstance(value, list):
                for i, item in enumerate(value):
                    if isinstance(item, dict):
                        replace_path_in_dict(item, old_path, new_path)
                    elif isinstance(item, str) and old_path in item:
                        value[i] = item.replace(old_path, new_path)
            elif isinstance(value, str) and old_path in value:
                input_dict[key] = value.replace(old_path, new_path)

    return input_dict


def copy_to_destination(source, destination_dir):
    # 确保目标文件夹存在
    if not os.path.exists(destination_dir):
        os.makedirs(destination_dir)

    destination = os.path.join(destination_dir, os.path.basename(source))
    existed_flag = os.path.exists(destination)

    if os.path.isfile(source):
        if existed_flag:
            os.remove(destination)
        shutil.copy(source, destination)

    elif os.path.isdir(source):
        if existed_flag:
            shutil.rmtree(destination)
        shutil.copytree(source, destination)

    else:
        print(f"{source}源路径不存在或既不是文件也不是文件夹")
        return None

    return destination


def draw_map(data, map_path, longitude_offset=0.01113, latitude_offset=0.00385):
    center = '{:.5f},{:.5f}'.format(data['longitude'].mean() + longitude_offset,
                                    data['latitude'].mean() + latitude_offset)
    marker = []
    for idx, row in data.iterrows():
        if idx % 1000 == 0:
            marker.append(
                '{:.5f},{:.5f}'.format(row['longitude'] + longitude_offset, row['latitude'] + latitude_offset))
    marker = '|'.join(marker)
    print(marker)
    # 你的百度地图API密钥
    api_key = 'kob5tDyCv7qDUGZrYjfLdQHTw7jtFkl3'
    # 静态地图服务的基础URL
    static_map_url = 'http://api.map.baidu.com/staticimage/v2/'
    params = {
        'ak': api_key,  # API密钥
        'center': center,
        'width': '960',  # 图片宽度
        'height': '540',  # 图片高度
        'zoom': '16',  # 地图缩放级别
        'markers': marker,  # 标记点（经度，纬度）：标记标签
        'output': 'png'  # 输出格式
    }
    full_url = f"{static_map_url}?{'&'.join([f'{k}={v}' for k, v in params.items()])}"
    response = requests.get(full_url)

    if response.status_code == 200:
        # 保存图片到文件
        with open(map_path, 'wb') as f:
            f.write(response.content)
        print(f"地图图片已保存为{map_path}")
        return 1
    else:
        print(f"请求失败，状态码：{response.status_code}")
        return 0


if __name__ == '__main__':
    d = {
        '20241111': {'longitude_offset': 0.01113, 'latitude_offset': 0.00385}
    }

    scenario_id = '20241112_073957_n000003'
    map_path = f'{scenario_id}.png'

    import pandas as pd
    path = f'/home/zhangliwei01/ZONE/TestProject/ES39/zpd_es39_manual_20241205_181840_3/01_Prediction/{scenario_id}/RawData/SAINSPVA_{scenario_id}_001_data.csv'
    data = pd.read_csv(path)
    draw_map(data, map_path, longitude_offset=0, latitude_offset=0)