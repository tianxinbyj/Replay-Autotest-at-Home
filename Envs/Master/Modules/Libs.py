"""  
Created on 2024/6/20.  
@author: Bu Yujun
"""

import os
import shutil


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
    if os.path.isfile(source):
        shutil.copy(source, destination)
    elif os.path.isdir(source):
        shutil.copytree(source, destination)
    else:
        print(f"{source}源路径不存在或既不是文件也不是文件夹")
        return None

    return destination


if __name__ == '__main__':
    import yaml
    scenario_config_yaml = '/home/byj/ZONE/TestProject/Pilot/1J5/Replay_Debug/04_TestData/TestConfig.yaml'
    with open(scenario_config_yaml, 'r', encoding='utf-8') as file:
        scenario_test_config = yaml.safe_load(file)

    old_path = '/home/byj/ZONE/TestProject/Pilot/1J5/Replay_Debug'
    new_path = '/home/byj/ZONE/TestProject/Pilot/1J5/Replay_Debug2'
    print(replace_path_in_dict(scenario_test_config, old_path, new_path))