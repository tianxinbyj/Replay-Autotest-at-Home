"""  
Created on 2024/6/20.  
@author: Bu Yujun
"""

import os


def get_project_path():
    """
    获取项目路径的函数。
    遍历当前文件的父目录，直到找到包含'Tests/requirements.txt'的路径为止。
    """
    folder = os.path.dirname(os.path.abspath(__file__))  # 获取当前文件所在的绝对路径的目录
    while True:
        if os.path.exists(os.path.join(folder, 'requirements.txt')):
            return folder
        parent_folder = os.path.dirname(folder)
        if parent_folder == folder:
            raise Exception("未找到项目路径")
        folder = parent_folder
