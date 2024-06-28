import os


def get_project_path():
    """
    获取项目路径的函数。
    遍历当前文件的父目录，直到找到包含'Tests/conftest.py'的路径为止。
    """
    folder = os.path.dirname(os.path.abspath(__file__))  # 获取当前文件所在的绝对路径的目录

    # 开始循环遍历目录
    while True:
        # 检查当前目录下是否存在'Tests/conftest.py'路径
        if os.path.exists(os.path.join(folder, 'Tests', 'conftest.py')):
            return folder  # 如果找到，则返回当前目录作为项目路径

        parent_folder = os.path.dirname(folder)  # 获取当前目录的父目录

        # 如果父目录和当前目录相同，说明已经到达文件系统的根目录
        if parent_folder == folder:
            raise Exception("未找到项目路径，请检查'Tests/conftest.py'是否存在于正确的位置。")

        folder = parent_folder  # 将父目录设置为当前目录，继续下一次循环
