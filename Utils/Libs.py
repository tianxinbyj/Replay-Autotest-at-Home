"""  
Author: Bu Yujun
Date: 6/21/24  
"""
import glob
import os
import subprocess
import yaml


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


def kill_tmux_session_if_exists(session_name):
    try:
        result = subprocess.run(['tmux', 'ls'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode != 0:
            print("Error listing tmux sessions:", result.stderr)
            return

        for line in result.stdout.splitlines():
            if session_name in line:
                subprocess.run(['tmux', 'kill-session', '-t', session_name], check=True)
                print(f"Killed session: {session_name}")
                return

        print(f"Session {session_name} does not exist.")

    except subprocess.CalledProcessError as e:
        print(f"Error killing tmux session: {e}")


def check_tmux_session_exists(session_name):
    try:
        cmd = f"tmux ls | grep '{session_name}'"
        result = subprocess.run(cmd, shell=True,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True)
        if result.stdout:
            return True
        else:
            return False

    except subprocess.CalledProcessError as e:
        print(f"Error checking tmux session: {e}")
        return False


def get_tmux_window_content(session_name, window_name):
    """
    获取tmux中指定窗口的内容。

    :param session_name: tmux会话的名称
    :param window_index: 窗口的索引（从0开始）
    :return: 窗口的内容
    """
    # 构造tmux命令，捕获指定窗口的内容
    command = [
        'tmux', 'capture-pane', '-pt', f'{session_name}:{window_name}', '-S', '-E'
    ]

    # 执行命令并捕获输出
    result = subprocess.run(command, capture_output=True, text=True)

    # 返回命令的输出
    return result.stdout


def get_bench_id():
    ids = glob.glob(os.path.join(get_project_path(), 'Replay0*'))
    if len(ids):
        return os.path.basename(ids[0])
    else:
        return None


def parse_bench_config():
    bench_id = get_bench_id()
    if not bench_id:
        return None

    bench_config_yaml = os.path.join(get_project_path(), 'Docs', 'Resources', 'bench_config.yaml')
    with open(bench_config_yaml, 'r', encoding='utf-8') as file:
        data = yaml.safe_load(file)
        if bench_id not in data:
            return None
        return data[bench_id]


def parse_test_encyclopaedia():
    test_encyclopaedia_yaml = os.path.join(get_project_path(), 'Docs', 'Resources', 'test_encyclopaedia.yaml')
    with open(test_encyclopaedia_yaml, 'r', encoding='utf-8') as file:
        data = yaml.safe_load(file)
        return data


def parse_code_variables():
    var_yaml = os.path.join(get_project_path(), 'Docs', 'Resources', 'variables.yaml')
    with open(var_yaml, 'r', encoding='utf-8') as file:
        data = yaml.safe_load(file)
        return data


bench_id = get_bench_id()
project_path = get_project_path()
bench_config = parse_bench_config()
test_encyclopaedia = parse_test_encyclopaedia()
variables = parse_code_variables()
TempFolder = os.path.join(project_path, 'Temp')
if not os.path.exists(TempFolder):
    os.makedirs(TempFolder)


if __name__ == '__main__':
    print(get_bench_id())
