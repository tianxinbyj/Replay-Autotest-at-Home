"""  
Author: Bu Yujun
Date: 6/21/24  
"""
import glob
import hashlib
import os
import platform
import re
import shutil
import signal
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


def kill_process_by_port(port, password='123456'):
    system = platform.system()

    if system == "Windows":
        # For Windows, use netstat and taskkill
        result = subprocess.run(["netstat", "-ano", "|", "findstr", f":{port}"], capture_output=True, text=True,
                                shell=True)
        if result.stdout:
            pid = result.stdout.split()[-1]
            subprocess.run(["taskkill", "/F", "/PID", pid])
        else:
            print(f"No process is listening on port {port} on Windows.")

    elif system == "Linux":
        # 使用正则表达式来匹配 PID
        command = ["sudo", "-S", "netstat", "-tulnp"]
        result = subprocess.run(command, input=f'{password}\n', encoding='utf-8', capture_output=True, text=True)
        output = result.stdout

        # 正则表达式匹配 PID（假设 PID 是一个或多个数字）
        pid_pattern = re.compile(r'\b(\d+)/')
        for line in output.splitlines():
            if f":{port} " in line:
                match = pid_pattern.search(line)
                if match:
                    pid = match.group(1)
                    kill_command = ["sudo", "-S", "kill", "-9", pid]
                    subprocess.run(kill_command, input=f'{password}\n', encoding='utf-8')
                    break
    else:
        print("Unsupported operating system.")


def kill_process_by_keyword(keyword):
    # 执行 ps aux 命令并获取输出
    result = subprocess.run(['ps', 'aux'], stdout=subprocess.PIPE, text=True)
    output = result.stdout

    # 解析输出，寻找包含关键字的进程，并提取PID
    pids = []
    for line in output.splitlines()[1:]:  # 跳过ps aux输出的标题行
        if keyword in line:
            pid = line.split()[1]  # ps aux输出的第二列是PID
            pids.append(pid)

    for pid in pids:
        try:
            # 使用os.kill发送SIGTERM信号来终止进程
            os.kill(int(pid), signal.SIGTERM)
            print(f"Killed process with PID {pid}")
        except ProcessLookupError:
            print(f"Process with PID {pid} not found or already terminated")
        except PermissionError:
            print(f"Permission denied when trying to kill PID {pid}")


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


def create_folder(path, update=True):
    if update:
        if os.path.exists(path):
            shutil.rmtree(path)
        os.makedirs(path)
    else:
        if not os.path.exists(path):
            os.makedirs(path)


def calculate_file_checksum(file_path, method='md5'):
    """
    计算文件的校验和（哈希值）。
    :param filepath: 文件的路径。
    :param method: 使用的哈希算法，默认为'md5'。
    :return: 文件的哈希值。
    """
    hash_func = getattr(hashlib, method)()
    with open(file_path, 'rb') as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_func.update(chunk)
    return hash_func.hexdigest()


bench_id = get_bench_id()
project_path = get_project_path()
bench_config = parse_bench_config()
test_encyclopaedia = parse_test_encyclopaedia()
variables = parse_code_variables()

TempFolder = os.path.join(project_path, 'Temp')
if not os.path.exists(TempFolder):
    os.makedirs(TempFolder)

font_size = 13
title_font = {
    'family': 'Ubuntu',
    'style': 'normal',
    'weight': 'normal',
    'color': 'lightskyblue',
    'size': font_size * 1.3,
}
axis_font = {
    'family': 'Ubuntu',
    'style': 'normal',
    'weight': 'normal',
    'color': 'black',
    'size': font_size,
}
axis_font_white = {
    'family': 'Ubuntu',
    'style': 'normal',
    'weight': 'normal',
    'color': 'white',
    'size': font_size,
}
text_font = {
    'family': 'sans-serif',
    'style': 'italic',
    'weight': 'normal',
    'color': 'black',
    'size': font_size * 0.65,
}
legend_font = {
    'family': 'Ubuntu',
    'style': 'normal',
    'weight': 'normal',
    'size': font_size * 0.8,
}
mpl_colors = ['#3682be', '#45a776', '#f05326', '#eed777', '#334f65', '#b3974e', '#38cb7d', '#ddae33', '#844bb3',
              '#93c555', '#5f6694', '#df3881'] * 2


if __name__ == '__main__':
    with open('123.yaml', 'w', encoding='utf-8') as f:
        yaml.dump(test_encyclopaedia,
                  f, encoding='utf-8', allow_unicode=True, sort_keys=False)
