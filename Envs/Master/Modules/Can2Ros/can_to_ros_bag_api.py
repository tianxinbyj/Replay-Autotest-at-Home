import argparse
import datetime
import os
import subprocess
import sys
import time


def check_tmux_installed():
    """检查系统是否安装了tmux"""
    try:
        subprocess.run(['tmux', '-V'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("错误: 未找到tmux。请先安装tmux: sudo apt-get install tmux")
        return False

def session_exists(session_name):
    """检查tmux会话是否存在"""
    try:
        subprocess.run(['tmux', 'has-session', '-t', session_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        return True
    except subprocess.CalledProcessError:
        return False

def window_exists(session_name, window_name):
    """检查tmux窗口是否存在于指定会话中"""
    try:
        # 获取会话中所有窗口的列表
        result = subprocess.run(
            ['tmux', 'list-windows', '-t', session_name, '-F', '#{window_name}'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        return window_name in result.stdout
    except subprocess.CalledProcessError:
        return False

def create_tmux_session(session_name):
    """创建新的tmux会话"""
    try:
        subprocess.run(['tmux', 'new-session', '-d', '-s', session_name], check=True)
        print(f"已创建tmux会话: {session_name}")
    except subprocess.CalledProcessError as e:
        print(f"创建会话失败: {e}")
        sys.exit(1)

def create_tmux_window(session_name, window_name):
    """在指定会话中创建新窗口"""
    try:
        subprocess.run(['tmux', 'new-window', '-t', session_name, '-n', window_name], check=True)
        print(f"已在会话 '{session_name}' 中创建窗口: {window_name}")
    except subprocess.CalledProcessError as e:
        print(f"创建窗口失败: {e}")
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description="convert data to ros2 bag.")
    parser.add_argument("-i", "--install_path", required=True, help="Specify the install path.")
    parser.add_argument("-m", "--mapping",  required=True, help="can to ros2 mapping csv file config")
    parser.add_argument("-d", "--data", required=True, help="can data file")
    parser.add_argument("-n", "--name", required=True, help="ros bag name")
    parser.add_argument("-r", "--docker", required=True, help="ros docker path")
    args = parser.parse_args()

    tmux_session = 'write_ros2_bag'
    now = datetime.datetime.now()
    timestamp_str = now.strftime("%Y-%m-%d_%H-%M-%S")
    tmux_window = f'write_ros2_bag_{timestamp_str}'

    # 检查tmux是否安装
    if not check_tmux_installed():
        sys.exit(1)

    # 检查会话是否存在，不存在则创建
    if not session_exists(tmux_session):
        create_tmux_session(tmux_session)
    else:
        print(f"tmux会话 '{tmux_session}' 已存在")

    # 检查窗口是否存在，不存在则创建
    if not window_exists(tmux_session, tmux_window):
        create_tmux_window(tmux_session, tmux_window)
    else:
        print(f"窗口 '{tmux_window}' 已存在于会话 '{tmux_session}' 中")


    os.system(f'tmux send-keys -t {tmux_session}:{tmux_window}  "bash {args.docker}" C-m')
    time.sleep(2)
    os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "source {args.install_path}/setup.bash" C-m')
    script_path = os.path.dirname(os.path.abspath(__file__))
    os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "cd {script_path}" C-m')
    os.system(f'tmux send-keys -t {tmux_session}:{tmux_window} "python3 can_to_ros_bag.py -m {args.mapping} -d {args.data} -n {args.name}" C-m')


if __name__ == '__main__':
    main()
