"""  
Author: Bu Yujun
Date: 6/21/24  
"""

import subprocess


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
