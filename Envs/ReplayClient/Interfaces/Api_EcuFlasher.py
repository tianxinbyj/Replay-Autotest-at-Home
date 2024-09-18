import subprocess
import sys
import time

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

from Envs.ReplayClient.Modules.EcuFlasher import main as auto_flash
from Envs.ReplayClient.Modules.EcuFlasher import parse_arguments

"""
This script is used to flash the firmware on the ECU.
Arg list:
1. -a or --artifact_path软件包的本地路径
2. -t or --targets 提供需要刷写的固件，包含j5a, j5b, s32g, switch，若不提供，则不刷写
3. --type 提供连接方式，用于切换网段。包含T1和lan，若不提供，默认T1
"""


def flash_firmware():
    args = parse_arguments()
    args_list = ['-a', args.artifact_path] + ['-t'] + args.targets + ['--type', args.type]

    # Save the original sys.argv
    original_sys_argv = sys.argv

    # Replace sys.argv
    sys.argv = [sys.argv[0]] + args_list

    # Call the main function
    auto_flash()

    # Restore the original sys.argv
    sys.argv = original_sys_argv

    print('============================准备关闭刷写窗口============================')
    time.sleep(5)
    kill_tmux_session_if_exists('ses_flash')


if __name__ == "__main__":
    flash_firmware()
