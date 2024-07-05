"""  
Author: Bu Yujun
Date: 6/21/24  
"""
import os
import re
import time
import sys

from Libs import get_project_path

sys.path.append(get_project_path())
from Utils.Libs import kill_tmux_session_if_exists, check_tmux_session_exists, get_tmux_window_content
from Utils.Libs import variables


class ReplayManagement:

    def __init__(self):
        self.task_id = 0
        self.tmux_session = variables['tmux_node']['replay'][0]
        self.tmux_window = variables['tmux_node']['replay'][1]

    def start_replay(self):
        kill_tmux_session_if_exists(self.tmux_session)

        os.system(f"tmux new-session -s {self.tmux_session} -n {self.tmux_window} -d")
        os.system(f"tmux send-keys -t {self.tmux_session}:{self.tmux_window} 'source activate base' C-m")
        time.sleep(1)
        os.system(f"tmux send-keys -t {self.tmux_session}:{self.tmux_window} 'cd ~/work/injectFile' C-m")
        os.system(f"tmux send-keys -t {self.tmux_session}:{self.tmux_window} 'python start_inject.py -e 0' C-m")

        t0 = time.time()

        while True:
            res = self.parse_replay_process()
            if res:
                self.task_id, _ = res
                print(f'回灌动作开始，task id = {self.task_id}')
                return True
            print('发出回灌指令，等待回灌开始')
            if time.time() - t0 > 15:
                self.task_id = 0
                print(f'回灌开始失败')
                return False
            time.sleep(1)

    def get_replay_process(self):
        res = self.parse_replay_process()
        if not res:
            return 0

        return res[1]

    def stop_replay(self):
        if check_tmux_session_exists(self.tmux_session):
            os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "exit" Enter')
            time.sleep(0.5)

        kill_tmux_session_if_exists(self.tmux_session)
        print(f'回灌动作结束，task id = {self.task_id}')
        self.task_id = 0

    def parse_replay_process(self):
        text = get_tmux_window_content(self.tmux_session, self.tmux_window).split('\n')
        if not any(['inject percent' in t for t in text]):
            return False

        task_id = None
        replay_process = None
        for t in text:
            if 'task_id' in t:
                matches = re.findall(r'task_id \d{1,5}', t)
                for match in matches:
                    try:
                        task_id = int(match.split(' ')[-1])
                        break
                    except:
                        task_id = None

            if 'inject percent' in t:
                try:
                    replay_process = eval(t.split(': ')[-1])
                except:
                    replay_process = None

        if task_id is None or replay_process is None:
            return False
        else:
            return task_id, replay_process


if __name__ == '__main__':
    replay_management = ReplayManagement()
    replay_management.start_replay()
    for _ in range(100):
        p = replay_management.get_replay_process()
        print(p)
        time.sleep(3)
