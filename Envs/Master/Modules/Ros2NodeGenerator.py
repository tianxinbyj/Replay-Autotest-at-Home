#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName : TestNodeGenerator.py
# @Time         : 9/7/22 2:16 PM
# @Author      : Bu Yujun

import glob
import os
import time
import pandas as pd

from Libs import get_project_path
from Utils.Libs import kill_tmux_session_if_exists, check_tmux_session_exists, docker_path
from Utils.Libs import kill_process_by_port, kill_process_by_keyword, bench_config



class Ros2NodeGenerator:

    def __init__(self, ws_folder, product='1J5'):
        self.built_packages = None
        self.created_packages = None
        self.src_lines = None
        self.make_lines = None
        self.xml_lines = None
        self.launch_lines = None
        self.topics = None
        self.msgs = None
        self.ws_folder = ws_folder
        self.product = product
        self.Packages = {}
        self.install_folder = os.path.join(self.ws_folder, 'install')
        self.src_folder = os.path.join(self.ws_folder, 'src')
        self.build_folder = os.path.join(self.ws_folder, 'build')
        self.password = bench_config['Master']['password']

        self.tmux_session = 'get_topics_session'
        self.tmux_window = 'get_topics_window'

    def get_topics(self):

        def get_lowerName(name):
            new_name = name[:-4]
            name_list = new_name.split('/')
            if '.hpp' in name:
                name_list[-1] = (''.join(name_list[-1].split('_'))).lower()
                lower_name = r'/'.join(name_list)
            else:
                name_list[-1] = name_list[-1].lower()
                lower_name = r'/'.join(name_list)
            return lower_name

        kill_tmux_session_if_exists(self.tmux_session)
        os.system(f'tmux new-session -s {self.tmux_session} -n {self.tmux_window} -d')
        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "bash {docker_path}" C-m')
        os.system('sleep 3')

        os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} '
                  f'"source {self.install_folder}/setup.bash" C-m')

        hpp_list = []
        msg_list = []
        for root, dirs, files in os.walk(self.install_folder):
            for f in files:
                ff = os.path.join(root, f)
                if 'include' in ff and '.hpp' in ff and 'detail' not in ff:
                    hpp_list.append(r'/'.join(ff.split(r'/')[-3:]))
                if 'share' in ff and '.msg' in ff and 'detail' not in ff:
                    msg_list.append(r'/'.join(ff.split(r'/')[-3:]))

        headings_all = ['index', 'topic', 'level_1', 'level_2', '.hpp', 'struct', 'msg_type', 'topic_num']
        headings_topic = ['index', 'package', 'topic', 'level_1', 'level_2', '.hpp', 'struct', 'msg_type', 'pub_count',
                          'sub_count']
        rows_all = []
        rows_topic = []
        lower_msg_list = [get_lowerName(msg) for msg in msg_list]
        topic_index = 0

        temp_info_txt = os.path.join(get_project_path(), 'Temp', 'temp_info.txt')
        if os.path.exists(temp_info_txt):
            os.remove(temp_info_txt)

        for index, item in enumerate(hpp_list):
            lower_name = get_lowerName(item)
            if lower_name in lower_msg_list:
                level_1 = item.split(r'/')[0]
                level_2 = item.split(r'/')[1]
                _hpp = item.split(r'/')[2][:-4]
                msg_index = lower_msg_list.index(lower_name)
                struct = msg_list[msg_index].split(r'/')[-1][:-4]
                msg_type = r'/'.join([level_1, level_2, struct])

                os.system(f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "ros2 topic find -c {msg_type} > {temp_info_txt}" C-m')
                topic_num = 0
                t0 = time.time()
                while True:
                    if os.path.exists(temp_info_txt):
                        time.sleep(2)
                        with open(temp_info_txt, 'r') as f:
                            captured_output = f.read()
                        topic_num = int(captured_output.split('\n')[0])
                        os.remove(temp_info_txt)
                        break
                    elif time.time() - t0 > 10:
                        break
                    else:
                        time.sleep(0.1)

                row = [index + 1, 'TBD', level_1, level_2, _hpp, struct, msg_type, topic_num]
                rows_all.append(row)
                print(row)

                if topic_num != 0:

                    os.system(
                        f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "ros2 topic find {msg_type} > {temp_info_txt}" C-m')
                    topics = []
                    t0 = time.time()
                    while True:
                        if os.path.exists(temp_info_txt):
                            time.sleep(1)
                            with open(temp_info_txt, 'r') as f:
                                captured_output = f.read()
                            topics = captured_output.split('\n')[:-1]
                            os.remove(temp_info_txt)
                            break
                        elif time.time() - t0 > 10:
                            break
                        else:
                            time.sleep(0.1)

                    for topic in topics:
                        pkg = topic.replace('/', '_')[1:].lower()
                        topic_index += 1

                        os.system(
                            f'tmux send-keys -t {self.tmux_session}:{self.tmux_window} "ros2 topic info {topic} > {temp_info_txt}" C-m')
                        pub_count, sub_count = 0, 0
                        t0 = time.time()
                        while True:
                            if os.path.exists(temp_info_txt):
                                time.sleep(1)
                                with open(temp_info_txt, 'r') as f:
                                    captured_output = f.read()
                                pub_count = int(captured_output.split('\n')[1][-1])
                                sub_count = int(captured_output.split('\n')[2][-1])
                                os.remove(temp_info_txt)
                                break
                            elif time.time() - t0 > 10:
                                break
                            else:
                                time.sleep(0.1)

                        row = [topic_index, pkg, topic, level_1, level_2, _hpp, struct, msg_type, pub_count, sub_count]
                        print(row)
                        rows_topic.append(row)

        self.msgs = pd.DataFrame(rows_all, columns=headings_all)
        self.topics = pd.DataFrame(rows_topic, columns=headings_topic)

        folder = os.path.join(get_project_path(), 'Docs', 'Resources', 'topics_and_msgs', '{:s}'.format(self.product))
        if not os.path.exists(folder):
            os.mkdir(folder)

        # 读取是否包含时间头
        topic_header_flag = []
        for idx, row in self.topics.iterrows():
            struct = row['struct']
            full_msg_path = os.path.join(self.install_folder,
                                         row['level_1'], 'share', row['level_1'],
                                         'msg', f'{struct}.msg')
            com_header_flag = False
            try:
                if os.path.exists(full_msg_path):
                    with open(full_msg_path, "r", encoding='utf-8') as f:
                        lines = f.readlines()
                        for line in lines:
                            if 'common_msgs/ComHeader header' in line:
                                com_header_flag = True
                                break
            except:
                pass

            topic_header_flag.append(com_header_flag)
        self.topics['topic_header_flag'] = topic_header_flag

        msg_file = os.path.join(folder, 'msgs.csv')
        self.msgs.to_csv(msg_file, index=False)
        topic_file = os.path.join(folder, 'topics.csv')
        self.topics.to_csv(topic_file, index=False)

    def initialize(self):
        # 读取topic和msg的对应关系
        topic_file = os.path.join(get_project_path(), 'Docs', 'Resources',
                              'topics_and_msgs', f'{self.product}', 'topics.csv')
        self.topics = pd.read_csv(topic_file, index_col=False)

        # 读取ros2 sub的模板
        subscription_template = os.path.join(get_project_path(), 'Docs', 'Resources',
                                             'topics_and_msgs', 'ros2_src_temp', 'python_subscriber.py')
        with open(subscription_template) as f:
            self.src_lines = f.readlines()

    def get_name(self, topic, port=0):
        folder = os.path.join('/tmp', 'TempRos2Functions')
        if not port:
            subscription_src = glob.glob(os.path.join(folder, f"{topic.replace('/', '')}Subscription*"))[0]
        else:
            # 如果有port传入，需要删除其他port不正确的py
            subscription_src = os.path.join(folder, f"{topic.replace('/', '')}Subscription_{port}.py")
            for f in glob.glob(os.path.join(folder, f"{topic.replace('/', '')}Subscription*")):
                if f != subscription_src:
                    os.remove(f)

        node_session = f'{os.path.splitext(os.path.basename(subscription_src))[0]}_session'
        node_window = f'{os.path.splitext(os.path.basename(subscription_src))[0]}_window'
        return subscription_src, node_session, node_window

    def generate_python_subscription(self, topic, host, port, variables):
        self.initialize()

        folder = os.path.join('/tmp', 'TempRos2Functions')
        if not os.path.exists(folder):
            os.makedirs(folder)

        data = self.topics[self.topics['topic'] == topic]
        if not len(data):
            print(f'没有找到{topic}对应的msg等信息')
            return

        subscription_src, _, _ = self.get_name(topic, port)
        kill_process_by_keyword(subscription_src)
        kill_process_by_port(port, self.password)

        new_src_lines = []
        dependency = data['level_1'].values[0]
        struct = data['struct'].values[0]
        topic_header_flag = data['topic_header_flag'].values[0]
        node_name = f"{topic.replace('/', '')}Subscription"

        for line in self.src_lines:
            if 'dependency' in line:
                line = line.replace('dependency', dependency)

            if 'topic_name' in line:
                line = line.replace('topic_name', topic)

            if 'node_name' in line:
                line = line.replace('node_name', node_name)

            if '_struct' in line:
                line = line.replace('_struct', struct)

            if 'host' in line:
                line = line.replace('host', host)

            if 'port' in line:
                line = line.replace('udp_port', str(port))

            # 插入内容
            if '# time_stamp' in line and topic_header_flag:
                line = "        msg_info['time_stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9\n"

            if '# msg_var' in line:
                line = ''
                for var in variables:
                    line += f"        if type({var}) in [int, float, str, bool]:\n"
                    line += f"            msg_info['{var}'] = {var}\n"
                    line += f"        elif type({var}) is np.ndarray:\n"
                    line += f"            msg_info['{var}'] = {var}.tolist()\n"

            new_src_lines.append(line)

        with open(subscription_src, 'w') as new_f:
            for line in new_src_lines:
                new_f.write(line)

    def run_python_subscription(self, topic):
        subscription_src, node_session, node_window = self.get_name(topic)
        print(f'运行{os.path.basename(subscription_src)}')

        # 创建一个tmux窗口，进入docker
        kill_tmux_session_if_exists(node_session)
        os.system(f'tmux new-session -s {node_session} -n {node_window} -d')
        os.system(f'tmux send-keys -t {node_session}:{node_window} "bash {docker_path}" C-m')
        time.sleep(2)
        os.system(f'tmux send-keys -t {node_session}:{node_window} "cd {self.install_folder}" C-m')
        os.system(f'tmux send-keys -t {node_session}:{node_window} "source setup.bash" C-m')
        os.system(f'tmux send-keys -t {node_session}:{node_window} "python3 {subscription_src}" C-m')

    def terminate_python_subscription(self, topic):
        subscription_src, node_session, node_window = self.get_name(topic)
        print(f'关闭{os.path.basename(subscription_src)}')

        port = int(subscription_src.split('_')[-1][:-3])
        kill_process_by_port(port, self.password)

        if check_tmux_session_exists(node_session):
            os.system(f'tmux send-keys -t {node_session}:{node_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {node_session}:{node_window} C-c')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {node_session}:{node_window} "exit" Enter')
            time.sleep(0.5)
            os.system(f'tmux send-keys -t {node_session}:{node_window} "exit" Enter')
            time.sleep(0.5)

        kill_tmux_session_if_exists(node_session)


if __name__ == "__main__":
    ws_folder = '/home/zhangliwei01/ZONE/TestProject/ES39/p_feature_20241118_010000/03_Workspace'
    RN = Ros2NodeGenerator(ws_folder=ws_folder, product='ES39')
    RN.get_topics()
