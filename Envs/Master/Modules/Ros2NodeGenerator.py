#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName : TestNodeGenerator.py
# @Time         : 9/7/22 2:16 PM
# @Author      : Bu Yujun

import os

import pandas as pd

from Libs import get_project_path

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

    def get_topics(self):
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
        lower_msg_list = [self.get_lowerName(msg) for msg in msg_list]
        topic_index = 0
        for index, item in enumerate(hpp_list):
            lower_name = self.get_lowerName(item)
            if lower_name in lower_msg_list:
                level_1 = item.split(r'/')[0]
                level_2 = item.split(r'/')[1]
                _hpp = item.split(r'/')[2][:-4]
                msg_index = lower_msg_list.index(lower_name)
                struct = msg_list[msg_index].split(r'/')[-1][:-4]
                msg_type = r'/'.join([level_1, level_2, struct])
                cmd = 'cd {:s}; ' \
                      'source install/setup.bash; ' \
                      'ros2 topic find -c {:s}'.format(self.ws_folder, msg_type)
                print(cmd)
                p = os.popen(cmd)
                topic_num = int(p.read().split('\n')[0])
                p.close()
                rows_all.append([index + 1, 'TBD', level_1, level_2, _hpp, struct, msg_type, topic_num])
                if topic_num != 0:
                    cmd = 'cd {:s}; ' \
                          'source install/setup.bash; ' \
                          'ros2 topic find {:s}'.format(self.ws_folder, msg_type)
                    p = os.popen(cmd)
                    topics = p.read().split('\n')[:-1]
                    p.close()
                    for topic in topics:
                        pkg = topic.replace('/', '_')[1:].lower()
                        topic_index += 1
                        cmd = 'cd {:s}; ' \
                              'source install/setup.bash; ' \
                              'ros2 topic info {:s}'.format(self.ws_folder, topic)
                        p = os.popen(cmd)
                        x = p.read()
                        pub_count, sub_count = int(x.split('\n')[1][-1]), int(x.split('\n')[2][-1])
                        p.close()
                        row = [topic_index, pkg, topic, level_1, level_2, _hpp, struct, msg_type, pub_count,
                               sub_count]
                        print(row)
                        rows_topic.append(row)

        self.msgs = pd.DataFrame(rows_all, columns=headings_all)
        self.topics = pd.DataFrame(rows_topic, columns=headings_topic)

        folder = os.path.join(get_project_path(), 'Docs', 'Resources', 'topics_and_msgs', '{:s}'.format(self.product))
        if not os.path.exists(folder):
            os.mkdir(folder)

        msg_file = os.path.join(folder, 'msgs.csv')
        self.msgs.to_csv(msg_file, index=False)
        topic_file = os.path.join(folder, 'topics.csv')
        self.topics.to_csv(topic_file, index=False)

    def get_lowerName(self, name):
        new_name = name[:-4]
        name_list = new_name.split('/')
        if '.hpp' in name:
            name_list[-1] = (''.join(name_list[-1].split('_'))).lower()
            lower_name = r'/'.join(name_list)
        else:
            name_list[-1] = name_list[-1].lower()
            lower_name = r'/'.join(name_list)
        return lower_name


if __name__ == "__main__":
    ws_folder = '/home/zhangliwei01/ZONE/TestProject/2J5/p_feature_20240924_030000/03_Workspace'
    RN = Ros2NodeGenerator(ws_folder=ws_folder, product='ES39')
    RN.get_topics()
