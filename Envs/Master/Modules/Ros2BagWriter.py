import copy
import csv
import glob
import multiprocessing as mp
import os
import shutil
import time
import warnings
from pathlib import Path
from ctypes import *

import numpy as np
import pandas as pd
import yaml
from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.typesys import Stores, get_typestore

from MsgParser import MsgParser, CppBasicType, Cpp2pydtype

warnings.filterwarnings("ignore")


class Ros2BagWriter:

    def __init__(self, workspace, product='ES37'):
        self.last_timestamp = None
        self.frame_id_saver = None
        self.time_saver = None
        self.product = product
        self.workspace = workspace
        self.install_folder = os.path.join(workspace, 'install')
        self.typestore = get_typestore(Stores.LATEST)
        msg_list = []
        for root, dirs, files in os.walk(self.install_folder):
            for f in files:
                ff = os.path.join(root, f)
                if 'share' in ff and '.msg' in ff and 'detail' not in ff:
                    msg_list.append(ff)

        # for root, dirs, files in os.walk('/opt/ros/rolling'):
        #     for f in files:
        #         ff = os.path.join(root, f)
        #         if 'share' in ff and '.msg' in ff and 'detail' not in ff:
        #             msg_list.append(ff)

        for pathstr in msg_list:
            msg_path = Path(pathstr)
            msg_def = msg_path.read_text(encoding='utf-8')
            temp = get_types_from_msg(msg_def, self.getMsgType(msg_path))
            self.typestore.register(temp)

        print(self.typestore.types.keys())

    def getMsgType(self, path: Path) -> str:
        name = path.relative_to(path.parents[2]).with_suffix('')
        if 'msg' not in name.parts:
            name = name.parent / 'msg' / name.name
        return str(name)

    def init_message(self, topic):

        def get_size(struct_name):
            if struct_name in CppBasicType.keys():
                return int(CppBasicType[struct_name] / 8)
            else:
                size = 0
                for value in struct_dict[struct_name].values():
                    size += get_size(value[0]) * value[1]
                return size

        msg_parser = MsgParser(self.workspace, [], self.product)
        msg_parser.parse_from_topic(topic)

        struct_dict = msg_parser.struct_dict
        struct_abbr_corr = msg_parser.struct_abbr_corr

        init_msg = {}
        for k, v in struct_dict.items():
            full_struct_name = struct_abbr_corr[k]
            class_dict = self.typestore.types[full_struct_name]
            kwargs = {}
            for sub_k, sub_v in v.items():
                if sub_k in ['local_time', 'msg_sequence'] or 'RSV' in sub_k:
                    continue

                if sub_v[0] in Cpp2pydtype.keys():
                    if sub_v[1] == 1:
                        kwargs[sub_k] = np.array([0], dtype=Cpp2pydtype[sub_v[0]])[0]
                    else:
                        kwargs[sub_k] = np.array([0] * sub_v[1], dtype=Cpp2pydtype[sub_v[0]])

                elif sub_v[0] in init_msg.keys():
                    if sub_v[1] == 1:
                        kwargs[sub_k] = init_msg[sub_v[0]]
                    else:
                        arr = np.empty(sub_v[1], dtype=np.object_)
                        for i in range(sub_v[1]):
                            arr[i] = init_msg[sub_v[0]]
                        kwargs[sub_k] = arr

            init_msg[k] = class_dict(**kwargs)

        last_key = list(struct_dict.keys())[-1]

        return init_msg[last_key], struct_abbr_corr[last_key]

    def try_write(self):

        ros2bag = '/home/zhangliwei01/rosbag_2020_03_24'
        if os.path.exists(ros2bag):
            shutil.rmtree(ros2bag)

        with Writer(ros2bag) as writer:

            new_topic = '/VA/Obstacles'
            topic = '/VA/Obstacles'
            message_zero, msg_type = self.init_message(topic)
            connection = writer.add_connection(topic=new_topic,
                                               msgtype=msg_type,
                                               typestore=self.typestore)

            for i in range(150):
                message = copy.deepcopy(message_zero)
                timestamp = time.time()
                message.obstacle_num = i % 64
                message.header.stamp.sec = int(timestamp)
                message.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
                message.exposure_time_stamp = int(timestamp * 1000)
                print(f'writing {new_topic} {msg_type} @ {timestamp}')
                outdata = self.typestore.serialize_cdr(message, msg_type)
                writer.write(connection, int(timestamp * 1e9), outdata)
                time.sleep(1 / 15)


if __name__ == '__main__':
    workspace = '/home/zhangliwei01/ZONE/TestProject/ES37_PP_Feature_20240611/03_Workspace'

    WW = Ros2BagWriter(workspace)
    WW.try_write()


