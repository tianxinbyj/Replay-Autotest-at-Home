import copy
import os
import shutil
import time
import warnings
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys import get_types_from_msg

from MsgParser import MsgParser, CppBasicType, Cpp2pydtype

warnings.filterwarnings("ignore")


class Ros2BagWriter:

    def __init__(self, workspace, product='ES39'):
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

        for root, dirs, files in os.walk('/opt/ros/rolling'):
            for f in files:
                ff = os.path.join(root, f)
                if 'share' in ff and '.msg' in ff and 'detail' not in ff:
                    msg_list.append(ff)

        for pathstr in msg_list:
            msg_path = Path(pathstr)
            msg_def = msg_path.read_text(encoding='utf-8')
            temp = get_types_from_msg(msg_def, self.getMsgType(msg_path))
            if list(temp.keys())[0] not in self.typestore.types.keys():
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

    def try_write(self, ros2bag_folder):

        if os.path.exists(ros2bag_folder):
            shutil.rmtree(ros2bag_folder)

        with Writer(ros2bag_folder) as writer:

            new_topic = '/VA/Obstacles'
            topic = '/VA/Obstacles'
            message_zero_1, msg_type_1 = self.init_message(topic)
            connection_1 = writer.add_connection(topic=new_topic,
                                               msgtype=msg_type_1,
                                               typestore=self.typestore)

            new_topic = '/VA/Lines'
            topic = '/VA/Lines'
            message_zero_2, msg_type_2 = self.init_message(topic)
            connection_2 = writer.add_connection(topic=new_topic,
                                               msgtype=msg_type_2,
                                               typestore=self.typestore)

            t0 = time.time()
            interval = 1/15
            t_range = np.arange(t0, t0 + 10, interval)

            for i, timestamp in enumerate(t_range):
                timestamp = float(timestamp)
                message = copy.deepcopy(message_zero_1)
                message.obstacle_num = i % 64
                message.header.stamp.sec = int(timestamp)
                message.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
                message.exposure_time_stamp = int(timestamp * 1000)
                for obstacle_data in message.obstacles:
                    obstacle_data.obstacle_id = i * 100
                print(f'writing {new_topic} {msg_type_1} @ {timestamp}')
                outdata = self.typestore.serialize_cdr(message, msg_type_1)
                writer.write(connection_1, int(timestamp * 1e9), outdata)

                message = copy.deepcopy(message_zero_2)
                message.lines_num = 5
                message.header.stamp.sec = int(timestamp)
                message.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
                message.exposure_time_stamp = int(timestamp * 1000)
                for line_data in message.lines:
                    line_data.line_id = i * 100
                    line_data.lines_3d_num = 1
                    for line_3d in line_data.lines_3d:
                        line_3d.width = 90
                print(f'writing {new_topic} {msg_type_2} @ {timestamp}')
                outdata = self.typestore.serialize_cdr(message, msg_type_2)
                writer.write(connection_2, int(timestamp * 1e9), outdata)

                time.sleep(1 / 15)


if __name__ == '__main__':
    workspace = '/home/vcar/ZONE'
    ros2bag_folder = '/home/vcar/ZONE/2345'

    WW = Ros2BagWriter(workspace)
    WW.try_write(ros2bag_folder)


