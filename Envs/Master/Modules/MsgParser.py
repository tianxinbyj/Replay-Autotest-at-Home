#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @FileName : MsgParser.py
# @Time         : 1/23/23 7:23 PM
# @Author      : Bu Yujun

import os
import pandas as pd
import time
import copy
from Libs import get_project_path

CppBasicType = {
    'double': 64,
    'float': 32,
    'uint64_t': 64,
    'int64_t': 64,
    'uint32_t': 32,
    'int32_t': 32,
    'uint16_t': 16,
    'int16_t': 16,
    'uint8_t': 8,
    'int8_t': 8,
    'bool': 8,
}

Cpp2pydtype = {
    'double': 'f8',
    'float': 'f4',
    'uint64_t': 'u8',
    'int64_t': 'i8',
    'uint32_t': 'u4',
    'int32_t': 'i4',
    'uint16_t': 'u2',
    'int16_t': 'i2',
    'uint8_t': 'u1',
    'int8_t': 'i1',
    'bool': 'bool',
}


class MsgParser:

    def __init__(self, ws_folder, ignore_var, product='ES37'):
        self.struct_dict = None
        self.struct_abbr_corr = {}
        self.ws_folder = ws_folder
        self.install_folder = os.path.join(ws_folder, 'install')
        self.product = product
        topic_file = os.path.join(get_project_path(), 'Docs/Resources/topics_and_msgs/{:s}/topics.csv'.format(product))
        self.topics = pd.read_csv(topic_file, index_col=False)
        self.other_include = ['/opt/ros/rolling/include']
        # 不欢迎的变量
        self.ignore_var = ignore_var

    # iteration
    def get_msg_struct(self, full_msg):

        def get_msg_file(dependency, hpp):
            path = os.path.join(self.install_folder, dependency, 'include',
                                dependency, dependency, 'msg/detail', f'{hpp}_struct.hpp')

            if os.path.exists(path):
                with open(path) as f:
                    return f.readlines()
            else:
                for inc in self.other_include:
                    print(dependency)
                    path = os.path.join(inc, dependency, 'msg/detail', f'{hpp}_struct.hpp')
                    if os.path.exists(path):
                        with open(path) as f:
                            return f.readlines()
                return []

        def get_hpp_from_msg(msg):
            pos = [0]
            for i in range(len(msg) - 1):
                if not msg[i].isupper() and msg[i + 1].isupper() and i + 1 not in pos:
                    pos.append(i + 1)
                elif msg[i].isupper() and msg[i + 1].islower() and i not in pos:
                    pos.append(i)
            pos.append(len(msg))
            parts = [msg[pos[j]: pos[j + 1]].lower() for j in range(len(pos) - 1)]
            return '_'.join(parts)

        if full_msg[-1] != '_':
            full_msg += '_'
        msg_split = full_msg.split('::')
        dependency = msg_split[0]
        hpp = get_hpp_from_msg(msg_split[-1])
        lines = get_msg_file(dependency, hpp)
        if not len(lines):
            print('{:s} is not found!'.format(full_msg))
            return False

        start_line_number = 0
        end_line_number = 0
        for i, line in enumerate(lines):
            if 'field types and members' in line:
                start_line_number = i + 1
            if 'setters for named parameter idiom' in line:
                end_line_number = i - 1

        struct = {}
        for i in range(start_line_number, end_line_number):
            line = lines[i].strip().strip('\n').strip(';')
            if 'using ' in line:
                line_1 = lines[i + 1].strip().strip('\n').strip(';')
                line_2 = lines[i + 2].strip().strip('\n').strip(';')
                # print(line_1, line_2)
                var_name = line_2.split(' ')[-1]
                # 不欢迎的变量
                if sum([igvar in var_name for igvar in self.ignore_var]) > 0:
                    continue

                if line_1 in CppBasicType.keys():
                    struct[var_name] = [line_1, line_1, 1]
                elif 'std::array<' in line_1:
                    temp = line_1[11: -1].split(', ')
                    if temp[0] in CppBasicType.keys():
                        struct[var_name] = [temp[0], temp[0], int(temp[-1])]
                    else:
                        temp_full_msg = temp[0].split('<ContainerAllocator>')[0]
                        res = self.get_msg_struct(temp_full_msg)
                        if not res:
                            continue
                        struct[var_name] = [res[0].split('::')[-1], res[1], int(temp[-1])]
                else:
                    temp_full_msg = line_1.split('<ContainerAllocator>')[0]
                    res = self.get_msg_struct(temp_full_msg)
                    if not res:
                        continue
                    struct[var_name] = [res[0].split('::')[-1], res[1], 1]
                struct[var_name].append(msg_split[-1])
                # struct_last_name, [udp_struct变量名，msg变量类型，个数]

                if msg_split[-1] not in self.struct_abbr_corr.keys():
                    self.struct_abbr_corr[msg_split[-1]] = '/'.join(msg_split)[:-1]

        return full_msg, struct

    def parse_from_topic(self, topic, existed_struct=None):
        if existed_struct is None:
            existed_struct = []
        row = self.topics[self.topics['topic'] == topic][['level_1', 'level_2', 'struct']].values[0].tolist()
        full_msg = '::'.join(row)
        print('===================================')
        print('正在解析 {:s}'.format(full_msg))
        msg = MsgStruct(topic, *self.get_msg_struct(full_msg), existed_struct)
        self.struct_dict = msg.struct_dict
        return msg.struct_name, msg.mapping_text, msg.cpp_struct_text, msg.python_struct_text, msg.existed_struct


class MsgStruct:

    def __init__(self, topic, full_struct_name, struct_dict, existed_struct=None):
        if existed_struct is None:
            existed_struct = []
        self.existed_struct = existed_struct
        self.full_struct_name = full_struct_name.strip('_')
        self.struct_name = full_struct_name.split('::')[-1]
        self.define_topic = topic.replace('/', '_')[1:]
        self.struct_dict = {}
        # print(struct_dict)
        self.mapping_text = []
        self.cpp_struct_text = []
        self.python_struct_text = []
        self.main_struct = 'm_structUDP.zone_{:s}'.format(self.define_topic)
        self.struct_list = [self.struct_name, struct_dict, 1, None]
        self.layer_index = {i + 2: 'i' * (i + 1) for i in range(5)}
        # 解析struct的层层结构
        self.parse_struct('aaa', self.struct_list)
        # udp struct和ros msg的mapping关系
        self.generate_mapping_text()
        # 为udp struct排序, 增加offset和补充长度 为 64位的倍数
        self.expand_struct()
        # udp struct和python结构体
        self.generate_struct_text()
        # self.show()

    def show(self):
        for key in self.struct_dict.keys():
            print('------------------')
            print(key, self.get_size(key))
            print(self.struct_dict[key].keys())
            print(self.struct_dict[key])
        print('===============================================')
        # for line in self.mapping_text:
        #     print(line)
        # print('===============================================')
        # for line in self.cpp_struct_text:
        #     print(line)
        # print('===============================================')
        # for line in self.python_struct_text:
        #     print(line)
        # print('===============================================')

    # iteration
    def parse_struct(self, var_name, struct_list):
        struct_name, var_type, var_num, last_node = struct_list
        if isinstance(var_type, dict):
            self.struct_dict[struct_name] = {}
            for key, value in var_type.items():
                self.parse_struct(key, value)
        if last_node is not None:
            self.struct_dict[last_node][var_name] = [struct_name, var_num]

    def generate_mapping_text(self):
        mp_text = [
            '    void {:s}_callback(const {:s}::SharedPtr msg)'.format(self.define_topic, self.full_struct_name),
            '    {',
            '        std:: chrono::system_clock::time_point now = std::chrono::system_clock::now();',
            '        auto t = double(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()) / 1000000;',
            '        {:s}.local_time = t;'.format(self.main_struct),
            '        {:s}.msg_sequence = index_{:s};'.format(self.main_struct, self.define_topic),
            '        std::cout << "{:s} msg_sequence = " << index_{:s} << std::endl;'.format(self.define_topic,
                                                                                             self.define_topic),
            '        index_{:s} ++;'.format(self.define_topic),
            '        ',
        ]
        self.mapping_text.extend(mp_text)
        self.generate_mapping(self.struct_name, '', 2)
        self.mapping_text.append('    }')
        self.mapping_text.append('    ')

    def expand_struct(self):
        # 结构体的排序
        items = sorted(self.struct_dict.items(),
                       key=lambda d: self.get_size(d[0]))
        self.struct_dict = dict(items)

        sub_struct = list(self.struct_dict.keys())
        new_dict = {k: list(set([vv[0] for vv in v.values() if vv[0] not in CppBasicType.keys()]))
                    for k, v in self.struct_dict.items()}
        while True:
            sub_struct_copy = copy.copy(sub_struct)
            for key, value in new_dict.items():
                for keykey in value:
                    index_key = sub_struct_copy.index(key)
                    index_keykey = sub_struct_copy.index(keykey)
                    if index_key < index_keykey:
                        sub_struct_copy[index_key] = keykey
                        sub_struct_copy[index_keykey] = key
            if sub_struct_copy == sub_struct:
                break
            sub_struct = sub_struct_copy

        self.struct_dict = {key: self.struct_dict[key] for key in sub_struct}
        self.struct_dict[self.struct_name]['local_time'] = ['double', 1]
        self.struct_dict[self.struct_name]['msg_sequence'] = ['int32_t', 1]
        for key in self.struct_dict.keys():
            # 排序
            items = sorted(self.struct_dict[key].items(),
                           key=lambda d: self.get_max_item_size(d[1][0]), reverse=True)
            self.struct_dict[key] = dict(items)

        # 成员偏移量必须是成员长度的整数倍，
        # 结构体长度必须是所有成员大小的公倍数，
        # 64位系统UDP发送的结构体必须是8的整数倍
        def add_offset(sub_udp_struct):
            new_udp_struct = {}
            offset = 0
            len_list = []
            for key, value in sub_udp_struct.items():
                max_item_size = self.get_max_item_size(value[0])
                len_list.append(max_item_size)
                new_udp_struct[key] = copy.copy(value)
                residue = offset % max_item_size
                # new_udp_struct[key].append(offset)
                if residue:
                    offset_key = 'RSV{:.0f}'.format(time.time() % 1 * 1e6)
                    new_udp_struct[offset_key] = ['int8_t', max_item_size - residue]
                    offset += max_item_size - residue
                    # new_udp_struct[k].append(offset)
                offset += self.get_size(value[0]) * value[1]
            residue = offset % max(len_list)
            if residue:
                offset_key = 'RSV{:.0f}'.format(time.time() % 1 * 1e6)
                new_udp_struct[offset_key] = ['int8_t', max(len_list) - residue]
                offset += max(len_list) - residue
            return new_udp_struct, offset

        for key in self.struct_dict.keys():
            # 补充offset
            self.struct_dict[key], offset = add_offset(self.struct_dict[key])
            if key == self.struct_name:
                residue = offset % 8
                if residue:
                    last_key = 'RSV{:.0f}'.format(time.time() % 1 * 1e6)
                    self.struct_dict[key][last_key] = ['int8_t', 8 - residue]

    def generate_struct_text(self):
        for key in self.struct_dict.keys():
            if key in self.existed_struct:
                continue

            self.cpp_struct_text.append('    struct {:s}'.format(key + ' {'))
            self.python_struct_text.append('{:s} = np.dtype(['.format(key))
            for var_name, var_type in self.struct_dict[key].items():
                if var_type[1] == 1:
                    cpp_text = '        {:s} {:s};'.format(var_type[0], var_name)
                    if var_type[0] in Cpp2pydtype.keys():
                        py_text = "    ('{:s}', '{:s}'),".format(var_name, Cpp2pydtype[var_type[0]])
                    else:
                        py_text = "    ('{:s}', {:s}),".format(var_name, var_type[0])
                else:
                    cpp_text = '        {:s} {:s}[{:d}];'.format(var_type[0], var_name, var_type[1])
                    if var_type[0] in Cpp2pydtype.keys():
                        py_text = "    ('{:s}', '{:s}', {:d}),".format(var_name, Cpp2pydtype[var_type[0]], var_type[1])
                    else:
                        py_text = "    ('{:s}', {:s}, {:d}),".format(var_name, var_type[0], var_type[1])
                self.cpp_struct_text.append(cpp_text)
                self.python_struct_text.append(py_text)
            self.cpp_struct_text.append('    };')
            self.cpp_struct_text.append('    ')
            self.python_struct_text.append('])')
            self.python_struct_text.append('')

            if key not in self.existed_struct:
                self.existed_struct.append(key)

    # iteration
    def generate_mapping(self, struct, prefix, layer):
        for key, value in self.struct_dict[struct].items():
            if value[1] == 1:
                if value[0] in CppBasicType.keys():
                    self.mapping_text.append(
                        '    ' * layer + '{:s}.{:s}{:s} = msg->{:s}{:s};' \
                        .format(self.main_struct, prefix, key, prefix, key)
                    )
                else:
                    struct = value[0]
                    prefix2 = prefix + key + '.'
                    self.generate_mapping(struct, prefix2, layer)
            else:
                index = self.layer_index[layer]
                self.mapping_text.append(
                    '    ' * layer + 'for (int {:s} = 0; {:s} < int(sizeof({:s}.{:s}{:s}) / sizeof({:s})); {:s} = {:s} + 1)' \
                    .format(index, index, self.main_struct, prefix, key, value[0], index, index)
                )
                self.mapping_text.append(
                    '    ' * layer + '{'
                )
                if value[0] in CppBasicType.keys():
                    self.mapping_text.append(
                        '    ' * (layer + 1) + '{:s}.{:s}{:s}[{:s}] = msg->{:s}{:s}[{:s}];' \
                        .format(self.main_struct, prefix, key, index, prefix, key, index)
                    )
                else:
                    struct = value[0]
                    prefix2 = prefix + key + '[{:s}].'.format(index)
                    layer2 = layer + 1
                    self.generate_mapping(struct, prefix2, layer2)

                self.mapping_text.append(
                    '    ' * layer + '}'
                )

    # iteration
    def get_size(self, struct_name):
        if struct_name in CppBasicType.keys():
            return int(CppBasicType[struct_name] / 8)
        else:
            size = 0
            for value in self.struct_dict[struct_name].values():
                size += self.get_size(value[0]) * value[1]
            return size

    # iteration
    def get_max_item_size(self, struct_name):
        if struct_name in CppBasicType.keys():
            return int(CppBasicType[struct_name] / 8)
        else:
            size_list = []
            for value in self.struct_dict[struct_name].values():
                size_list.append(self.get_max_item_size(value[0]))
            return max(size_list)


if __name__ == "__main__":
    ignore_var = ['cov', '_std', '_var', 'var_', 'std_', '_dev', 'dev_', 'accel', 'reserve']
    ignore_var = []
    ws_folder = '/home/hp/artifacts/ZPD_EP39/RC11'
    MP = MsgParser(ws_folder, ignore_var)
    full_msg1 = 'common_msgs::msg::ComHeader'
    print(MP.get_msg_struct(full_msg1))
    # # topic = '/PI/EG/EgoMotionInfo'
    # # topic = '/SAFrontRadarObject'
    topic = '/VA/Obstacles'
    struct_name, mapping_text, cpp_struct_text, python_struct_text, existed_struct = MP.parse_from_topic(topic)

