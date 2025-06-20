import argparse
import os
import shutil

import rclpy
from rclpy.time import Time
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from gnss_imu_msgs.msg import Gnss, Imu, Inspva
from vehicle_msgs.msg import VehicleStatusIpd, VehicleMotionIpd
import pandas as pd
import numpy as np
import rclpy.serialization
import builtin_interfaces.msg
import time
import rosbag2_py
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

def split_by_operators(s):
    """将字符串按运算符和括号分割成标记列表"""
    operators = ['+', '-', '*', '/', '=', '>', '<', '>=', '<=', '==', '!=', '&&', '||', '!', '(', ')']
    tokens = []
    current = ''
    i = 0
    while i < len(s):
        # 检查是否是多字符运算符
        for op in sorted(operators, key=len, reverse=True):
            if s.startswith(op, i):
                if current:
                    tokens.append(current)
                    current = ''
                tokens.append(op)
                i += len(op)
                break
        else:
            # 不是运算符，添加到当前标记
            current += s[i]
            i += 1
    # 添加最后一个标记
    if current:
        tokens.append(current)
    return tokens


def extract_signal_names(s):
    """从字符串中提取信号名（包含字母的标记）"""
    if s == '/':
        return None
    elif is_int(s) or is_float(s):
        return float(s)
    else:
        tokens = split_by_operators(s)
        signal_names = []
        for token in tokens:
            if any(c.isalpha() for c in token) and not token.isdigit():
                signal_names.append('I' + token)
            else:
                signal_names.append(token)
        return signal_names


def read_can_data(file_name, simulation_start_time, duration):
    start_time = time.time()
    try:
        # 读取数据并设置索引
        can_data = pd.read_csv(file_name).set_index('timestamp')
        max_timestamp = can_data.index.max()
        sample_duration = max_timestamp - simulation_start_time

        # 生成目标时间点序列
        target_times = np.array(
            [simulation_start_time + i * duration for i in range(int(sample_duration / duration) + 1)])

        # 使用searchsorted快速定位每个目标时间点的位置
        indices = np.searchsorted(can_data.index, target_times, side='right') - 1

        # 过滤掉无效索引（即小于0的索引）
        valid_indices = indices[indices >= 0]

        # 提取对应行
        if len(valid_indices) > 0:
            result = can_data.iloc[valid_indices]
        else:
            result = pd.DataFrame(columns=can_data.columns)
    except Exception as e:
        print(f"Error reading CAN data: {e}")
        result = pd.DataFrame()
    end_time = time.time()
    print(f"read_can_data 函数运行时间: {end_time - start_time} 秒")
    return result


def convert_data_type(raw_value, target_type):
    """将原始值转换为指定的数据类型"""
    try:
        # 标准化类型名称（移除后缀）
        base_type = target_type.replace('_t', '')

        if base_type == 'bool':
            # 处理布尔值（支持多种表示方式）
            if isinstance(raw_value, str):
                return raw_value.lower() in ['true', '1', 'yes', 'y']
            return bool(raw_value)

        elif base_type == 'float' or base_type == 'float32' or base_type == 'float64' or base_type == 'double':
            # 处理浮点数
            return float(raw_value)

        elif base_type == 'string':
            # 处理字符串
            return str(raw_value)

        elif base_type.startswith('uint'):
            # 处理无符号整数
            bits = int(base_type.replace('uint', ''))
            max_val = (1 << bits) - 1  # 2^bits - 1
            value = int(raw_value)
            return max(0, min(value, max_val))  # 确保在有效范围内

        elif base_type.startswith('int'):
            # 处理有符号整数
            bits = int(base_type.replace('int', ''))
            max_val = (1 << (bits - 1)) - 1  # 2^(bits-1) - 1
            min_val = - (1 << (bits - 1))  # -2^(bits-1)
            value = int(raw_value)
            return max(min_val, min(value, max_val))  # 确保在有效范围内

        else:
            # 未知类型，返回原始值
            return raw_value

    except Exception as e:
        # 转换失败，记录错误并返回默认值
        print(f"类型转换失败: {raw_value} -> {target_type} ({str(e)})")
        if target_type in ['float', 'float32', 'float64', 'double']:
            return 0.0
        elif target_type.startswith(('uint', 'int')):
            return 0
        elif target_type == 'bool':
            return False
        else:
            return ""


def is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def create_message_class(topic_info):
    message_class_group = {}
    for topic in topic_info:
        if topic['name'] == '/SA/GNSS':
            gnss_msg = Gnss()
            message_class_group['/SA/GNSS'] = gnss_msg
        elif topic['name'] == '/SA/IMU':
            imu_msg = Imu()
            message_class_group['/SA/IMU'] = imu_msg
        elif topic['name'] == '/SA/INSPVA':
            inspva_msg = Inspva()
            message_class_group['/SA/INSPVA'] = inspva_msg
        elif topic['name'] == '/VA/VehicleStatusIpd':
            vehicle_status_ipd_msg = VehicleStatusIpd()
            message_class_group['/VA/VehicleStatusIpd'] = vehicle_status_ipd_msg
        elif topic['name'] == '/VA/VehicleMotionIpd':
            vehicle_motion_ipd_msg = VehicleMotionIpd()
            message_class_group['/VA/VehicleMotionIpd'] = vehicle_motion_ipd_msg
    return message_class_group

def group_csv_data(csv_data):
    # 获取列名（如果没有表头，pandas 会自动生成整数索引作为列名）
    columns = csv_data.columns

    # 按第一列分组并转换为所需的字典结构
    result_dict = {}
    for group_name, group_data in csv_data.groupby(columns[0]):
        group_list = []
        for _, row in group_data.iterrows():
            item = {
                'msg_type': row[columns[1]],
                'msg_name': row[columns[2]],
                'can_signal_name': row[columns[3]]
            }
            group_list.append(item)
        result_dict[group_name] = group_list
    return result_dict



def create_message(csv_data_group, can_data, msg_class, topic_name, index):
    """创建并填充ROS消息"""
    msg = msg_class()
    if topic_name in csv_data_group.keys():
        for item in csv_data_group[topic_name]:
            signal_name = extract_signal_names(item['can_signal_name'])
            if signal_name is not None:
                msg_value = 0
                if isinstance(signal_name, float):
                    msg_value = signal_name
                elif isinstance(signal_name, list) and len(signal_name) == 1:
                    msg_value = can_data[signal_name[0]].iloc[index]
                elif isinstance(signal_name, list) and len(signal_name) == 3:
                    signal_name_raw = ''.join(signal_name)
                    signal_name_replace = signal_name_raw.replace(signal_name[0], f"can_data[\'{signal_name[0]}\'].iloc[{index}]")
                    # print(signal_name_raw)
                    msg_value = eval(signal_name_replace)
                    # print(f"{signal_name[0]}信号值为{can_data[signal_name[0]].iloc[index]}，计算出的值为{msg_value}")
                msg_value = convert_data_type(msg_value, item['msg_type'])
                setattr(msg, item['msg_name'], msg_value)
    return msg


def write_ros_bag(csv_data_path='/home/hp/Replay-Autotest-at-Home/Envs/Master/Modules/Can2Ros/config/CAN2ROS_ES37.csv', can_data_path='/home/hp/temp/20250324_144918_n000001/ASCParseData/CAN_Trace.csv', name='/home/hp/temp/20250324_144918_n000001/ROSBAG/TEST'):
    start_total_time = time.time()
    rclpy.init()

    try:
        # 创建一个SequentialWriter对象
        writer = rosbag2_py.SequentialWriter()

        # 配置存储选项
        storage_options = rosbag2_py.StorageOptions(
            uri=name,
            storage_id='sqlite3')

        # 配置转换选项
        converter_options = ConverterOptions('', '')

        # 打开写入器
        start_open_time = time.time()
        writer.open(storage_options, converter_options)
        end_open_time = time.time()
        print(f"打开写入器运行时间: {end_open_time - start_open_time} 秒")

        # 定义QoS配置
        # qos_profile = QoSProfile(
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1,
        #     reliability=QoSReliabilityPolicy.RELIABLE,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        #     liveliness=QoSLivelinessPolicy.AUTOMATIC,
        #     avoid_ros_namespace_conventions=False
        # )
        # qos_str = "- history: 1\n  depth: 1\n  reliability: 2\n  durability:2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec:9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n\   sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions:false"
        qos_str = ""
        # 定义主题元数据
        topic_infos = [
            {
                'name': '/SA/GNSS',
                'type': 'gnss_imu_msgs/msg/Gnss',
                'serialization_format': 'cdr',
                'qos': qos_str
            },
            {
                'name': '/SA/IMU',
                'type': 'gnss_imu_msgs/msg/Imu',
                'serialization_format': 'cdr',
                'qos': qos_str
            },
            {
                'name': '/SA/INSPVA',
                'type': 'gnss_imu_msgs/msg/Inspva',
                'serialization_format': 'cdr',
                'qos': qos_str
            },
            {
                'name': '/VA/VehicleStatusIpd',
                'type': 'vehicle_msgs/msg/VehicleStatusIpd',
                'serialization_format': 'cdr',
                'qos': qos_str
            },
            {
                'name': '/VA/VehicleMotionIpd',
                'type': 'vehicle_msgs/msg/VehicleMotionIpd',
                'serialization_format': 'cdr',
                'qos': qos_str
            }
        ]

        # 为每个主题添加元数据
        start_add_topic_time = time.time()
        for info in topic_infos:
            topic_metadata = TopicMetadata(
                name=info['name'],
                type=info['type'],
                serialization_format=info['serialization_format'],
                offered_qos_profiles=info['qos']
            )
            writer.create_topic(topic_metadata)
        end_add_topic_time = time.time()
        print(f"添加主题元数据运行时间: {end_add_topic_time - start_add_topic_time} 秒")

        # 读取数据
        start_read_data_time = time.time()
        csv_data = pd.read_csv(csv_data_path)
        csv_data_grouped = group_csv_data(csv_data)
        simulation_start_time = 0.000176
        can_data = read_can_data(can_data_path, simulation_start_time, 0.01)
        end_read_data_time = time.time()
        print(f"读取数据运行时间: {end_read_data_time - start_read_data_time} 秒")

        # 确定要处理的消息数量
        num_messages = len(can_data)
        UINT32_MAX = 2 ** 32 - 1

        start_publish_time = time.time()

        for i in range(num_messages):
            start_ros_time = time.time()
            # 发布inspva消息
            inspva_msg = create_message(csv_data_grouped, can_data, Inspva, '/SA/INSPVA', i)
            inspva_msg.header.stamp.sec = int(inspva_msg.utc_time_us / 1000)
            inspva_msg.header.stamp.nanosec = int(inspva_msg.utc_time_us % 1000 * 1000000)
            inspva_msg.header.seq = i % UINT32_MAX
            inspva_msg.time_stamp.time_stamp_s = int(inspva_msg.utc_time_us / 1000)
            inspva_msg.time_stamp.time_stamp_ns = int(inspva_msg.utc_time_us % 1000 * 1000000)
            if inspva_msg.utc_time_us == 0:
                continue
            serialized_inspva_msg = rclpy.serialization.serialize_message(inspva_msg)
            writer.write('/SA/INSPVA', serialized_inspva_msg, inspva_msg.utc_time_us*1000000)

            if i % 20 == 0:
                # 发布GNSS消息
                gnss_msg = create_message(csv_data_grouped, can_data, Gnss, '/SA/GNSS', i)
                gnss_msg.header.stamp.sec = int(inspva_msg.utc_time_us / 1000)
                gnss_msg.header.stamp.nanosec = int(inspva_msg.utc_time_us % 1000 * 1000000)
                gnss_msg.header.seq = (i // 20) % UINT32_MAX
                gnss_msg.time_stamp.time_stamp_s = int(inspva_msg.utc_time_us / 1000)
                gnss_msg.time_stamp.time_stamp_ns = int(inspva_msg.utc_time_us % 1000 * 1000000)
                serialized_gnss_msg = rclpy.serialization.serialize_message(gnss_msg)
                writer.write('/SA/GNSS', serialized_gnss_msg, inspva_msg.utc_time_us*1000000)

            # 发布IMU消息
            imu_msg = create_message(csv_data_grouped, can_data, Imu, '/SA/IMU', i)
            imu_msg.header.stamp.sec = int(inspva_msg.utc_time_us / 1000)
            imu_msg.header.stamp.nanosec = int(inspva_msg.utc_time_us % 1000 * 1000000)
            imu_msg.header.seq = i % UINT32_MAX
            imu_msg.time_stamp.time_stamp_s = int(inspva_msg.utc_time_us / 1000)
            imu_msg.time_stamp.time_stamp_ns = int(inspva_msg.utc_time_us % 1000 * 1000000)
            serialized_imu_msg = rclpy.serialization.serialize_message(imu_msg)
            writer.write('/SA/IMU', serialized_imu_msg, inspva_msg.utc_time_us*1000000)

            if i % 2 == 0:
                # 发布VehicleStatusIpd消息
                vehicle_status_ipd_msg = create_message(csv_data_grouped, can_data, VehicleStatusIpd, '/VA/VehicleStatusIpd', i)
                vehicle_status_ipd_msg.header.stamp.sec = int(inspva_msg.utc_time_us / 1000)
                vehicle_status_ipd_msg.header.stamp.nanosec = int(inspva_msg.utc_time_us % 1000 * 1000000)
                vehicle_status_ipd_msg.header.seq = (i // 2) % UINT32_MAX
                serialized_vehicle_status_ipd_msg = rclpy.serialization.serialize_message(vehicle_status_ipd_msg)
                writer.write('/VA/VehicleStatusIpd', serialized_vehicle_status_ipd_msg, inspva_msg.utc_time_us*1000000)

                # 发布VehicleMotionIpd消息
                vehicle_motion_ipd_msg = create_message(csv_data_grouped, can_data, VehicleMotionIpd, '/VA/VehicleMotionIpd', i)
                vehicle_motion_ipd_msg.header.stamp.sec = int(inspva_msg.utc_time_us / 1000)
                vehicle_motion_ipd_msg.header.stamp.nanosec = int(inspva_msg.utc_time_us % 1000 * 1000000)
                vehicle_motion_ipd_msg.header.seq = (i // 2) % UINT32_MAX
                serialized_vehicle_motion_ipd_msg = rclpy.serialization.serialize_message(vehicle_motion_ipd_msg)
                writer.write('/VA/VehicleMotionIpd', serialized_vehicle_motion_ipd_msg, inspva_msg.utc_time_us*1000000)
        end_publish_time = time.time()
        print(f"消息发布运行时间: {end_publish_time - start_publish_time} 秒")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # 确保资源被正确释放
        if 'writer' in locals() and hasattr(writer, 'shutdown'):
            start_shutdown_time = time.time()
            writer.shutdown()
            end_shutdown_time = time.time()
            print(f"关闭写入器运行时间: {end_shutdown_time - start_shutdown_time} 秒")
        rclpy.shutdown()
    end_total_time = time.time()
    print(f"代码总运行时间: {end_total_time - start_total_time} 秒")


def main():
    parser = argparse.ArgumentParser(description="convert data to ros2 bag.")
    parser.add_argument("-m", "--mapping",  required=True, help="can to ros2 mapping csv file config")
    parser.add_argument("-d", "--data", required=True, help="can data file")
    parser.add_argument("-n", "--name", required=True, help="ros bag name")
    args = parser.parse_args()

    write_ros_bag(args.mapping, args.data, args.name)


if __name__ == '__main__':
    main()
