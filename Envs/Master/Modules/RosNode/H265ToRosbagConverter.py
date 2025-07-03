"""
@Author: BU YUJUN
@Date: 2025/4/7 14:04
"""

import argparse
import os
import shutil

import pandas as pd
import rclpy
import yaml
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage
import rosbag2_py
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py import TopicMetadata


class H265ToRosbagConverter:

    def __init__(self, h265_config_path, ros2bag_path):

        if os.path.exists(ros2bag_path):
            shutil.rmtree(ros2bag_path)

        with open(h265_config_path, 'r', encoding='utf-8') as file:
            h265_config = yaml.safe_load(file)

        # 配置输出.db3文件
        self.writer = SequentialWriter()
        storage_options = StorageOptions(
            uri=ros2bag_path,
            storage_id='sqlite3'
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.writer.open(storage_options, converter_options)

        # 创建Topic信息
        for topic in h265_config['h265']:
            topic_metadata = TopicMetadata(
                name=topic,
                type="sensor_msgs/msg/CompressedImage",
                serialization_format="cdr"
            )
            self.writer.create_topic(topic_metadata)

            frames = self.parse_h265_frames(h265_config['h265'][topic]['H265_path'])
            timestamp = pd.read_csv(h265_config['h265'][topic]['timestamp_path'], index_col=False)

            for i, frame in enumerate(frames):
                self.write_to_bag(topic, frame, i, timestamp)

    def parse_h265_frames(self, h265_path):

        frames = []
        if not isinstance(h265_path, list):
            # 解析H.265裸流文件，分离出各帧
            with open(h265_path, 'rb') as f:
                data = f.read()

            delimiter = b'\x00\x00\x00\x01\x40'
            positions = []
            start = 0

            # 查找所有匹配位置
            while True:
                pos = data.find(delimiter, start)
                if pos == -1:
                    break
                positions.append(pos)
                start = pos + 1  # 继续从下一个位置查找

            for i, pos in enumerate(positions):
                if i == len(positions) - 1:
                    frames.append(data[pos:])
                else:
                    frames.append(data[pos:positions[i + 1]])

            return frames

        else:
            for h265_file in h265_path:
                with open(h265_file, 'rb') as f:
                    frames.append(f.read())

        return frames

    def write_to_bag(self, topic, nal_data, frame_id, timestamp):
        if frame_id < len(timestamp):
            time_stamp = timestamp.at[frame_id, 'time_stamp']
            msg = CompressedImage()
            msg.header.stamp.sec = int(time_stamp)
            msg.header.stamp.nanosec = int(time_stamp*1e9 - int(time_stamp)*1e9)
            msg.header.frame_id = f'frame_{frame_id}'
            msg.format = 'h265'
            msg.data = bytes(nal_data)
            print(topic, round(time_stamp, 3), len(msg.data.tolist()), msg.data.tolist()[:30])

            # 写入数据库
            self.writer.write(
                topic,
                serialize_message(msg),
                int(time_stamp*1e9),
            )

    def close(self):
        del self.writer


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="process raw data")
    parser.add_argument("-f", "--h265_config_path", type=str, required=True, help="h265_path")
    parser.add_argument("-r", "--ros2bag_path", type=str, required=True, help="ros2bag_path")
    args = parser.parse_args()

    try:
        # 初始化ROS 2上下文（无节点）
        rclpy.init()
        converter = H265ToRosbagConverter(args.h265_config_path, args.ros2bag_path)
        converter.close()
    finally:
        rclpy.shutdown()