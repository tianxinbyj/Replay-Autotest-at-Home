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

    def __init__(self, h265_config_path, rosbag_path):

        if os.path.exists(rosbag_path):
            shutil.rmtree(rosbag_path)

        with open(h265_config_path, 'r', encoding='utf-8') as file:
            h265_config = yaml.safe_load(file)

        # 配置输出.db3文件
        self.writer = SequentialWriter()
        storage_options = StorageOptions(
            uri=rosbag_path,
            storage_id='sqlite3'
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.writer.open(storage_options, converter_options)

        # 创建Topic信息
        for topic in h265_config:
            topic_metadata = TopicMetadata(
                name=topic,
                type="sensor_msgs/msg/CompressedImage",
                serialization_format="cdr"
            )
            self.writer.create_topic(topic_metadata)

            frames = self.parse_h265_frames(h265_config[topic]['H265_path'])
            timestamp = pd.read_csv(h265_config[topic]['timestamp_path'], index_col=False)

            for i, frame in enumerate(frames):
                self.write_to_bag(topic, frame, i, timestamp)

    def parse_h265_frames(self, h265_path):
        """解析H.265裸流文件，分离出各帧"""
        with open(h265_path, 'rb') as f:
            data = f.read()

        frames = []
        start = 0

        # 查找NAL单元起始码 (0x00000001),
        # 需要合并非00 00 00 01 02开头的（非关键帧 (P/B 帧)）
        one_frame = b''
        while True:
            # 查找下一个起始码
            pos = data.find(b'\x00\x00\x00\x01', start + 1)
            if pos == -1:
                # 添加最后一帧
                one_frame += data[start:]
                frames.append(one_frame)
                break
            # 添加当前帧
            f = data[start:pos]
            if f[4] != 2:
                one_frame += f
            else:
                if len(one_frame):
                    frames.append(one_frame)
                frames.append(data[start:pos])
                one_frame = b''
            start = pos

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
            print(topic, time_stamp, len(msg.data.tolist()), msg.data.tolist()[:30])

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
    parser.add_argument("-r", "--rosbag_path", type=str, required=True, help="rosbag_path")
    args = parser.parse_args()

    try:
        # 初始化ROS 2上下文（无节点）
        rclpy.init()
        converter = H265ToRosbagConverter(args.h265_config_path, args.rosbag_path)
        converter.close()
    finally:
        rclpy.shutdown()